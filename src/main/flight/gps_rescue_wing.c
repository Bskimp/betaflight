/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include "platform.h"

#ifdef USE_WING
#ifdef USE_GPS_RESCUE

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/config.h"
#include "drivers/time.h"

#include "fc/core.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/autopilot.h"
#include "flight/autopilot_wing.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/position.h"

#include "io/gps.h"
#include "rx/rx.h"
#include "pg/autopilot.h"
#include "pg/gps_rescue.h"
#include "sensors/acceleration.h"

#include "gps_rescue.h"

// ---- Wing rescue phases ----

typedef enum {
    RESCUE_WING_IDLE = 0,
    RESCUE_WING_INITIALIZE,
    RESCUE_WING_ACQUIRE_HEADING,
    RESCUE_WING_CLIMB,
    RESCUE_WING_TURN_TO_HOME,
    RESCUE_WING_CRUISE,
    RESCUE_WING_ORBIT,
    RESCUE_WING_ABORT,
    RESCUE_WING_COMPLETE,
} rescueWingPhase_e;

// ---- Wing rescue failure modes ----

typedef enum {
    RESCUE_WING_HEALTHY = 0,
    RESCUE_WING_NO_HOME_POINT,
    RESCUE_WING_GPS_LOST,
    RESCUE_WING_LOW_SATS,
    RESCUE_WING_STALL,
    RESCUE_WING_NO_PROGRESS,
} rescueWingFailure_e;

// ---- Constants ----

static const float taskIntervalS = 1.0f / TASK_GPS_RESCUE_RATE_HZ;

#define HEADING_ACQUIRE_TIMEOUT_S   10.0f
#define ALTITUDE_TOLERANCE_CM       500.0f   // 5m tolerance for altitude target
#define HEADING_ON_TARGET_DECI      300      // 30 deg in decidegrees
#define INITIAL_CLIMB_CM            1000.0f  // 10m minimum initial climb
#define ORBIT_REENTRY_FACTOR        1.5f     // drift beyond 1.5x orbit radius -> re-cruise
#define CLOSE_TO_HOME_M             5.0f     // too-close abort threshold

// Sanity check thresholds
#define SANITY_CHECK_INTERVAL_S     1.0f     // run sanity checks at 1Hz
#define LOW_SATS_TIMEOUT_S          10.0f    // abort after 10s with low sats
#define STALL_TIMEOUT_S             5.0f     // abort after 5s of stall speed
#define NO_PROGRESS_TIMEOUT_S       12.0f    // abort after 12s of no progress toward home
#define NO_PROGRESS_THRESHOLD_M     5.0f     // must close at least 5m in the check window

// ---- Module state ----

static rescueWingPhase_e rescuePhase = RESCUE_WING_IDLE;
static rescueWingFailure_e rescueFailure = RESCUE_WING_HEALTHY;
static float cogGain = 1.0f;

// Sensor cache (updated every tick, deltas on new GPS only)
static float currentAltitudeCm;
static float climbRateCmS;
static uint16_t groundSpeedCmS;
static float distanceToHomeM;
static int16_t directionToHomeDeci;

// Navigation targets (computed per phase, smoothed before output)
static float targetRollCd;
static float targetPitchCd;
static float targetThrottle;

// Phase and rescue timing
static float targetAltitudeCm;
static float phaseTimerS;

// Orbit state
static int8_t orbitDirection;       // +1 = CW (right turns), -1 = CCW (left turns)
static bool orbitDirectionSet;

// Sanity check state
static float sanityTimerS;
static float secondsLowSats;
static float secondsStalled;
static float secondsNoProgress;
static float previousDistanceToHomeM;

// ---- Phase-dependent COG gain schedule ----

static float getCogGainForPhase(rescueWingPhase_e phase)
{
    switch (phase) {
    case RESCUE_WING_ACQUIRE_HEADING:
        return 3.0f;
    case RESCUE_WING_CLIMB:
        return 2.5f;
    case RESCUE_WING_TURN_TO_HOME:
        return 1.5f;
    case RESCUE_WING_CRUISE:
        return 2.5f;
    case RESCUE_WING_ORBIT:
        return 1.5f;
    case RESCUE_WING_ABORT:
        return 1.0f;
    default:
        return 1.0f;
    }
}

// ---- Sensor update ----

static void sensorUpdate(bool newGpsData)
{
    currentAltitudeCm = wingGetRescueAltitudeCm();

    if (newGpsData) {
        distanceToHomeM = (float)GPS_distanceToHome;
        directionToHomeDeci = GPS_directionToHome;
        groundSpeedCmS = gpsSol.groundSpeed;
        climbRateCmS = (float)getEstimatedVario();
    }
}

// ---- Rescue start / stop ----

static bool wasActiveRecently;
static timeUs_t lastActiveTimeUs;

static void rescueStart(void)
{
    rescuePhase = RESCUE_WING_INITIALIZE;
    rescueFailure = RESCUE_WING_HEALTHY;
    phaseTimerS = 0.0f;
    targetRollCd = 0.0f;
    targetPitchCd = 0.0f;
    targetThrottle = 0.0f;
    targetAltitudeCm = 0.0f;
    orbitDirectionSet = false;
    sanityTimerS = 0.0f;
    secondsLowSats = 0.0f;
    secondsStalled = 0.0f;
    secondsNoProgress = 0.0f;
    previousDistanceToHomeM = distanceToHomeM;
    wingResetHeadingValidity();
}

static void rescueStop(void)
{
    // Only mark as recently active if rescue was actually running (not IDLE).
    // rescueStop() is called every tick when GPS_RESCUE_MODE is inactive,
    // so without this guard, wasActiveRecently would always be true and
    // the first activation would incorrectly take the quick-reentry path.
    if (rescuePhase != RESCUE_WING_IDLE) {
        wasActiveRecently = true;
        lastActiveTimeUs = micros();
    }
    rescuePhase = RESCUE_WING_IDLE;
    rescueFailure = RESCUE_WING_HEALTHY;
    wingSetRescueInactive();

    // clear debug so blackbox doesn't show stale frozen values
    for (int i = 0; i < 8; i++) {
        DEBUG_SET(DEBUG_GPS_RESCUE_WING, i, 0);
    }
}

// ---- Sanity checks ----

static void performSanityChecks(void)
{
    const gpsRescueConfig_t *cfg = gpsRescueConfig();

    if (rescuePhase == RESCUE_WING_IDLE || rescuePhase == RESCUE_WING_INITIALIZE) {
        rescueFailure = RESCUE_WING_HEALTHY;
        secondsStalled = 0.0f;
        return;
    }

    // Already in abort -- don't re-check
    if (rescuePhase == RESCUE_WING_ABORT) {
        return;
    }

    // Phase-independent stall persistence: accumulated every tick (not
    // gated to CRUISE/ORBIT) so it survives CRUISE<->ACQUIRE_HEADING
    // reverts. A real stall drops groundspeed below both stallSpeedCmS
    // and minHeadingSpeedCmS; the resulting validity revert would reset
    // a phase-gated counter each loop, masking the stall indefinitely.
    if (groundSpeedCmS < cfg->stallSpeedCmS) {
        secondsStalled += taskIntervalS;
        if (secondsStalled >= STALL_TIMEOUT_S && rescueFailure == RESCUE_WING_HEALTHY) {
            rescueFailure = RESCUE_WING_STALL;
            return;
        }
    } else {
        secondsStalled = 0.0f;
    }

    // Run the remaining checks at ~1Hz to avoid reacting to transient GPS noise
    sanityTimerS += taskIntervalS;
    if (sanityTimerS < SANITY_CHECK_INTERVAL_S) {
        return;
    }
    sanityTimerS = 0.0f;

    // GPS fix lost
    if (!STATE(GPS_FIX)) {
        rescueFailure = RESCUE_WING_GPS_LOST;
        return;
    }

    // Low satellite count
    if (gpsSol.numSat < cfg->minSats) {
        secondsLowSats += SANITY_CHECK_INTERVAL_S;
        if (secondsLowSats >= LOW_SATS_TIMEOUT_S) {
            rescueFailure = RESCUE_WING_LOW_SATS;
            return;
        }
    } else {
        secondsLowSats = 0.0f;
    }

    // No progress toward home during cruise
    if (rescuePhase == RESCUE_WING_CRUISE) {
        const float progressM = previousDistanceToHomeM - distanceToHomeM;
        if (progressM < NO_PROGRESS_THRESHOLD_M) {
            secondsNoProgress += SANITY_CHECK_INTERVAL_S;
            if (secondsNoProgress >= NO_PROGRESS_TIMEOUT_S) {
                rescueFailure = RESCUE_WING_NO_PROGRESS;
                return;
            }
        } else {
            secondsNoProgress = 0.0f;
        }
        previousDistanceToHomeM = distanceToHomeM;
    }
}

// Handle failure: route to ABORT based on sanity config
static void handleFailure(void)
{
    if (rescueFailure == RESCUE_WING_HEALTHY) {
        return;
    }

    const gpsRescueConfig_t *cfg = gpsRescueConfig();
    const bool hardFailsafe = !isRxReceivingSignal();

    switch (cfg->sanityChecks) {
    case RESCUE_SANITY_ON:
        rescuePhase = RESCUE_WING_ABORT;
        phaseTimerS = 0.0f;
        break;
    case RESCUE_SANITY_FS_ONLY:
        if (hardFailsafe) {
            rescuePhase = RESCUE_WING_ABORT;
            phaseTimerS = 0.0f;
        }
        break;
    default:
        // Even with sanity off, abort if no home and no RC link
        if (cfg->allowArmingWithoutFix && !STATE(GPS_FIX_HOME) && hardFailsafe) {
            rescuePhase = RESCUE_WING_ABORT;
            phaseTimerS = 0.0f;
        }
        break;
    }
}

// ---- Phase logic ----

static void updatePhase(void)
{
    const gpsRescueConfig_t *cfg = gpsRescueConfig();
    const float returnAltCm = (float)cfg->returnAltitudeM * 100.0f;
    const int16_t currentHeadingDeci = (int16_t)gpsSol.groundCourse;

    phaseTimerS += taskIntervalS;

    switch (rescuePhase) {

    case RESCUE_WING_IDLE:
        targetRollCd = 0.0f;
        targetPitchCd = 0.0f;
        targetThrottle = 0.0f;
        break;

    case RESCUE_WING_INITIALIZE:
    {
        // Seed smoothing filters FIRST so any ABORT exit below has valid
        // filter state (previously an INITIALIZE->ABORT path left the
        // PT1 filters holding stale values from a prior rescue session).
        const bool quickReentry = wasActiveRecently
            && (cmpTimeUs(micros(), lastActiveTimeUs) < 500000); // 500ms
        wasActiveRecently = false;

        if (quickReentry) {
            wingReseedSmoothing();
        } else {
            const float initialThrottle = (float)cfg->cruiseThrottle / 100.0f;
            wingInitSmoothing(initialThrottle, (float)TASK_GPS_RESCUE_RATE_HZ);
        }

        // No home point -> abort
        if (!STATE(GPS_FIX_HOME)) {
            rescuePhase = RESCUE_WING_ABORT;
            phaseTimerS = 0.0f;
            break;
        }

        // Too close to home -> abort (enforces minStartDistM from config)
        if (distanceToHomeM < fmaxf(CLOSE_TO_HOME_M, (float)cfg->minStartDistM)) {
            rescuePhase = RESCUE_WING_ABORT;
            phaseTimerS = 0.0f;
            break;
        }

        // Set target altitude: at least initialClimb above current, or return altitude
        targetAltitudeCm = fmaxf(currentAltitudeCm + INITIAL_CLIMB_CM, returnAltCm);

        // Transition to heading acquisition
        rescuePhase = RESCUE_WING_ACQUIRE_HEADING;
        phaseTimerS = 0.0f;
        break;
    }

    case RESCUE_WING_ACQUIRE_HEADING:
    {
        // Wings level, climb gently, wait for COG to stabilize
        targetRollCd = 0.0f;

        const float altErrorCm = targetAltitudeCm - currentAltitudeCm;
        float pitchDeg = wingAltitudeControl(altErrorCm, climbRateCmS, cfg->altP);
        targetPitchCd = pitchDeg * 100.0f;
        targetThrottle = wingThrottleControl(pitchDeg, cfg->cruiseThrottle, cfg->minThrottle);

        // Check heading validity
        wingUpdateHeadingValidity(groundSpeedCmS, cfg->minHeadingSpeedCmS);

        if (wingIsHeadingValid()) {
            // Decide next phase based on current altitude
            if (currentAltitudeCm >= (targetAltitudeCm - ALTITUDE_TOLERANCE_CM)) {
                rescuePhase = RESCUE_WING_TURN_TO_HOME;
            } else {
                rescuePhase = RESCUE_WING_CLIMB;
            }
            phaseTimerS = 0.0f;
            break;
        }

        // Heading acquisition timeout
        if (phaseTimerS >= HEADING_ACQUIRE_TIMEOUT_S) {
            rescuePhase = RESCUE_WING_ABORT;
            phaseTimerS = 0.0f;
        }
        break;
    }

    case RESCUE_WING_CLIMB:
    {
        // Re-check heading validity; revert if COG becomes unreliable
        wingUpdateHeadingValidity(groundSpeedCmS, cfg->minHeadingSpeedCmS);
        if (!wingIsHeadingValid()) {
            rescuePhase = RESCUE_WING_ACQUIRE_HEADING;
            phaseTimerS = 0.0f;
            wingReseedSmoothing();
            break;
        }

        // Climb to target altitude while tracking heading toward home
        const int16_t headingErr = headingErrorDeci(directionToHomeDeci, currentHeadingDeci);
        targetRollCd = wingHeadingControl(headingErr, cfg->navP, cfg->maxBankAngle);

        const float altErrorCm = targetAltitudeCm - currentAltitudeCm;
        float pitchDeg = wingAltitudeControl(altErrorCm, climbRateCmS, cfg->altP);
        pitchDeg += wingTurnCompensation(targetRollCd / 100.0f, cfg->turnCompensation);
        targetPitchCd = pitchDeg * 100.0f;
        targetThrottle = wingThrottleControl(pitchDeg, cfg->cruiseThrottle, cfg->minThrottle);

        // Transition when near target altitude
        if (currentAltitudeCm >= (targetAltitudeCm - ALTITUDE_TOLERANCE_CM)) {
            if (abs(headingErr) > HEADING_ON_TARGET_DECI) {
                rescuePhase = RESCUE_WING_TURN_TO_HOME;
            } else {
                rescuePhase = RESCUE_WING_CRUISE;
            }
            phaseTimerS = 0.0f;
        }
        break;
    }

    case RESCUE_WING_TURN_TO_HOME:
    {
        // Re-check heading validity; revert if COG becomes unreliable
        wingUpdateHeadingValidity(groundSpeedCmS, cfg->minHeadingSpeedCmS);
        if (!wingIsHeadingValid()) {
            rescuePhase = RESCUE_WING_ACQUIRE_HEADING;
            phaseTimerS = 0.0f;
            wingReseedSmoothing();
            break;
        }

        // At altitude, turn toward home
        const int16_t headingErr = headingErrorDeci(directionToHomeDeci, currentHeadingDeci);
        targetRollCd = wingHeadingControl(headingErr, cfg->navP, cfg->maxBankAngle);

        const float altErrorCm = targetAltitudeCm - currentAltitudeCm;
        float pitchDeg = wingAltitudeControl(altErrorCm, climbRateCmS, cfg->altP);
        pitchDeg += wingTurnCompensation(targetRollCd / 100.0f, cfg->turnCompensation);
        targetPitchCd = pitchDeg * 100.0f;
        targetThrottle = wingThrottleControl(pitchDeg, cfg->cruiseThrottle, cfg->minThrottle);

        // Transition when pointed at home
        if (abs(headingErr) < HEADING_ON_TARGET_DECI) {
            rescuePhase = RESCUE_WING_CRUISE;
            phaseTimerS = 0.0f;
        }
        break;
    }

    case RESCUE_WING_CRUISE:
    {
        // Re-check heading validity; revert if COG becomes unreliable
        wingUpdateHeadingValidity(groundSpeedCmS, cfg->minHeadingSpeedCmS);
        if (!wingIsHeadingValid()) {
            rescuePhase = RESCUE_WING_ACQUIRE_HEADING;
            phaseTimerS = 0.0f;
            wingReseedSmoothing();
            break;
        }

        // Fly toward home with heading + altitude control
        const int16_t headingErr = headingErrorDeci(directionToHomeDeci, currentHeadingDeci);
        targetRollCd = wingHeadingControl(headingErr, cfg->navP, cfg->maxBankAngle);

        const float altErrorCm = targetAltitudeCm - currentAltitudeCm;
        float pitchDeg = wingAltitudeControl(altErrorCm, climbRateCmS, cfg->altP);
        pitchDeg += wingTurnCompensation(targetRollCd / 100.0f, cfg->turnCompensation);
        targetPitchCd = pitchDeg * 100.0f;
        targetThrottle = wingThrottleControl(pitchDeg, cfg->cruiseThrottle, cfg->minThrottle);

        // Check for orbit entry
        const float orbitRadiusM = wingClampOrbitRadius(groundSpeedCmS, cfg->maxBankAngle, cfg->orbitRadiusM);
        if (distanceToHomeM <= orbitRadiusM) {
            // Choose orbit direction from heading error (pick the shorter
            // turn into orbit). Previously used sign(targetRollCd), which
            // biased toward whichever way the plane happened to be banked
            // at the transition instant -- approach geometry tended to
            // lock the orbit into the same direction flight after flight.
            if (!orbitDirectionSet) {
                orbitDirection = (headingErr >= 0) ? 1 : -1;
                orbitDirectionSet = true;
            }
            rescuePhase = RESCUE_WING_ORBIT;
            phaseTimerS = 0.0f;
        }
        break;
    }

    case RESCUE_WING_ORBIT:
    {
        // Fly tangent to orbit circle around home
        // CW (right turns): tangent = directionToHome - 90 deg
        // CCW (left turns): tangent = directionToHome + 90 deg
        const int16_t tangentDeci = directionToHomeDeci - orbitDirection * 900;
        const int16_t headingErr = headingErrorDeci(tangentDeci, currentHeadingDeci);
        targetRollCd = wingHeadingControl(headingErr, cfg->navP, cfg->maxBankAngle);

        // Altitude hold at orbit altitude (at least minLoiterAlt)
        const float orbitAltCm = fmaxf(targetAltitudeCm, (float)cfg->minLoiterAltM * 100.0f);
        const float altErrorCm = orbitAltCm - currentAltitudeCm;
        float pitchDeg = wingAltitudeControl(altErrorCm, climbRateCmS, cfg->altP);
        pitchDeg += wingTurnCompensation(targetRollCd / 100.0f, cfg->turnCompensation);
        targetPitchCd = pitchDeg * 100.0f;
        targetThrottle = wingThrottleControl(pitchDeg, cfg->cruiseThrottle, cfg->minThrottle);

        // If drifted too far from home, re-enter cruise to approach again
        const float orbitRadiusM = wingClampOrbitRadius(groundSpeedCmS, cfg->maxBankAngle, cfg->orbitRadiusM);
        if (distanceToHomeM > orbitRadiusM * ORBIT_REENTRY_FACTOR) {
            rescuePhase = RESCUE_WING_CRUISE;
            phaseTimerS = 0.0f;
        }
        break;
    }

    case RESCUE_WING_ABORT:
    {
        // Energy-first recovery: level wings, abort throttle
        targetRollCd = 0.0f;
        targetPitchCd = 0.0f;
        targetThrottle = (float)cfg->abortThrottle / 100.0f;
        break;
    }

    case RESCUE_WING_COMPLETE:
        // Terminal: shouldn't reach here, orbit is the terminal state
        break;
    }
}

// ---- Apply outputs ----

static void applyOutputs(void)
{
    if (rescuePhase == RESCUE_WING_IDLE) {
        gpsRescueAngle[AI_ROLL] = 0.0f;
        gpsRescueAngle[AI_PITCH] = 0.0f;
        return;
    }

    // Smooth all outputs through PT1 filters
    float rollCd = targetRollCd;
    float pitchCd = targetPitchCd;
    float throttle = targetThrottle;
    wingApplySmoothing(&rollCd, &pitchCd, &throttle);

    // Assign to rescue angle outputs (centidegrees)
    gpsRescueAngle[AI_ROLL] = rollCd;
    gpsRescueAngle[AI_PITCH] = pitchCd;

    // Set throttle for autopilot interface
    wingSetRescueThrottle(throttle);

    // Debug output
    const int16_t headingErr = headingErrorDeci(directionToHomeDeci, (int16_t)gpsSol.groundCourse);
    wingRescueDebug((uint8_t)rescuePhase, headingErr, rollCd, pitchCd,
                    distanceToHomeM, groundSpeedCmS, throttle);
}

// ---- Public interface ----

float gpsRescueAngle[RP_AXIS_COUNT] = { 0, 0 };

void gpsRescueInit(void)
{
    rescuePhase = RESCUE_WING_IDLE;
    wasActiveRecently = false;
    lastActiveTimeUs = 0;
    cogGain = 1.0f;
    gpsRescueAngle[AI_ROLL] = 0.0f;
    gpsRescueAngle[AI_PITCH] = 0.0f;
}

void gpsRescueUpdate(void)
{
    static uint16_t gpsStamp = 0;
    const bool newGpsData = gpsHasNewData(&gpsStamp);

    if (!FLIGHT_MODE(GPS_RESCUE_MODE)) {
        rescueStop();
    } else if (FLIGHT_MODE(GPS_RESCUE_MODE) && rescuePhase == RESCUE_WING_IDLE) {
        rescueStart();
    }

    sensorUpdate(newGpsData);

    // Crash detection: if aircraft has impacted, disarm immediately
    if (rescuePhase != RESCUE_WING_IDLE && crashRecoveryModeActive()) {
        setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
        disarm(DISARM_REASON_FAILSAFE);
        rescueStop();
        return;
    }

    // Sanity checks at ~1Hz, then handle any failures
    performSanityChecks();
    handleFailure();

    updatePhase();

    cogGain = getCogGainForPhase(rescuePhase);

    applyOutputs();
}

float gpsRescueGetYawRate(void)
{
    return 0.0f;
}

float gpsRescueGetImuYawCogGain(void)
{
    return cogGain;
}

bool gpsRescueIsConfigured(void)
{
    return sensors(SENSOR_ACC) && sensors(SENSOR_GPS);
}

bool gpsRescueIsAvailable(void)
{
    if (!gpsRescueIsConfigured()) {
        return false;
    }
    // Must have GPS fix and sufficient satellites to actually navigate
    if (!STATE(GPS_FIX) || gpsSol.numSat < gpsRescueConfig()->minSats) {
        return false;
    }
    // Must have a home point (unless allowArmingWithoutFix, which permits
    // arming without home -- but rescue still needs home once activated)
    if (!STATE(GPS_FIX_HOME)) {
        return false;
    }
    return true;
}

bool gpsRescueIsDisabled(void)
{
    return !gpsRescueIsConfigured();
}

const char *gpsRescueGetPhaseName(void)
{
    switch (rescuePhase) {
    case RESCUE_WING_IDLE:            return "IDLE";
    case RESCUE_WING_INITIALIZE:      return "INIT";
    case RESCUE_WING_ACQUIRE_HEADING: return "ACQH";
    case RESCUE_WING_CLIMB:           return "CLMB";
    case RESCUE_WING_TURN_TO_HOME:    return "TURN";
    case RESCUE_WING_CRUISE:          return "CRSE";
    case RESCUE_WING_ORBIT:           return "ORBT";
    case RESCUE_WING_ABORT:           return "ABRT";
    case RESCUE_WING_COMPLETE:        return "DONE";
    default:                          return "????";
    }
}

#ifdef USE_MAG
bool gpsRescueDisableMag(void)
{
    // wings use COG for heading, not magnetometer
    return true;
}
#endif

#endif // USE_GPS_RESCUE

#endif // USE_WING
