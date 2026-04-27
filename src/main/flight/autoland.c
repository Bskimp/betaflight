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

#include "platform.h"

#ifdef USE_WING

#include <math.h>
#include <stddef.h>

#include "common/maths.h"

#include "build/debug.h"

#include "flight/autoland.h"
#include "flight/nav_geom.h"
#include "flight/nav_wind.h"

// Milliseconds spent in AL_ENTRY before auto-advancing. Short dwell lets
// the wing attitude controller settle on the new pitch setpoint.
#define AL_ENTRY_DWELL_MS 500

// Earth distance per degree of latitude (constant ~111.2 km). Used for
// small-distance flat-earth pattern waypoint derivation.
#define M_PER_DEG_LAT 111195.0f

// Tangent bearing is (bearing_to_center ± 90°); cross-track correction
// nudges it inward/outward from the orbit radius. This gain controls
// how hard we pull toward the radius -- radians of bearing adjustment
// per metre of radius error. Keep small; bank comes from the P loop.
#define ORBIT_CROSS_TRACK_GAIN (0.01f)
#define ORBIT_CROSS_TRACK_MAX_RAD (M_PIf / 4.0f)   // ±45° max adjustment

// Maximum per-sample course change we'll count as "the wing actually
// turned that much" -- rejects GPS teleport jumps. At ~5 Hz sampling
// and realistic wing turn rates this is generous.
#define ORBIT_MAX_DELTA_RAD (M_PIf / 4.0f)

// Touchdown quiescence thresholds. "Quiet" = both accel and gyro
// below these bounds on the most recent IMU sample.
#define TD_QUIET_ACCEL_G      1.2f
#define TD_QUIET_GYRO_DEG_S   30.0f
// Short quiescence required to declare touchdown from AL_FLARE.
// Longer quiescence (touchdown_quiescence_ms from config) is required
// before AL_TOUCHDOWN -> AL_COMPLETE triggers the disarm.
#define TD_SHORT_QUIESCENCE_MS 500

// ---- Watchdog table (ms). Index by autolandPhase_e.
// 0 = no watchdog (IDLE / COMPLETE; their exits are event-driven).
// All other phases must have a non-zero budget so a stuck state
// machine force-aborts rather than silently continues (invariant #1).
static const uint32_t phaseWatchdogMs[AL_PHASE_COUNT] = {
    [AL_IDLE]           = 0,
    [AL_ENTRY]           = 2000,
    [AL_ORBIT_DESCENT]   = 60000,
    [AL_DOWNWIND]        = 45000,
    [AL_BASE]            = 20000,
    [AL_FINAL]           = 30000,
    [AL_STRAIGHT_IN]     = 45000,
    [AL_FLARE]           = 10000,
    [AL_TOUCHDOWN]       = 5000,
    [AL_COMPLETE]        = 0,
    [AL_ABORT]           = 2000,
};

static const char * const phaseNames[AL_PHASE_COUNT] = {
    [AL_IDLE]          = "IDLE",
    [AL_ENTRY]         = "ENTRY",
    [AL_ORBIT_DESCENT] = "ORBIT_DESCENT",
    [AL_DOWNWIND]      = "DOWNWIND",
    [AL_BASE]          = "BASE",
    [AL_FINAL]         = "FINAL",
    [AL_STRAIGHT_IN]   = "STRAIGHT_IN",
    [AL_FLARE]         = "FLARE",
    [AL_TOUCHDOWN]     = "TOUCHDOWN",
    [AL_COMPLETE]      = "COMPLETE",
    [AL_ABORT]         = "ABORT",
};

typedef struct {
    autolandPhase_e       phase;
    timeUs_t              phaseStartUs;
    timeUs_t              sequenceStartUs;
    autolandAbortCause_e  lastAbort;

    // Arm-session lock (invariant #8). Set on terminal phase
    // (COMPLETE / ABORT). Cleared only by autolandInit() -- which the
    // arming path will call on each arm event (Phase 7 wiring).
    bool                  armSessionConsumed;

    // Throttle override. Active during flight phases whenever the
    // sequence-config snapshot is valid. mixer.c reads via
    // autolandGetThrottleOverride().
    bool                  throttleOverrideActive;
    float                 throttleOverrideValue;

    // Baro reference, latched once at autolandRequestEntry() time
    // (invariant #2). Units: centimetres above whatever the sensor's
    // local zero happens to be -- we only ever use differences.
    bool                  baroRefLatched;
    float                 baroRefCm;

    // Most recent baro reading pushed in via autolandUpdate(). Used
    // to derive AGL = lastBaroCm - baroRefCm.
    float                 lastBaroCm;

    // Home coordinates, latched at RequestEntry. AL_STRAIGHT_IN and
    // AL_FINAL steer toward this point via the waypoint controller.
    float                 homeLatDeg;
    float                 homeLonDeg;

    // Last-known sensor sample. Updated lazily as the scheduler pushes
    // data in. Controllers read from here each tick.
    autolandSensorSample_t lastSensor;
    bool                  lastSensorValid;

    // Latest roll command (degrees) produced by the waypoint
    // controller. Pitch is stateless so no parallel field.
    float                 rollSetpointDeg;

    // Config snapshot. Copied from PG at RequestEntry time so live
    // edits to wingAutolandConfig() don't perturb an active sequence.
    autolandSequenceConfig_t snapshotCfg;
    bool                  cfgSnapshotValid;

    // Entry mode -- selects the flight-phase path out of AL_ENTRY.
    autolandEntryMode_e   entryMode;

    // AL_ORBIT_DESCENT tracking.
    float                 cumulativeHeadingChangeRad;  // reset on orbit-descent entry
    float                 lastObservedCourseRad;
    bool                  hasLastCourse;
    timeUs_t              orbitEnteredUs;              // for loiter timeout

    // AL_BASE tracking.
    float                 baseEnterCourseRad;
    float                 baseHeadingChangeRad;

    // Pattern waypoints, derived at AL_ORBIT_DESCENT -> AL_DOWNWIND handoff
    // from the wind estimate. Coordinates in decimal degrees.
    float                 wpDownwindEndLat, wpDownwindEndLon;   // also base start
    float                 wpFinalStartLat,  wpFinalStartLon;
    bool                  patternWaypointsValid;

    // Commit latch (invariant #5). Set when the wing drops below
    // commit_altitude_cm during a flight phase. While latched,
    // external aborts are refused so stick jitter or a swiped
    // BOXAUTOLAND can't break the flare. Internal aborts (watchdog,
    // GPS/baro loss) bypass the latch deliberately -- invariant #6.
    bool                  commitLatched;

    // Touchdown quiescence tracker. `quietSinceUs == 0` means we are
    // not currently quiet; any non-zero value is the timestamp at
    // which continuous quiet began. AL_FLARE uses TD_SHORT_QUIESCENCE_MS
    // to declare touchdown; AL_TOUCHDOWN uses snapshotCfg
    // .touchdown_quiescence_ms before AL_COMPLETE.
    timeUs_t              quietSinceUs;

    // One-shot flag set at AL_TOUCHDOWN -> AL_COMPLETE. Phase 7 arming
    // wiring polls autolandShouldDisarm() and issues the actual
    // disarm on a rising edge. Cleared by autolandInit() only.
    bool                  disarmRequested;
} autolandState_t;

static autolandState_t state;

// True for phases where autoland actively drives surfaces + throttle.
// Terminal phases (IDLE/COMPLETE/ABORT) and the pre-entry state don't
// produce control output.
static bool isFlightPhase(autolandPhase_e phase)
{
    switch (phase) {
        case AL_ENTRY:
        case AL_ORBIT_DESCENT:
        case AL_DOWNWIND:
        case AL_BASE:
        case AL_FINAL:
        case AL_STRAIGHT_IN:
        case AL_FLARE:
        case AL_TOUCHDOWN:
            return true;
        default:
            return false;
    }
}

// Flat-earth conversion from "metres offset from (lat, lon)" to a
// new (lat, lon). Adequate for pattern-waypoint work where distances
// are typically well under 1 km. deltaN/deltaE are offsets in metres
// north and east respectively.
static void offsetLatLon(float lat0, float lon0,
                         float deltaN, float deltaE,
                         float *outLat, float *outLon)
{
    *outLat = lat0 + (deltaN / M_PER_DEG_LAT);
    const float metersPerDegLon = M_PER_DEG_LAT * cosf(lat0 * (M_PIf / 180.0f));
    if (fabsf(metersPerDegLon) < 1.0f) {
        *outLon = lon0;
    } else {
        *outLon = lon0 + (deltaE / metersPerDegLon);
    }
}

// Derive the downwind-end / base-start / final-start waypoints from the
// wind vector. Called at the AL_ORBIT_DESCENT -> AL_DOWNWIND transition
// after the wind estimator has converged. Pattern direction is
// right-hand (base turn left from downwind heading) for Phase 5b; a
// future iteration could make it configurable.
static void derivePatternWaypoints(float windFromRad)
{
    const autolandSequenceConfig_t *cfg = &state.snapshotCfg;

    // Unit vector pointing in the direction the wind is coming FROM
    // (i.e. the direction the wing will fly on final).
    const float finalN =  cosf(windFromRad);
    const float finalE =  sinf(windFromRad);

    // final_start = home - final_distance_m * finalHeadingUnit
    // (i.e. the point we begin the final leg from; downwind of home).
    const float finalStartDeltaN = -(float)cfg->final_distance_m * finalN;
    const float finalStartDeltaE = -(float)cfg->final_distance_m * finalE;
    offsetLatLon(state.homeLatDeg, state.homeLonDeg,
                 finalStartDeltaN, finalStartDeltaE,
                 &state.wpFinalStartLat, &state.wpFinalStartLon);

    // Pattern direction: left turn from downwind to final means the
    // downwind leg runs to the RIGHT of the final track (looking from
    // above). Perpendicular-to-final, clockwise-90-degrees vector is
    // (finalE, -finalN). Use it to offset base-start sideways.
    const float perpN =  finalE;
    const float perpE = -finalN;

    const float baseStartDeltaN = finalStartDeltaN + (float)cfg->base_radius_m * perpN;
    const float baseStartDeltaE = finalStartDeltaE + (float)cfg->base_radius_m * perpE;
    offsetLatLon(state.homeLatDeg, state.homeLonDeg,
                 baseStartDeltaN, baseStartDeltaE,
                 &state.wpDownwindEndLat, &state.wpDownwindEndLon);

    state.patternWaypointsValid = true;
}

// Orbit controller. Produces a roll setpoint that keeps the wing
// circling around `homeLat/Lon` at `radiusM`, turning clockwise (the
// default orbit direction used by the wing rescue state machine).
// Tangent bearing = bearing_to_center + 90°; cross-track term nudges
// the bearing inward/outward to regulate the radius.
static float computeOrbitRoll(float curLat, float curLon,
                              float courseRad,
                              float homeLat, float homeLon,
                              float radiusM,
                              uint8_t maxBankDeg, uint8_t navP)
{
    const float bearingToHome = navGeomBearingRad(curLat, curLon, homeLat, homeLon);
    const float distanceM     = navGeomDistanceM(curLat, curLon, homeLat, homeLon);

    // Tangent for clockwise orbit: +90° from bearing-to-center.
    float tangent = bearingToHome + (M_PIf / 2.0f);
    if (tangent >= 2.0f * M_PIf) {
        tangent -= 2.0f * M_PIf;
    }

    // Cross-track correction: if we're outside the radius, bias inward;
    // if inside, bias outward. Clamped so we never pull harder than 45°
    // from pure tangent.
    const float radiusErr = distanceM - radiusM;
    float xtrackRad = ORBIT_CROSS_TRACK_GAIN * radiusErr;
    if (xtrackRad >  ORBIT_CROSS_TRACK_MAX_RAD) xtrackRad =  ORBIT_CROSS_TRACK_MAX_RAD;
    if (xtrackRad < -ORBIT_CROSS_TRACK_MAX_RAD) xtrackRad = -ORBIT_CROSS_TRACK_MAX_RAD;

    // When outside the radius, we want to turn MORE toward home (reduce
    // tangent angle). Subtract the correction.
    float desiredHeading = tangent - xtrackRad;
    while (desiredHeading < 0.0f)          desiredHeading += 2.0f * M_PIf;
    while (desiredHeading >= 2.0f * M_PIf) desiredHeading -= 2.0f * M_PIf;

    const float errRad = navGeomHeadingErrorRad(courseRad, desiredHeading);
    const float errDeg = errRad * (180.0f / M_PIf);
    float bankDeg = (float)navP * 0.01f * errDeg;
    const float clamp = (float)maxBankDeg;
    if (bankDeg >  clamp) bankDeg =  clamp;
    if (bankDeg < -clamp) bankDeg = -clamp;
    return bankDeg;
}

// Waypoint bank-angle controller. Proportional on heading error,
// clamped to max_bank_angle_deg. Phase 5a drives AL_STRAIGHT_IN;
// Phase 5b also uses it for AL_DOWNWIND / AL_BASE / AL_FINAL.
static float computeWaypointRoll(float curLat, float curLon,
                                 float courseRad,
                                 float tgtLat, float tgtLon,
                                 uint8_t maxBankDeg, uint8_t navP)
{
    const float targetBearing = navGeomBearingRad(curLat, curLon, tgtLat, tgtLon);
    const float errRad        = navGeomHeadingErrorRad(courseRad, targetBearing);
    const float errDeg        = errRad * (180.0f / M_PIf);
    // navP is expressed in hundredths per the rescue convention --
    // navP = 30 means 0.3 deg bank per 1 deg heading error.
    float bankDeg = (float)navP * 0.01f * errDeg;
    const float clamp = (float)maxBankDeg;
    if (bankDeg >  clamp) bankDeg =  clamp;
    if (bankDeg < -clamp) bankDeg = -clamp;
    return bankDeg;
}

// Refresh both throttle + roll setpoints from the latest sensor cache.
// Called from autolandUpdate() after sample intake. Pitch is stateless
// (pure function of phase + cfg) so it's computed in
// autolandGetPitchSetpoint() rather than here.
static void updateControllers(void)
{
    if (!isFlightPhase(state.phase) || !state.cfgSnapshotValid) {
        state.throttleOverrideActive = false;
        state.throttleOverrideValue  = 0.0f;
        state.rollSetpointDeg        = 0.0f;
        return;
    }

    const autolandSequenceConfig_t *cfg = &state.snapshotCfg;
    const float aglCm = state.lastBaroCm - state.baroRefCm;

    // Throttle policy (pitch-only glide controller; no PI loop):
    //   throttle_cut_alt_cm == 0  -> cut immediately at entry
    //   throttle_cut_alt_cm  > 0  -> hold cruise_throttle_pct while
    //                                AGL >= threshold, cut below.
    if (cfg->throttle_cut_alt_cm == 0 || aglCm < (float)cfg->throttle_cut_alt_cm) {
        state.throttleOverrideValue = 0.0f;
    } else {
        state.throttleOverrideValue = (float)cfg->cruise_throttle_pct / 100.0f;
    }
    state.throttleOverrideActive = true;

    // Roll. Dispatch per phase:
    //   AL_ORBIT_DESCENT: orbit controller (tangent + cross-track)
    //   AL_DOWNWIND:      waypoint controller toward downwind-end
    //   AL_BASE:          waypoint controller toward final-start
    //   AL_FINAL / AL_STRAIGHT_IN: waypoint controller toward home
    //   Everything else: wings-level (stateless).
    if (!state.lastSensorValid || !state.lastSensor.gpsValid) {
        state.rollSetpointDeg = 0.0f;
    } else {
        switch (state.phase) {
            case AL_ORBIT_DESCENT:
                state.rollSetpointDeg = computeOrbitRoll(
                    state.lastSensor.latDeg, state.lastSensor.lonDeg,
                    state.lastSensor.groundCourseRad,
                    state.homeLatDeg, state.homeLonDeg,
                    (float)cfg->orbit_radius_m,
                    cfg->max_bank_angle_deg, cfg->nav_p_gain);
                break;
            case AL_DOWNWIND:
                if (state.patternWaypointsValid) {
                    state.rollSetpointDeg = computeWaypointRoll(
                        state.lastSensor.latDeg, state.lastSensor.lonDeg,
                        state.lastSensor.groundCourseRad,
                        state.wpDownwindEndLat, state.wpDownwindEndLon,
                        cfg->max_bank_angle_deg, cfg->nav_p_gain);
                } else {
                    state.rollSetpointDeg = 0.0f;
                }
                break;
            case AL_BASE:
                if (state.patternWaypointsValid) {
                    state.rollSetpointDeg = computeWaypointRoll(
                        state.lastSensor.latDeg, state.lastSensor.lonDeg,
                        state.lastSensor.groundCourseRad,
                        state.wpFinalStartLat, state.wpFinalStartLon,
                        cfg->max_bank_angle_deg, cfg->nav_p_gain);
                } else {
                    state.rollSetpointDeg = 0.0f;
                }
                break;
            case AL_FINAL:
            case AL_STRAIGHT_IN:
                state.rollSetpointDeg = computeWaypointRoll(
                    state.lastSensor.latDeg, state.lastSensor.lonDeg,
                    state.lastSensor.groundCourseRad,
                    state.homeLatDeg, state.homeLonDeg,
                    cfg->max_bank_angle_deg, cfg->nav_p_gain);
                break;
            default:
                state.rollSetpointDeg = 0.0f;
                break;
        }
    }
}

// ---- Private helpers ----

static bool isTerminalPhase(autolandPhase_e phase)
{
    return phase == AL_COMPLETE || phase == AL_ABORT;
}

static void clearThrottleOverride(void)
{
    state.throttleOverrideActive = false;
    state.throttleOverrideValue = 0.0f;
}

// ---- Public API ----

void autolandInit(void)
{
    state.phase              = AL_IDLE;
    state.phaseStartUs       = 0;
    state.sequenceStartUs    = 0;
    state.lastAbort          = AL_ABORT_NONE;
    state.armSessionConsumed = false;
    state.baroRefLatched     = false;
    state.baroRefCm          = 0.0f;
    state.lastBaroCm         = 0.0f;
    state.homeLatDeg         = 0.0f;
    state.homeLonDeg         = 0.0f;
    state.lastSensorValid    = false;
    state.rollSetpointDeg    = 0.0f;
    state.cfgSnapshotValid   = false;
    state.entryMode          = AL_ENTRY_STRAIGHT_IN;
    state.cumulativeHeadingChangeRad = 0.0f;
    state.lastObservedCourseRad = 0.0f;
    state.hasLastCourse      = false;
    state.orbitEnteredUs     = 0;
    state.baseEnterCourseRad = 0.0f;
    state.baseHeadingChangeRad = 0.0f;
    state.wpDownwindEndLat   = 0.0f;
    state.wpDownwindEndLon   = 0.0f;
    state.wpFinalStartLat    = 0.0f;
    state.wpFinalStartLon    = 0.0f;
    state.patternWaypointsValid = false;
    state.commitLatched      = false;
    state.quietSinceUs       = 0;
    state.disarmRequested    = false;
    clearThrottleOverride();
}

void autolandUpdate(timeUs_t currentTimeUs,
                    const autolandSensorSample_t *sample)
{
    // Cache the new sample (NULL means "nothing new this tick" --
    // we keep the last one).
    if (sample != NULL) {
        state.lastSensor      = *sample;
        state.lastSensorValid = true;
        if (sample->baroValid) {
            state.lastBaroCm = sample->baroCm;
        }
    }

    // IDLE and COMPLETE are sink states -- nothing to tick.
    if (state.phase == AL_IDLE || state.phase == AL_COMPLETE) {
        updateControllers();
        return;
    }

    // Watchdog (invariant #1). Internal-safety abort bypasses
    // autolandAbort() so it succeeds even when the commit latch is
    // set (invariant #6).
    const uint32_t watchdogMs = phaseWatchdogMs[state.phase];
    if (watchdogMs > 0) {
        const uint32_t elapsedMs =
            (uint32_t)((currentTimeUs - state.phaseStartUs) / 1000);
        if (elapsedMs > watchdogMs) {
            state.lastAbort = AL_ABORT_WATCHDOG;
            autolandTransition(AL_ABORT, currentTimeUs);
            return;
        }
    }

    // Sensor-validity checks. Any flight-phase tick with a fresh
    // sample that says GPS or baro is gone triggers a safety abort
    // (invariant #6). We only check if we actually have a fresh
    // sample -- a stream of NULL samples is "no new info," not
    // "sensors are dead."
    if (isFlightPhase(state.phase) && sample != NULL) {
        if (!sample->gpsValid) {
            state.lastAbort = AL_ABORT_GPS_LOSS;
            autolandTransition(AL_ABORT, currentTimeUs);
            return;
        }
        if (!sample->baroValid) {
            state.lastAbort = AL_ABORT_BARO_LOSS;
            autolandTransition(AL_ABORT, currentTimeUs);
            return;
        }
    }

    // ---- Accumulators that feed transition logic below ----

    // IMU quiescence tracker for touchdown detection / auto-disarm.
    // `quietSinceUs == 0` means we aren't currently in a quiet window;
    // any non-zero value is the timestamp when continuous quiet began.
    if (sample != NULL && sample->imuValid) {
        const bool quiet = (sample->accelMagG  < TD_QUIET_ACCEL_G)
                        && (sample->gyroRateDegS < TD_QUIET_GYRO_DEG_S);
        if (quiet) {
            if (state.quietSinceUs == 0) {
                state.quietSinceUs = currentTimeUs;
            }
        } else {
            state.quietSinceUs = 0;
        }
    }

    // Commit latch (invariant #5). Once the wing crosses below
    // commit_altitude_cm during a flight phase, external aborts are
    // refused until the sequence terminates.
    if (isFlightPhase(state.phase) && state.cfgSnapshotValid
        && !state.commitLatched) {
        const float aglCm = state.lastBaroCm - state.baroRefCm;
        if (aglCm <= (float)state.snapshotCfg.commit_altitude_cm) {
            state.commitLatched = true;
        }
    }

    // Per-tick heading change tracking. Used by AL_ORBIT_DESCENT
    // (orbit count) and AL_BASE (90° turn complete).
    if (sample != NULL && sample->gpsValid) {
        if (state.hasLastCourse) {
            const float delta = navGeomHeadingErrorRad(state.lastObservedCourseRad,
                                                       sample->groundCourseRad);
            // Reject big jumps (GPS glitch, teleport) -- only count
            // realistic continuous turns.
            if (fabsf(delta) < ORBIT_MAX_DELTA_RAD) {
                state.cumulativeHeadingChangeRad += fabsf(delta);
                if (state.phase == AL_BASE) {
                    state.baseHeadingChangeRad += fabsf(delta);
                }
            }
        }
        state.lastObservedCourseRad = sample->groundCourseRad;
        state.hasLastCourse         = true;

        // Feed the wind estimator while orbiting. Samples outside
        // AL_ORBIT_DESCENT are ignored -- wind gets reset on entry.
        if (state.phase == AL_ORBIT_DESCENT) {
            navWindAddSample(sample->groundCourseRad, sample->groundSpeedCmS,
                             currentTimeUs);
        }
    }

    // Phase-specific transition logic.
    switch (state.phase) {
        case AL_ENTRY: {
            // Brief dwell so the attitude controller settles on the
            // new pitch setpoint, then advance based on entry mode.
            const uint32_t dwellMs =
                (uint32_t)((currentTimeUs - state.phaseStartUs) / 1000);
            if (dwellMs >= AL_ENTRY_DWELL_MS) {
                if (state.entryMode == AL_ENTRY_ORBIT_DESCENT) {
                    autolandTransition(AL_ORBIT_DESCENT, currentTimeUs);
                } else {
                    autolandTransition(AL_STRAIGHT_IN, currentTimeUs);
                }
            }
            break;
        }
        case AL_ORBIT_DESCENT: {
            // Two transition paths:
            //  (a) wind converged AND we've done enough orbits -> DOWNWIND
            //  (b) loiter timeout expired -> STRAIGHT_IN (fallback)
            const autolandSequenceConfig_t *cfg = &state.snapshotCfg;
            if (state.cfgSnapshotValid) {
                const uint32_t elapsedS =
                    (uint32_t)((currentTimeUs - state.orbitEnteredUs) / (1000 * 1000));

                const navWindEstimate_t *e = navWindGetEstimate();
                const float orbitsDone = state.cumulativeHeadingChangeRad / (2.0f * M_PIf);
                const bool windReady =
                    (e != NULL && e->status == NAV_WIND_VALID
                     && orbitsDone >= (float)cfg->orbits_before_descent);

                if (windReady) {
                    autolandTransition(AL_DOWNWIND, currentTimeUs);
                } else if (elapsedS >= cfg->loiter_timeout_s) {
                    autolandTransition(AL_STRAIGHT_IN, currentTimeUs);
                }
            }
            break;
        }
        case AL_DOWNWIND: {
            // Advance to AL_BASE once we've flown past downwind_distance
            // measured from home. (Simple radial-distance trigger; a
            // future iteration could measure along the downwind axis.)
            if (state.lastSensorValid && state.cfgSnapshotValid) {
                const float dist = navGeomDistanceM(
                    state.lastSensor.latDeg, state.lastSensor.lonDeg,
                    state.homeLatDeg, state.homeLonDeg);
                if (dist >= (float)state.snapshotCfg.downwind_distance_m) {
                    autolandTransition(AL_BASE, currentTimeUs);
                }
            }
            break;
        }
        case AL_BASE: {
            // Advance to AL_FINAL after a 90° heading change from base
            // entry -- the base turn is complete and we're on final.
            if (state.baseHeadingChangeRad >= (M_PIf / 2.0f)) {
                autolandTransition(AL_FINAL, currentTimeUs);
            }
            break;
        }
        case AL_FINAL:
        case AL_STRAIGHT_IN: {
            // Both fall into AL_FLARE at flare_start_alt.
            if (state.cfgSnapshotValid) {
                const float aglCm = state.lastBaroCm - state.baroRefCm;
                if (aglCm <= (float)state.snapshotCfg.flare_start_alt_cm) {
                    autolandTransition(AL_FLARE, currentTimeUs);
                }
            }
            break;
        }
        case AL_FLARE: {
            // AL_FLARE -> AL_TOUCHDOWN when any of three signals fire
            // (invariant #3, independent-OR). Throttle is already 0
            // from Phase 4 so the prop is commanded off regardless;
            // ESC brake (if configured) takes care of actively
            // stopping the prop. Disarm happens at AL_COMPLETE.
            if (state.cfgSnapshotValid) {
                const autolandSequenceConfig_t *cfg = &state.snapshotCfg;
                const float aglCm = state.lastBaroCm - state.baroRefCm;

                const bool tdBaro = (aglCm <= (float)cfg->touchdown_alt_threshold_cm);

                // touchdown_accel_threshold is in 0.1g units (e.g. 30 = 3.0g).
                const bool tdAccel = state.lastSensorValid
                                  && state.lastSensor.imuValid
                                  && (state.lastSensor.accelMagG * 10.0f
                                      >= (float)cfg->touchdown_accel_threshold);

                const bool tdQuiet = (state.quietSinceUs != 0)
                                  && ((currentTimeUs - state.quietSinceUs) / 1000
                                      >= TD_SHORT_QUIESCENCE_MS);

                if (tdBaro || tdAccel || tdQuiet) {
                    autolandTransition(AL_TOUCHDOWN, currentTimeUs);
                }
            }
            break;
        }
        case AL_TOUCHDOWN: {
            // Wait for a longer continuous quiet window (from config)
            // before committing to the disarm. Slides on the ground
            // count as motion and reset quietSinceUs, so we don't
            // disarm until the wing actually comes to rest.
            if (state.cfgSnapshotValid
                && state.quietSinceUs != 0
                && (currentTimeUs - state.quietSinceUs) / 1000
                    >= (uint32_t)state.snapshotCfg.touchdown_quiescence_ms) {
                state.disarmRequested = true;
                autolandTransition(AL_COMPLETE, currentTimeUs);
            }
            break;
        }
        case AL_ABORT:
            // Hand-back window -- pilot input governs. Watchdog forces
            // us back to IDLE.
            break;
        default:
            break;
    }

    updateControllers();

    // Blackbox debug. Slots are autoland-specific intent that isn't
    // recoverable from standard fields (motors/attitude/GPS/baro are
    // already logged separately).
    //   0: state machine phase
    //   1: AGL cm (autoland's launch-relative altitude reference)
    //   2: commanded roll setpoint, deg * 10 (vs. logged attitude.roll)
    //   3: low byte = last abort cause; upper bits = status flags
    const uint16_t flags =
          ((state.lastSensorValid && state.lastSensor.gpsValid) ? (1u << 8)  : 0u)
        | (state.cfgSnapshotValid                               ? (1u << 9)  : 0u)
        | (state.throttleOverrideActive                         ? (1u << 10) : 0u)
        | (state.armSessionConsumed                             ? (1u << 11) : 0u)
        | (state.baroRefLatched                                 ? (1u << 12) : 0u);
    DEBUG_SET(DEBUG_AUTOLAND, 0, (int32_t)state.phase);
    DEBUG_SET(DEBUG_AUTOLAND, 1, (int32_t)(state.lastBaroCm - state.baroRefCm));
    DEBUG_SET(DEBUG_AUTOLAND, 2, (int32_t)(state.rollSetpointDeg * 10.0f));
    DEBUG_SET(DEBUG_AUTOLAND, 3, (int32_t)((uint32_t)state.lastAbort | (uint32_t)flags));
}

bool autolandCanEnter(void)
{
    return state.phase == AL_IDLE && !state.armSessionConsumed;
}

bool autolandRequestEntry(timeUs_t currentTimeUs,
                          float initialBaroCm,
                          float homeLatDeg,
                          float homeLonDeg,
                          autolandEntryMode_e entryMode,
                          const autolandSequenceConfig_t *cfg)
{
    if (!autolandCanEnter()) {
        return false;
    }
    if (cfg == NULL) {
        // A NULL cfg leaves us with no controller outputs -- refuse
        // rather than enter a half-configured sequence.
        return false;
    }

    // Snapshot the config (invariant: values frozen for the whole
    // sequence, insulating us from live PG edits in configurator).
    state.snapshotCfg      = *cfg;
    state.cfgSnapshotValid = true;
    state.entryMode        = entryMode;

    // Latch the baro reference (invariant #2, one-shot).
    state.baroRefCm       = initialBaroCm;
    state.lastBaroCm      = initialBaroCm;
    state.baroRefLatched  = true;

    // Home position -- target for the waypoint controller.
    state.homeLatDeg = homeLatDeg;
    state.homeLonDeg = homeLonDeg;

    state.sequenceStartUs = currentTimeUs;
    autolandTransition(AL_ENTRY, currentTimeUs);

    updateControllers();
    return true;
}

bool autolandAbort(autolandAbortCause_e cause, timeUs_t currentTimeUs)
{
    // Invariant #5: while commit is latched, external aborts are
    // refused. Internal safety aborts (watchdog, GPS/baro loss) should
    // call autolandTransition(AL_ABORT, ...) directly and set lastAbort
    // inline -- those bypass this commit-latch check deliberately.
    if (state.commitLatched) {
        return false;
    }
    // Already aborting / done -- nothing to do, but don't return false
    // (caller might retry indefinitely and lose context).
    if (state.phase == AL_ABORT || state.phase == AL_COMPLETE) {
        return true;
    }
    state.lastAbort = cause;
    autolandTransition(AL_ABORT, currentTimeUs);
    return true;
}

void autolandTransition(autolandPhase_e next, timeUs_t currentTimeUs)
{
    if (next >= AL_PHASE_COUNT) {
        return;
    }

    state.phase = next;
    state.phaseStartUs = currentTimeUs;

    // Terminal transitions consume the arm session (invariant #8) and
    // drop the throttle override so the pilot's stick command takes
    // effect within the tick (invariant #4 pre-commit abort budget).
    if (isTerminalPhase(next)) {
        state.armSessionConsumed = true;
        clearThrottleOverride();
        // Baro ref + commit latch remain set -- they describe the
        // completed sequence. autolandInit() on the next arm clears.
    }

    // Entering IDLE is only legal via autolandInit(); if a caller
    // transitions directly to IDLE we still clear override so the
    // state is consistent.
    if (next == AL_IDLE) {
        clearThrottleOverride();
    }

    // Phase-entry side effects -- resetting orbit counters, deriving
    // pattern waypoints, snapshotting heading for base-turn tracking.
    // Deliberately a switch (not a table) so related side effects sit
    // next to the code that consumes them.
    switch (next) {
        case AL_ORBIT_DESCENT:
            navWindReset();
            state.cumulativeHeadingChangeRad = 0.0f;
            // Deliberately don't touch hasLastCourse / lastObservedCourseRad:
            // any previous sample is within ORBIT_MAX_DELTA_RAD of the next
            // (the wing is physically turning, not teleporting), and reusing
            // the last course keeps the first post-transition sample counted
            // instead of dropping it.
            state.orbitEnteredUs             = currentTimeUs;
            state.patternWaypointsValid      = false;
            break;
        case AL_DOWNWIND: {
            // Derive pattern waypoints from the wind vector that
            // nav_wind converged on during orbit-descent.
            const navWindEstimate_t *e = navWindGetEstimate();
            if (e != NULL && e->status == NAV_WIND_VALID) {
                derivePatternWaypoints(e->windFromHeadingRad);
            }
            break;
        }
        case AL_BASE:
            // Snapshot course at base entry to track the 90° heading
            // change used as the AL_BASE -> AL_FINAL trigger.
            state.baseEnterCourseRad  = state.lastSensor.groundCourseRad;
            state.baseHeadingChangeRad = 0.0f;
            break;
        case AL_TOUCHDOWN:
            // Fresh count for the disarm quiescence window. A short-
            // quiet touchdown detection in AL_FLARE may have already
            // accumulated up to TD_SHORT_QUIESCENCE_MS; we don't want
            // that to shortcut the longer post-touchdown window.
            state.quietSinceUs = 0;
            break;
        default:
            break;
    }

    // Entering a new flight phase re-evaluates throttle/pitch so the
    // controller reflects the new target (e.g. FLARE uses flare_pitch,
    // not glide_pitch). IDLE/ABORT/COMPLETE fall through with cleared
    // overrides from above -- updateControllers will keep them off.
    if (isFlightPhase(next)) {
        updateControllers();
    }
}

bool autolandIsActive(void)
{
    return state.phase != AL_IDLE
        && state.phase != AL_COMPLETE
        && state.phase != AL_ABORT;
}

autolandPhase_e autolandGetPhase(void)
{
    return state.phase;
}

autolandAbortCause_e autolandGetLastAbortCause(void)
{
    return state.lastAbort;
}

uint32_t autolandGetPhaseElapsedMs(timeUs_t currentTimeUs)
{
    if (state.phase == AL_IDLE || state.phaseStartUs == 0) {
        return 0;
    }
    return (uint32_t)((currentTimeUs - state.phaseStartUs) / 1000);
}

const char *autolandPhaseName(autolandPhase_e phase)
{
    if (phase >= AL_PHASE_COUNT) {
        return "INVALID";
    }
    const char *name = phaseNames[phase];
    return name ? name : "UNKNOWN";
}

bool autolandGetThrottleOverride(float *outThrottle)
{
    if (!state.throttleOverrideActive || outThrottle == NULL) {
        return false;
    }
    *outThrottle = state.throttleOverrideValue;
    return true;
}

bool autolandGetPitchSetpoint(float *outDeg)
{
    if (outDeg == NULL
        || !state.cfgSnapshotValid
        || !isFlightPhase(state.phase)) {
        return false;
    }
    // AL_FLARE uses flare_pitch_deg; every other flight phase uses
    // the cruise glide target.
    if (state.phase == AL_FLARE) {
        *outDeg = (float)state.snapshotCfg.flare_pitch_deg;
    } else {
        *outDeg = (float)state.snapshotCfg.glide_pitch_deg;
    }
    return true;
}

bool autolandGetRollSetpoint(float *outDeg)
{
    // Phase 5a: waypoint controller drives AL_STRAIGHT_IN toward home;
    // all other flight phases remain wings-level until Phase 5b wires
    // orbit + pattern turns. rollSetpointDeg is computed in
    // updateControllers() so this is a straight read.
    if (outDeg == NULL
        || !state.cfgSnapshotValid
        || !isFlightPhase(state.phase)) {
        return false;
    }
    *outDeg = state.rollSetpointDeg;
    return true;
}

bool autolandGetBaroReferenceCm(float *outCm)
{
    if (outCm == NULL || !state.baroRefLatched) {
        return false;
    }
    *outCm = state.baroRefCm;
    return true;
}

bool autolandShouldDisarm(void)
{
    return state.disarmRequested;
}

bool autolandIsCommitLatched(void)
{
    return state.commitLatched;
}

#endif // USE_WING
