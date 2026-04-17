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

#include "build/debug.h"
#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/vector.h"
#include "fc/rc.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/position.h"
#include "rx/rx.h"
#include "sensors/gyro.h"

#include "pg/autopilot.h"
#include "autopilot.h"

#ifdef USE_GPS_RESCUE

#include "io/gps.h"
#include "pg/gps_rescue.h"

// ---- Internal constants (not CLI-exposed in v1) ----

#define ALT_D_GAIN                  15      // altitude D gain
#define MIN_PITCH_DEG             (-20)     // max nose-up climb angle in Betaflight pitch sign
#define MAX_PITCH_DEG               15      // max nose-down dive angle in Betaflight pitch sign
#define CLIMB_THROTTLE_GAIN         20      // extra throttle per degree nose-up pitch
#define COG_HISTORY_SIZE            10      // samples for heading stability check
#define HEADING_VALID_DWELL_TICKS   10      // ~1s at rescue task rate (100Hz / 10 = 10 ticks)
#define HEADING_INVALID_DWELL_TICKS 20      // ~2s of bad heading before declaring invalid (hysteresis)
#define COMMAND_SMOOTHING_CUTOFF_HZ 2.0f    // PT1 cutoff for output smoothing
#define ORBIT_SAFETY_MARGIN         1.2f    // 20% safety margin on min orbit radius
#define GRAVITY_CMSS                981.0f  // cm/s^2

// ---- Module state ----

static pt1Filter_t rollSmoothFilter;
static pt1Filter_t pitchSmoothFilter;
static pt1Filter_t throttleSmoothFilter;

static int16_t cogHistory[COG_HISTORY_SIZE];
static uint8_t cogIndex;
static uint8_t headingValidCounter;
static uint8_t headingInvalidCounter;
static bool headingValid;
static bool wingRescueActive;
static float wingRescueThrottle;

#endif // USE_GPS_RESCUE

float autopilotAngle[RP_AXIS_COUNT];

void resetPositionControl(unsigned taskRateHz)
{
    UNUSED(taskRateHz);
}

void autopilotInit(void)
{
}

void resetAltitudeControl(void)
{
}

void altitudeControl(float targetAltitudeCm, float taskIntervalS, float targetAltitudeVelCmS, float velLimitCmS)
{
    UNUSED(targetAltitudeCm);
    UNUSED(taskIntervalS);
    UNUSED(targetAltitudeVelCmS);
    UNUSED(velLimitCmS);
}

void setSticksActiveStatus(bool areSticksActive)
{
    UNUSED(areSticksActive);
}

bool positionControl(void)
{
    return false;
}

bool isBelowLandingAltitude(void)
{
    return false;
}

float getAutopilotThrottle(void)
{
#ifdef USE_GPS_RESCUE
    if (wingRescueActive) {
        return wingRescueThrottle;
    }
#endif
    return 0.0f;
}

bool isAutopilotInControl(void)
{
#ifdef USE_GPS_RESCUE
    return wingRescueActive;
#else
    return false;
#endif
}

// ============================================================
// Wing GPS Rescue navigation primitives
// ============================================================

#ifdef USE_GPS_RESCUE

// ---- Altitude source (W2 decision: use relative/zeroed altitude) ----

float wingGetRescueAltitudeCm(void)
{
    return getAltitudeCm();
}

// ---- Heading wrap: shortest path between two headings ----
// inputs/outputs in decidegrees, result in [-1800, 1800]

int16_t headingErrorDeci(int16_t target, int16_t current)
{
    int16_t error = target - current;
    if (error > 1800) {
        error -= 3600;
    } else if (error < -1800) {
        error += 3600;
    }
    return error;
}

// ---- Heading control: bank-to-turn ----
// errorDeci: heading error in decidegrees
// navP: P gain from config
// maxBankAngle: max bank in degrees
// returns: target roll in centidegrees (for gpsRescueAngle[AI_ROLL])

float wingHeadingControl(int16_t errorDeci, uint8_t navP, uint8_t maxBankAngle)
{
    // P controller: error (decideg) * gain / 10 -> degrees -> centidegrees
    float rollDeg = (float)errorDeci * (float)navP / 100.0f;
    rollDeg = constrainf(rollDeg, -(float)maxBankAngle, (float)maxBankAngle);
    return rollDeg * 100.0f;
}

// ---- Altitude control: pitch targeting ----
// altErrorCm: (target - current) in cm
// climbRateCmS: vertical velocity in cm/s (positive = climbing)
// altP: P gain from config
// returns: target pitch in Betaflight sign (+ nose-down, - nose-up)

float wingAltitudeControl(float altErrorCm, float climbRateCmS, uint8_t altP)
{
    // In Betaflight pitch sign, a positive altitude error needs nose-up (negative pitch).
    const float pTerm = -altErrorCm * (float)altP / 1000.0f;

    // Positive climb rate should suppress further nose-up, and vice versa.
    const float dTerm = climbRateCmS * (float)ALT_D_GAIN / 1000.0f;

    float pitchDeg = pTerm + dTerm;
    pitchDeg = constrainf(pitchDeg, (float)MIN_PITCH_DEG, (float)MAX_PITCH_DEG);
    return pitchDeg;
}

// ---- Throttle control: stall prevention ----
// pitchDeg: current commanded pitch in Betaflight sign (+ nose-down, - nose-up)
// cruiseThrottle, minThrottle: from config (percent 0-100)
// returns: throttle as fraction [0.0, 1.0]

float wingThrottleControl(float pitchDeg, uint8_t cruiseThrottle, uint8_t minThrottle)
{
    float throttle = (float)cruiseThrottle / 100.0f;

    // Add throttle for nose-up commands, which correspond to climbing in BF sign.
    if (pitchDeg < 0.0f) {
        throttle += -pitchDeg * (float)CLIMB_THROTTLE_GAIN / 10000.0f;
    }

    // hard floor at minThrottle
    const float floor = (float)minThrottle / 100.0f;
    throttle = constrainf(throttle, floor, 1.0f);
    return throttle;
}

// ---- Turn compensation: pitch-up in banks ----
// bankAngleDeg: current bank angle in degrees (signed or absolute)
// returns: additional pitch in Betaflight sign, negative for nose-up in turns

float wingTurnCompensation(float bankAngleDeg, uint8_t turnCompPct)
{
    const float absBankAngleDeg = fabsf(bankAngleDeg);

    if (absBankAngleDeg < 1.0f || turnCompPct == 0) {
        return 0.0f;
    }
    // 1/cos(bank) - 1 gives the load factor increase
    const float bankRad = absBankAngleDeg * RAD;
    const float cosBank = cos_approx(bankRad);
    const float scale = (float)turnCompPct / 100.0f;
    if (cosBank < 0.5f) {
        // Extreme bank: cap compensation and keep it nose-up in BF sign.
        return -(1.0f / 0.5f - 1.0f) * scale * fabsf((float)MIN_PITCH_DEG);
    }
    const float loadIncrease = 1.0f / cosBank - 1.0f;
    return -loadIncrease * scale * fabsf((float)MIN_PITCH_DEG);
}

// ---- Orbit radius clamping ----
// Computes minimum achievable turn radius from speed and max bank
// groundSpeedCmS: current GPS groundspeed in cm/s
// maxBankAngle: max bank in degrees
// configRadiusM: desired orbit radius in meters
// returns: clamped orbit radius in meters (never below physics + safety margin)

float wingClampOrbitRadius(uint16_t groundSpeedCmS, uint8_t maxBankAngle, uint16_t configRadiusM)
{
    // r_min = v^2 / (g * tan(bank))
    const float speedMs = (float)groundSpeedCmS / 100.0f;
    const float bankRad = (float)maxBankAngle * RAD;
    const float tanBank = tan_approx(bankRad);

    float minRadiusM = 10.0f; // absolute floor
    if (tanBank > 0.01f) {
        minRadiusM = (speedMs * speedMs) / (GRAVITY_CMSS / 100.0f * tanBank);
        minRadiusM *= ORBIT_SAFETY_MARGIN;
    }

    return fmaxf((float)configRadiusM, minRadiusM);
}

// ---- Heading validity: COG stability check ----

void wingResetHeadingValidity(void)
{
    for (int i = 0; i < COG_HISTORY_SIZE; i++) {
        cogHistory[i] = 0;
    }
    cogIndex = 0;
    headingValidCounter = 0;
    headingInvalidCounter = 0;
    headingValid = false;
}

static int16_t computeCogSpread(void)
{
    // find max angular spread in the COG history buffer (with wrap)
    int16_t minCog = cogHistory[0];
    int16_t maxCog = cogHistory[0];
    for (int i = 1; i < COG_HISTORY_SIZE; i++) {
        if (cogHistory[i] < minCog) {
            minCog = cogHistory[i];
        }
        if (cogHistory[i] > maxCog) {
            maxCog = cogHistory[i];
        }
    }
    int16_t spread = maxCog - minCog;
    // handle wrap: if spread > 1800 decideg, the actual spread is 3600 - spread
    if (spread > 1800) {
        spread = 3600 - spread;
    }
    return spread;
}

bool wingUpdateHeadingValidity(uint16_t groundSpeedCmS, uint16_t minHeadingSpeedCmS)
{
    // record current GPS course in circular buffer
    cogHistory[cogIndex] = gpsSol.groundCourse;
    cogIndex = (cogIndex + 1) % COG_HISTORY_SIZE;

    const bool speedOk = groundSpeedCmS >= minHeadingSpeedCmS;
    const int16_t spread = computeCogSpread();
    const bool cogStable = spread < 150; // 15 degrees in decidegrees

    if (speedOk && cogStable) {
        headingInvalidCounter = 0;
        if (headingValidCounter < HEADING_VALID_DWELL_TICKS) {
            headingValidCounter++;
        }
    } else {
        if (headingValid) {
            // Already valid: require sustained bad readings before going invalid (hysteresis)
            headingInvalidCounter++;
            if (headingInvalidCounter >= HEADING_INVALID_DWELL_TICKS) {
                headingValidCounter = 0;
                headingValid = false;
            }
        } else {
            headingValidCounter = 0;
        }
    }

    if (!headingValid) {
        headingValid = (headingValidCounter >= HEADING_VALID_DWELL_TICKS);
    }
    return headingValid;
}

bool wingIsHeadingValid(void)
{
    return headingValid;
}

// ---- Output smoothing (L12) ----

void wingInitSmoothing(float currentThrottle, float rescueTaskHz)
{
    const float gain = pt1FilterGain(COMMAND_SMOOTHING_CUTOFF_HZ, 1.0f / rescueTaskHz);
    pt1FilterInit(&rollSmoothFilter, gain);
    pt1FilterInit(&pitchSmoothFilter, gain);
    pt1FilterInit(&throttleSmoothFilter, gain);

    // seed roll/pitch at zero -- gpsRescueAngle is ADDED to the existing
    // angle-mode target in pid.c, so seeding from current attitude would
    // double-count and cause an immediate nose-down pitch step.
    rollSmoothFilter.state = 0.0f;
    pitchSmoothFilter.state = 0.0f;
    throttleSmoothFilter.state = currentThrottle;

    wingRescueActive = true;
    wingRescueThrottle = currentThrottle;
}

void wingReseedSmoothing(void)
{
    // Re-seed roll/pitch at zero -- gpsRescueAngle is additive on top of
    // the angle-mode target, so seeding from attitude would double-count.
    rollSmoothFilter.state = 0.0f;
    pitchSmoothFilter.state = 0.0f;
    // throttle filter state is left as-is to avoid thrust step
    wingRescueActive = true;
}

void wingApplySmoothing(float *rollCd, float *pitchCd, float *throttle)
{
    *rollCd = pt1FilterApply(&rollSmoothFilter, *rollCd);
    *pitchCd = pt1FilterApply(&pitchSmoothFilter, *pitchCd);
    *throttle = pt1FilterApply(&throttleSmoothFilter, *throttle);
}

void wingSetRescueThrottle(float throttle)
{
    wingRescueThrottle = constrainf(throttle, 0.0f, 1.0f);
}

void wingSetRescueInactive(void)
{
    wingRescueActive = false;
    wingRescueThrottle = 0.0f;
}

// ---- Debug output ----

void wingRescueDebug(uint8_t phase, int16_t headingErrDeci, float targetRollCd,
                     float targetPitchCd, float distanceM, uint16_t groundSpeedCmS,
                     float throttle)
{
    DEBUG_SET(DEBUG_GPS_RESCUE_WING, 0, phase);
    DEBUG_SET(DEBUG_GPS_RESCUE_WING, 1, headingValid ? 1 : 0);
    DEBUG_SET(DEBUG_GPS_RESCUE_WING, 2, headingErrDeci);
    DEBUG_SET(DEBUG_GPS_RESCUE_WING, 3, lrintf(targetRollCd / 10.0f));
    DEBUG_SET(DEBUG_GPS_RESCUE_WING, 4, lrintf(targetPitchCd / 10.0f));
    DEBUG_SET(DEBUG_GPS_RESCUE_WING, 5, lrintf(distanceM));
    DEBUG_SET(DEBUG_GPS_RESCUE_WING, 6, groundSpeedCmS);
    DEBUG_SET(DEBUG_GPS_RESCUE_WING, 7, lrintf(throttle * 1000.0f));
}

#endif // USE_GPS_RESCUE

#endif // USE_WING
