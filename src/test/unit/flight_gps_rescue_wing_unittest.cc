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
#include <stdbool.h>
#include <limits.h>
#include <cmath>

extern "C" {
    #include "platform.h"
    #include "build/debug.h"

    #include "common/axis.h"
    #include "common/filter.h"
    #include "common/maths.h"
    #include "common/vector.h"

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
    #include "pg/pg.h"
    #include "pg/pg_ids.h"

    #include "sensors/acceleration.h"
    #include "sensors/gyro.h"
    #include "sensors/sensors.h"

    #include "flight/gps_rescue.h"

    PG_REGISTER(autopilotConfig_t, autopilotConfig, PG_AUTOPILOT, 0);

    PG_REGISTER_WITH_RESET_TEMPLATE(gpsRescueConfig_t, gpsRescueConfig, PG_GPS_RESCUE, 8);
    PG_RESET_TEMPLATE(gpsRescueConfig_t, gpsRescueConfig,
        .allowArmingWithoutFix = false,
        .minSats = 8,
        .maxBankAngle = 25,
        .orbitRadiusM = 50,
        .returnAltitudeM = 50,
        .minLoiterAltM = 25,
        .cruiseThrottle = 50,
        .minThrottle = 30,
        .abortThrottle = 45,
        .navP = 30,
        .altP = 30,
        .turnCompensation = 50,
        .minHeadingSpeedCmS = 400,
        .stallSpeedCmS = 200,
        .minStartDistM = 30,
        .sanityChecks = RESCUE_SANITY_ON,
    );
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// ---- Controllable stubs ----

static float stubAltitudeCm = 0.0f;
static int16_t stubEstimatedVario = 0;
static bool stubRxReceiving = true;
static bool stubCrashRecovery = false;
static bool stubSensorsAcc = true;
static bool stubSensorsGps = true;
static bool stubDisarmed = false;

extern "C" {
    uint8_t armingFlags = 0;
    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode;
    uint16_t flightModeFlags = 0;
    uint8_t stateFlags = 0;

    acc_t acc;
    attitudeEulerAngles_t attitude;
    gpsSolutionData_t gpsSol;
    float rcCommand[4];

    uint16_t GPS_distanceToHome;
    int16_t GPS_directionToHome;
    uint32_t GPS_distanceToHomeCm;

    float getAltitudeCm(void) { return stubAltitudeCm; }
    float getAltitudeDerivative(void) { return 0.0f; }
    float getCosTiltAngle(void) { return 1.0f; }
    int16_t getEstimatedVario(void) { return stubEstimatedVario; }
    float getGpsDataIntervalSeconds(void) { return 0.1f; }
    float getGpsDataFrequencyHz(void) { return 10.0f; }

    bool gpsHasNewData(uint16_t *gpsStamp) { UNUSED(*gpsStamp); return true; }
    void GPS_distance2d(const gpsLocation_t *, const gpsLocation_t *, vector2_t *) { }

    void parseRcChannels(const char *input, rxConfig_t *rxConfig) { UNUSED(input); UNUSED(rxConfig); }
    throttleStatus_e calculateThrottleStatus(void) { return THROTTLE_LOW; }

    bool isRxReceivingSignal(void) { return stubRxReceiving; }
    bool crashRecoveryModeActive(void) { return stubCrashRecovery; }

    bool sensors(uint32_t mask) {
        bool result = true;
        if ((mask & SENSOR_ACC) && !stubSensorsAcc) result = false;
        if ((mask & SENSOR_GPS) && !stubSensorsGps) result = false;
        return result;
    }

    void disarm(flightLogDisarmReason_e reason) { UNUSED(reason); stubDisarmed = true; }
    void setArmingDisabled(armingDisableFlags_e flag) { UNUSED(flag); }
    void tryArm(void) { }

    void writeEEPROM(void) { }
    void systemReset(void) { }

    timeUs_t currentTimeUs = 0;
    timeUs_t micros(void) { return currentTimeUs; }
    boxBitmask_t rcModeActivationMask;
}

// ---- Test helpers ----

static void resetTestState(void)
{
    stubAltitudeCm = 0.0f;
    stubEstimatedVario = 0;
    stubRxReceiving = true;
    stubCrashRecovery = false;
    stubSensorsAcc = true;
    stubSensorsGps = true;
    stubDisarmed = false;

    flightModeFlags = 0;
    stateFlags = GPS_FIX_HOME | GPS_FIX;

    gpsSol.numSat = 12;
    gpsSol.groundSpeed = 800;
    gpsSol.groundCourse = 0;

    GPS_distanceToHome = 200;
    GPS_directionToHome = 1800;

    attitude.values.roll = 0;
    attitude.values.pitch = 0;

    gpsRescueConfigMutable()->allowArmingWithoutFix = false;
    gpsRescueConfigMutable()->minSats = 8;
    gpsRescueConfigMutable()->maxBankAngle = 25;
    gpsRescueConfigMutable()->orbitRadiusM = 50;
    gpsRescueConfigMutable()->returnAltitudeM = 50;
    gpsRescueConfigMutable()->minLoiterAltM = 25;
    gpsRescueConfigMutable()->cruiseThrottle = 50;
    gpsRescueConfigMutable()->minThrottle = 30;
    gpsRescueConfigMutable()->abortThrottle = 45;
    gpsRescueConfigMutable()->navP = 30;
    gpsRescueConfigMutable()->altP = 30;
    gpsRescueConfigMutable()->turnCompensation = 50;
    gpsRescueConfigMutable()->minHeadingSpeedCmS = 400;
    gpsRescueConfigMutable()->stallSpeedCmS = 200;
    gpsRescueConfigMutable()->minStartDistM = 30;
    gpsRescueConfigMutable()->sanityChecks = RESCUE_SANITY_ON;

    gpsRescueInit();
}

// =================================================================
// Public interface query tests
// =================================================================

TEST(GpsRescueWingTest, InitResetsState)
{
    gpsRescueInit();
    EXPECT_FLOAT_EQ(gpsRescueAngle[AI_ROLL], 0.0f);
    EXPECT_FLOAT_EQ(gpsRescueAngle[AI_PITCH], 0.0f);
    EXPECT_FLOAT_EQ(gpsRescueGetImuYawCogGain(), 1.0f);
}

TEST(GpsRescueWingTest, YawRateAlwaysZero)
{
    EXPECT_FLOAT_EQ(gpsRescueGetYawRate(), 0.0f);
}

TEST(GpsRescueWingTest, IsConfiguredWithSensors)
{
    stubSensorsAcc = true;
    stubSensorsGps = true;
    EXPECT_TRUE(gpsRescueIsConfigured());
}

TEST(GpsRescueWingTest, IsNotConfiguredWithoutAcc)
{
    stubSensorsAcc = false;
    stubSensorsGps = true;
    EXPECT_FALSE(gpsRescueIsConfigured());
}

TEST(GpsRescueWingTest, IsNotConfiguredWithoutGps)
{
    stubSensorsAcc = true;
    stubSensorsGps = false;
    EXPECT_FALSE(gpsRescueIsConfigured());
}

TEST(GpsRescueWingTest, IsDisabledWhenNotConfigured)
{
    stubSensorsAcc = false;
    EXPECT_TRUE(gpsRescueIsDisabled());
}

TEST(GpsRescueWingTest, IsNotDisabledWhenConfigured)
{
    stubSensorsAcc = true;
    stubSensorsGps = true;
    EXPECT_FALSE(gpsRescueIsDisabled());
}

TEST(GpsRescueWingTest, IsAvailableWithFixAndSats)
{
    stubSensorsAcc = true;
    stubSensorsGps = true;
    stateFlags = GPS_FIX_HOME | GPS_FIX;
    gpsSol.numSat = 12;
    EXPECT_TRUE(gpsRescueIsAvailable());
}

TEST(GpsRescueWingTest, IsNotAvailableWithoutFix)
{
    stubSensorsAcc = true;
    stubSensorsGps = true;
    stateFlags = GPS_FIX_HOME; // has home, but no current fix
    gpsSol.numSat = 12;
    EXPECT_FALSE(gpsRescueIsAvailable());
}

TEST(GpsRescueWingTest, IsNotAvailableLowSats)
{
    stubSensorsAcc = true;
    stubSensorsGps = true;
    stateFlags = GPS_FIX_HOME | GPS_FIX;
    gpsRescueConfigMutable()->minSats = 8;
    gpsSol.numSat = 3; // below minSats=8
    EXPECT_FALSE(gpsRescueIsAvailable());
}

TEST(GpsRescueWingTest, IsNotAvailableWithoutHomeEvenIfAllowed)
{
    stubSensorsAcc = true;
    stubSensorsGps = true;
    stateFlags = 0; // no home, no fix
    gpsRescueConfigMutable()->allowArmingWithoutFix = true;
    EXPECT_FALSE(gpsRescueIsAvailable());
}

TEST(GpsRescueWingTest, IsNotAvailableWithoutHomeIfNotAllowed)
{
    stubSensorsAcc = true;
    stubSensorsGps = true;
    stateFlags = 0; // no home, no fix
    gpsRescueConfigMutable()->allowArmingWithoutFix = false;
    EXPECT_FALSE(gpsRescueIsAvailable());
}

#ifdef USE_MAG
TEST(GpsRescueWingTest, DisablesMag)
{
    EXPECT_TRUE(gpsRescueDisableMag());
}
#endif

// =================================================================
// State machine activation tests
// =================================================================

TEST(GpsRescueWingTest, UpdateIdleNoRescueMode)
{
    resetTestState();
    flightModeFlags = 0; // not in rescue mode

    gpsRescueUpdate();

    // should remain idle, angles zero
    EXPECT_FLOAT_EQ(gpsRescueAngle[AI_ROLL], 0.0f);
    EXPECT_FLOAT_EQ(gpsRescueAngle[AI_PITCH], 0.0f);
}

TEST(GpsRescueWingTest, UpdateActivatesOnRescueMode)
{
    resetTestState();
    stubAltitudeCm = 1000.0f;
    flightModeFlags = GPS_RESCUE_MODE;

    // first update: start -> initialize -> acquire heading
    gpsRescueUpdate();

    // COG gain should change from default 1.0 to heading acquisition gain (3.0)
    EXPECT_FLOAT_EQ(gpsRescueGetImuYawCogGain(), 3.0f);
}

TEST(GpsRescueWingTest, UpdateDeactivatesOnModeClear)
{
    resetTestState();
    stubAltitudeCm = 1000.0f;
    flightModeFlags = GPS_RESCUE_MODE;

    gpsRescueUpdate(); // activate

    flightModeFlags = 0; // deactivate
    gpsRescueUpdate();

    // Should return to idle, COG gain back to 1.0
    EXPECT_FLOAT_EQ(gpsRescueGetImuYawCogGain(), 1.0f);
    EXPECT_FLOAT_EQ(gpsRescueAngle[AI_ROLL], 0.0f);
}

// =================================================================
// Abort scenarios
// =================================================================

TEST(GpsRescueWingTest, AbortsWithoutHomeFix)
{
    resetTestState();
    stateFlags = GPS_FIX; // has GPS fix but no home
    stubAltitudeCm = 1000.0f;
    flightModeFlags = GPS_RESCUE_MODE;

    gpsRescueUpdate(); // start -> initialize -> no home -> abort

    // in abort, COG gain = 1.0
    EXPECT_FLOAT_EQ(gpsRescueGetImuYawCogGain(), 1.0f);
}

TEST(GpsRescueWingTest, AbortsInsideMinStartDistance)
{
    resetTestState();
    GPS_distanceToHome = 20;
    gpsRescueConfigMutable()->minStartDistM = 30;
    stubAltitudeCm = 1000.0f;
    flightModeFlags = GPS_RESCUE_MODE;

    gpsRescueUpdate();

    EXPECT_FLOAT_EQ(gpsRescueGetImuYawCogGain(), 1.0f);
}

TEST(GpsRescueWingTest, CrashRecoveryDisarms)
{
    resetTestState();
    stubAltitudeCm = 5000.0f;
    flightModeFlags = GPS_RESCUE_MODE;

    gpsRescueUpdate(); // activate

    stubCrashRecovery = true;
    gpsRescueUpdate(); // should detect crash and disarm

    EXPECT_TRUE(stubDisarmed);
}

// =================================================================
// COG gain schedule tests (indirect via phase transitions)
// =================================================================

TEST(GpsRescueWingTest, CogGainIdleIsDefault)
{
    resetTestState();
    flightModeFlags = 0;

    gpsRescueUpdate();

    EXPECT_FLOAT_EQ(gpsRescueGetImuYawCogGain(), 1.0f);
}

TEST(GpsRescueWingTest, CogGainAcquireHeadingIs3)
{
    resetTestState();
    stubAltitudeCm = 1000.0f;
    flightModeFlags = GPS_RESCUE_MODE;

    gpsRescueUpdate(); // initialize -> acquire heading

    EXPECT_FLOAT_EQ(gpsRescueGetImuYawCogGain(), 3.0f);
}

TEST(GpsRescueWingTest, AcquireHeadingCommandsNoseUpWhenBelowReturnAltitude)
{
    resetTestState();
    stubAltitudeCm = 0.0f;
    stubEstimatedVario = 0;
    flightModeFlags = GPS_RESCUE_MODE;

    gpsRescueUpdate(); // INITIALIZE -> ACQUIRE_HEADING (transition only)
    gpsRescueUpdate(); // ACQUIRE_HEADING body runs, sets pitch

    EXPECT_FLOAT_EQ(gpsRescueGetImuYawCogGain(), 3.0f);
    EXPECT_LT(gpsRescueAngle[AI_PITCH], 0.0f);
}

TEST(GpsRescueWingTest, ClimbCommandsNoseUpWhenBelowReturnAltitude)
{
    resetTestState();
    stubAltitudeCm = 0.0f;
    stubEstimatedVario = 0;
    flightModeFlags = GPS_RESCUE_MODE;

    for (int i = 0; i < 11; i++) {
        gpsRescueUpdate();
    }

    EXPECT_FLOAT_EQ(gpsRescueGetImuYawCogGain(), 2.5f);
    EXPECT_LT(gpsRescueAngle[AI_PITCH], 0.0f);
}

TEST(GpsRescueWingTest, ClimbRevertsToAcquireHeadingWhenHeadingBecomesInvalid)
{
    resetTestState();
    stubAltitudeCm = 0.0f;
    flightModeFlags = GPS_RESCUE_MODE;

    for (int i = 0; i < 11; i++) {
        gpsRescueUpdate();
    }

    EXPECT_FLOAT_EQ(gpsRescueGetImuYawCogGain(), 2.5f);

    // Oscillate COG to invalidate heading. Need > HEADING_INVALID_DWELL_TICKS (20)
    // *unstable* updates after the buffer first sees oscillation, so allow margin.
    for (int i = 0; i < 25; i++) {
        gpsSol.groundCourse = (i % 2 == 0) ? 0 : 1800;
        gpsRescueUpdate();
    }

    EXPECT_FLOAT_EQ(gpsRescueGetImuYawCogGain(), 3.0f);

    gpsRescueUpdate();

    EXPECT_FLOAT_EQ(gpsRescueGetImuYawCogGain(), 3.0f);
    EXPECT_FLOAT_EQ(gpsRescueAngle[AI_ROLL], 0.0f);
}

TEST(GpsRescueWingTest, CogGainAbortIs1)
{
    resetTestState();
    stateFlags = GPS_FIX; // no home -> will abort
    flightModeFlags = GPS_RESCUE_MODE;

    gpsRescueUpdate(); // initialize -> abort

    EXPECT_FLOAT_EQ(gpsRescueGetImuYawCogGain(), 1.0f);
}

TEST(GpsRescueWingTest, AbortOnHeadingAcquireTimeout)
{
    resetTestState();
    stubAltitudeCm = 1000.0f;
    flightModeFlags = GPS_RESCUE_MODE;
    // Keep ground speed below minHeadingSpeedCmS so validity never acquires.
    gpsSol.groundSpeed = 100;

    // HEADING_ACQUIRE_TIMEOUT_S=10s @ 100Hz = 1000 ticks; run with margin.
    for (int i = 0; i < 1100; i++) {
        gpsRescueUpdate();
    }

    // ACQUIRE_HEADING -> ABORT after timeout: COG gain collapses to 1.0
    EXPECT_FLOAT_EQ(gpsRescueGetImuYawCogGain(), 1.0f);
}

TEST(GpsRescueWingTest, AbortLatchesThroughApplyOutputs)
{
    resetTestState();
    stateFlags = GPS_FIX; // no home -> abort on first initialize tick
    flightModeFlags = GPS_RESCUE_MODE;

    gpsRescueUpdate(); // INITIALIZE -> ABORT (transition only)
    gpsRescueUpdate(); // ABORT body runs, sets level + abort throttle

    ASSERT_FLOAT_EQ(gpsRescueGetImuYawCogGain(), 1.0f);

    // Exit-persistence: over many mixer ticks, ABORT must not revert to
    // any earlier phase and the commanded angles must stay at 0 (wings level).
    for (int i = 0; i < 50; i++) {
        gpsRescueUpdate();
    }
    EXPECT_FLOAT_EQ(gpsRescueGetImuYawCogGain(), 1.0f);
    // PT1 smoothing converges asymptotically; tolerance swallows float noise
    EXPECT_NEAR(gpsRescueAngle[AI_ROLL], 0.0f, 0.001f);
    EXPECT_NEAR(gpsRescueAngle[AI_PITCH], 0.0f, 0.001f);
}

// =================================================================
// Sanity check / failure detection scenarios
// =================================================================

// Drive rescue state machine to an active phase (CLIMB) suitable for
// GPS_LOST / LOW_SATS detector tests (those checks run in any active phase).
static void driveToClimb(void)
{
    stubAltitudeCm = 1000.0f;
    gpsSol.groundSpeed = 800;
    gpsSol.groundCourse = 0;
    GPS_directionToHome = 0;
    GPS_distanceToHome = 500;
    flightModeFlags = GPS_RESCUE_MODE;

    // INITIALIZE -> ACQUIRE_HEADING (+ HEADING_VALID_DWELL 10 ticks) -> CLIMB
    for (int i = 0; i < 15; i++) {
        gpsRescueUpdate();
    }
}

// Drive rescue to CRUISE (needed for STALL and NO_PROGRESS checks).
static void driveToCruise(void)
{
    stubAltitudeCm = 4000.0f; // below target 5000 so INITIALIZE picks 5000
    gpsSol.groundSpeed = 800;
    gpsSol.groundCourse = 0;
    GPS_directionToHome = 0;       // headingErr=0 -> CLIMB transitions to CRUISE
    GPS_distanceToHome = 500;
    flightModeFlags = GPS_RESCUE_MODE;

    // Phase progression through ACQUIRE_HEADING and into CLIMB
    for (int i = 0; i < 15; i++) {
        gpsRescueUpdate();
    }
    // Simulate reaching target altitude so CLIMB transitions to CRUISE
    stubAltitudeCm = 5000.0f;
    for (int i = 0; i < 3; i++) {
        gpsRescueUpdate();
    }
}

TEST(GpsRescueWingTest, AbortsOnGpsFixLostMidClimb)
{
    resetTestState();
    driveToClimb();
    ASSERT_STREQ(gpsRescueGetPhaseName(), "CLMB");

    // Drop GPS fix; detector fires on next SANITY_CHECK_INTERVAL (1s = 100 ticks)
    stateFlags = GPS_FIX_HOME; // clear GPS_FIX bit
    for (int i = 0; i < 110; i++) {
        gpsRescueUpdate();
    }

    EXPECT_STREQ(gpsRescueGetPhaseName(), "ABRT");
}

TEST(GpsRescueWingTest, AbortsOnLowSatsMidClimb)
{
    resetTestState();
    driveToClimb();
    ASSERT_STREQ(gpsRescueGetPhaseName(), "CLMB");

    // Drop sats below minSats (default 8); must persist LOW_SATS_TIMEOUT_S (10s)
    gpsSol.numSat = 5;
    for (int i = 0; i < 1200; i++) {
        gpsRescueUpdate();
    }

    EXPECT_STREQ(gpsRescueGetPhaseName(), "ABRT");
}

TEST(GpsRescueWingTest, AbortsOnStallMidCruise)
{
    resetTestState();
    // Heading stays valid here (groundSpeed >= minHeadingSpeedCmS).
    // Separate test below covers the harder case where heading goes invalid.
    gpsRescueConfigMutable()->stallSpeedCmS = 500;
    driveToCruise();
    ASSERT_STREQ(gpsRescueGetPhaseName(), "CRSE");

    gpsSol.groundSpeed = 400;
    for (int i = 0; i < 700; i++) {
        gpsRescueUpdate();
    }

    EXPECT_STREQ(gpsRescueGetPhaseName(), "ABRT");
}

TEST(GpsRescueWingTest, AbortsOnStallEvenWhenHeadingInvalid)
{
    // Regression test: previously the CRUISE->ACQUIRE_HEADING revert on
    // invalid heading short-circuited the phase-gated stall counter, so a
    // real stall (low groundspeed trips BOTH stall and heading thresholds)
    // could cycle indefinitely without aborting. The phase-independent
    // stall counter must accumulate across the revert cycle and abort.
    resetTestState();
    driveToCruise();
    ASSERT_STREQ(gpsRescueGetPhaseName(), "CRSE");

    // Drop groundSpeed below BOTH thresholds (stallSpeedCmS=200,
    // minHeadingSpeedCmS=400). Heading goes invalid; phase will cycle
    // between CRUISE and ACQUIRE_HEADING, but stall counter persists.
    gpsSol.groundSpeed = 100;
    for (int i = 0; i < 700; i++) {
        gpsRescueUpdate();
    }

    EXPECT_STREQ(gpsRescueGetPhaseName(), "ABRT");
}

TEST(GpsRescueWingTest, AbortsOnNoProgressMidCruise)
{
    resetTestState();
    driveToCruise();
    ASSERT_STREQ(gpsRescueGetPhaseName(), "CRSE");

    // Keep distance-to-home constant -> progressM=0 per sanity check.
    // NO_PROGRESS_TIMEOUT_S=12s -> 1200 ticks + margin.
    GPS_distanceToHome = 500;
    for (int i = 0; i < 1400; i++) {
        gpsRescueUpdate();
    }

    EXPECT_STREQ(gpsRescueGetPhaseName(), "ABRT");
}

// ---- handleFailure() routing tests ----

TEST(GpsRescueWingTest, SanityOnRoutesGpsLostToAbort)
{
    resetTestState();
    gpsRescueConfigMutable()->sanityChecks = RESCUE_SANITY_ON;
    stubRxReceiving = true; // soft-failsafe: RX intact
    driveToClimb();
    ASSERT_STREQ(gpsRescueGetPhaseName(), "CLMB");

    stateFlags = GPS_FIX_HOME;
    for (int i = 0; i < 110; i++) {
        gpsRescueUpdate();
    }

    EXPECT_STREQ(gpsRescueGetPhaseName(), "ABRT");
}

TEST(GpsRescueWingTest, SanityFsOnlyRoutesToAbortOnHardFailsafe)
{
    resetTestState();
    gpsRescueConfigMutable()->sanityChecks = RESCUE_SANITY_FS_ONLY;
    stubRxReceiving = false; // hard failsafe: RX lost
    driveToClimb();
    ASSERT_STREQ(gpsRescueGetPhaseName(), "CLMB");

    stateFlags = GPS_FIX_HOME;
    for (int i = 0; i < 110; i++) {
        gpsRescueUpdate();
    }

    EXPECT_STREQ(gpsRescueGetPhaseName(), "ABRT");
}

TEST(GpsRescueWingTest, SanityFsOnlyDoesNotRouteOnSoftFailsafe)
{
    // Negative case: FS_ONLY must NOT abort if RX is intact, even with GPS_LOST
    resetTestState();
    gpsRescueConfigMutable()->sanityChecks = RESCUE_SANITY_FS_ONLY;
    stubRxReceiving = true; // RX intact -> no hard failsafe
    driveToClimb();
    ASSERT_STREQ(gpsRescueGetPhaseName(), "CLMB");

    stateFlags = GPS_FIX_HOME;
    for (int i = 0; i < 110; i++) {
        gpsRescueUpdate();
    }

    // Phase must NOT be ABRT -- sanity config prevents routing
    EXPECT_STRNE(gpsRescueGetPhaseName(), "ABRT");
}
