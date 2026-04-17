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

    #include "fc/runtime_config.h"

    #include "flight/imu.h"
    #include "flight/position.h"

    #include "io/gps.h"

    #include "fc/rc_controls.h"

    #include "pg/autopilot.h"
    #include "pg/gps_rescue.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"

    #include "rx/rx.h"
    #include "sensors/gyro.h"

    #include "flight/autopilot_wing.h"

    PG_REGISTER(autopilotConfig_t, autopilotConfig, PG_AUTOPILOT, 0);
    PG_REGISTER(gpsRescueConfig_t, gpsRescueConfig, PG_GPS_RESCUE, 0);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// ---- Controllable stubs ----

static float stubAltitudeCm = 0.0f;
static int16_t stubEstimatedVario = 0;

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

    bool isRxReceivingSignal(void) { return true; }
}

// =================================================================
// headingErrorDeci tests
// =================================================================

TEST(AutopilotWingTest, HeadingErrorZero)
{
    EXPECT_EQ(headingErrorDeci(1800, 1800), 0);
}

TEST(AutopilotWingTest, HeadingErrorPositive)
{
    // target 200 deg, current 100 deg -> +100 deg = +1000 decideg
    EXPECT_EQ(headingErrorDeci(2000, 1000), 1000);
}

TEST(AutopilotWingTest, HeadingErrorNegative)
{
    // target 100 deg, current 200 deg -> -100 deg = -1000 decideg
    EXPECT_EQ(headingErrorDeci(1000, 2000), -1000);
}

TEST(AutopilotWingTest, HeadingErrorWrapPositive)
{
    // target 10 deg (100), current 350 deg (3500) -> should be +200 not -3400
    EXPECT_EQ(headingErrorDeci(100, 3500), 200);
}

TEST(AutopilotWingTest, HeadingErrorWrapNegative)
{
    // target 350 deg (3500), current 10 deg (100) -> should be -200 not +3400
    EXPECT_EQ(headingErrorDeci(3500, 100), -200);
}

TEST(AutopilotWingTest, HeadingErrorExactly180)
{
    // 180 deg difference -> should be +1800 (positive direction)
    EXPECT_EQ(headingErrorDeci(2700, 900), 1800);
}

TEST(AutopilotWingTest, HeadingErrorExactlyMinus180)
{
    // -180 deg difference -> should be -1800
    EXPECT_EQ(headingErrorDeci(900, 2700), -1800);
}

// =================================================================
// wingHeadingControl tests
// =================================================================

TEST(AutopilotWingTest, HeadingControlZeroError)
{
    float roll = wingHeadingControl(0, 30, 25);
    EXPECT_FLOAT_EQ(roll, 0.0f);
}

TEST(AutopilotWingTest, HeadingControlSmallError)
{
    // error = 100 decideg (10 deg), navP = 30, maxBank = 25
    // rollDeg = 100 * 30 / 100 = 30 -> clamped to 25 -> 2500 centideg
    float roll = wingHeadingControl(100, 30, 25);
    EXPECT_FLOAT_EQ(roll, 2500.0f);
}

TEST(AutopilotWingTest, HeadingControlNegativeError)
{
    // error = -50 decideg (5 deg left), navP = 30
    // rollDeg = -50 * 30 / 100 = -15 -> within +-25 -> -1500 centideg
    float roll = wingHeadingControl(-50, 30, 25);
    EXPECT_FLOAT_EQ(roll, -1500.0f);
}

TEST(AutopilotWingTest, HeadingControlClampsToMaxBank)
{
    // large error should be clamped to maxBankAngle
    float roll = wingHeadingControl(1800, 30, 25);
    EXPECT_FLOAT_EQ(roll, 2500.0f); // 25 deg * 100

    roll = wingHeadingControl(-1800, 30, 25);
    EXPECT_FLOAT_EQ(roll, -2500.0f);
}

// =================================================================
// wingAltitudeControl tests
// =================================================================

TEST(AutopilotWingTest, AltitudeControlAtTarget)
{
    // no altitude error, no climb rate -> zero pitch
    float pitch = wingAltitudeControl(0.0f, 0.0f, 30);
    EXPECT_FLOAT_EQ(pitch, 0.0f);
}

TEST(AutopilotWingTest, AltitudeControlBelow)
{
    // 1000 cm below target should command nose-up in Betaflight sign.
    float pitch = wingAltitudeControl(1000.0f, 0.0f, 30);
    EXPECT_FLOAT_EQ(pitch, -20.0f);
}

TEST(AutopilotWingTest, AltitudeControlAbove)
{
    // 500 cm above target should command nose-down in Betaflight sign.
    float pitch = wingAltitudeControl(-500.0f, 0.0f, 30);
    EXPECT_FLOAT_EQ(pitch, 15.0f);
}

TEST(AutopilotWingTest, AltitudeControlDamping)
{
    // Positive climb rate adds nose-down damping in Betaflight sign.
    float pitch = wingAltitudeControl(0.0f, 200.0f, 30);
    EXPECT_FLOAT_EQ(pitch, 3.0f);
}

TEST(AutopilotWingTest, AltitudeControlClampsDive)
{
    // Way above target clamps at the configured nose-down limit.
    float pitch = wingAltitudeControl(-5000.0f, 0.0f, 30);
    EXPECT_FLOAT_EQ(pitch, 15.0f);
}

// =================================================================
// wingThrottleControl tests
// =================================================================

TEST(AutopilotWingTest, ThrottleCruiseLevel)
{
    // level flight, cruise = 50%, min = 30%
    float thr = wingThrottleControl(0.0f, 50, 30);
    EXPECT_FLOAT_EQ(thr, 0.5f);
}

TEST(AutopilotWingTest, ThrottleClimbBoost)
{
    // Negative pitch is nose-up in Betaflight sign, so it gets climb boost.
    float thr = wingThrottleControl(-10.0f, 50, 30);
    EXPECT_FLOAT_EQ(thr, 0.52f);
}

TEST(AutopilotWingTest, ThrottleNoBoostOnDive)
{
    // Positive pitch is nose-down in Betaflight sign, so cruise stays unchanged.
    float thr = wingThrottleControl(10.0f, 50, 30);
    EXPECT_FLOAT_EQ(thr, 0.5f); // cruise, no reduction
}

TEST(AutopilotWingTest, ThrottleMinFloor)
{
    // very low cruise (below min) should be floored
    float thr = wingThrottleControl(0.0f, 10, 30);
    EXPECT_FLOAT_EQ(thr, 0.3f); // floored to minThrottle
}

TEST(AutopilotWingTest, ThrottleMaxClamp)
{
    // Extreme nose-up command with high cruise should clamp at full throttle.
    float thr = wingThrottleControl(-30.0f, 95, 30);
    EXPECT_LE(thr, 1.0f);
}

// =================================================================
// wingTurnCompensation tests
// =================================================================

TEST(AutopilotWingTest, TurnCompensationZeroBank)
{
    EXPECT_FLOAT_EQ(wingTurnCompensation(0.0f, 50), 0.0f);
}

TEST(AutopilotWingTest, TurnCompensationSmallBank)
{
    // very small bank (0.5 deg) -> below 1 deg threshold -> 0
    EXPECT_FLOAT_EQ(wingTurnCompensation(0.5f, 50), 0.0f);
}

TEST(AutopilotWingTest, TurnCompensationDisabled)
{
    // turnCompPct = 0 -> always returns 0 regardless of bank
    EXPECT_FLOAT_EQ(wingTurnCompensation(30.0f, 0), 0.0f);
}

TEST(AutopilotWingTest, TurnCompensation30deg)
{
    // 30 deg bank: 1/cos(30) - 1 = 1/0.866 - 1 ~= 0.1547
    // compensation = 0.1547 * 50/100 * 20 ~= 1.547
    float comp = wingTurnCompensation(30.0f, 50);
    EXPECT_NEAR(comp, -1.547f, 0.1f);
}

TEST(AutopilotWingTest, TurnCompensationHighGain)
{
    // same 30 deg bank at 100% -> double the compensation
    float comp = wingTurnCompensation(30.0f, 100);
    EXPECT_NEAR(comp, -3.094f, 0.1f);
}

TEST(AutopilotWingTest, TurnCompensationIsSymmetricForNegativeBank)
{
    const float compRight = wingTurnCompensation(30.0f, 50);
    const float compLeft = wingTurnCompensation(-30.0f, 50);
    EXPECT_FLOAT_EQ(compLeft, compRight);
}

TEST(AutopilotWingTest, TurnCompensationExtremeBank)
{
    // 80 deg bank: cos(80) ~= 0.174 < 0.5 -> capped
    // capped = (1/0.5 - 1) * 50/100 * 20 = 1.0 * 10 = 10.0
    float comp = wingTurnCompensation(80.0f, 50);
    EXPECT_FLOAT_EQ(comp, -10.0f);
}

// =================================================================
// wingClampOrbitRadius tests
// =================================================================

TEST(AutopilotWingTest, OrbitRadiusUsesConfigWhenSafe)
{
    // slow speed, config radius 50m should dominate
    float radius = wingClampOrbitRadius(500, 25, 50); // 5 m/s
    EXPECT_GE(radius, 50.0f);
}

TEST(AutopilotWingTest, OrbitRadiusClampsPhysics)
{
    // fast speed: 30 m/s (3000 cm/s), maxBank = 25, config = 10m
    // r_min = 30^2 / (9.81 * tan(25)) = 900 / (9.81 * 0.466) = 900/4.57 ~= 197m * 1.2 ~= 236m
    float radius = wingClampOrbitRadius(3000, 25, 10);
    EXPECT_GT(radius, 200.0f);
}

TEST(AutopilotWingTest, OrbitRadiusMinFloor)
{
    // zero speed -> should still return at least config radius or 10m floor
    float radius = wingClampOrbitRadius(0, 25, 50);
    EXPECT_GE(radius, 10.0f);
}

// =================================================================
// wingGetRescueAltitudeCm tests
// =================================================================

TEST(AutopilotWingTest, RescueAltitudeReturnsRelative)
{
    stubAltitudeCm = 5000.0f;
    EXPECT_FLOAT_EQ(wingGetRescueAltitudeCm(), 5000.0f);
}

// =================================================================
// Heading validity tests
// =================================================================

TEST(AutopilotWingTest, HeadingValidityInitiallyFalse)
{
    wingResetHeadingValidity();
    EXPECT_FALSE(wingIsHeadingValid());
}

TEST(AutopilotWingTest, HeadingValidityRequiresSpeedAndStability)
{
    wingResetHeadingValidity();

    // fill COG buffer with stable heading (all same), adequate speed
    gpsSol.groundCourse = 1800; // 180 deg
    for (int i = 0; i < 20; i++) { // more than HEADING_VALID_DWELL_TICKS
        wingUpdateHeadingValidity(500, 400); // 5 m/s > 4 m/s threshold
    }
    EXPECT_TRUE(wingIsHeadingValid());
}

TEST(AutopilotWingTest, HeadingValidityFailsOnLowSpeed)
{
    wingResetHeadingValidity();

    gpsSol.groundCourse = 1800;
    for (int i = 0; i < 20; i++) {
        wingUpdateHeadingValidity(100, 400); // 1 m/s < 4 m/s threshold
    }
    EXPECT_FALSE(wingIsHeadingValid());
}

TEST(AutopilotWingTest, HeadingValidityFailsOnUnstableCog)
{
    wingResetHeadingValidity();

    // alternate COG wildly: 0 and 1800 decideg (0 and 180 deg)
    for (int i = 0; i < 20; i++) {
        gpsSol.groundCourse = (i % 2 == 0) ? 0 : 1800;
        wingUpdateHeadingValidity(500, 400);
    }
    EXPECT_FALSE(wingIsHeadingValid());
}

TEST(AutopilotWingTest, HeadingValidityResetsCleanly)
{
    wingResetHeadingValidity();

    // first achieve valid heading
    gpsSol.groundCourse = 900;
    for (int i = 0; i < 20; i++) {
        wingUpdateHeadingValidity(500, 400);
    }
    EXPECT_TRUE(wingIsHeadingValid());

    // reset should clear it
    wingResetHeadingValidity();
    EXPECT_FALSE(wingIsHeadingValid());
}

// =================================================================
// Output smoothing tests
// =================================================================

TEST(AutopilotWingTest, SmoothingStartsFromZeroAngleOffset)
{
    // Roll and pitch start from zero because gpsRescueAngle is additive.
    wingInitSmoothing(0.5f, 100.0f);

    // First output should move toward the target, not jump straight to it.
    float roll = 1000.0f, pitch = 500.0f, throttle = 0.5f;
    wingApplySmoothing(&roll, &pitch, &throttle);

    EXPECT_GT(roll, 0.0f);
    EXPECT_LT(roll, 300.0f);
    EXPECT_GT(pitch, 0.0f);
    EXPECT_LT(pitch, 150.0f);
    EXPECT_NEAR(throttle, 0.5f, 0.05f);
}

TEST(AutopilotWingTest, SmoothingConverges)
{
    wingInitSmoothing(0.0f, 100.0f);

    // apply constant target repeatedly -> should converge
    float roll, pitch, throttle;
    for (int i = 0; i < 200; i++) {
        roll = 2000.0f;
        pitch = 1000.0f;
        throttle = 0.8f;
        wingApplySmoothing(&roll, &pitch, &throttle);
    }

    EXPECT_NEAR(roll, 2000.0f, 1.0f);
    EXPECT_NEAR(pitch, 1000.0f, 1.0f);
    EXPECT_NEAR(throttle, 0.8f, 0.01f);
}

TEST(AutopilotWingTest, SetRescueThrottleClamped)
{
    wingInitSmoothing(0.0f, 100.0f);

    wingSetRescueThrottle(1.5f);
    // getAutopilotThrottle should return clamped value
    float thr = getAutopilotThrottle();
    EXPECT_LE(thr, 1.0f);

    wingSetRescueThrottle(-0.5f);
    thr = getAutopilotThrottle();
    EXPECT_GE(thr, 0.0f);
}

TEST(AutopilotWingTest, SetRescueInactive)
{
    wingInitSmoothing(0.5f, 100.0f);
    EXPECT_TRUE(isAutopilotInControl());

    wingSetRescueInactive();
    EXPECT_FALSE(isAutopilotInControl());
    EXPECT_FLOAT_EQ(getAutopilotThrottle(), 0.0f);
}
