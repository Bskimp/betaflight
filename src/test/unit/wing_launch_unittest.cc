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

extern "C" {

    #include "platform.h"
    #include "build/debug.h"

    #include "common/maths.h"

    #include "fc/rc_modes.h"
    #include "fc/runtime_config.h"

    #include "flight/imu.h"
    #include "flight/pid.h"
    #include "flight/wing_launch.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// Mock state
static uint32_t mockTimeUs = 0;
static bool mockLaunchSwitchActive = true;

extern "C" {
    uint8_t armingFlags = 0;
    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode;
    uint16_t flightModeFlags = 0;
    uint8_t stateFlags = 0;

    attitudeEulerAngles_t attitude;

    bool IS_RC_MODE_ACTIVE(boxId_e boxId) {
        if (boxId == BOXAUTOLAUNCH) {
            return mockLaunchSwitchActive;
        }
        return false;
    }

    timeUs_t micros(void) { return mockTimeUs; }
    uint32_t millis(void) { return mockTimeUs / 1000; }
}

static pidProfile_t testProfile;

static void resetTestState(void)
{
    mockTimeUs = 1000000; // start at 1 second
    mockLaunchSwitchActive = true;
    armingFlags = 0; // start DISARMED

    memset(&testProfile, 0, sizeof(testProfile));
    testProfile.wing_launch_accel_thresh = 25;   // unused now but keep in profile
    testProfile.wing_launch_motor_delay = 100;
    testProfile.wing_launch_motor_ramp = 500;
    testProfile.wing_launch_throttle = 75;
    testProfile.wing_launch_climb_time = 3000;
    testProfile.wing_launch_climb_angle = 45;
    testProfile.wing_launch_transition = 1000;
    testProfile.wing_launch_max_tilt = 45;
    testProfile.wing_launch_idle_thr = 0;

    memset(&attitude, 0, sizeof(attitude));

    wingLaunchInit(&testProfile);
}

static void advanceTimeMs(uint32_t ms)
{
    mockTimeUs += ms * 1000;
}

// simulate arming — sets ARMED flag and runs one update so the state machine sees the edge
static void simulateArm(void)
{
    armingFlags = 0x01; // ARMED = (1 << 0)
    wingLaunchUpdate(mockTimeUs);
    advanceTimeMs(1);
}

// helper: arm and advance through to CLIMBING state
static void advanceToClimbing(void)
{
    simulateArm();
    // now in MOTOR_DELAY — wait through it
    advanceTimeMs(100);
    wingLaunchUpdate(mockTimeUs); // → MOTOR_RAMP
    advanceTimeMs(500);
    wingLaunchUpdate(mockTimeUs); // → CLIMBING
}

// --- Tests ---

TEST(WingLaunchTest, InitialStateIsIdle)
{
    resetTestState();
    EXPECT_EQ(wingLaunchGetState(), WING_LAUNCH_IDLE);
    EXPECT_FALSE(isWingLaunchActive());
    EXPECT_FALSE(isWingLaunchInProgress());
    EXPECT_FLOAT_EQ(wingLaunchGetThrottle(), 0.0f);
}

TEST(WingLaunchTest, PitchPreDeflectsWhenSwitchOn)
{
    resetTestState();
    // AUTOLAUNCH switch is on, not armed — should pre-deflect elevator
    EXPECT_FLOAT_EQ(wingLaunchGetPitchAngle(), 45.0f);
}

TEST(WingLaunchTest, NoPitchDeflectWhenSwitchOff)
{
    resetTestState();
    mockLaunchSwitchActive = false;
    EXPECT_FLOAT_EQ(wingLaunchGetPitchAngle(), 0.0f);
}

TEST(WingLaunchTest, ArmTriggersLaunchSequence)
{
    resetTestState();
    EXPECT_EQ(wingLaunchGetState(), WING_LAUNCH_IDLE);

    simulateArm();
    EXPECT_EQ(wingLaunchGetState(), WING_LAUNCH_MOTOR_DELAY);
}

TEST(WingLaunchTest, NoTriggerWithoutSwitch)
{
    resetTestState();
    mockLaunchSwitchActive = false;

    armingFlags = 0x01;
    wingLaunchUpdate(mockTimeUs);
    advanceTimeMs(1);

    // switch is off so update returns early — state stays IDLE
    EXPECT_EQ(wingLaunchGetState(), WING_LAUNCH_IDLE);
}

TEST(WingLaunchTest, FullStateSequence)
{
    resetTestState();

    // arm triggers MOTOR_DELAY
    simulateArm();
    EXPECT_EQ(wingLaunchGetState(), WING_LAUNCH_MOTOR_DELAY);

    // wait through motor delay (100ms)
    advanceTimeMs(100);
    wingLaunchUpdate(mockTimeUs);
    EXPECT_EQ(wingLaunchGetState(), WING_LAUNCH_MOTOR_RAMP);

    // wait through motor ramp (500ms)
    advanceTimeMs(500);
    wingLaunchUpdate(mockTimeUs);
    EXPECT_EQ(wingLaunchGetState(), WING_LAUNCH_CLIMBING);
    EXPECT_NEAR(wingLaunchGetThrottle(), 0.75f, 0.01f);

    // wait through climb time (3000ms)
    advanceTimeMs(3000);
    wingLaunchUpdate(mockTimeUs);
    EXPECT_EQ(wingLaunchGetState(), WING_LAUNCH_TRANSITION);

    // wait through transition (1000ms)
    advanceTimeMs(1000);
    wingLaunchUpdate(mockTimeUs);
    EXPECT_EQ(wingLaunchGetState(), WING_LAUNCH_COMPLETE);

    EXPECT_FALSE(isWingLaunchActive());
    EXPECT_FALSE(isWingLaunchInProgress());
}

TEST(WingLaunchTest, MotorRampIsLinear)
{
    resetTestState();

    simulateArm();
    advanceTimeMs(100);
    wingLaunchUpdate(mockTimeUs); // → MOTOR_RAMP
    EXPECT_EQ(wingLaunchGetState(), WING_LAUNCH_MOTOR_RAMP);

    // at 50% through ramp (250ms of 500ms)
    advanceTimeMs(250);
    wingLaunchUpdate(mockTimeUs);
    EXPECT_NEAR(wingLaunchGetThrottle(), 0.375f, 0.05f); // 50% of 0.75

    // at 100% through ramp
    advanceTimeMs(250);
    wingLaunchUpdate(mockTimeUs);
    EXPECT_EQ(wingLaunchGetState(), WING_LAUNCH_CLIMBING);
    EXPECT_NEAR(wingLaunchGetThrottle(), 0.75f, 0.01f);
}

TEST(WingLaunchTest, PitchAngleDuringLaunch)
{
    resetTestState();

    advanceToClimbing();
    EXPECT_EQ(wingLaunchGetState(), WING_LAUNCH_CLIMBING);
    EXPECT_FLOAT_EQ(wingLaunchGetPitchAngle(), 45.0f);

    // during transition, pitch should decrease
    advanceTimeMs(3000);
    wingLaunchUpdate(mockTimeUs); // → TRANSITION
    EXPECT_EQ(wingLaunchGetState(), WING_LAUNCH_TRANSITION);

    advanceTimeMs(500); // 50% through transition
    wingLaunchUpdate(mockTimeUs);
    EXPECT_NEAR(wingLaunchGetPitchAngle(), 22.5f, 2.0f);
}

TEST(WingLaunchTest, AbortOnExcessiveRoll)
{
    resetTestState();

    advanceToClimbing();
    EXPECT_EQ(wingLaunchGetState(), WING_LAUNCH_CLIMBING);

    // exceed max tilt (45 degrees = 450 decidegrees)
    attitude.values.roll = 500; // 50 degrees
    advanceTimeMs(1);
    wingLaunchUpdate(mockTimeUs);

    // should abort → complete
    wingLaunchUpdate(mockTimeUs);
    EXPECT_EQ(wingLaunchGetState(), WING_LAUNCH_COMPLETE);
}

TEST(WingLaunchTest, AbortOnSwitchOff)
{
    resetTestState();

    advanceToClimbing();
    EXPECT_EQ(wingLaunchGetState(), WING_LAUNCH_CLIMBING);

    // turn off launch switch
    mockLaunchSwitchActive = false;
    advanceTimeMs(1);
    wingLaunchUpdate(mockTimeUs);
    EXPECT_EQ(wingLaunchGetState(), WING_LAUNCH_ABORT);
}

TEST(WingLaunchTest, ResetOnDisarm)
{
    resetTestState();

    simulateArm();
    EXPECT_NE(wingLaunchGetState(), WING_LAUNCH_IDLE);

    // disarm
    armingFlags = 0;
    advanceTimeMs(1);
    wingLaunchUpdate(mockTimeUs);
    EXPECT_EQ(wingLaunchGetState(), WING_LAUNCH_IDLE);
}

TEST(WingLaunchTest, ThrottleNegativeWhenComplete)
{
    resetTestState();

    // run full sequence
    simulateArm();
    advanceTimeMs(100);
    wingLaunchUpdate(mockTimeUs);
    advanceTimeMs(500);
    wingLaunchUpdate(mockTimeUs);
    advanceTimeMs(3000);
    wingLaunchUpdate(mockTimeUs);
    advanceTimeMs(1000);
    wingLaunchUpdate(mockTimeUs);
    EXPECT_EQ(wingLaunchGetState(), WING_LAUNCH_COMPLETE);
    EXPECT_LT(wingLaunchGetThrottle(), 0.0f); // negative = no override
}

TEST(WingLaunchTest, AbortOnPitchDive)
{
    resetTestState();

    advanceToClimbing();
    EXPECT_EQ(wingLaunchGetState(), WING_LAUNCH_CLIMBING);

    // pitch nose-down past max tilt (45 degrees = 450 decidegrees)
    attitude.values.pitch = -500; // -50 degrees
    advanceTimeMs(1);
    wingLaunchUpdate(mockTimeUs);

    // should abort
    wingLaunchUpdate(mockTimeUs);
    EXPECT_EQ(wingLaunchGetState(), WING_LAUNCH_COMPLETE);
}

TEST(WingLaunchTest, AbortOnRollDuringMotorRamp)
{
    resetTestState();

    simulateArm();
    advanceTimeMs(100);
    wingLaunchUpdate(mockTimeUs); // → MOTOR_RAMP
    EXPECT_EQ(wingLaunchGetState(), WING_LAUNCH_MOTOR_RAMP);

    // exceed max tilt during ramp
    attitude.values.roll = 500; // 50 degrees
    advanceTimeMs(1);
    wingLaunchUpdate(mockTimeUs);
    EXPECT_EQ(wingLaunchGetState(), WING_LAUNCH_ABORT);
}

// cmpTimeUs is static inline in common/time.h — no stub needed
