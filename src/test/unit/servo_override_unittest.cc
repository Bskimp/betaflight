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

#include <cstdint>

extern "C" {
    #include "platform.h"
    #include "common/time.h"
    #include "fc/runtime_config.h"
    #include "flight/servo_override.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// ---- Mocks ------------------------------------------------------------
//
// servo_override.c references:
//   - micros() from drivers/time.h
//   - armingFlags global from fc/runtime_config.h (via ARMING_FLAG macro)
//
// Tests drive both directly: mockMicros for the auto-clear timer,
// armingFlags bit ARMED for the arm-state branches.
extern "C" {
    static timeUs_t mockMicros = 0;
    timeUs_t micros(void) { return mockMicros; }

    uint8_t armingFlags = 0;
}

static void resetGlobals()
{
    mockMicros = 0;
    armingFlags = 0;
}

static void setArmed(bool armed)
{
    if (armed) {
        armingFlags |= ARMED;
    } else {
        armingFlags &= ~ARMED;
    }
}

// ---- Tests ------------------------------------------------------------

class ServoOverrideTest : public ::testing::Test {
protected:
    void SetUp() override {
        resetGlobals();
        // Drain any leftover state from a previous test by force-clearing
        // via the armed-state path: arm, update, disarm.
        setArmed(true);
        servoOverrideUpdate();
        setArmed(false);
    }
};

TEST_F(ServoOverrideTest, RequestSucceedsWithValidArgs) {
    EXPECT_TRUE(servoOverrideRequest(0, 1500, 1000));
    EXPECT_TRUE(servoOverrideIsActive(0));
    EXPECT_EQ(1500, servoOverrideGetPwm());
}

TEST_F(ServoOverrideTest, RequestRejectedWhenArmed) {
    setArmed(true);
    EXPECT_FALSE(servoOverrideRequest(0, 1500, 1000));
    EXPECT_FALSE(servoOverrideIsActive(0));
}

TEST_F(ServoOverrideTest, RequestRejectedOnInvalidServoIndex) {
    EXPECT_FALSE(servoOverrideRequest(MAX_SUPPORTED_SERVOS, 1500, 1000));
    EXPECT_FALSE(servoOverrideRequest(255, 1500, 1000));
}

TEST_F(ServoOverrideTest, RequestRejectedOnPwmOutOfRange) {
    EXPECT_FALSE(servoOverrideRequest(0, 999, 1000));
    EXPECT_FALSE(servoOverrideRequest(0, 2001, 1000));
    EXPECT_FALSE(servoOverrideRequest(0, 0, 1000));
}

TEST_F(ServoOverrideTest, RequestRejectedOnDurationOutOfRange) {
    EXPECT_FALSE(servoOverrideRequest(0, 1500, 0));
    EXPECT_FALSE(servoOverrideRequest(0, 1500, SERVO_OVERRIDE_MAX_DURATION_MS + 1));
}

TEST_F(ServoOverrideTest, IsActiveOnlyForRequestedIndex) {
    EXPECT_TRUE(servoOverrideRequest(2, 1500, 1000));
    EXPECT_FALSE(servoOverrideIsActive(0));
    EXPECT_FALSE(servoOverrideIsActive(1));
    EXPECT_TRUE(servoOverrideIsActive(2));
    EXPECT_FALSE(servoOverrideIsActive(3));
}

TEST_F(ServoOverrideTest, AutoClearAfterDurationElapsed) {
    mockMicros = 0;
    EXPECT_TRUE(servoOverrideRequest(0, 1700, 100));   // 100ms
    EXPECT_TRUE(servoOverrideIsActive(0));

    // Advance to just before expiry: still active.
    mockMicros = 99 * 1000;
    servoOverrideUpdate();
    EXPECT_TRUE(servoOverrideIsActive(0));

    // Advance past expiry: cleared.
    mockMicros = 100 * 1000;
    servoOverrideUpdate();
    EXPECT_FALSE(servoOverrideIsActive(0));
}

TEST_F(ServoOverrideTest, ArmedDuringActiveOverrideForceClears) {
    EXPECT_TRUE(servoOverrideRequest(0, 1500, 5000));
    EXPECT_TRUE(servoOverrideIsActive(0));

    // Closes the start-disarmed-then-arm race that handler-time
    // refusal alone would miss. Arm AFTER override is active.
    setArmed(true);
    servoOverrideUpdate();
    EXPECT_FALSE(servoOverrideIsActive(0));
}

TEST_F(ServoOverrideTest, NewRequestSupersedesOld) {
    EXPECT_TRUE(servoOverrideRequest(1, 1200, 5000));
    EXPECT_TRUE(servoOverrideIsActive(1));
    EXPECT_EQ(1200, servoOverrideGetPwm());

    EXPECT_TRUE(servoOverrideRequest(2, 1800, 1000));
    EXPECT_FALSE(servoOverrideIsActive(1));   // old idx no longer active
    EXPECT_TRUE(servoOverrideIsActive(2));
    EXPECT_EQ(1800, servoOverrideGetPwm());
}

TEST_F(ServoOverrideTest, UpdateNoopWhenInactive) {
    // Should be safe to call repeatedly with no override pending.
    servoOverrideUpdate();
    servoOverrideUpdate();
    EXPECT_FALSE(servoOverrideIsActive(0));
}
