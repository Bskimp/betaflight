/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <http://www.gnu.org/licenses/>.
 */

// Golden-vector round-trip tests for MSP2 wing-launch config messages.
// Same cross-repo pattern as wing_msp_unittest.cc — the hex bytes here
// must round-trip through the configurator's wing_launch_msp.test.js.
// A field reorder or signed/unsigned flip on either side fails both.

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

extern "C" {
    #include "platform.h"
    #include "common/streambuf.h"
    #include "flight/pid.h"
    #include "msp/msp_wing_launch.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// 15-byte golden vector for MSP2_WING_LAUNCH.
// Canonical values include a negative int16 (wing_launch_climb_angle = -15)
// so a readU16 vs readI16 bug surfaces immediately.
static const uint8_t WING_LAUNCH_GOLDEN[] = {
    0x19,                          // wing_launch_accel_thresh (25)
    0x64, 0x00,                    // wing_launch_motor_delay (100)
    0xF4, 0x01,                    // wing_launch_motor_ramp (500)
    0x4B,                          // wing_launch_throttle (75)
    0xB8, 0x0B,                    // wing_launch_climb_time (3000)
    0xF1, 0xFF,                    // wing_launch_climb_angle (-15, int16)
    0xE8, 0x03,                    // wing_launch_transition (1000)
    0x2D,                          // wing_launch_max_tilt (45)
    0x00,                          // wing_launch_idle_thr (0)
    0x14,                          // wing_launch_stick_override (20)
};

static void populateCanonicalWingLaunch(pidProfile_t *p)
{
    memset(p, 0, sizeof(*p));
    p->wing_launch_accel_thresh   = 25;
    p->wing_launch_motor_delay    = 100;
    p->wing_launch_motor_ramp     = 500;
    p->wing_launch_throttle       = 75;
    p->wing_launch_climb_time     = 3000;
    p->wing_launch_climb_angle    = -15;
    p->wing_launch_transition     = 1000;
    p->wing_launch_max_tilt       = 45;
    p->wing_launch_idle_thr       = 0;
    p->wing_launch_stick_override = 20;
}

TEST(WingLaunchMspUnitTest, ConfigEncodesToGoldenVector)
{
    pidProfile_t profile;
    populateCanonicalWingLaunch(&profile);

    uint8_t buffer[32] = {0};
    sbuf_t sbuf;
    sbufInit(&sbuf, buffer, buffer + sizeof(buffer));

    serializeWingLaunch(&sbuf, &profile);

    const size_t written = (size_t)(sbuf.ptr - buffer);
    EXPECT_EQ(sizeof(WING_LAUNCH_GOLDEN), written);
    EXPECT_EQ(0, memcmp(buffer, WING_LAUNCH_GOLDEN, sizeof(WING_LAUNCH_GOLDEN)));
}

TEST(WingLaunchMspUnitTest, ConfigDecodesGoldenVector)
{
    pidProfile_t profile;
    memset(&profile, 0, sizeof(profile));

    sbuf_t sbuf;
    sbufInit(&sbuf, (uint8_t *)WING_LAUNCH_GOLDEN,
             (uint8_t *)WING_LAUNCH_GOLDEN + sizeof(WING_LAUNCH_GOLDEN));

    const bool ok = deserializeWingLaunch(&sbuf, &profile);
    EXPECT_TRUE(ok);

    pidProfile_t expected;
    populateCanonicalWingLaunch(&expected);

    EXPECT_EQ(expected.wing_launch_accel_thresh,   profile.wing_launch_accel_thresh);
    EXPECT_EQ(expected.wing_launch_motor_delay,    profile.wing_launch_motor_delay);
    EXPECT_EQ(expected.wing_launch_motor_ramp,     profile.wing_launch_motor_ramp);
    EXPECT_EQ(expected.wing_launch_throttle,       profile.wing_launch_throttle);
    EXPECT_EQ(expected.wing_launch_climb_time,     profile.wing_launch_climb_time);
    EXPECT_EQ(expected.wing_launch_climb_angle,    profile.wing_launch_climb_angle);
    EXPECT_EQ(expected.wing_launch_transition,     profile.wing_launch_transition);
    EXPECT_EQ(expected.wing_launch_max_tilt,       profile.wing_launch_max_tilt);
    EXPECT_EQ(expected.wing_launch_idle_thr,       profile.wing_launch_idle_thr);
    EXPECT_EQ(expected.wing_launch_stick_override, profile.wing_launch_stick_override);
}

TEST(WingLaunchMspUnitTest, ConfigRoundTrips)
{
    pidProfile_t source;
    populateCanonicalWingLaunch(&source);

    uint8_t buffer[32] = {0};
    sbuf_t writer;
    sbufInit(&writer, buffer, buffer + sizeof(buffer));
    serializeWingLaunch(&writer, &source);
    const size_t written = (size_t)(writer.ptr - buffer);

    pidProfile_t roundTripped;
    memset(&roundTripped, 0, sizeof(roundTripped));
    sbuf_t reader;
    sbufInit(&reader, buffer, buffer + written);
    EXPECT_TRUE(deserializeWingLaunch(&reader, &roundTripped));

    // Explicit signed-field check — if the int16 isn't preserved, this
    // catches it before anything else.
    EXPECT_EQ(source.wing_launch_climb_angle, roundTripped.wing_launch_climb_angle);
    EXPECT_EQ(-15, roundTripped.wing_launch_climb_angle);
}

TEST(WingLaunchMspUnitTest, DeserializeRejectsTruncatedPayload)
{
    pidProfile_t profile;
    memset(&profile, 0, sizeof(profile));

    // Feed only the first 8 bytes of the 15-byte golden vector.
    sbuf_t sbuf;
    sbufInit(&sbuf, (uint8_t *)WING_LAUNCH_GOLDEN, (uint8_t *)WING_LAUNCH_GOLDEN + 8);

    EXPECT_FALSE(deserializeWingLaunch(&sbuf, &profile));
}
