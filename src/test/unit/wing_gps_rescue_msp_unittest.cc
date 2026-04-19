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

// Golden-vector round-trip tests for MSP2 wing GPS-rescue config.
// Same cross-repo pattern as wing_launch_msp_unittest.cc — the hex
// bytes here must round-trip through the configurator's
// wing_gps_rescue_msp.test.js. A field reorder or size flip on
// either side fails both suites in sync.

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

extern "C" {
    #include "platform.h"
    #include "common/streambuf.h"
    #include "pg/gps_rescue_wing.h"
    #include "msp/msp_wing_gps_rescue.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// 22-byte golden vector for MSP2_WING_GPS_RESCUE.
// All fields unsigned — no signed-variant trap like wing_launch, but
// the u8/u16 boundaries still catch any field reorder.
static const uint8_t WING_GPS_RESCUE_GOLDEN[] = {
    0x00,                          // allowArmingWithoutFix (false)
    0x08,                          // minSats (8)
    0x19,                          // maxBankAngle (25)
    0x32, 0x00,                    // orbitRadiusM (50)
    0x32, 0x00,                    // returnAltitudeM (50)
    0x19, 0x00,                    // minLoiterAltM (25)
    0x32,                          // cruiseThrottle (50)
    0x1E,                          // minThrottle (30)
    0x2D,                          // abortThrottle (45)
    0x1E,                          // navP (30)
    0x1E,                          // altP (30)
    0x32,                          // turnCompensation (50)
    0x90, 0x01,                    // minHeadingSpeedCmS (400)
    0xC8, 0x00,                    // stallSpeedCmS (200)
    0x1E, 0x00,                    // minStartDistM (30)
    0x01,                          // sanityChecks (1)
};

static void populateCanonicalWingGpsRescue(gpsRescueConfig_t *c)
{
    memset(c, 0, sizeof(*c));
    c->allowArmingWithoutFix = 0;
    c->minSats               = 8;
    c->maxBankAngle          = 25;
    c->orbitRadiusM          = 50;
    c->returnAltitudeM       = 50;
    c->minLoiterAltM         = 25;
    c->cruiseThrottle        = 50;
    c->minThrottle           = 30;
    c->abortThrottle         = 45;
    c->navP                  = 30;
    c->altP                  = 30;
    c->turnCompensation      = 50;
    c->minHeadingSpeedCmS    = 400;
    c->stallSpeedCmS         = 200;
    c->minStartDistM         = 30;
    c->sanityChecks          = 1;
}

TEST(WingGpsRescueMspUnitTest, ConfigEncodesToGoldenVector)
{
    gpsRescueConfig_t cfg;
    populateCanonicalWingGpsRescue(&cfg);

    uint8_t buffer[48] = {0};
    sbuf_t sbuf;
    sbufInit(&sbuf, buffer, buffer + sizeof(buffer));

    serializeWingGpsRescue(&sbuf, &cfg);

    const size_t written = (size_t)(sbuf.ptr - buffer);
    EXPECT_EQ(sizeof(WING_GPS_RESCUE_GOLDEN), written);
    EXPECT_EQ(0, memcmp(buffer, WING_GPS_RESCUE_GOLDEN, sizeof(WING_GPS_RESCUE_GOLDEN)));
}

TEST(WingGpsRescueMspUnitTest, ConfigDecodesGoldenVector)
{
    gpsRescueConfig_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    sbuf_t sbuf;
    sbufInit(&sbuf, (uint8_t *)WING_GPS_RESCUE_GOLDEN,
             (uint8_t *)WING_GPS_RESCUE_GOLDEN + sizeof(WING_GPS_RESCUE_GOLDEN));

    const bool ok = deserializeWingGpsRescue(&sbuf, &cfg);
    EXPECT_TRUE(ok);

    gpsRescueConfig_t expected;
    populateCanonicalWingGpsRescue(&expected);

    EXPECT_EQ(expected.allowArmingWithoutFix, cfg.allowArmingWithoutFix);
    EXPECT_EQ(expected.minSats,               cfg.minSats);
    EXPECT_EQ(expected.maxBankAngle,          cfg.maxBankAngle);
    EXPECT_EQ(expected.orbitRadiusM,          cfg.orbitRadiusM);
    EXPECT_EQ(expected.returnAltitudeM,       cfg.returnAltitudeM);
    EXPECT_EQ(expected.minLoiterAltM,         cfg.minLoiterAltM);
    EXPECT_EQ(expected.cruiseThrottle,        cfg.cruiseThrottle);
    EXPECT_EQ(expected.minThrottle,           cfg.minThrottle);
    EXPECT_EQ(expected.abortThrottle,         cfg.abortThrottle);
    EXPECT_EQ(expected.navP,                  cfg.navP);
    EXPECT_EQ(expected.altP,                  cfg.altP);
    EXPECT_EQ(expected.turnCompensation,      cfg.turnCompensation);
    EXPECT_EQ(expected.minHeadingSpeedCmS,    cfg.minHeadingSpeedCmS);
    EXPECT_EQ(expected.stallSpeedCmS,         cfg.stallSpeedCmS);
    EXPECT_EQ(expected.minStartDistM,         cfg.minStartDistM);
    EXPECT_EQ(expected.sanityChecks,          cfg.sanityChecks);
}

TEST(WingGpsRescueMspUnitTest, ConfigRoundTrips)
{
    gpsRescueConfig_t source;
    populateCanonicalWingGpsRescue(&source);

    uint8_t buffer[48] = {0};
    sbuf_t writer;
    sbufInit(&writer, buffer, buffer + sizeof(buffer));
    serializeWingGpsRescue(&writer, &source);
    const size_t written = (size_t)(writer.ptr - buffer);

    gpsRescueConfig_t roundTripped;
    memset(&roundTripped, 0, sizeof(roundTripped));
    sbuf_t reader;
    sbufInit(&reader, buffer, buffer + written);
    EXPECT_TRUE(deserializeWingGpsRescue(&reader, &roundTripped));

    EXPECT_EQ(0, memcmp(&source, &roundTripped, sizeof(source)));
}

TEST(WingGpsRescueMspUnitTest, DeserializeRejectsTruncatedPayload)
{
    gpsRescueConfig_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    // Feed only the first 10 bytes of the 22-byte golden vector.
    sbuf_t sbuf;
    sbufInit(&sbuf, (uint8_t *)WING_GPS_RESCUE_GOLDEN,
             (uint8_t *)WING_GPS_RESCUE_GOLDEN + 10);

    EXPECT_FALSE(deserializeWingGpsRescue(&sbuf, &cfg));
}
