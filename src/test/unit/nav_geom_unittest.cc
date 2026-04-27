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

#include <cmath>

extern "C" {
    #include "platform.h"
    #include "common/maths.h"
    #include "flight/nav_geom.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

namespace {

// Decimal-degrees home field, arbitrary but stable across tests.
constexpr float HOME_LAT = 45.0000f;
constexpr float HOME_LON = -75.0000f;

// One degree of latitude is always ~111 km. Longitude at 45 deg lat is
// ~78.7 km (111 * cos(45deg)). Used as a sanity check for haversine.
constexpr float M_PER_DEG_LAT   = 111195.0f;
constexpr float M_PER_DEG_LON_45 = 78626.0f;

} // namespace

TEST(NavGeomTest, DistanceZeroForSamePoint)
{
    EXPECT_NEAR(navGeomDistanceM(HOME_LAT, HOME_LON, HOME_LAT, HOME_LON),
                0.0f, 0.1f);
}

TEST(NavGeomTest, DistanceOneDegreeLatitude)
{
    // Moving 1 deg north = ~111.2 km regardless of longitude.
    const float d = navGeomDistanceM(HOME_LAT, HOME_LON,
                                     HOME_LAT + 1.0f, HOME_LON);
    EXPECT_NEAR(d, M_PER_DEG_LAT, 300.0f);   // 0.3% tolerance
}

TEST(NavGeomTest, DistanceOneDegreeLongitudeAt45Lat)
{
    const float d = navGeomDistanceM(HOME_LAT, HOME_LON,
                                     HOME_LAT, HOME_LON + 1.0f);
    EXPECT_NEAR(d, M_PER_DEG_LON_45, 300.0f);
}

TEST(NavGeomTest, DistanceSymmetric)
{
    const float ab = navGeomDistanceM(HOME_LAT, HOME_LON,
                                      HOME_LAT + 0.01f, HOME_LON + 0.01f);
    const float ba = navGeomDistanceM(HOME_LAT + 0.01f, HOME_LON + 0.01f,
                                      HOME_LAT, HOME_LON);
    EXPECT_NEAR(ab, ba, 0.01f);
}

TEST(NavGeomTest, BearingDueNorth)
{
    // Target is directly north. Compass heading should be ~0 rad.
    const float b = navGeomBearingRad(HOME_LAT, HOME_LON,
                                      HOME_LAT + 0.01f, HOME_LON);
    EXPECT_NEAR(b, 0.0f, 0.01f);
}

TEST(NavGeomTest, BearingDueEast)
{
    // Target is directly east. Compass heading ~PI/2.
    const float b = navGeomBearingRad(HOME_LAT, HOME_LON,
                                      HOME_LAT, HOME_LON + 0.01f);
    EXPECT_NEAR(b, M_PIf / 2.0f, 0.01f);
}

TEST(NavGeomTest, BearingDueSouth)
{
    const float b = navGeomBearingRad(HOME_LAT, HOME_LON,
                                      HOME_LAT - 0.01f, HOME_LON);
    EXPECT_NEAR(b, M_PIf, 0.01f);
}

TEST(NavGeomTest, BearingDueWest)
{
    const float b = navGeomBearingRad(HOME_LAT, HOME_LON,
                                      HOME_LAT, HOME_LON - 0.01f);
    EXPECT_NEAR(b, 3.0f * M_PIf / 2.0f, 0.01f);
}

TEST(NavGeomTest, BearingWrapsIntoPositiveRange)
{
    // Every possible bearing we produce should be in [0, 2*PI).
    const float b = navGeomBearingRad(HOME_LAT, HOME_LON,
                                      HOME_LAT - 0.01f, HOME_LON - 0.01f);
    EXPECT_GE(b, 0.0f);
    EXPECT_LT(b, 2.0f * M_PIf);
}

TEST(NavGeomTest, HeadingErrorZeroForIdenticalHeadings)
{
    EXPECT_FLOAT_EQ(navGeomHeadingErrorRad(0.0f, 0.0f), 0.0f);
    EXPECT_FLOAT_EQ(navGeomHeadingErrorRad(M_PIf, M_PIf), 0.0f);
}

TEST(NavGeomTest, HeadingErrorPositiveForClockwiseTarget)
{
    // Current: due north. Target: due east. Should steer right (+pi/2).
    EXPECT_NEAR(navGeomHeadingErrorRad(0.0f, M_PIf / 2.0f),
                M_PIf / 2.0f, 1e-5f);
}

TEST(NavGeomTest, HeadingErrorNegativeForCounterclockwiseTarget)
{
    // Current: due north. Target: due west. Should steer left (-pi/2).
    EXPECT_NEAR(navGeomHeadingErrorRad(0.0f, 3.0f * M_PIf / 2.0f),
                -M_PIf / 2.0f, 1e-5f);
}

TEST(NavGeomTest, HeadingErrorWrapsAcrossNorth)
{
    // Current: 10 deg east of north. Target: 10 deg west of north.
    // Shortest path is -20 deg, not +340 deg.
    const float cur = 10.0f * M_PIf / 180.0f;
    const float tgt = 350.0f * M_PIf / 180.0f;
    EXPECT_NEAR(navGeomHeadingErrorRad(cur, tgt),
                -20.0f * M_PIf / 180.0f, 1e-5f);
}

TEST(NavGeomTest, HeadingErrorAlwaysInPiRange)
{
    // Stress: random-ish headings should always produce |error| <= PI.
    const float headings[] = { 0.1f, 0.5f * M_PIf, M_PIf, 1.5f * M_PIf,
                               2.0f * M_PIf - 0.1f };
    for (float a : headings) {
        for (float b : headings) {
            const float e = navGeomHeadingErrorRad(a, b);
            EXPECT_GE(e, -M_PIf - 1e-5f);
            EXPECT_LE(e,  M_PIf + 1e-5f);
        }
    }
}
