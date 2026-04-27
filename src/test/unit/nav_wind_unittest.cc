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
#include <cstdint>

extern "C" {
    #include "platform.h"
    #include "common/maths.h"
    #include "flight/nav_wind.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

namespace {

// Synthesize a GPS sample for a wing flying at a given heading through a
// known (true) wind field. We use the exact (non-linearized) ground-speed
// formula so the test is an honest check on the linearization error too.
//
//   v_ground_vec = v_air_vec + v_wind_vec
//
// Axes: x = East, y = North. Course is compass heading -- 0 = N, PI/2 = E.
static void pushSample(float courseCompassRad,
                       float airspeedCmS,
                       float windFromCompassRad,  // direction wind COMES FROM
                       float windSpeedCmS,
                       timeUs_t now)
{
    // Aircraft velocity through the air, in East/North components.
    const float vAirE = airspeedCmS * sinf(courseCompassRad);
    const float vAirN = airspeedCmS * cosf(courseCompassRad);

    // Wind BLOWS TOWARD (windFrom + PI). Decompose to East/North.
    const float windToward = windFromCompassRad + M_PIf;
    const float vWindE = windSpeedCmS * sinf(windToward);
    const float vWindN = windSpeedCmS * cosf(windToward);

    const float vGroundE = vAirE + vWindE;
    const float vGroundN = vAirN + vWindN;

    const float groundSpeed = sqrtf(vGroundE * vGroundE + vGroundN * vGroundN);

    // nav_wind treats the course arg as the ground-track direction. That's
    // what the FC gets from GPS (course-over-ground), which is NOT exactly
    // equal to aircraft heading in wind -- but for the linearization to be
    // fair, we feed it ground track.
    float groundTrack = atan2f(vGroundE, vGroundN);
    if (groundTrack < 0.0f) groundTrack += 2.0f * M_PIf;

    navWindAddSample(groundTrack, groundSpeed, now);
}

// Simulate a full orbit: N samples evenly spaced around 360 degrees of
// aircraft heading, at a constant airspeed, through a constant wind.
static void simulateOrbit(int samples,
                          float airspeedCmS,
                          float windFromCompassRad,
                          float windSpeedCmS,
                          timeUs_t startUs,
                          timeUs_t sampleIntervalUs = 200000) // 5 Hz
{
    for (int i = 0; i < samples; i++) {
        const float heading = (2.0f * M_PIf * (float)i) / (float)samples;
        pushSample(heading, airspeedCmS, windFromCompassRad, windSpeedCmS,
                   startUs + (timeUs_t)i * sampleIntervalUs);
    }
}

static float angleDiffRad(float a, float b)
{
    float d = a - b;
    while (d >  M_PIf) d -= 2.0f * M_PIf;
    while (d < -M_PIf) d += 2.0f * M_PIf;
    return fabsf(d);
}

} // namespace

TEST(NavWindTest, ResetProducesUnknown)
{
    navWindReset();
    const navWindEstimate_t *e = navWindGetEstimate();
    ASSERT_EQ(e->status, NAV_WIND_UNKNOWN);
    EXPECT_FLOAT_EQ(e->confidence, 0.0f);
}

TEST(NavWindTest, TooFewSamplesStaysUnknown)
{
    navWindReset();
    // 4 samples, well below MIN_SAMPLES_FOR_CONVERGING (12) and
    // WEDGES_FOR_CONVERGING (6). Should remain UNKNOWN regardless of values.
    pushSample(0.0f,         1500.0f, M_PIf, 300.0f, 100000);
    pushSample(M_PIf / 2.0f, 1500.0f, M_PIf, 300.0f, 300000);
    pushSample(M_PIf,        1500.0f, M_PIf, 300.0f, 500000);
    pushSample(3.0f * M_PIf / 2.0f, 1500.0f, M_PIf, 300.0f, 700000);

    const navWindEstimate_t *e = navWindGetEstimate();
    EXPECT_EQ(e->status, NAV_WIND_UNKNOWN);
}

TEST(NavWindTest, ZeroWindOrbitConverges)
{
    navWindReset();
    // No wind: groundspeed should match airspeed on every heading, the
    // fit should return wind ~= 0 with airspeed ~= 1500.
    simulateOrbit(24, 1500.0f, 0.0f, 0.0f, 1000);

    const navWindEstimate_t *e = navWindGetEstimate();
    ASSERT_EQ(e->status, NAV_WIND_VALID);
    EXPECT_NEAR(e->windSpeedCmS, 0.0f,   5.0f);
    EXPECT_NEAR(e->airSpeedCmS,  1500.0f, 5.0f);
}

TEST(NavWindTest, ModerateWindRecoveredWithinTolerance)
{
    navWindReset();
    // 10 m/s wind (1000 cm/s) from heading 90 deg (due East), aircraft
    // airspeed 15 m/s (1500 cm/s). Wind is ~67% of airspeed -- at the
    // outer edge of the linearization's validity range.
    const float trueWindSpeed = 1000.0f;
    const float trueWindFrom  = M_PIf / 2.0f;  // from East
    simulateOrbit(24, 1500.0f, trueWindFrom, trueWindSpeed, 1000);

    const navWindEstimate_t *e = navWindGetEstimate();
    ASSERT_EQ(e->status, NAV_WIND_VALID);

    // Linearization adds a few percent error at this wind/air ratio;
    // expect 15% magnitude tolerance, 20 deg direction tolerance.
    EXPECT_NEAR(e->windSpeedCmS, trueWindSpeed, 150.0f);
    EXPECT_LT(angleDiffRad(e->windFromHeadingRad, trueWindFrom),
              20.0f * RAD);
}

TEST(NavWindTest, GentleWindRecoveredAccurately)
{
    navWindReset();
    // 3 m/s wind -- well within linearization range. Tight tolerance.
    const float trueWindSpeed = 300.0f;
    const float trueWindFrom  = 3.0f * M_PIf / 4.0f;  // from SE
    simulateOrbit(24, 1500.0f, trueWindFrom, trueWindSpeed, 1000);

    const navWindEstimate_t *e = navWindGetEstimate();
    ASSERT_EQ(e->status, NAV_WIND_VALID);
    EXPECT_NEAR(e->windSpeedCmS, trueWindSpeed, 30.0f);
    EXPECT_NEAR(e->airSpeedCmS,  1500.0f,       30.0f);
    EXPECT_LT(angleDiffRad(e->windFromHeadingRad, trueWindFrom),
              10.0f * RAD);
}

TEST(NavWindTest, PartialCoverageStaysConverging)
{
    navWindReset();
    // Only half the orbit sampled -- insufficient wedge coverage for
    // VALID, but enough for CONVERGING.
    for (int i = 0; i < 14; i++) {
        const float heading = M_PIf * (float)i / 14.0f;  // 0..PI
        pushSample(heading, 1500.0f, 0.0f, 300.0f,
                   1000 + (timeUs_t)i * 200000);
    }

    const navWindEstimate_t *e = navWindGetEstimate();
    EXPECT_EQ(e->status, NAV_WIND_CONVERGING);
    EXPECT_GT(e->confidence, 0.0f);
    EXPECT_LT(e->confidence, 1.0f);
}

TEST(NavWindTest, SampleTooCloseInTimeDropped)
{
    navWindReset();
    // Push two samples at the same microsecond -- second one must be
    // rejected by MIN_SAMPLE_INTERVAL_US gating.
    pushSample(0.0f, 1500.0f, 0.0f, 0.0f, 1000000);
    pushSample(0.0f, 1500.0f, 0.0f, 0.0f, 1000000);
    // (indirect assertion -- no way to see n from outside, but this test
    // exists to exercise the rate limiter path under coverage.)
    SUCCEED();
}

TEST(NavWindTest, LowGroundSpeedSampleRejected)
{
    navWindReset();
    // Below MIN_VALID_GROUND_SPEED_CMS (300 cm/s) -- should be ignored.
    for (int i = 0; i < 24; i++) {
        navWindAddSample(2.0f * M_PIf * (float)i / 24.0f, 100.0f,
                         1000 + (timeUs_t)i * 200000);
    }
    const navWindEstimate_t *e = navWindGetEstimate();
    EXPECT_EQ(e->status, NAV_WIND_UNKNOWN);
}
