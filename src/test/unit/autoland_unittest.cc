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
#include <cstring>

extern "C" {
    #include "platform.h"
    #include "common/maths.h"
    #include "flight/autoland.h"
    #include "flight/nav_wind.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

static constexpr timeUs_t MS = 1000;
static constexpr timeUs_t S  = 1000 * MS;

static constexpr float HOME_LAT = 45.0f;
static constexpr float HOME_LON = -75.0f;

static constexpr float LAUNCH_BARO_CM = 4000.0f;

static autolandSequenceConfig_t makeDefaultCfg()
{
    autolandSequenceConfig_t cfg{};
    cfg.glide_pitch_deg        = 5;
    cfg.throttle_cut_alt_cm    = 0;
    cfg.cruise_throttle_pct    = 40;
    cfg.commit_altitude_cm     = 600;
    cfg.flare_start_alt_cm     = 150;
    cfg.flare_pitch_deg        = 8;
    cfg.max_bank_angle_deg     = 25;
    cfg.nav_p_gain             = 30;
    cfg.orbit_radius_m         = 50;
    cfg.loiter_timeout_s       = 60;
    cfg.orbits_before_descent  = 2;
    cfg.downwind_distance_m    = 150;
    cfg.base_radius_m          = 40;
    cfg.final_distance_m       = 80;
    cfg.touchdown_alt_threshold_cm = 30;
    cfg.touchdown_accel_threshold  = 30;   // 3.0 g
    cfg.touchdown_quiescence_ms    = 2000;
    return cfg;
}

static autolandSensorSample_t sampleAt(float lat, float lon, float courseRad,
                                       float groundSpeedCmS, float aglCm)
{
    autolandSensorSample_t s{};
    s.baroCm          = LAUNCH_BARO_CM + aglCm;
    s.baroValid       = true;
    s.latDeg          = lat;
    s.lonDeg          = lon;
    s.groundCourseRad = courseRad;
    s.groundSpeedCmS  = groundSpeedCmS;
    s.gpsValid        = true;
    // "In-flight" IMU defaults: at-rest accel, gently banking gyro.
    // Tests that care about touchdown detection override these.
    s.accelMagG       = 1.0f;
    s.gyroRateDegS    = 10.0f;
    s.imuValid        = true;
    return s;
}

// "On ground, motionless" sample -- wing has landed and stopped sliding.
// Trips the quiescence tracker.
static autolandSensorSample_t sampleAtQuiet(float aglCm)
{
    auto s = sampleAt(HOME_LAT, HOME_LON, 0.0f, 0.0f, aglCm);
    s.accelMagG    = 1.0f;    // just gravity
    s.gyroRateDegS = 1.0f;    // no rotation
    return s;
}

static autolandSensorSample_t sampleAtHome(float aglCm)
{
    return sampleAt(HOME_LAT, HOME_LON, 0.0f, 1500.0f, aglCm);
}

class AutolandTest : public ::testing::Test {
protected:
    void SetUp() override {
        autolandInit();
        navWindReset();
    }
};

static bool enterStraightIn(timeUs_t now)
{
    auto cfg = makeDefaultCfg();
    return autolandRequestEntry(now, LAUNCH_BARO_CM, HOME_LAT, HOME_LON,
                                AL_ENTRY_STRAIGHT_IN, &cfg);
}

static bool enterOrbitDescent(timeUs_t now)
{
    auto cfg = makeDefaultCfg();
    return autolandRequestEntry(now, LAUNCH_BARO_CM, HOME_LAT, HOME_LON,
                                AL_ENTRY_ORBIT_DESCENT, &cfg);
}

// Fly a complete wind-informative orbit to give nav_wind enough data
// to go VALID. Samples span 360° of course at ~5 Hz.
static timeUs_t simulateWindOrbit(timeUs_t startUs)
{
    constexpr int N = 24;
    timeUs_t t = startUs;
    for (int i = 0; i < N; i++) {
        const float course = 2.0f * M_PIf * (float)i / (float)N;
        auto s = sampleAt(HOME_LAT, HOME_LON + 0.001f, course, 1500.0f, 5000.0f);
        autolandUpdate(t, &s);
        t += 200 * MS;
    }
    return t;
}

// ---- State machine basics ----

TEST_F(AutolandTest, InitLeavesMachineInIdle)
{
    EXPECT_EQ(autolandGetPhase(), AL_IDLE);
    EXPECT_TRUE(autolandCanEnter());
}

TEST_F(AutolandTest, RequestEntryWithNullCfgRefused)
{
    EXPECT_FALSE(autolandRequestEntry(1 * S, LAUNCH_BARO_CM, HOME_LAT, HOME_LON,
                                      AL_ENTRY_STRAIGHT_IN, nullptr));
    EXPECT_EQ(autolandGetPhase(), AL_IDLE);
}

TEST_F(AutolandTest, RequestEntryTransitionsToEntry)
{
    ASSERT_TRUE(enterStraightIn(1 * S));
    EXPECT_EQ(autolandGetPhase(), AL_ENTRY);
}

TEST_F(AutolandTest, AbortLocksArmSessionReinitClears)
{
    enterStraightIn(1 * S);
    autolandAbort(AL_ABORT_PILOT, 2 * S);
    EXPECT_FALSE(autolandCanEnter());
    autolandInit();
    EXPECT_TRUE(autolandCanEnter());
}

TEST_F(AutolandTest, PhaseNamesDefinedForEveryPhase)
{
    for (int p = 0; p < AL_PHASE_COUNT; p++) {
        EXPECT_GT(strlen(autolandPhaseName((autolandPhase_e)p)), 0u);
    }
}

// ---- Baro + pitch (Phase 4) ----

TEST_F(AutolandTest, BaroReferenceLatchedAtEntryAndNeverRezeroed)
{
    enterStraightIn(1 * S);
    float ref;
    ASSERT_TRUE(autolandGetBaroReferenceCm(&ref));
    EXPECT_FLOAT_EQ(ref, LAUNCH_BARO_CM);

    auto s = sampleAtHome(3000.0f);
    autolandUpdate(2 * S, &s);
    ASSERT_TRUE(autolandGetBaroReferenceCm(&ref));
    EXPECT_FLOAT_EQ(ref, LAUNCH_BARO_CM);
}

TEST_F(AutolandTest, PitchSetpointUsesFlarePitchInFlare)
{
    auto cfg = makeDefaultCfg();
    cfg.flare_pitch_deg = 11;
    autolandRequestEntry(1 * S, LAUNCH_BARO_CM, HOME_LAT, HOME_LON,
                         AL_ENTRY_STRAIGHT_IN, &cfg);
    autolandTransition(AL_FLARE, 2 * S);
    float pitch;
    ASSERT_TRUE(autolandGetPitchSetpoint(&pitch));
    EXPECT_FLOAT_EQ(pitch, 11.0f);
}

TEST_F(AutolandTest, ThrottleStaysAtCruiseUntilCrossingCutAlt)
{
    auto cfg = makeDefaultCfg();
    cfg.throttle_cut_alt_cm = 1500;
    cfg.cruise_throttle_pct = 40;
    autolandRequestEntry(1 * S, LAUNCH_BARO_CM, HOME_LAT, HOME_LON,
                         AL_ENTRY_STRAIGHT_IN, &cfg);

    auto high = sampleAtHome(2500.0f);
    autolandUpdate(2 * S, &high);
    float throttle;
    ASSERT_TRUE(autolandGetThrottleOverride(&throttle));
    EXPECT_FLOAT_EQ(throttle, 0.40f);

    auto low = sampleAtHome(1000.0f);
    autolandUpdate(3 * S, &low);
    ASSERT_TRUE(autolandGetThrottleOverride(&throttle));
    EXPECT_FLOAT_EQ(throttle, 0.0f);
}

// ---- Phase 5a: straight-in path ----

TEST_F(AutolandTest, StraightInEntryAutoAdvancesAfterDwell)
{
    enterStraightIn(1 * S);
    auto s = sampleAtHome(5000.0f);
    autolandUpdate(1 * S + 600 * MS, &s);
    EXPECT_EQ(autolandGetPhase(), AL_STRAIGHT_IN);
}

TEST_F(AutolandTest, StraightInFlareTransitionOnAgl)
{
    enterStraightIn(1 * S);
    auto high = sampleAtHome(5000.0f);
    autolandUpdate(1 * S + 600 * MS, &high);
    ASSERT_EQ(autolandGetPhase(), AL_STRAIGHT_IN);

    auto low = sampleAtHome(140.0f);
    autolandUpdate(2 * S, &low);
    EXPECT_EQ(autolandGetPhase(), AL_FLARE);
}

TEST_F(AutolandTest, StraightInRollSteersTowardHome)
{
    enterStraightIn(1 * S);
    auto s = sampleAtHome(5000.0f);
    autolandUpdate(1 * S + 600 * MS, &s);
    ASSERT_EQ(autolandGetPhase(), AL_STRAIGHT_IN);

    // NW of home, heading north -- home is east-ish: should bank right.
    auto offPos = sampleAt(HOME_LAT + 0.005f, HOME_LON - 0.005f,
                           0.0f, 1500.0f, 5000.0f);
    autolandUpdate(2 * S, &offPos);
    float roll;
    ASSERT_TRUE(autolandGetRollSetpoint(&roll));
    EXPECT_GT(roll, 0.0f);
    EXPECT_LE(roll, 25.0f);
}

TEST_F(AutolandTest, GpsLossDuringFlightAborts)
{
    enterStraightIn(1 * S);
    auto s = sampleAtHome(5000.0f);
    autolandUpdate(1 * S + 600 * MS, &s);

    auto noGps = s;
    noGps.gpsValid = false;
    autolandUpdate(2 * S, &noGps);
    EXPECT_EQ(autolandGetPhase(), AL_ABORT);
    EXPECT_EQ(autolandGetLastAbortCause(), AL_ABORT_GPS_LOSS);
}

TEST_F(AutolandTest, BaroLossDuringFlightAborts)
{
    enterStraightIn(1 * S);
    auto s = sampleAtHome(5000.0f);
    autolandUpdate(1 * S + 600 * MS, &s);

    auto noBaro = s;
    noBaro.baroValid = false;
    autolandUpdate(2 * S, &noBaro);
    EXPECT_EQ(autolandGetPhase(), AL_ABORT);
    EXPECT_EQ(autolandGetLastAbortCause(), AL_ABORT_BARO_LOSS);
}

// ---- Phase 5b: orbit path + pattern ----

TEST_F(AutolandTest, OrbitEntryModeAdvancesToOrbitDescent)
{
    enterOrbitDescent(1 * S);
    auto s = sampleAtHome(5000.0f);
    autolandUpdate(1 * S + 600 * MS, &s);
    EXPECT_EQ(autolandGetPhase(), AL_ORBIT_DESCENT);
}

TEST_F(AutolandTest, OrbitTimeoutFallsBackToStraightIn)
{
    auto cfg = makeDefaultCfg();
    cfg.loiter_timeout_s      = 10;
    cfg.orbits_before_descent = 99;   // effectively unreachable
    autolandRequestEntry(1 * S, LAUNCH_BARO_CM, HOME_LAT, HOME_LON,
                         AL_ENTRY_ORBIT_DESCENT, &cfg);

    auto s = sampleAtHome(5000.0f);
    autolandUpdate(1 * S + 600 * MS, &s);
    ASSERT_EQ(autolandGetPhase(), AL_ORBIT_DESCENT);

    // Push a few samples across the 10s timeout; wind never converges
    // (samples don't sweep 360°) so we should hit the fallback.
    timeUs_t t = 1 * S + 700 * MS;
    for (int i = 0; i < 6; i++) {
        // Keep the course identical so wedge coverage stays minimal.
        autolandUpdate(t, &s);
        t += 2 * S;
    }
    EXPECT_EQ(autolandGetPhase(), AL_STRAIGHT_IN);
}

TEST_F(AutolandTest, OrbitDescendsToDownwindOnWindConvergence)
{
    auto cfg = makeDefaultCfg();
    cfg.loiter_timeout_s      = 120;
    cfg.orbits_before_descent = 1;   // just need one orbit's worth
    autolandRequestEntry(1 * S, LAUNCH_BARO_CM, HOME_LAT, HOME_LON,
                         AL_ENTRY_ORBIT_DESCENT, &cfg);

    // Initial sample to exit AL_ENTRY dwell.
    auto s = sampleAtHome(5000.0f);
    autolandUpdate(1 * S + 600 * MS, &s);
    ASSERT_EQ(autolandGetPhase(), AL_ORBIT_DESCENT);

    // Simulate a full wind-informative orbit + some extra for robustness.
    timeUs_t t = simulateWindOrbit(2 * S);
    simulateWindOrbit(t);  // second orbit so wedge coverage is full

    // Wind estimator should be VALID; cumulative heading change >= 1 orbit.
    EXPECT_EQ(autolandGetPhase(), AL_DOWNWIND);
}

TEST_F(AutolandTest, DownwindTransitionsToBaseOnDistance)
{
    // Skip straight to AL_DOWNWIND via explicit transition so the test
    // doesn't depend on the full orbit simulation.
    enterOrbitDescent(1 * S);
    auto s = sampleAtHome(5000.0f);
    autolandUpdate(1 * S + 600 * MS, &s);
    ASSERT_EQ(autolandGetPhase(), AL_ORBIT_DESCENT);

    autolandTransition(AL_DOWNWIND, 2 * S);

    // Position well within downwind_distance_m (150) -- still DOWNWIND.
    auto near = sampleAt(HOME_LAT + 0.0005f, HOME_LON, 0.0f, 1500.0f, 5000.0f);
    autolandUpdate(3 * S, &near);
    EXPECT_EQ(autolandGetPhase(), AL_DOWNWIND);

    // Far enough to trigger AL_BASE -- 0.002 deg lat ~= 220 m.
    auto far = sampleAt(HOME_LAT + 0.002f, HOME_LON, 0.0f, 1500.0f, 5000.0f);
    autolandUpdate(4 * S, &far);
    EXPECT_EQ(autolandGetPhase(), AL_BASE);
}

TEST_F(AutolandTest, BaseTransitionsToFinalAfter90DegTurn)
{
    enterOrbitDescent(1 * S);
    auto s = sampleAt(HOME_LAT, HOME_LON, 0.0f, 1500.0f, 5000.0f);
    autolandUpdate(1 * S + 600 * MS, &s);
    autolandTransition(AL_BASE, 2 * S);

    // Feed cumulative heading changes. Each ~30 deg delta; after 3 we
    // should be in AL_FINAL.
    auto heading30 = sampleAt(HOME_LAT, HOME_LON,
                              30.0f * M_PIf / 180.0f, 1500.0f, 5000.0f);
    autolandUpdate(3 * S, &heading30);
    EXPECT_EQ(autolandGetPhase(), AL_BASE);

    auto heading60 = sampleAt(HOME_LAT, HOME_LON,
                              60.0f * M_PIf / 180.0f, 1500.0f, 5000.0f);
    autolandUpdate(4 * S, &heading60);
    EXPECT_EQ(autolandGetPhase(), AL_BASE);

    auto heading91 = sampleAt(HOME_LAT, HOME_LON,
                              91.0f * M_PIf / 180.0f, 1500.0f, 5000.0f);
    autolandUpdate(5 * S, &heading91);
    EXPECT_EQ(autolandGetPhase(), AL_FINAL);
}

TEST_F(AutolandTest, FinalTransitionsToFlareOnAgl)
{
    enterOrbitDescent(1 * S);
    auto s = sampleAtHome(5000.0f);
    autolandUpdate(1 * S + 600 * MS, &s);
    autolandTransition(AL_FINAL, 2 * S);

    auto low = sampleAtHome(140.0f);
    autolandUpdate(3 * S, &low);
    EXPECT_EQ(autolandGetPhase(), AL_FLARE);
}

TEST_F(AutolandTest, OrbitControllerProducesBankSetpoint)
{
    enterOrbitDescent(1 * S);
    // Push well into orbit-descent and place the wing outside the
    // orbit radius so cross-track correction plus tangent both demand
    // a non-zero bank.
    auto s = sampleAt(HOME_LAT + 0.001f, HOME_LON, 0.0f, 1500.0f, 5000.0f);
    autolandUpdate(1 * S + 600 * MS, &s);
    ASSERT_EQ(autolandGetPhase(), AL_ORBIT_DESCENT);

    autolandUpdate(2 * S, &s);
    float roll;
    ASSERT_TRUE(autolandGetRollSetpoint(&roll));
    EXPECT_NE(roll, 0.0f);
    EXPECT_LE(fabsf(roll), 25.0f);   // clamped by max_bank_angle
}

TEST_F(AutolandTest, GpsLossInOrbitAborts)
{
    enterOrbitDescent(1 * S);
    auto s = sampleAtHome(5000.0f);
    autolandUpdate(1 * S + 600 * MS, &s);
    ASSERT_EQ(autolandGetPhase(), AL_ORBIT_DESCENT);

    auto noGps = s;
    noGps.gpsValid = false;
    autolandUpdate(2 * S, &noGps);
    EXPECT_EQ(autolandGetPhase(), AL_ABORT);
    EXPECT_EQ(autolandGetLastAbortCause(), AL_ABORT_GPS_LOSS);
}

// ---- Phase 6: touchdown detection + commit latch + disarm ----

TEST_F(AutolandTest, CommitLatchesBelowCommitAltitude)
{
    auto cfg = makeDefaultCfg();
    cfg.commit_altitude_cm = 600;   // 20 ft
    autolandRequestEntry(1 * S, LAUNCH_BARO_CM, HOME_LAT, HOME_LON,
                         AL_ENTRY_STRAIGHT_IN, &cfg);
    EXPECT_FALSE(autolandIsCommitLatched());

    auto high = sampleAtHome(2000.0f);    // well above commit alt
    autolandUpdate(1 * S + 600 * MS, &high);
    EXPECT_FALSE(autolandIsCommitLatched());

    auto low = sampleAtHome(400.0f);      // below commit alt
    autolandUpdate(2 * S, &low);
    EXPECT_TRUE(autolandIsCommitLatched());
}

TEST_F(AutolandTest, ExternalAbortRefusedOnceCommitLatched)
{
    enterStraightIn(1 * S);
    auto high = sampleAtHome(2000.0f);
    autolandUpdate(1 * S + 600 * MS, &high);
    ASSERT_EQ(autolandGetPhase(), AL_STRAIGHT_IN);

    auto low = sampleAtHome(300.0f);      // below commit_alt (600)
    autolandUpdate(2 * S, &low);
    ASSERT_TRUE(autolandIsCommitLatched());

    // Pilot abort is refused -- state machine keeps flying to flare.
    EXPECT_FALSE(autolandAbort(AL_ABORT_PILOT, 3 * S));
    EXPECT_NE(autolandGetPhase(), AL_ABORT);
}

TEST_F(AutolandTest, InternalAbortBypassesCommitLatch)
{
    enterStraightIn(1 * S);
    auto high = sampleAtHome(2000.0f);
    autolandUpdate(1 * S + 600 * MS, &high);
    auto low = sampleAtHome(300.0f);
    autolandUpdate(2 * S, &low);
    ASSERT_TRUE(autolandIsCommitLatched());

    // GPS loss is an internal/safety abort -- must bypass the latch
    // (invariant #6: degraded modes never silently continue).
    auto noGps = low;
    noGps.gpsValid = false;
    autolandUpdate(3 * S, &noGps);
    EXPECT_EQ(autolandGetPhase(), AL_ABORT);
    EXPECT_EQ(autolandGetLastAbortCause(), AL_ABORT_GPS_LOSS);
}

TEST_F(AutolandTest, TouchdownViaBaroFromFlare)
{
    enterStraightIn(1 * S);
    auto s = sampleAtHome(5000.0f);
    autolandUpdate(1 * S + 600 * MS, &s);
    autolandTransition(AL_FLARE, 2 * S);

    // Baro drops below touchdown_alt_threshold_cm (30 default).
    auto wheelsDown = sampleAtHome(20.0f);
    wheelsDown.accelMagG = 1.0f;        // no spike
    wheelsDown.gyroRateDegS = 10.0f;    // not quiet, not touching quiet timer
    autolandUpdate(3 * S, &wheelsDown);
    EXPECT_EQ(autolandGetPhase(), AL_TOUCHDOWN);
}

TEST_F(AutolandTest, TouchdownViaAccelSpikeFromFlare)
{
    auto cfg = makeDefaultCfg();
    cfg.touchdown_accel_threshold = 30;  // 3.0g
    autolandRequestEntry(1 * S, LAUNCH_BARO_CM, HOME_LAT, HOME_LON,
                         AL_ENTRY_STRAIGHT_IN, &cfg);
    auto s = sampleAtHome(5000.0f);
    autolandUpdate(1 * S + 600 * MS, &s);
    autolandTransition(AL_FLARE, 2 * S);

    // Still airborne per baro (100cm > 30cm), but impact spike hits.
    auto impact = sampleAtHome(100.0f);
    impact.accelMagG = 4.5f;            // way above 3.0g threshold
    autolandUpdate(3 * S, &impact);
    EXPECT_EQ(autolandGetPhase(), AL_TOUCHDOWN);
}

TEST_F(AutolandTest, TouchdownViaQuiescenceFromFlare)
{
    enterStraightIn(1 * S);
    auto s = sampleAtHome(5000.0f);
    autolandUpdate(1 * S + 600 * MS, &s);
    autolandTransition(AL_FLARE, 2 * S);

    // Push quiet samples at 100ms intervals until the short quiescence
    // window (500ms) elapses.
    auto quiet = sampleAtQuiet(100.0f);
    for (int i = 0; i < 7; i++) {
        autolandUpdate(2 * S + (timeUs_t)i * 100 * MS, &quiet);
    }
    EXPECT_EQ(autolandGetPhase(), AL_TOUCHDOWN);
}

TEST_F(AutolandTest, DisarmFiresAfterTouchdownQuiescenceWindow)
{
    auto cfg = makeDefaultCfg();
    cfg.touchdown_quiescence_ms = 1000;
    autolandRequestEntry(1 * S, LAUNCH_BARO_CM, HOME_LAT, HOME_LON,
                         AL_ENTRY_STRAIGHT_IN, &cfg);
    EXPECT_FALSE(autolandShouldDisarm());

    autolandTransition(AL_TOUCHDOWN, 2 * S);
    auto quiet = sampleAtQuiet(20.0f);

    // Not enough quiet yet -- disarm must not fire.
    autolandUpdate(2 * S + 500 * MS, &quiet);
    EXPECT_EQ(autolandGetPhase(), AL_TOUCHDOWN);
    EXPECT_FALSE(autolandShouldDisarm());

    // After 1000ms continuous quiet, disarm fires and we complete.
    autolandUpdate(2 * S + 1600 * MS, &quiet);
    EXPECT_EQ(autolandGetPhase(), AL_COMPLETE);
    EXPECT_TRUE(autolandShouldDisarm());
}

TEST_F(AutolandTest, MotionInterruptsDisarmQuiescence)
{
    auto cfg = makeDefaultCfg();
    cfg.touchdown_quiescence_ms = 1000;
    autolandRequestEntry(1 * S, LAUNCH_BARO_CM, HOME_LAT, HOME_LON,
                         AL_ENTRY_STRAIGHT_IN, &cfg);
    autolandTransition(AL_TOUCHDOWN, 2 * S);

    // Half a quiet window, then a bouncy sample, then more quiet.
    // Should not disarm -- the motion resets the quiet timer.
    auto quiet = sampleAtQuiet(20.0f);
    auto bumpy = sampleAtQuiet(20.0f);
    bumpy.accelMagG = 2.0f;    // above the quiet accel threshold

    autolandUpdate(2 * S + 500 * MS, &quiet);
    autolandUpdate(2 * S + 700 * MS, &bumpy);
    autolandUpdate(2 * S + 1500 * MS, &quiet);   // 800ms of quiet since bump

    EXPECT_EQ(autolandGetPhase(), AL_TOUCHDOWN);
    EXPECT_FALSE(autolandShouldDisarm());
}

TEST_F(AutolandTest, DisarmFlagClearsOnReinit)
{
    auto cfg = makeDefaultCfg();
    cfg.touchdown_quiescence_ms = 500;
    autolandRequestEntry(1 * S, LAUNCH_BARO_CM, HOME_LAT, HOME_LON,
                         AL_ENTRY_STRAIGHT_IN, &cfg);
    autolandTransition(AL_TOUCHDOWN, 2 * S);
    auto quiet = sampleAtQuiet(20.0f);
    // Two ticks -- first latches quietSinceUs, second accumulates the
    // 500ms window and fires the disarm.
    autolandUpdate(2 * S + 100 * MS, &quiet);
    autolandUpdate(2 * S + 700 * MS, &quiet);
    ASSERT_TRUE(autolandShouldDisarm());

    autolandInit();
    EXPECT_FALSE(autolandShouldDisarm());
    EXPECT_FALSE(autolandIsCommitLatched());
}
