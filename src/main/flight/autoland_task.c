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

#include "platform.h"

#ifdef USE_WING

#include <math.h>

#include "common/maths.h"

#include "fc/core.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/autoland.h"
#include "flight/autoland_task.h"
#include "flight/imu.h"
#include "flight/position.h"

#include "io/gps.h"

#include "pg/autoland.h"
#include "pg/gps_rescue_wing.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

// ---- BOXAUTOLAND edge-detect state ----

// Tracks previous tick's BOX state so we can fire on rising/falling
// edges only. Reset to current state on arm so a pilot who armed with
// the switch already ON doesn't get autoland triggered immediately --
// requires an actual toggle post-arm.
static bool boxAutolandPrev;

// One-shot guard so we don't repeatedly call disarm() after the first
// AL_COMPLETE quiescence fires. Cleared by autolandOnArm().
static bool disarmIssued;

// ---- Sensor sample glue (Option B: pure adapter, pass by value) ----

// Build the autoland sensor sample from BF globals. Tolerates partial
// data -- each field is flagged individually so the state machine can
// react to partial availability (e.g. baro ok but GPS lost).
static void buildSensorSample(autolandSensorSample_t *out)
{
    // Baro: getEstimatedAltitudeCm() fuses baro + GPS altitude and is
    // the same source RTH uses. Absolute value is arbitrary (zero at
    // boot); autoland only cares about differences from its own
    // latched reference.
    out->baroCm    = (float)getEstimatedAltitudeCm();
    out->baroValid = sensors(SENSOR_BARO);

    // GPS: convert from BF's int32 deg*1e7 / deg*10 to floats. On
    // targets compiled without USE_GPS (e.g. 512 KB F7 wings) autoland
    // is still built but stays inert -- gpsValid is permanently false
    // and the trigger path below refuses entry.
#ifdef USE_GPS
    out->gpsValid = STATE(GPS_FIX) && STATE(GPS_FIX_HOME);
    if (out->gpsValid) {
        out->latDeg          = (float)gpsSol.llh.lat * 1.0e-7f;
        out->lonDeg          = (float)gpsSol.llh.lon * 1.0e-7f;
        out->groundCourseRad = (float)gpsSol.groundCourse * (M_PIf / 1800.0f);
        out->groundSpeedCmS  = (float)gpsSol.groundSpeed;
    } else {
        out->latDeg          = 0.0f;
        out->lonDeg          = 0.0f;
        out->groundCourseRad = 0.0f;
        out->groundSpeedCmS  = 0.0f;
    }
#else
    out->gpsValid        = false;
    out->latDeg          = 0.0f;
    out->lonDeg          = 0.0f;
    out->groundCourseRad = 0.0f;
    out->groundSpeedCmS  = 0.0f;
#endif

    // IMU. acc.accADC is in units where 1g == acc.dev.acc_1G. Gyro
    // gyroADCf is in deg/s per-axis.
    out->imuValid = sensors(SENSOR_ACC) && sensors(SENSOR_GYRO);
    if (out->imuValid && acc.dev.acc_1G > 0) {
        const float inv1G = 1.0f / (float)acc.dev.acc_1G;
        const float ax = (float)acc.accADC.x * inv1G;
        const float ay = (float)acc.accADC.y * inv1G;
        const float az = (float)acc.accADC.z * inv1G;
        out->accelMagG = sqrtf(ax * ax + ay * ay + az * az);

        const float gx = gyro.gyroADCf[0];
        const float gy = gyro.gyroADCf[1];
        const float gz = gyro.gyroADCf[2];
        out->gyroRateDegS = sqrtf(gx * gx + gy * gy + gz * gz);
    } else {
        out->accelMagG    = 0.0f;
        out->gyroRateDegS = 0.0f;
    }
}

// Snapshot the PG-backed config into the sequence-config struct we pass
// into autolandRequestEntry. Pulls a few fields from gpsRescueConfig
// so the wing's rescue-side tuning (bank angle, nav P, orbit radius)
// applies automatically.
static void snapshotSequenceConfig(autolandSequenceConfig_t *out)
{
    const wingAutolandConfig_t *ac = wingAutolandConfig();
    out->glide_pitch_deg           = ac->glide_pitch_deg;
    out->throttle_cut_alt_cm       = ac->throttle_cut_alt_cm;
    out->cruise_throttle_pct       = ac->cruise_throttle_pct;
    out->commit_altitude_cm        = ac->commit_altitude_cm;
    out->flare_start_alt_cm        = ac->flare_start_alt_cm;
    out->flare_pitch_deg           = ac->flare_pitch_deg;
    out->loiter_timeout_s          = ac->loiter_timeout_s;
    out->orbits_before_descent     = ac->orbits_before_descent;
    out->downwind_distance_m       = ac->downwind_distance_m;
    out->base_radius_m             = ac->base_radius_m;
    out->final_distance_m          = ac->final_distance_m;
    out->touchdown_alt_threshold_cm = ac->touchdown_alt_threshold_cm;
    out->touchdown_accel_threshold = ac->touchdown_accel_threshold;
    out->touchdown_quiescence_ms   = ac->touchdown_quiescence_ms;

#ifdef USE_GPS_RESCUE
    // Reuse rescue tuning so pilots don't have a parallel tuning surface.
    const gpsRescueConfig_t *rc = gpsRescueConfig();
    out->max_bank_angle_deg = rc->maxBankAngle;
    out->nav_p_gain         = rc->navP;
    out->orbit_radius_m     = rc->orbitRadiusM;
#else
    out->max_bank_angle_deg = 25;
    out->nav_p_gain         = 30;
    out->orbit_radius_m     = 50;
#endif
}

// ---- Public API ----

void autolandOnArm(void)
{
    autolandInit();
    // Snapshot the BOX state so a pilot arming with the switch already
    // ON doesn't get autoland triggered immediately -- they have to
    // actually toggle it after arm.
    boxAutolandPrev = IS_RC_MODE_ACTIVE(BOXAUTOLAND);
    disarmIssued    = false;
}

void autolandTaskTick(timeUs_t currentTimeUs)
{
    autolandSensorSample_t sample;
    buildSensorSample(&sample);
    autolandUpdate(currentTimeUs, &sample);

    // ---- Manual trigger (BOXAUTOLAND edge-detect) ----

    const bool boxNow       = IS_RC_MODE_ACTIVE(BOXAUTOLAND);
    const bool risingEdge   = !boxAutolandPrev && boxNow;
    const bool fallingEdge  =  boxAutolandPrev && !boxNow;
    boxAutolandPrev = boxNow;

    const wingAutolandConfig_t *ac = wingAutolandConfig();
    const bool masterEnabled = (ac->enabled != 0);
    const bool manualAllowed = (ac->trigger_manual != 0);

#ifdef USE_GPS
    // Manual trigger requires a home-fix to have any meaning; without
    // GPS + a set home point the waypoint controller has nowhere to
    // steer to. Gate the entry-request on the home-fix flag.
    const bool homeKnown = STATE(GPS_FIX_HOME);
#else
    const bool homeKnown = false;
#endif

    if (risingEdge && masterEnabled && manualAllowed && homeKnown
        && autolandCanEnter()) {
        autolandSequenceConfig_t cfg;
        snapshotSequenceConfig(&cfg);
        const float initialBaroCm = (float)getEstimatedAltitudeCm();
#ifdef USE_GPS
        const float homeLatDeg    = (float)GPS_home_llh.lat * 1.0e-7f;
        const float homeLonDeg    = (float)GPS_home_llh.lon * 1.0e-7f;
#else
        const float homeLatDeg    = 0.0f;
        const float homeLonDeg    = 0.0f;
#endif
        autolandRequestEntry(currentTimeUs, initialBaroCm,
                             homeLatDeg, homeLonDeg,
                             AL_ENTRY_STRAIGHT_IN, &cfg);
    } else if (fallingEdge && autolandIsActive()) {
        // Pilot flipped the switch off -- request abort. Refused if
        // commit-latched (invariant #5). autolandAbort() handles that.
        autolandAbort(AL_ABORT_PILOT, currentTimeUs);
    }

    // ---- Disarm on landing ----

    if (!disarmIssued && autolandShouldDisarm()) {
        disarm(DISARM_REASON_LANDING);
        disarmIssued = true;
    }
}

#endif // USE_WING
