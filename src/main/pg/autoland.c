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

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "autoland.h"

PG_REGISTER_WITH_RESET_TEMPLATE(wingAutolandConfig_t, wingAutolandConfig, PG_WING_AUTOLAND_CONFIG, 0);

PG_RESET_TEMPLATE(wingAutolandConfig_t, wingAutolandConfig,
    // Master + triggers -- default master OFF. Autoland never engages
    // until the pilot explicitly enables it in configurator.
    .enabled = 0,
    .trigger_manual = 1,
    .trigger_rth_timeout = 1,
    .trigger_low_batt = 0,
    .trigger_failsafe = 1,

    // Timing
    .loiter_timeout_s = 60,
    .orbits_before_descent = 2,

    // Pattern geometry (reasonable starting points; tune on bench/field)
    .approach_altitude_m = 60,
    .downwind_distance_m = 150,
    .base_radius_m = 40,
    .final_distance_m = 80,
    .commit_altitude_cm = 600,          // ~20 ft AGL

    // Glide / flare
    .glide_pitch_deg = 5,
    .throttle_cut_alt_cm = 0,           // default: cut at AL_ENTRY
    .cruise_throttle_pct = 40,
    .flare_start_alt_cm = 150,          // ~5 ft AGL
    .flare_pitch_deg = 8,

    // Touchdown
    .touchdown_accel_threshold = 30,    // 3.0 g spike
    .touchdown_alt_threshold_cm = 30,   // 1 ft AGL
    .touchdown_quiescence_ms = 2000,
    .min_pattern_sats = 8,
);

#endif // USE_WING
