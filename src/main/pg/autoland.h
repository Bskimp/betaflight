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

#pragma once

#ifdef USE_WING

#include <stdint.h>

#include "pg/pg.h"

typedef struct wingAutolandConfig_s {
    // Master + per-trigger gating. Default master OFF so autoland never
    // engages without explicit user opt-in. BOXAUTOLAND rc mode stays
    // *assignable* in modes tab even when master off -- it just no-ops.
    uint8_t  enabled;                       // 0 = off, 1 = on
    uint8_t  trigger_manual;                // BOXAUTOLAND switch
    uint8_t  trigger_rth_timeout;           // rescue orbit timeout -> autoland
    uint8_t  trigger_low_batt;              // low-battery auto-trigger
    uint8_t  trigger_failsafe;              // failsafe stage 3 auto-trigger

    // Timing
    uint16_t loiter_timeout_s;              // rescue orbit seconds before handoff
    uint8_t  orbits_before_descent;         // 1-3, default 2

    // Approach pattern (wind-aware downwind -> base -> final)
    uint16_t approach_altitude_m;           // AL_ENTRY altitude (m above home)
    uint16_t downwind_distance_m;           // pattern downwind leg length
    uint16_t base_radius_m;                 // base-turn radius
    uint16_t final_distance_m;              // final-approach distance from home
    uint16_t commit_altitude_cm;            // below this: latched, no abort

    // Glide / flare
    uint8_t  glide_pitch_deg;               // target glide pitch angle
    uint16_t throttle_cut_alt_cm;           // 0 = cut at AL_ENTRY, >0 = staged cut
    uint8_t  cruise_throttle_pct;           // % throttle held if cut_alt > 0
    uint16_t flare_start_alt_cm;            // AGL altitude to start flare
    uint8_t  flare_pitch_deg;               // flare pitch angle

    // Touchdown / post-land
    uint8_t  touchdown_accel_threshold;     // g-force spike threshold (0.1g units)
    uint16_t touchdown_alt_threshold_cm;    // baro AGL below = touchdown candidate
    uint16_t touchdown_quiescence_ms;       // ms of no motion to auto-disarm
    uint8_t  min_pattern_sats;              // min GPS sats during pattern
} wingAutolandConfig_t;

PG_DECLARE(wingAutolandConfig_t, wingAutolandConfig);

#endif // USE_WING
