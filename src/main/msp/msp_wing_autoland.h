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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/streambuf.h"
#include "pg/autoland.h"

#ifdef USE_WING

// MSP2_WING_AUTOLAND: serialize all 22 wing autoland config fields.
// Wire format: 31 bytes, little-endian, all unsigned.
//
// Byte layout:
//   [0]      uint8   enabled
//   [1]      uint8   trigger_manual
//   [2]      uint8   trigger_rth_timeout
//   [3]      uint8   trigger_low_batt
//   [4]      uint8   trigger_failsafe
//   [5..6]   uint16  loiter_timeout_s
//   [7]      uint8   orbits_before_descent
//   [8..9]   uint16  approach_altitude_m
//   [10..11] uint16  downwind_distance_m
//   [12..13] uint16  base_radius_m
//   [14..15] uint16  final_distance_m
//   [16..17] uint16  commit_altitude_cm
//   [18]     uint8   glide_pitch_deg
//   [19..20] uint16  throttle_cut_alt_cm
//   [21]     uint8   cruise_throttle_pct
//   [22..23] uint16  flare_start_alt_cm
//   [24]     uint8   flare_pitch_deg
//   [25]     uint8   touchdown_accel_threshold
//   [26..27] uint16  touchdown_alt_threshold_cm
//   [28..29] uint16  touchdown_quiescence_ms
//   [30]     uint8   min_pattern_sats
void serializeWingAutoland(sbuf_t *dst, const wingAutolandConfig_t *cfg);

// MSP2_SET_WING_AUTOLAND: deserialize into cfg. Returns true on success,
// false if the payload is shorter than 31 bytes (cfg left untouched).
// Does NOT write EEPROM; caller handles persistence via MSP_EEPROM_WRITE.
bool deserializeWingAutoland(sbuf_t *src, wingAutolandConfig_t *cfg);

#endif // USE_WING
