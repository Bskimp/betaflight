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
#include "flight/pid.h"

#ifdef USE_WING_LAUNCH

// MSP2_WING_LAUNCH: serialize 10 wing auto-launch config fields from
// the current PID profile. Wire format: 15 bytes, little-endian.
//
// Byte layout:
//   [0]    uint8   wing_launch_accel_thresh
//   [1..2] uint16  wing_launch_motor_delay
//   [3..4] uint16  wing_launch_motor_ramp
//   [5]    uint8   wing_launch_throttle
//   [6..7] uint16  wing_launch_climb_time
//   [8..9] int16   wing_launch_climb_angle
//   [10..11] uint16 wing_launch_transition
//   [12]   uint8   wing_launch_max_tilt
//   [13]   uint8   wing_launch_idle_thr
//   [14]   uint8   wing_launch_stick_override
void serializeWingLaunch(sbuf_t *dst, const pidProfile_t *profile);

// MSP2_SET_WING_LAUNCH: deserialize into profile. Returns true on
// success, false if the payload is shorter than 15 bytes (profile
// left untouched). Does NOT write EEPROM; caller handles persistence
// via MSP_EEPROM_WRITE.
bool deserializeWingLaunch(sbuf_t *src, pidProfile_t *profile);

#endif // USE_WING_LAUNCH
