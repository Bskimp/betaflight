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
#include "pg/gps_rescue_wing.h"

#if defined(USE_WING) && defined(USE_GPS_RESCUE)

// MSP2_WING_GPS_RESCUE: serialize all 16 wing GPS rescue config fields.
// Wire format: 22 bytes, little-endian, all unsigned.
//
// Byte layout:
//   [0]      uint8   allowArmingWithoutFix
//   [1]      uint8   minSats
//   [2]      uint8   maxBankAngle
//   [3..4]   uint16  orbitRadiusM
//   [5..6]   uint16  returnAltitudeM
//   [7..8]   uint16  minLoiterAltM
//   [9]      uint8   cruiseThrottle
//   [10]     uint8   minThrottle
//   [11]     uint8   abortThrottle
//   [12]     uint8   navP
//   [13]     uint8   altP
//   [14]     uint8   turnCompensation
//   [15..16] uint16  minHeadingSpeedCmS
//   [17..18] uint16  stallSpeedCmS
//   [19..20] uint16  minStartDistM
//   [21]     uint8   sanityChecks
void serializeWingGpsRescue(sbuf_t *dst, const gpsRescueConfig_t *cfg);

// MSP2_SET_WING_GPS_RESCUE: deserialize into cfg. Returns true on
// success, false if the payload is shorter than 22 bytes (cfg left
// untouched). Does NOT write EEPROM; caller handles persistence via
// MSP_EEPROM_WRITE.
bool deserializeWingGpsRescue(sbuf_t *src, gpsRescueConfig_t *cfg);

#endif // USE_WING && USE_GPS_RESCUE
