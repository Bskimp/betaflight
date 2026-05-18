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

#ifdef USE_WING

// MSP2_WING_TUNING: serialize 26 wing config fields from profile.
// Wire format: see .plan/WIRE_FORMAT.md (39 bytes, little-endian).
void serializeWingTuning(sbuf_t *dst, const pidProfile_t *profile);

// MSP2_SET_WING_TUNING: deserialize into profile. Uses sbufBytesRemaining()
// guards; returns true if all fields present, false if short. Does NOT
// write EEPROM -- caller handles persistence via MSP_EEPROM_WRITE.
bool deserializeWingTuning(sbuf_t *src, pidProfile_t *profile);

// MSP2_GET_WING_CAPABILITIES: return a u16 bitfield indicating which
// wing-fork-specific features this firmware build supports. The
// configurator uses this to gate sub-tabs and yaw_type options that
// would no-op on mainline (post-betaflight#13719) builds, which have
// USE_WING + MSP2_WING_TUNING only.
//
// Bit layout (append-only — older configurators ignore unknown high bits):
//   bit 0: WING_TUNING       — MSP2_WING_TUNING fields (s_*, SPA, TPA)
//   bit 1: WING_LAUNCH       — MSP2_WING_LAUNCH (auto-launch)
//   bit 2: WING_GPS_RESCUE   — MSP2_WING_GPS_RESCUE
//   bit 3: WING_AUTOLAND     — MSP2_WING_AUTOLAND
//   bit 4: COMBINED_YAW      — yaw_type=COMBINED enum support
uint16_t getWingCapabilitiesBitfield(void);

#endif // USE_WING
