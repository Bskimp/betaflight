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

// Wing GPS rescue MSP2 message serialization.
//
// Fields live in gpsRescueConfig_t (pg/gps_rescue_wing.h). Wire is
// fixed-size 22 bytes, all unsigned. Parameter group is the wing
// variant only — multirotor builds don't include this file.

#include "platform.h"

#if defined(USE_WING) && defined(USE_GPS_RESCUE)

#include "msp/msp_wing_gps_rescue.h"

void serializeWingGpsRescue(sbuf_t *dst, const gpsRescueConfig_t *cfg)
{
    sbufWriteU8(dst,  cfg->allowArmingWithoutFix);
    sbufWriteU8(dst,  cfg->minSats);
    sbufWriteU8(dst,  cfg->maxBankAngle);
    sbufWriteU16(dst, cfg->orbitRadiusM);
    sbufWriteU16(dst, cfg->returnAltitudeM);
    sbufWriteU16(dst, cfg->minLoiterAltM);
    sbufWriteU8(dst,  cfg->cruiseThrottle);
    sbufWriteU8(dst,  cfg->minThrottle);
    sbufWriteU8(dst,  cfg->abortThrottle);
    sbufWriteU8(dst,  cfg->navP);
    sbufWriteU8(dst,  cfg->altP);
    sbufWriteU8(dst,  cfg->turnCompensation);
    sbufWriteU16(dst, cfg->minHeadingSpeedCmS);
    sbufWriteU16(dst, cfg->stallSpeedCmS);
    sbufWriteU16(dst, cfg->minStartDistM);
    sbufWriteU8(dst,  cfg->sanityChecks);
}

bool deserializeWingGpsRescue(sbuf_t *src, gpsRescueConfig_t *cfg)
{
    if (sbufBytesRemaining(src) < 22) {
        return false;
    }
    cfg->allowArmingWithoutFix = sbufReadU8(src);
    cfg->minSats               = sbufReadU8(src);
    cfg->maxBankAngle          = sbufReadU8(src);
    cfg->orbitRadiusM          = sbufReadU16(src);
    cfg->returnAltitudeM       = sbufReadU16(src);
    cfg->minLoiterAltM         = sbufReadU16(src);
    cfg->cruiseThrottle        = sbufReadU8(src);
    cfg->minThrottle           = sbufReadU8(src);
    cfg->abortThrottle         = sbufReadU8(src);
    cfg->navP                  = sbufReadU8(src);
    cfg->altP                  = sbufReadU8(src);
    cfg->turnCompensation      = sbufReadU8(src);
    cfg->minHeadingSpeedCmS    = sbufReadU16(src);
    cfg->stallSpeedCmS         = sbufReadU16(src);
    cfg->minStartDistM         = sbufReadU16(src);
    cfg->sanityChecks          = sbufReadU8(src);
    return true;
}

#endif // USE_WING && USE_GPS_RESCUE
