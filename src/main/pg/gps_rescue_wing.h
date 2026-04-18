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

typedef struct gpsRescue_s {
    uint8_t  allowArmingWithoutFix;
    uint8_t  minSats;
    uint8_t  maxBankAngle;          // degrees, max roll during rescue turns
    uint16_t orbitRadiusM;          // meters, orbit radius at home
    uint16_t returnAltitudeM;       // meters, cruise altitude for rescue
    uint16_t minLoiterAltM;         // meters, minimum safe orbit altitude
    uint8_t  cruiseThrottle;        // percent, cruise throttle
    uint8_t  minThrottle;           // percent, stall prevention floor
    uint8_t  abortThrottle;         // percent, abort/recovery throttle
    uint8_t  navP;                  // heading P gain
    uint8_t  altP;                  // altitude P gain
    uint8_t  turnCompensation;      // percent, nose-up pitch compensation in banked turns (0-100)
    uint16_t minHeadingSpeedCmS;    // cm/s, min groundspeed for valid heading
    uint16_t stallSpeedCmS;         // cm/s, groundspeed below which stall is declared (during cruise/orbit only)
    uint16_t minStartDistM;         // meters, min distance to attempt rescue
    uint8_t  sanityChecks;          // rescue sanity check mode
} gpsRescueConfig_t;

PG_DECLARE(gpsRescueConfig_t, gpsRescueConfig);

#endif // USE_WING
