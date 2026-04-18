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

#ifdef USE_GPS_RESCUE

#include "flight/gps_rescue.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "gps_rescue.h"

PG_REGISTER_WITH_RESET_TEMPLATE(gpsRescueConfig_t, gpsRescueConfig, PG_GPS_RESCUE, 10);

PG_RESET_TEMPLATE(gpsRescueConfig_t, gpsRescueConfig,
    .allowArmingWithoutFix = false,
    .minSats = 8,
    .maxBankAngle = 25,
    .orbitRadiusM = 50,
    .returnAltitudeM = 50,
    .minLoiterAltM = 25,
    .cruiseThrottle = 50,
    .minThrottle = 30,
    .abortThrottle = 45,
    .navP = 30,
    .altP = 30,
    .turnCompensation = 50,
    .minHeadingSpeedCmS = 400,
    .stallSpeedCmS = 200,
    .minStartDistM = 30,
    .sanityChecks = 1,
);

#endif // USE_GPS_RESCUE

#endif // USE_WING
