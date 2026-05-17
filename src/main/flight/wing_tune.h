/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "platform.h"

#ifdef USE_WING

#include <stdint.h>
#include "common/axis.h"

// Always-on capture buffer for wing-tuning blackbox fields.
//
// Stock DEBUG_SET only writes to debug[] when debugMode matches the mode arg,
// so only one debug mode's data ever reaches the log. This buffer mirrors
// the same data across all wing-relevant debug modes simultaneously, so the
// blackbox can log any combination of them in a single flight.
//
// Each blackbox field group (SPA, WING_SETPOINT, S_TERM, TPA, WING_LAUNCH)
// has an independent toggle in the Configurator's "Debug Fields included"
// panel via the existing fields_disabled_mask bitmask.
typedef struct wingTuneData_s {
    int32_t spa[XYZ_AXIS_COUNT];          // DEBUG_SPA: pidRuntime.spa[axis] * 1000
    int32_t setpointRaw[XYZ_AXIS_COUNT];  // DEBUG_WING_SETPOINT slots 0,2,4
    int32_t setpointAdj[XYZ_AXIS_COUNT];  // DEBUG_WING_SETPOINT slots 1,3,5
    int32_t sTermRaw[XYZ_AXIS_COUNT];     // DEBUG_S_TERM slots 0,2,4
    int32_t sTermPost[XYZ_AXIS_COUNT];    // DEBUG_S_TERM slots 1,3,5
    int32_t tpa[6];                       // DEBUG_TPA: factor, roll, pitch, throttle, airspeed*10, arg
    int32_t launch[8];                    // DEBUG_WING_LAUNCH: state, elapsed, throttle, accel, pitch, roll, ramp, climbRem
} wingTuneData_t;

extern wingTuneData_t wingTuneData;

void wingTuneCapture(uint8_t mode, uint8_t index, int32_t value);

#endif // USE_WING
