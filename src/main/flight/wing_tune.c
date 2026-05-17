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

#include "platform.h"

#ifdef USE_WING

#include <stdint.h>
#include <string.h>

#include "build/debug.h"
#include "flight/wing_tune.h"

wingTuneData_t wingTuneData;

// Called from the extended DEBUG_SET macro on every DEBUG_SET site.
// Hot path: most calls dispatch to default and return immediately.
void wingTuneCapture(uint8_t mode, uint8_t index, int32_t value)
{
    switch (mode) {
    case DEBUG_SPA:
        if (index < XYZ_AXIS_COUNT) {
            wingTuneData.spa[index] = value;
        }
        break;
    case DEBUG_WING_SETPOINT:
        // Slots are interleaved: 2*axis = raw, 2*axis+1 = adjusted.
        if (index < XYZ_AXIS_COUNT * 2) {
            if ((index & 1) == 0) {
                wingTuneData.setpointRaw[index >> 1] = value;
            } else {
                wingTuneData.setpointAdj[index >> 1] = value;
            }
        }
        break;
    case DEBUG_S_TERM:
        // Same interleaving: 2*axis = raw, 2*axis+1 = post-TPA.
        if (index < XYZ_AXIS_COUNT * 2) {
            if ((index & 1) == 0) {
                wingTuneData.sTermRaw[index >> 1] = value;
            } else {
                wingTuneData.sTermPost[index >> 1] = value;
            }
        }
        break;
    case DEBUG_TPA:
        if (index < 6) {
            wingTuneData.tpa[index] = value;
        }
        break;
    case DEBUG_WING_LAUNCH:
        if (index < 8) {
            wingTuneData.launch[index] = value;
        }
        break;
    default:
        break;
    }
}

#endif // USE_WING
