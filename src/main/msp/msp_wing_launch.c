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

// Wing auto-launch MSP2 message serialization.
//
// Fields live in pidProfile_t under #ifdef USE_WING_LAUNCH (pid.h
// lines 326-335). Wire is fixed-size 15 bytes; signed field is
// wing_launch_climb_angle (int16, -N..+N degrees). Int16 written
// via sbufWriteU16 cast to match the int-read / uint-write
// convention from MSP_SET_PID_ADVANCED — streambuf only exposes
// U8/U16/U32 writers.

#include "platform.h"

#ifdef USE_WING_LAUNCH

#include "msp/msp_wing_launch.h"

void serializeWingLaunch(sbuf_t *dst, const pidProfile_t *profile)
{
    sbufWriteU8(dst,  profile->wing_launch_accel_thresh);
    sbufWriteU16(dst, profile->wing_launch_motor_delay);
    sbufWriteU16(dst, profile->wing_launch_motor_ramp);
    sbufWriteU8(dst,  profile->wing_launch_throttle);
    sbufWriteU16(dst, profile->wing_launch_climb_time);
    sbufWriteU16(dst, (uint16_t)profile->wing_launch_climb_angle);
    sbufWriteU16(dst, profile->wing_launch_transition);
    sbufWriteU8(dst,  profile->wing_launch_max_tilt);
    sbufWriteU8(dst,  profile->wing_launch_idle_thr);
    sbufWriteU8(dst,  profile->wing_launch_stick_override);
}

bool deserializeWingLaunch(sbuf_t *src, pidProfile_t *profile)
{
    if (sbufBytesRemaining(src) < 15) {
        return false;
    }
    profile->wing_launch_accel_thresh   = sbufReadU8(src);
    profile->wing_launch_motor_delay    = sbufReadU16(src);
    profile->wing_launch_motor_ramp     = sbufReadU16(src);
    profile->wing_launch_throttle       = sbufReadU8(src);
    profile->wing_launch_climb_time     = sbufReadU16(src);
    profile->wing_launch_climb_angle    = (int16_t)sbufReadU16(src);
    profile->wing_launch_transition     = sbufReadU16(src);
    profile->wing_launch_max_tilt       = sbufReadU8(src);
    profile->wing_launch_idle_thr       = sbufReadU8(src);
    profile->wing_launch_stick_override = sbufReadU8(src);
    return true;
}

#endif // USE_WING_LAUNCH
