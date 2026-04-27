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

// Wing autoland MSP2 message serialization.
//
// Fields live in wingAutolandConfig_t (pg/autoland.h). Wire is fixed-size
// 31 bytes, all unsigned. Wing-fork only.

#include "platform.h"

#ifdef USE_WING

#include "msp/msp_wing_autoland.h"

void serializeWingAutoland(sbuf_t *dst, const wingAutolandConfig_t *cfg)
{
    sbufWriteU8(dst,  cfg->enabled);
    sbufWriteU8(dst,  cfg->trigger_manual);
    sbufWriteU8(dst,  cfg->trigger_rth_timeout);
    sbufWriteU8(dst,  cfg->trigger_low_batt);
    sbufWriteU8(dst,  cfg->trigger_failsafe);
    sbufWriteU16(dst, cfg->loiter_timeout_s);
    sbufWriteU8(dst,  cfg->orbits_before_descent);
    sbufWriteU16(dst, cfg->approach_altitude_m);
    sbufWriteU16(dst, cfg->downwind_distance_m);
    sbufWriteU16(dst, cfg->base_radius_m);
    sbufWriteU16(dst, cfg->final_distance_m);
    sbufWriteU16(dst, cfg->commit_altitude_cm);
    sbufWriteU8(dst,  cfg->glide_pitch_deg);
    sbufWriteU16(dst, cfg->throttle_cut_alt_cm);
    sbufWriteU8(dst,  cfg->cruise_throttle_pct);
    sbufWriteU16(dst, cfg->flare_start_alt_cm);
    sbufWriteU8(dst,  cfg->flare_pitch_deg);
    sbufWriteU8(dst,  cfg->touchdown_accel_threshold);
    sbufWriteU16(dst, cfg->touchdown_alt_threshold_cm);
    sbufWriteU16(dst, cfg->touchdown_quiescence_ms);
    sbufWriteU8(dst,  cfg->min_pattern_sats);
}

bool deserializeWingAutoland(sbuf_t *src, wingAutolandConfig_t *cfg)
{
    if (sbufBytesRemaining(src) < 31) {
        return false;
    }
    cfg->enabled                    = sbufReadU8(src);
    cfg->trigger_manual             = sbufReadU8(src);
    cfg->trigger_rth_timeout        = sbufReadU8(src);
    cfg->trigger_low_batt           = sbufReadU8(src);
    cfg->trigger_failsafe           = sbufReadU8(src);
    cfg->loiter_timeout_s           = sbufReadU16(src);
    cfg->orbits_before_descent      = sbufReadU8(src);
    cfg->approach_altitude_m        = sbufReadU16(src);
    cfg->downwind_distance_m        = sbufReadU16(src);
    cfg->base_radius_m              = sbufReadU16(src);
    cfg->final_distance_m           = sbufReadU16(src);
    cfg->commit_altitude_cm         = sbufReadU16(src);
    cfg->glide_pitch_deg            = sbufReadU8(src);
    cfg->throttle_cut_alt_cm        = sbufReadU16(src);
    cfg->cruise_throttle_pct        = sbufReadU8(src);
    cfg->flare_start_alt_cm         = sbufReadU16(src);
    cfg->flare_pitch_deg            = sbufReadU8(src);
    cfg->touchdown_accel_threshold  = sbufReadU8(src);
    cfg->touchdown_alt_threshold_cm = sbufReadU16(src);
    cfg->touchdown_quiescence_ms    = sbufReadU16(src);
    cfg->min_pattern_sats           = sbufReadU8(src);
    return true;
}

#endif // USE_WING
