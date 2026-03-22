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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_VTOL

#include "config/feature.h"

#include "fc/runtime_config.h"

#include "common/filter.h"

#include "flight/mixer.h"
#include "flight/mixer_init.h"
#include "flight/mixer_profile.h"
#include "flight/servos.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

PG_REGISTER_ARRAY_WITH_RESET_FN(mixerProfile_t, MAX_MIXER_PROFILE_COUNT, mixerProfiles, PG_MIXER_PROFILE, 0);

void pgResetFn_mixerProfiles(mixerProfile_t *profiles)
{
    memset(profiles, 0, sizeof(mixerProfile_t) * MAX_MIXER_PROFILE_COUNT);

    // profile 0 defaults to current mixer mode, multirotor platform
    profiles[0].mixerMode = MIXER_CUSTOM;
    profiles[0].platformType = PLATFORM_MULTIROTOR;

    // profile 1 defaults to airplane
    profiles[1].mixerMode = MIXER_CUSTOM_AIRPLANE;
    profiles[1].platformType = PLATFORM_AIRPLANE;
}

static uint8_t activeMixerProfileIndex = 0;

void mixerProfileInit(void)
{
    activeMixerProfileIndex = 0;
}

uint8_t mixerProfileGetActiveIndex(void)
{
    return activeMixerProfileIndex;
}

// Apply motor mix from the specified profile into mixerRuntime
static void mixerProfileApplyMotorMix(uint8_t index)
{
    const mixerProfile_t *profile = mixerProfiles(index);

    // re-init mixer with the profile's mixer mode (sets currentMixerMode internally)
    mixerInit(profile->mixerMode);

    // override with custom motor mix from profile
    mixerRuntime.motorCount = 0;
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        if (profile->motorMix[i].throttle == 0.0f) {
            break;
        }
        mixerRuntime.currentMixer[i] = profile->motorMix[i];
        mixerRuntime.motorCount++;
    }
}

// Apply servo mix from the specified profile into current servo mixer
static void mixerProfileApplyServoMix(uint8_t index)
{
    const mixerProfile_t *profile = mixerProfiles(index);
    loadServoMixerFromRules(profile->servoMix, MAX_SERVO_RULES);
}

bool mixerProfileSelect(uint8_t index)
{
    if (index >= MAX_MIXER_PROFILE_COUNT) {
        return false;
    }

    // PR1: disarmed switching only
    if (ARMING_FLAG(ARMED)) {
        return false;
    }

    if (index == activeMixerProfileIndex) {
        return true;
    }

    activeMixerProfileIndex = index;

    // reload motor and servo mixes from the new profile
    mixerProfileApplyMotorMix(index);
    mixerProfileApplyServoMix(index);

    // re-init ESC endpoints and profile-dependent mixer settings
    initEscEndpoints();
    mixerInitProfile();
    mixerResetDisarmedMotors();

    return true;
}

void mixerProfileGetSuperset(uint8_t *maxMotors, uint8_t *maxServos)
{
    uint8_t mMax = 0;
    uint8_t sMax = 0;

    for (int p = 0; p < MAX_MIXER_PROFILE_COUNT; p++) {
        const mixerProfile_t *profile = mixerProfiles(p);

        // count non-zero motor entries
        uint8_t mc = 0;
        for (int m = 0; m < MAX_SUPPORTED_MOTORS; m++) {
            if (profile->motorMix[m].throttle != 0.0f) {
                mc++;
            }
        }
        if (mc > mMax) {
            mMax = mc;
        }

        // count non-zero servo entries (by rate != 0)
        uint8_t sc = 0;
        for (int s = 0; s < MAX_SERVO_RULES; s++) {
            if (profile->servoMix[s].rate != 0) {
                sc++;
            }
        }
        if (sc > sMax) {
            sMax = sc;
        }
    }

    *maxMotors = mMax;
    *maxServos = sMax;
}

#endif // USE_VTOL
