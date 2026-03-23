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
#include <math.h>

#include "platform.h"

#ifdef USE_VTOL

#include "build/debug.h"

#include "common/filter.h"
#include "common/maths.h"

#include "drivers/time.h"

#include "config/config.h"
#include "config/feature.h"

#include "fc/controlrate_profile.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/mixer.h"
#include "flight/mixer_init.h"
#include "flight/mixer_profile.h"
#include "flight/pid.h"
#include "flight/servos.h"

#include "common/sensor_alignment.h"
#include "sensors/boardalignment.h"

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

// ---- state ----

static uint8_t activeMixerProfileIndex = 0;

static struct {
    mixerTransitionState_e state;
    uint32_t startTimeMs;
    uint16_t durationMs;
    float progress;         // 0.0 .. 1.0
    uint8_t fromProfile;
    uint8_t toProfile;
} transition;

// ---- internal helpers ----

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

// Activate a profile immediately (no armed check — internal use only)
static void mixerProfileActivateInternal(uint8_t index)
{
    activeMixerProfileIndex = index;

    // reload motor and servo mixes from the new profile
    mixerProfileApplyMotorMix(index);
    mixerProfileApplyServoMix(index);

    // re-init ESC endpoints and profile-dependent mixer settings
    initEscEndpoints();
    mixerInitProfile();
    mixerResetDisarmedMotors();

    // apply per-profile IMU orientation rotation
    boardAlignmentSetVtolRotation(
        mixerConfig()->mixer_imu_orientation_roll[index],
        mixerConfig()->mixer_imu_orientation_pitch[index],
        mixerConfig()->mixer_imu_orientation_yaw[index]
    );

    // switch linked PID/rate profiles if configured
    const uint8_t linkedPid = mixerConfig()->mixer_linked_pid_profile[index];
    if (linkedPid > 0) {
        changePidProfile(linkedPid - 1);
    }
    const uint8_t linkedRate = mixerConfig()->mixer_linked_rate_profile[index];
    if (linkedRate > 0) {
        changeControlRateProfile(linkedRate - 1);
    }
}

// ---- public API ----

void mixerProfileInit(void)
{
    activeMixerProfileIndex = 0;
    transition.state = MIXER_TRANSITION_IDLE;
}

void mixerProfileApplyActive(void)
{
    mixerProfileActivateInternal(activeMixerProfileIndex);
}

uint8_t mixerProfileGetActiveIndex(void)
{
    return activeMixerProfileIndex;
}

// Disarmed-only profile switch (CLI, MSP)
bool mixerProfileSelect(uint8_t index)
{
    if (index >= MAX_MIXER_PROFILE_COUNT) {
        return false;
    }

    if (ARMING_FLAG(ARMED)) {
        return false;
    }

    if (index == activeMixerProfileIndex) {
        return true;
    }

    mixerProfileActivateInternal(index);

    return true;
}

// Start a blended transition to a new profile
static void mixerProfileTransitionStart(uint8_t toProfile)
{
    if (transition.state != MIXER_TRANSITION_IDLE) {
        return;
    }
    if (toProfile == activeMixerProfileIndex) {
        return;
    }
    if (toProfile >= MAX_MIXER_PROFILE_COUNT) {
        return;
    }

    transition.fromProfile = activeMixerProfileIndex;
    transition.toProfile = toProfile;
    transition.startTimeMs = millis();
    transition.durationMs = mixerConfig()->mixer_transition_time;
    transition.progress = 0.0f;
    transition.state = MIXER_TRANSITION_BLENDING;

    // switch PID/rate profiles immediately so the controller runs target gains during blend
    const uint8_t linkedPid = mixerConfig()->mixer_linked_pid_profile[toProfile];
    if (linkedPid > 0) {
        changePidProfile(linkedPid - 1);
    }
    const uint8_t linkedRate = mixerConfig()->mixer_linked_rate_profile[toProfile];
    if (linkedRate > 0) {
        changeControlRateProfile(linkedRate - 1);
    }
}

// Finalize transition — snap to target profile
static void mixerProfileTransitionFinalize(void)
{
    mixerProfileActivateInternal(transition.toProfile);
    pidResetIterm();
    transition.state = MIXER_TRANSITION_IDLE;
    transition.progress = 0.0f;
}

void mixerProfileTransitionUpdate(uint32_t currentTimeMs)
{
    if (transition.state != MIXER_TRANSITION_BLENDING) {
        return;
    }

    if (transition.durationMs == 0) {
        // instant switch
        mixerProfileTransitionFinalize();
        return;
    }

    const float elapsed = (float)(currentTimeMs - transition.startTimeMs);
    const float linear = constrainf(elapsed / (float)transition.durationMs, 0.0f, 1.0f);
    transition.progress = linear;

    DEBUG_SET(DEBUG_MIXER_PROFILE, 0, activeMixerProfileIndex);
    DEBUG_SET(DEBUG_MIXER_PROFILE, 1, transition.toProfile);
    DEBUG_SET(DEBUG_MIXER_PROFILE, 2, lrintf(transition.progress * 1000.0f));
    DEBUG_SET(DEBUG_MIXER_PROFILE, 3, transition.state);

    if (linear >= 1.0f) {
        mixerProfileTransitionFinalize();
    }
}

bool mixerProfileTransitionInProgress(void)
{
    return transition.state == MIXER_TRANSITION_BLENDING;
}

float mixerProfileTransitionProgress(void)
{
    return transition.progress;
}

// ---- AUX mode processing ----

void mixerProfileUpdateFromRcModes(void)
{
    if (mixerConfig()->mixer_profile_count < 2) {
        return;
    }

    // determine target profile from AUX switch state
    const bool boxActive = IS_RC_MODE_ACTIVE(BOXMIXERPROFILE);
    const uint8_t targetProfile = boxActive ? 1 : 0;

    if (targetProfile == activeMixerProfileIndex && transition.state == MIXER_TRANSITION_IDLE) {
        return;
    }

    // if transition already heading to this target, nothing to do
    if (transition.state == MIXER_TRANSITION_BLENDING && transition.toProfile == targetProfile) {
        return;
    }

    // block switching during certain flight modes
    if (FLIGHT_MODE(GPS_RESCUE_MODE) || FLIGHT_MODE(FAILSAFE_MODE)) {
        return;
    }

    if (ARMING_FLAG(ARMED)) {
        // armed: use blended transition
        mixerProfileTransitionStart(targetProfile);
    } else {
        // disarmed: immediate switch
        mixerProfileActivateInternal(targetProfile);
    }
}

// ---- failsafe integration ----

void mixerProfileOnFailsafe(void)
{
    if (mixerConfig()->mixer_profile_count < 2) {
        return;
    }

    const uint8_t targetProfile = mixerConfig()->failsafe_mixer_profile;
    const uint8_t action = mixerConfig()->failsafe_mixer_action;

    if (targetProfile == activeMixerProfileIndex && transition.state == MIXER_TRANSITION_IDLE) {
        return;
    }

    switch (action) {
    case FAILSAFE_MIXER_SWITCH_IMMEDIATE:
        // abort any in-progress transition and snap to target
        transition.state = MIXER_TRANSITION_IDLE;
        mixerProfileActivateInternal(targetProfile);
        pidResetIterm();
        break;

    case FAILSAFE_MIXER_HOLD_CURRENT:
    default:
        // stay on current profile, do nothing
        break;
    }
}

// ---- superset calculation ----

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

// ---- transition blending helpers (PR3) ----

static uint8_t countProfileMotors(const mixerProfile_t *profile)
{
    uint8_t count = 0;
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        if (profile->motorMix[i].throttle == 0.0f) {
            break;
        }
        count++;
    }
    return count;
}

static uint8_t countProfileServoRules(const mixerProfile_t *profile)
{
    uint8_t count = 0;
    for (int i = 0; i < MAX_SERVO_RULES; i++) {
        if (profile->servoMix[i].rate == 0) {
            break;
        }
        count++;
    }
    return count;
}

void mixerProfileGetTransitionMixes(const motorMixer_t **fromMix, uint8_t *fromCount,
                                     const motorMixer_t **toMix, uint8_t *toCount)
{
    const mixerProfile_t *from = mixerProfiles(transition.fromProfile);
    const mixerProfile_t *to = mixerProfiles(transition.toProfile);

    *fromMix = from->motorMix;
    *fromCount = countProfileMotors(from);
    *toMix = to->motorMix;
    *toCount = countProfileMotors(to);
}

void mixerProfileGetTransitionServoMixes(const servoMixer_t **fromRules, uint8_t *fromCount,
                                          const servoMixer_t **toRules, uint8_t *toCount)
{
    const mixerProfile_t *from = mixerProfiles(transition.fromProfile);
    const mixerProfile_t *to = mixerProfiles(transition.toProfile);

    *fromRules = from->servoMix;
    *fromCount = countProfileServoRules(from);
    *toRules = to->servoMix;
    *toCount = countProfileServoRules(to);
}

const uint8_t *mixerProfileGetTransitionOnlyFlags(uint8_t profileIndex)
{
    return mixerProfiles(profileIndex)->motorTransitionOnly;
}

#endif // USE_VTOL
