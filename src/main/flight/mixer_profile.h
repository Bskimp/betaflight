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

#ifdef USE_VTOL

#include "pg/pg.h"

#include "drivers/motor.h"
#include "flight/mixer.h"
#include "flight/servos.h"

#define MAX_MIXER_PROFILE_COUNT 2

typedef enum {
    PLATFORM_MULTIROTOR = 0,
    PLATFORM_AIRPLANE,
    PLATFORM_TYPE_COUNT
} platformType_e;

typedef enum {
    FAILSAFE_MIXER_SWITCH_IMMEDIATE = 0,
    FAILSAFE_MIXER_HOLD_CURRENT,
    FAILSAFE_MIXER_ACTION_COUNT
} failsafeMixerAction_e;

typedef enum {
    MIXER_TRANSITION_IDLE = 0,
    MIXER_TRANSITION_BLENDING,
} mixerTransitionState_e;

typedef struct mixerProfile_s {
    uint8_t mixerMode;              // mixerMode_e
    uint8_t platformType;           // platformType_e
    motorMixer_t motorMix[MAX_SUPPORTED_MOTORS];
    uint8_t motorTransitionOnly[MAX_SUPPORTED_MOTORS]; // 1 = motor only active when BOXMIXERTRANSITION is on
    servoMixer_t servoMix[MAX_SERVO_RULES];
} mixerProfile_t;

PG_DECLARE_ARRAY(mixerProfile_t, MAX_MIXER_PROFILE_COUNT, mixerProfiles);

void mixerProfileInit(void);
bool mixerProfileSelect(uint8_t index);
uint8_t mixerProfileGetActiveIndex(void);
void mixerProfileGetSuperset(uint8_t *maxMotors, uint8_t *maxServos);

// transition state machine (PR2)
void mixerProfileUpdateFromRcModes(void);
void mixerProfileTransitionUpdate(uint32_t currentTimeMs);
bool mixerProfileTransitionInProgress(void);
float mixerProfileTransitionProgress(void);
void mixerProfileOnFailsafe(void);

// transition blending (PR3) — get FROM and TO motor/servo mixes for dual evaluation
void mixerProfileGetTransitionMixes(const motorMixer_t **fromMix, uint8_t *fromCount,
                                     const motorMixer_t **toMix, uint8_t *toCount);
void mixerProfileGetTransitionServoMixes(const servoMixer_t **fromRules, uint8_t *fromCount,
                                          const servoMixer_t **toRules, uint8_t *toCount);
const uint8_t *mixerProfileGetTransitionOnlyFlags(uint8_t profileIndex);

#endif // USE_VTOL
