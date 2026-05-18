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

#include "platform.h"

#ifdef USE_WING

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"
#include "drivers/time.h"
#include "fc/runtime_config.h"
#include "flight/mixer.h"
#include "flight/servo_override.h"
#include "flight/servos.h"

static struct {
    bool active;
    uint8_t servoIdx;
    uint16_t pwm;
    timeUs_t endTimeUs;
} state;

static void clearState(void)
{
    state.active = false;
    state.servoIdx = 0;
    state.pwm = 0;
    state.endTimeUs = 0;
}

bool servoOverrideRequest(uint8_t servoIdx, uint16_t pwm, uint16_t durationMs)
{
    if (ARMING_FLAG(ARMED)) {
        return false;
    }
    if (servoIdx >= MAX_SUPPORTED_SERVOS) {
        return false;
    }
    if (pwm < SERVO_OVERRIDE_PWM_MIN || pwm > SERVO_OVERRIDE_PWM_MAX) {
        return false;
    }
    if (durationMs == 0 || durationMs > SERVO_OVERRIDE_MAX_DURATION_MS) {
        return false;
    }

    state.active = true;
    state.servoIdx = servoIdx;
    state.pwm = pwm;
    state.endTimeUs = micros() + (timeUs_t)durationMs * 1000;
    return true;
}

void servoOverrideUpdate(void)
{
    if (!state.active) {
        return;
    }
    // Continuous armed-state force-clear: closes the start-disarmed-
    // then-arm race that handler-time refusal alone misses.
    if (ARMING_FLAG(ARMED)) {
        clearState();
        return;
    }
    if ((int32_t)(micros() - state.endTimeUs) >= 0) {
        clearState();
    }
}

bool servoOverrideIsActive(uint8_t servoIdx)
{
    return state.active && state.servoIdx == servoIdx;
}

uint16_t servoOverrideGetPwm(void)
{
    return state.pwm;
}

#endif // USE_WING
