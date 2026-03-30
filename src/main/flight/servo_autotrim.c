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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#if defined(USE_SERVOS) && defined(USE_WING)

#include "build/debug.h"

#include "common/maths.h"

#include "config/config.h"

#include "drivers/pwm_output.h"
#include "drivers/time.h"

#include "fc/core.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/servos.h"
#include "flight/servo_autotrim.h"

#include "io/beeper.h"

#include "rx/rx.h"

#include "sensors/gyro.h"

#define AUTOTRIM_MAX_OFFSET       100   // max microseconds offset per servo
#define AUTOTRIM_MIN_SAMPLES      200   // minimum samples before applying (~2s at 100Hz servo rate)
#define AUTOTRIM_STICK_DEADBAND    50   // ~5% of +/-500 rcCommand range
#define AUTOTRIM_MAX_ATT_DECI     300   // max attitude for sampling (30 deg in decidegrees)
#define AUTOTRIM_MAX_GYRO_RATE     15.0f // max gyro rate for sampling (deg/s)

typedef struct {
    bool active;
    bool applied;
    bool pendingSave;
    bool eligible[MAX_SUPPORTED_SERVOS];
    int32_t accumulator[MAX_SUPPORTED_SERVOS];
    uint32_t sampleCount[MAX_SUPPORTED_SERVOS];
} servoAutoTrimState_t;

static servoAutoTrimState_t autoTrimState;

// result display timer — shows TRIM SAVED / TRIM FAIL for ~3 seconds
static uint32_t resultDisplayEndTimeMs;
#define AUTOTRIM_RESULT_DISPLAY_MS 3000

void servoAutoTrimInit(void)
{
    memset(&autoTrimState, 0, sizeof(autoTrimState));
    resultDisplayEndTimeMs = 0;
}

// Check if a servo has stabilization mix rules (roll/pitch/yaw PID sources)
static bool isServoEligibleForAutoTrim(int servoIndex)
{
    for (int i = 0; i < MAX_SERVO_RULES; i++) {
        const servoMixer_t *rule = customServoMixers(i);
        if (rule->rate == 0) {
            break; // empty rule — end of list
        }
        if (rule->targetChannel == servoIndex && rule->inputSource <= INPUT_STABILIZED_YAW) {
            return true;
        }
    }
    return false;
}

static bool canSampleAutoTrim(void)
{
    if (!ARMING_FLAG(ARMED)) {
        return false;
    }

    if (!wasThrottleRaised()) {
        return false;
    }

    // no PID correction in passthrough — servo output is raw stick
    if (FLIGHT_MODE(PASSTHRU_MODE)) {
        return false;
    }

    // sticks must be near center
    if (ABS(rcCommand[ROLL]) > AUTOTRIM_STICK_DEADBAND) {
        return false;
    }
    if (ABS(rcCommand[PITCH]) > AUTOTRIM_STICK_DEADBAND) {
        return false;
    }

    // must be roughly level
    if (ABS(attitude.values.roll) > AUTOTRIM_MAX_ATT_DECI) {
        return false;
    }
    if (ABS(attitude.values.pitch) > AUTOTRIM_MAX_ATT_DECI) {
        return false;
    }

    // gyro rates must be low — flying straight, not oscillating
    if (fabsf(gyro.gyroADCf[FD_ROLL]) > AUTOTRIM_MAX_GYRO_RATE) {
        return false;
    }
    if (fabsf(gyro.gyroADCf[FD_PITCH]) > AUTOTRIM_MAX_GYRO_RATE) {
        return false;
    }

    return true;
}

// find first eligible servo for debug output
static int firstEligibleServo(void)
{
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        if (autoTrimState.eligible[i]) {
            return i;
        }
    }
    return -1;
}

static void servoAutoTrimApply(void)
{
    bool anyApplied = false;

    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        if (autoTrimState.eligible[i] && autoTrimState.sampleCount[i] >= AUTOTRIM_MIN_SAMPLES) {
            int16_t offset = autoTrimState.accumulator[i] / (int32_t)autoTrimState.sampleCount[i];
            offset = constrain(offset, -AUTOTRIM_MAX_OFFSET, AUTOTRIM_MAX_OFFSET);

            if (ABS(offset) >= 2) {
                servoParamsMutable(i)->middle += offset;
                servoParamsMutable(i)->middle = constrain(
                    servoParamsMutable(i)->middle,
                    PWM_RANGE_MIN,
                    PWM_RANGE_MAX
                );
                anyApplied = true;
                DEBUG_SET(DEBUG_SERVO_AUTOTRIM, 3, offset);
            }
        }
    }

    if (anyApplied) {
        autoTrimState.applied = true;
        autoTrimState.pendingSave = true;
    }

    resultDisplayEndTimeMs = millis() + AUTOTRIM_RESULT_DISPLAY_MS;
}

void servoAutoTrimUpdate(void)
{
    if (!ARMING_FLAG(ARMED)) {
        return;
    }

    // mode deactivated — apply if we have data
    if (!IS_RC_MODE_ACTIVE(BOXAUTOTRIM)) {
        if (autoTrimState.active) {
            servoAutoTrimApply();
            autoTrimState.active = false;
        }
        return;
    }

    // mode just activated — reset accumulators and compute eligible set
    if (!autoTrimState.active) {
        memset(autoTrimState.accumulator, 0, sizeof(autoTrimState.accumulator));
        memset(autoTrimState.sampleCount, 0, sizeof(autoTrimState.sampleCount));
        autoTrimState.applied = false;
        autoTrimState.pendingSave = false;
        resultDisplayEndTimeMs = 0;

        for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            autoTrimState.eligible[i] = isServoEligibleForAutoTrim(i);
        }

        autoTrimState.active = true;
    }

    const bool sampling = canSampleAutoTrim();

    // debug[0]: state (0=off, 1=active not sampling, 2=sampling)
    DEBUG_SET(DEBUG_SERVO_AUTOTRIM, 0, sampling ? 2 : 1);

    if (!sampling) {
        return;
    }

    // accumulate servo offset from midpoint on eligible surfaces
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        if (autoTrimState.eligible[i]) {
            autoTrimState.accumulator[i] += servo[i] - servoParams(i)->middle;
            autoTrimState.sampleCount[i]++;
        }
    }

    // debug[1]: sample count, debug[2]: running average offset
    const int dbgServo = firstEligibleServo();
    if (dbgServo >= 0) {
        DEBUG_SET(DEBUG_SERVO_AUTOTRIM, 1, autoTrimState.sampleCount[dbgServo]);
        DEBUG_SET(DEBUG_SERVO_AUTOTRIM, 2,
            autoTrimState.accumulator[dbgServo] / (int32_t)autoTrimState.sampleCount[dbgServo]);
    }
}

void servoAutoTrimOnDisarm(void)
{
    if (autoTrimState.pendingSave) {
        writeEEPROM();
        beeperConfirmationBeeps(3);
        autoTrimState.pendingSave = false;
    }
}

bool servoAutoTrimIsActive(void)
{
    return autoTrimState.active;
}

bool servoAutoTrimWasApplied(void)
{
    return autoTrimState.applied;
}

bool servoAutoTrimResultPending(void)
{
    return (resultDisplayEndTimeMs != 0) && (millis() < resultDisplayEndTimeMs);
}

#endif // USE_SERVOS && USE_WING
