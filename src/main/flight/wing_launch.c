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
#include <math.h>

#include "platform.h"

#ifdef USE_WING_LAUNCH

#include "build/debug.h"

#include "drivers/time.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/time.h"

#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "fc/rc_controls.h"

#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/wing_launch.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"

static wingLaunchState_e launchState = WING_LAUNCH_IDLE;
static timeUs_t stateStartTimeUs = 0;
static bool wasArmed = false;
static float motorOutput = 0.0f;
static float transitionFactor = 0.0f;
static bool throttleGatePassed = false;

// config cache (populated from pidProfile in wingLaunchInit)
static uint16_t motorDelayMs = 100;
static uint16_t motorRampMs = 500;
static float launchThrottle = 0.75f;
static uint16_t climbTimeMs = 3000;
static float climbAngleDeg = 45.0f;
static uint16_t transitionMs = 1000;
static float maxTiltDeg = 45.0f;
static float idleThrottle = 0.0f;
static float accelThreshG = 0.0f;
static float stickOverrideThresh = 0.0f;

static void transitionToState(wingLaunchState_e newState, timeUs_t currentTimeUs)
{
    launchState = newState;
    stateStartTimeUs = currentTimeUs;
}

static timeDelta_t stateElapsedMs(timeUs_t currentTimeUs)
{
    return cmpTimeUs(currentTimeUs, stateStartTimeUs) / 1000;
}

void wingLaunchInit(const pidProfile_t *pidProfile)
{
    motorDelayMs = pidProfile->wing_launch_motor_delay;
    motorRampMs = pidProfile->wing_launch_motor_ramp;
    launchThrottle = pidProfile->wing_launch_throttle / 100.0f;
    climbTimeMs = pidProfile->wing_launch_climb_time;
    climbAngleDeg = (float)pidProfile->wing_launch_climb_angle;
    transitionMs = pidProfile->wing_launch_transition;
    maxTiltDeg = (float)pidProfile->wing_launch_max_tilt;
    idleThrottle = pidProfile->wing_launch_idle_thr / 100.0f;
    accelThreshG = pidProfile->wing_launch_accel_thresh / 10.0f;
    stickOverrideThresh = pidProfile->wing_launch_stick_override / 100.0f * 500.0f;

    launchState = WING_LAUNCH_IDLE;
    stateStartTimeUs = 0;
    wasArmed = false;
    motorOutput = idleThrottle;
    transitionFactor = 0.0f;
    throttleGatePassed = false;
}

static bool isStickOverrideActive(void)
{
    if (stickOverrideThresh <= 0.0f) {
        return false;
    }
    return ABS(rcCommand[ROLL]) > stickOverrideThresh
        || ABS(rcCommand[PITCH]) > stickOverrideThresh
        || ABS(rcCommand[YAW]) > stickOverrideThresh;
}

void wingLaunchUpdate(timeUs_t currentTimeUs)
{
    // abort if switch turned off or disarmed mid-launch
    if (!IS_RC_MODE_ACTIVE(BOXAUTOLAUNCH) || !ARMING_FLAG(ARMED)) {
        if (launchState != WING_LAUNCH_IDLE && launchState != WING_LAUNCH_COMPLETE) {
            motorOutput = 0.0f;
            pidResetIterm();
            DISABLE_FLIGHT_MODE(ANGLE_MODE);
            transitionToState(WING_LAUNCH_COMPLETE, currentTimeUs);
        }
        if (!ARMING_FLAG(ARMED)) {
            wingLaunchReset();
        }
        return;
    }

    const float rollAngleDeg = ABS(attitude.values.roll) / 10.0f;
    const float pitchAngleDeg = attitude.values.pitch / 10.0f;

    switch (launchState) {
    case WING_LAUNCH_IDLE:
        if (ARMING_FLAG(ARMED) && !wasArmed) {
            pidResetIterm();
            // always require throw detection — motors stay idle until accel spike
            transitionToState(WING_LAUNCH_DETECTED, currentTimeUs);
        }
        wasArmed = ARMING_FLAG(ARMED);
        break;

    case WING_LAUNCH_DETECTED:
        motorOutput = idleThrottle;
        pidResetIterm(); // prevent I-term windup while aircraft is held in hand
        // throttle gate: require pilot to raise throttle before throw detection
        if (!throttleGatePassed) {
            const float throttleGatePwm = PWM_RANGE_MIN + launchThrottle * PWM_RANGE;
            if (rcData[THROTTLE] >= throttleGatePwm) {
                throttleGatePassed = true;
            }
        }
        if (throttleGatePassed && acc.accMagnitude > accelThreshG) {
            transitionToState(WING_LAUNCH_MOTOR_DELAY, currentTimeUs);
        }
        // no tilt abort here — motors are at idle, pilot may hold plane at any angle
        // tilt safety activates in MOTOR_RAMP and CLIMBING when motors are actually live
        break;

    case WING_LAUNCH_MOTOR_DELAY:
        if (stateElapsedMs(currentTimeUs) >= motorDelayMs) {
            motorOutput = idleThrottle;
            transitionToState(WING_LAUNCH_MOTOR_RAMP, currentTimeUs);
        }
        break;

    case WING_LAUNCH_MOTOR_RAMP:
    {
        const timeDelta_t elapsedMs = stateElapsedMs(currentTimeUs);
        if (elapsedMs >= motorRampMs) {
            motorOutput = launchThrottle;
            transitionToState(WING_LAUNCH_CLIMBING, currentTimeUs);
        } else {
            const float rampProgress = (float)elapsedMs / (float)motorRampMs;
            motorOutput = idleThrottle + (launchThrottle - idleThrottle) * rampProgress;
        }
        // safety check during ramp
        if (rollAngleDeg > maxTiltDeg || pitchAngleDeg < -maxTiltDeg) {
            transitionToState(WING_LAUNCH_ABORT, currentTimeUs);
        }
        if (isStickOverrideActive()) {
            transitionToState(WING_LAUNCH_TRANSITION, currentTimeUs);
        }
        break;
    }

    case WING_LAUNCH_CLIMBING:
        motorOutput = launchThrottle;
        if (stateElapsedMs(currentTimeUs) >= climbTimeMs) {
            transitionToState(WING_LAUNCH_TRANSITION, currentTimeUs);
        }
        // safety: abort on excessive roll or dive
        if (rollAngleDeg > maxTiltDeg || pitchAngleDeg < -maxTiltDeg) {
            transitionToState(WING_LAUNCH_ABORT, currentTimeUs);
        }
        if (isStickOverrideActive()) {
            transitionToState(WING_LAUNCH_TRANSITION, currentTimeUs);
        }
        break;

    case WING_LAUNCH_TRANSITION:
    {
        const timeDelta_t elapsedMs = stateElapsedMs(currentTimeUs);
        if (elapsedMs >= transitionMs) {
            transitionFactor = 1.0f;
            pidResetIterm();
            DISABLE_FLIGHT_MODE(ANGLE_MODE);
            transitionToState(WING_LAUNCH_COMPLETE, currentTimeUs);
        } else {
            transitionFactor = (float)elapsedMs / (float)transitionMs;
            motorOutput = launchThrottle; // mixer blends toward pilot stick
        }
        break;
    }

    case WING_LAUNCH_ABORT:
        motorOutput = 0.0f;
        pidResetIterm();
        DISABLE_FLIGHT_MODE(ANGLE_MODE);
        transitionToState(WING_LAUNCH_COMPLETE, currentTimeUs);
        break;

    case WING_LAUNCH_COMPLETE:
    default:
        break;
    }

    DEBUG_SET(DEBUG_WING_LAUNCH, 0, launchState);
    DEBUG_SET(DEBUG_WING_LAUNCH, 1, stateElapsedMs(currentTimeUs));
    DEBUG_SET(DEBUG_WING_LAUNCH, 2, lrintf(motorOutput * 1000));
    DEBUG_SET(DEBUG_WING_LAUNCH, 3, lrintf(acc.accMagnitude * 100));
    DEBUG_SET(DEBUG_WING_LAUNCH, 4, lrintf(attitude.values.pitch));
    DEBUG_SET(DEBUG_WING_LAUNCH, 5, lrintf(attitude.values.roll));
    DEBUG_SET(DEBUG_WING_LAUNCH, 6, lrintf(transitionFactor * 1000));
    DEBUG_SET(DEBUG_WING_LAUNCH, 7, launchState == WING_LAUNCH_CLIMBING
        ? MAX(0, (int16_t)(climbTimeMs - stateElapsedMs(currentTimeUs))) : 0);
}

void wingLaunchReset(void)
{
    launchState = WING_LAUNCH_IDLE;
    wasArmed = false;
    motorOutput = idleThrottle;
    transitionFactor = 0.0f;
    throttleGatePassed = false;
}

bool isWingLaunchActive(void)
{
    return launchState >= WING_LAUNCH_DETECTED && launchState <= WING_LAUNCH_CLIMBING;
}

bool isWingLaunchInProgress(void)
{
    return launchState >= WING_LAUNCH_DETECTED && launchState <= WING_LAUNCH_TRANSITION;
}

float wingLaunchGetThrottle(void)
{
    switch (launchState) {
    case WING_LAUNCH_IDLE:
    case WING_LAUNCH_DETECTED:
    case WING_LAUNCH_MOTOR_DELAY:
        return idleThrottle;
    case WING_LAUNCH_MOTOR_RAMP:
    case WING_LAUNCH_CLIMBING:
    case WING_LAUNCH_TRANSITION:
        return motorOutput;
    default:
        return -1.0f;
    }
}

float wingLaunchGetPitchAngle(void)
{
    if (launchState == WING_LAUNCH_IDLE && IS_RC_MODE_ACTIVE(BOXAUTOLAUNCH)) {
        return climbAngleDeg;
    }
    if (launchState == WING_LAUNCH_TRANSITION) {
        return climbAngleDeg * (1.0f - transitionFactor);
    }
    if (isWingLaunchInProgress()) {
        return climbAngleDeg;
    }
    return 0.0f;
}

float wingLaunchGetTransitionFactor(void)
{
    return transitionFactor;
}

int32_t wingLaunchGetClimbTimeRemainingMs(void)
{
    if (launchState == WING_LAUNCH_CLIMBING) {
        const timeDelta_t elapsed = cmpTimeUs(micros(), stateStartTimeUs) / 1000;
        const int32_t remaining = (int32_t)(climbTimeMs - elapsed);
        return (remaining > 0) ? remaining : 0;
    }
    return 0;
}

wingLaunchState_e wingLaunchGetState(void)
{
    return launchState;
}

bool wingLaunchIsThrottleGatePassed(void)
{
    return throttleGatePassed;
}

#endif // USE_WING_LAUNCH
