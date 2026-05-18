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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef USE_WING

// Hard caps applied to any single override request. Anything larger
// fails at the request boundary, not silently clamped, so the wizard
// gets an explicit error and can surface it.
#define SERVO_OVERRIDE_MAX_DURATION_MS 5000
#define SERVO_OVERRIDE_PWM_MIN         1000
#define SERVO_OVERRIDE_PWM_MAX         2000

// Wizard-driven single-servo override. Wing-fork only. Used by the
// Plane Setup Wizard (Identity / Direction / Endpoints steps) to pulse
// one servo at a time without RC input. Refused while armed.
//
// Returns false if any of:
//   - craft is armed (ARMING_FLAG(ARMED))
//   - servoIdx >= MAX_SUPPORTED_SERVOS
//   - pwm outside [SERVO_OVERRIDE_PWM_MIN, SERVO_OVERRIDE_PWM_MAX]
//   - durationMs == 0 or > SERVO_OVERRIDE_MAX_DURATION_MS
// Supersedes any previously active override.
bool servoOverrideRequest(uint8_t servoIdx, uint16_t pwm, uint16_t durationMs);

// Update the override state. Called once per servo update cycle from
// writeServos(). Force-clears if armed (closes the start-disarmed-
// then-arm race) and expires if duration elapsed.
void servoOverrideUpdate(void);

// True iff the given servo index is currently overridden.
bool servoOverrideIsActive(uint8_t servoIdx);

// Current override PWM (us). Only meaningful when
// servoOverrideIsActive() returns true for the same index.
uint16_t servoOverrideGetPwm(void);

#endif // USE_WING
