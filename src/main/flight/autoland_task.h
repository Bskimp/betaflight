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

// Thin runtime glue between autoland.c (pure state machine + controller)
// and Betaflight's main loop. autoland.c stays passive and unit-testable;
// this file pulls the sensor globals, handles the BOXAUTOLAND edge, and
// polls the disarm flag. Live on the wing-fork only.

#pragma once

#ifdef USE_WING

#include "common/time.h"

// Called once per main-loop tick while armed. Builds a sensor sample
// from baro / GPS / IMU globals, watches the BOXAUTOLAND toggle edges
// (manual trigger) and polls autolandShouldDisarm() to issue a disarm
// on rising edge. Safe to call when autoland master is disabled --
// triggers are gated internally.
void autolandTaskTick(timeUs_t currentTimeUs);

// Called immediately after a successful arm event (ENABLE_ARMING_FLAG
// (ARMED) in tryArm). Resets the state machine to IDLE, clears the
// arm-session lock, and snapshots the BOXAUTOLAND state so the first
// tick after arm doesn't misread a pre-held switch as a fresh trigger.
void autolandOnArm(void);

#endif // USE_WING
