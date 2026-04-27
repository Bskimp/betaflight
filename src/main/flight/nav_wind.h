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

// Passive wind estimator. Consumer (rescue / autoland / RTH) pushes
// GPS course-over-ground + ground-speed samples during an orbit. When
// heading coverage is sufficient we solve a linearized sine fit
// (ground_speed = airspeed + w_N*cos(course) + w_E*sin(course))
// via the closed-form 3x3 normal equations.
//
// GPS-only by design -- no compass required. Works for sport wings
// without a mag. Linearization holds while wind < ~30% of airspeed
// (essentially always true for recreational flight).

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"

typedef enum {
    NAV_WIND_UNKNOWN = 0,   // no samples yet, or heading coverage too low
    NAV_WIND_CONVERGING,    // samples coming in, partial fit available
    NAV_WIND_VALID,         // coverage + sample count good; fit trusted
} navWindStatus_e;

// NOTE on staleness: this module deliberately doesn't track "now" or
// apply a STALE filter. Callers already know the current time and can
// compare `lastUpdateUs` themselves. Keeping nav_wind time-agnostic
// makes it trivial to unit test and reusable outside the flight loop.

typedef struct {
    navWindStatus_e status;
    float windSpeedCmS;         // magnitude of horizontal wind vector (cm/s)
    float windFromHeadingRad;   // meteorological "from" heading, [0, 2*PI)
    float airSpeedCmS;          // estimated airspeed -- byproduct of the fit
    float confidence;           // heuristic [0.0 .. 1.0]
    timeUs_t lastUpdateUs;      // time of most recently incorporated sample
} navWindEstimate_t;

// Clear accumulator state. Call this at the moment orbit / loiter begins
// so stale data from a previous flight leg doesn't contaminate the fit.
void navWindReset(void);

// Feed one GPS sample. `courseRad` is the GPS course-over-ground in
// radians ([0, 2*PI) is preferred but any angle works -- we use cos/sin).
// `groundSpeedCmS` is the GPS-reported ground speed. Called at whatever
// rate the consumer wants (typical: 5-10 Hz during orbit).
//
// Samples below MIN_VALID_GROUND_SPEED_CMS are rejected (GPS course is
// noisy when the aircraft is nearly stationary); the function is a
// no-op in that case.
void navWindAddSample(float courseRad, float groundSpeedCmS, timeUs_t now);

// Returns a pointer to the current estimate. Never NULL. The struct is
// updated lazily; repeated calls without new samples return the same
// numbers. Consumers should check `.status` before trusting values --
// treat UNKNOWN / STALE as "wind unknown, use straight-in fallback."
const navWindEstimate_t *navWindGetEstimate(void);
