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

#include "platform.h"

#include <math.h>

#include "common/maths.h"

#include "flight/nav_wind.h"

// ---- Tunables (fixed at compile time for Phase 1; promote to PG later
// if real-world tuning needs them exposed) ----

// GPS course becomes unreliable below this groundspeed. Samples at or
// below are discarded.
#define MIN_VALID_GROUND_SPEED_CMS      300.0f

// Rate-limit pushed samples. A GPS firing at 10 Hz has 100 ms spacing;
// anything tighter than this is probably the same tick twice and gets
// dropped so we don't double-count.
#define MIN_SAMPLE_INTERVAL_US          50000

// Circle is diced into this many wedges; we count how many have been
// visited for the coverage heuristic.
#define WEDGE_COUNT                     16
#define WEDGES_FOR_CONVERGING           6       // partial fit trusted-ish
#define WEDGES_FOR_VALID                12      // ~270 deg of coverage

// Minimum total samples required before we even call the fit CONVERGING.
#define MIN_SAMPLES_FOR_CONVERGING      12

// Linearized solve is only well-conditioned when the normal-equation
// determinant is clearly above zero. Below this threshold we refuse to
// report VALID.
#define MIN_DETERMINANT                 1.0e-3f

// ---- Internal state ----

typedef struct {
    // Running sums for the linearized model
    //   v_i ~ a + b*cos(theta_i) + c*sin(theta_i)
    // where a = airspeed, b = wind_N, c = wind_E.
    float    n;           // sample count as float (used in normal equations)
    float    sumC;        // sum cos(theta)
    float    sumS;        // sum sin(theta)
    float    sumCC;       // sum cos^2
    float    sumSS;       // sum sin^2
    float    sumCS;       // sum cos*sin
    float    sumV;        // sum v
    float    sumVC;       // sum v*cos
    float    sumVS;       // sum v*sin

    uint32_t wedgeMask;        // bit per WEDGE_COUNT wedge visited
    uint8_t  wedgesVisited;    // popcount of wedgeMask
    timeUs_t lastSampleUs;     // wall time of last accepted sample

    navWindEstimate_t cached;  // last computed estimate, returned by getter
    bool      dirty;           // true if accumulator changed since last solve
} navWindState_t;

static navWindState_t state;

// ---- Helpers ----

static uint8_t wedgeIndex(float courseRad)
{
    // Normalize course into [0, 2*PI) then map to wedge index.
    float normalized = fmodf(courseRad, 2.0f * M_PIf);
    if (normalized < 0.0f) {
        normalized += 2.0f * M_PIf;
    }
    const float wedgeWidth = (2.0f * M_PIf) / (float)WEDGE_COUNT;
    int idx = (int)(normalized / wedgeWidth);
    if (idx >= WEDGE_COUNT) {
        idx = WEDGE_COUNT - 1;
    } else if (idx < 0) {
        idx = 0;
    }
    return (uint8_t)idx;
}

static uint8_t popcount16(uint32_t mask)
{
    uint8_t c = 0;
    while (mask) {
        c += (uint8_t)(mask & 1U);
        mask >>= 1U;
    }
    return c;
}

// Cramer's rule on the 3x3 normal-equation matrix.
// [ n     sC    sS  ] [a]   [sV  ]
// [ sC    sCC   sCS ] [b] = [sVC ]
// [ sS    sCS   sSS ] [c]   [sVS ]
//
// Returns determinant; a/b/c are filled only if |det| is large enough.
static float solveNormalEquations(float *a, float *b, float *c)
{
    const float n   = state.n;
    const float sC  = state.sumC;
    const float sS  = state.sumS;
    const float sCC = state.sumCC;
    const float sSS = state.sumSS;
    const float sCS = state.sumCS;
    const float sV  = state.sumV;
    const float sVC = state.sumVC;
    const float sVS = state.sumVS;

    // Main determinant
    const float det =
          n   * (sCC * sSS - sCS * sCS)
        - sC  * (sC  * sSS - sCS * sS)
        + sS  * (sC  * sCS - sCC * sS);

    if (fabsf(det) < MIN_DETERMINANT) {
        return det;
    }

    const float detA =
          sV  * (sCC * sSS - sCS * sCS)
        - sC  * (sVC * sSS - sCS * sVS)
        + sS  * (sVC * sCS - sCC * sVS);

    const float detB =
          n   * (sVC * sSS - sVS * sCS)
        - sV  * (sC  * sSS - sCS * sS)
        + sS  * (sC  * sVS - sVC * sS);

    const float detC =
          n   * (sCC * sVS - sCS * sVC)
        - sC  * (sC  * sVS - sVC * sS)
        + sV  * (sC  * sCS - sCC * sS);

    *a = detA / det;
    *b = detB / det;
    *c = detC / det;
    return det;
}

static void recompute(void)
{
    navWindEstimate_t *e = &state.cached;

    if (state.wedgesVisited < WEDGES_FOR_CONVERGING || state.n < (float)MIN_SAMPLES_FOR_CONVERGING) {
        e->status = NAV_WIND_UNKNOWN;
        e->confidence = 0.0f;
        // Leave previous speed/heading numbers in place; they'll read stale
        // but the caller should ignore them when status != VALID.
        return;
    }

    float airspeed, windN, windE;
    const float det = solveNormalEquations(&airspeed, &windN, &windE);

    if (fabsf(det) < MIN_DETERMINANT) {
        e->status = NAV_WIND_UNKNOWN;
        e->confidence = 0.0f;
        return;
    }

    const float windMag = sqrtf(windN * windN + windE * windE);

    // Meteorological "from" heading. The model has peak groundspeed at
    // course = atan2(windE, windN) -- that's where we have tailwind, so
    // the wind is blowing FROM the opposite heading (add PI).
    float fromHeading = atan2f(windE, windN) + M_PIf;
    if (fromHeading >= 2.0f * M_PIf) {
        fromHeading -= 2.0f * M_PIf;
    } else if (fromHeading < 0.0f) {
        fromHeading += 2.0f * M_PIf;
    }

    e->windSpeedCmS        = windMag;
    e->windFromHeadingRad  = fromHeading;
    e->airSpeedCmS         = airspeed;
    e->lastUpdateUs        = state.lastSampleUs;

    if (state.wedgesVisited >= WEDGES_FOR_VALID) {
        e->status = NAV_WIND_VALID;
    } else {
        e->status = NAV_WIND_CONVERGING;
    }

    // Simple confidence: wedge-coverage fraction clipped to 1.0.
    const float frac = (float)state.wedgesVisited / (float)WEDGES_FOR_VALID;
    e->confidence = (frac > 1.0f) ? 1.0f : frac;
}

// ---- Public API ----

void navWindReset(void)
{
    state.n = 0.0f;
    state.sumC = state.sumS = 0.0f;
    state.sumCC = state.sumSS = state.sumCS = 0.0f;
    state.sumV = state.sumVC = state.sumVS = 0.0f;
    state.wedgeMask = 0U;
    state.wedgesVisited = 0;
    state.lastSampleUs = 0;
    state.cached.status = NAV_WIND_UNKNOWN;
    state.cached.windSpeedCmS = 0.0f;
    state.cached.windFromHeadingRad = 0.0f;
    state.cached.airSpeedCmS = 0.0f;
    state.cached.confidence = 0.0f;
    state.cached.lastUpdateUs = 0;
    state.dirty = false;
}

void navWindAddSample(float courseRad, float groundSpeedCmS, timeUs_t now)
{
    if (groundSpeedCmS < MIN_VALID_GROUND_SPEED_CMS) {
        return;
    }
    if (state.lastSampleUs != 0 && (now - state.lastSampleUs) < MIN_SAMPLE_INTERVAL_US) {
        return;
    }

    const float cth = cosf(courseRad);
    const float sth = sinf(courseRad);

    state.n     += 1.0f;
    state.sumC  += cth;
    state.sumS  += sth;
    state.sumCC += cth * cth;
    state.sumSS += sth * sth;
    state.sumCS += cth * sth;
    state.sumV  += groundSpeedCmS;
    state.sumVC += groundSpeedCmS * cth;
    state.sumVS += groundSpeedCmS * sth;

    const uint8_t wedge = wedgeIndex(courseRad);
    const uint32_t bit = 1U << wedge;
    if ((state.wedgeMask & bit) == 0U) {
        state.wedgeMask |= bit;
        state.wedgesVisited = popcount16(state.wedgeMask);
    }

    state.lastSampleUs = now;
    state.dirty = true;
}

const navWindEstimate_t *navWindGetEstimate(void)
{
    // Recompute on every get. The math is a handful of floats --
    // negligible on F4/F7 with the FPU -- and a stateless getter is
    // much easier to reason about and unit test than a lazy one.
    recompute();
    return &state.cached;
}
