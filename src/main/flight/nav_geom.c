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

#include "flight/nav_geom.h"

#define EARTH_RADIUS_M  6371000.0f

static float degToRad(float d)
{
    return d * (M_PIf / 180.0f);
}

float navGeomDistanceM(float lat1Deg, float lon1Deg,
                       float lat2Deg, float lon2Deg)
{
    const float phi1    = degToRad(lat1Deg);
    const float phi2    = degToRad(lat2Deg);
    const float dPhi    = degToRad(lat2Deg - lat1Deg);
    const float dLambda = degToRad(lon2Deg - lon1Deg);

    const float sinHalfDPhi    = sinf(dPhi    * 0.5f);
    const float sinHalfDLambda = sinf(dLambda * 0.5f);

    const float a = sinHalfDPhi * sinHalfDPhi
                  + cosf(phi1) * cosf(phi2) * sinHalfDLambda * sinHalfDLambda;
    const float c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
    return EARTH_RADIUS_M * c;
}

float navGeomBearingRad(float lat1Deg, float lon1Deg,
                        float lat2Deg, float lon2Deg)
{
    const float phi1    = degToRad(lat1Deg);
    const float phi2    = degToRad(lat2Deg);
    const float dLambda = degToRad(lon2Deg - lon1Deg);

    const float y = sinf(dLambda) * cosf(phi2);
    const float x = cosf(phi1) * sinf(phi2)
                  - sinf(phi1) * cosf(phi2) * cosf(dLambda);

    float bearing = atan2f(y, x);
    if (bearing < 0.0f) {
        bearing += 2.0f * M_PIf;
    }
    return bearing;
}

float navGeomHeadingErrorRad(float currentRad, float targetRad)
{
    float diff = targetRad - currentRad;
    // Wrap into [-PI, PI]. Two while-loops instead of fmodf -- simpler
    // and faster for the small corrections this ever sees in practice.
    while (diff >  M_PIf) diff -= 2.0f * M_PIf;
    while (diff < -M_PIf) diff += 2.0f * M_PIf;
    return diff;
}
