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

// Passive navigation geometry helpers: great-circle distance, initial
// bearing, signed heading error. Pure functions -- no BF globals, no
// sensor coupling. Shared by autoland, RTH, and any future nav code
// that needs lat/lon math.

#pragma once

#include <stdbool.h>

// Great-circle distance between two surface points in metres. Lat/lon
// in decimal degrees. Uses the haversine formula -- accurate to ~0.5%
// over distances relevant to sport flying (<10 km).
float navGeomDistanceM(float lat1Deg, float lon1Deg,
                       float lat2Deg, float lon2Deg);

// Initial bearing (compass heading, 0 = North) from point 1 toward
// point 2, in radians [0, 2*PI). Result is the direction you would
// START flying to reach the target -- not the rhumb line.
float navGeomBearingRad(float lat1Deg, float lon1Deg,
                        float lat2Deg, float lon2Deg);

// Signed heading error: target - current, normalized to [-PI, PI].
// Positive result = target is clockwise from current (turn right in
// compass convention). Useful as the proportional term for a bank-
// angle controller.
float navGeomHeadingErrorRad(float currentRad, float targetRad);
