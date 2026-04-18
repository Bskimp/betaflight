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

#include <stdbool.h>
#include "common/axis.h"

#ifdef USE_WING

extern float autopilotAngle[RP_AXIS_COUNT]; // NOTE: ANGLES ARE IN CENTIDEGREES

void autopilotInit(void);
void resetAltitudeControl(void);
void setSticksActiveStatus(bool areSticksActive);
void resetPositionControl(unsigned taskRateHz);
bool positionControl(void);
void altitudeControl(float targetAltitudeCm, float taskIntervalS, float targetAltitudeVelCmS, float velLimitCmS);

bool isBelowLandingAltitude(void);
float getAutopilotThrottle(void);
bool isAutopilotInControl(void);

#ifdef USE_GPS_RESCUE
// Wing GPS rescue navigation primitives
int16_t headingErrorDeci(int16_t target, int16_t current);
float wingGetRescueAltitudeCm(void);
float wingHeadingControl(int16_t errorDeci, uint8_t navP, uint8_t maxBankAngle);
// Returns Betaflight pitch in degrees: positive = nose-down, negative = nose-up.
float wingAltitudeControl(float altErrorCm, float climbRateCmS, uint8_t altP);
// pitchDeg uses Betaflight pitch sign: positive = nose-down, negative = nose-up.
float wingThrottleControl(float pitchDeg, uint8_t cruiseThrottle, uint8_t minThrottle);
// Returns additional turn compensation in Betaflight pitch sign (negative for nose-up).
// turnCompPct: strength 0-100, from gps_rescue_turn_compensation CLI param.
float wingTurnCompensation(float bankAngleDeg, uint8_t turnCompPct);
float wingClampOrbitRadius(uint16_t groundSpeedCmS, uint8_t maxBankAngle, uint16_t configRadiusM);

// Heading validity
void wingResetHeadingValidity(void);
bool wingUpdateHeadingValidity(uint16_t groundSpeedCmS, uint16_t minHeadingSpeedCmS);
bool wingIsHeadingValid(void);

// Output smoothing
void wingInitSmoothing(float currentThrottle, float rescueTaskHz);
void wingReseedSmoothing(void);
void wingApplySmoothing(float *rollCd, float *pitchCd, float *throttle);
void wingSetRescueThrottle(float throttle);
void wingSetRescueInactive(void);

// Debug
void wingRescueDebug(uint8_t phase, int16_t headingErrDeci, float targetRollCd,
                     float targetPitchCd, float distanceM, uint16_t groundSpeedCmS,
                     float throttle);
#endif // USE_GPS_RESCUE

#endif // USE_WING
