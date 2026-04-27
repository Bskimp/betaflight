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

// Wing autoland state machine. Phase 2 ships the skeleton only:
// state enum, watchdog-driven transitions to ABORT, throttle-override
// plumbing, arm-session lock. Phase 4 adds the glide controller,
// Phase 5 adds pattern geometry, Phase 6 adds touchdown detection,
// Phase 7 wires the four triggers (manual / RTH timeout / low batt /
// failsafe) that call autolandRequestEntry().
//
// Single-writer / single-reader throttle override (invariant #7):
// autoland.c is the only module that mutates the override; mixer.c
// is the only consumer, reached via autolandGetThrottleOverride().
//
// ESC / motor configuration requirement:
// Autoland commands throttle = 0.0 from AL_FLARE onward (and earlier
// if throttle_cut_alt_cm > 0 is configured). "Throttle 0" does NOT
// mean "prop stopped" on its own -- without ESC braking, the prop
// will keep windmilling from airflow until disarm at AL_COMPLETE
// kills the PWM/DSHOT output. Pilots who want props actively stopped
// at touchdown should enable `motor_brake` in their ESC configuration
// (or have an ESC with built-in brake). Autoland's responsibility
// ends at commanding throttle 0 and issuing the disarm signal.

#pragma once

#ifdef USE_WING

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"

typedef enum {
    AL_IDLE = 0,
    AL_ENTRY,              // baro ref latch, sequence start
    AL_ORBIT_DESCENT,      // descent during final orbit, wait on wind solve
    AL_DOWNWIND,           // pattern: downwind leg (tailwind, covers ground)
    AL_BASE,               // pattern: base turn + continuing descent
    AL_FINAL,              // pattern: into-wind final toward home
    AL_STRAIGHT_IN,        // wind-unknown fallback, direct approach to home
    AL_FLARE,              // commit-latched flare below commit_alt
    AL_TOUCHDOWN,          // waiting on 3-signal OR touchdown detection
    AL_COMPLETE,           // landed, waiting for disarm
    AL_ABORT,              // handing control back to pilot
    AL_PHASE_COUNT,
} autolandPhase_e;

typedef enum {
    AL_ABORT_NONE = 0,
    AL_ABORT_PILOT,        // BOXAUTOLAND off / stick override / RC rx alive
    AL_ABORT_GPS_LOSS,
    AL_ABORT_BARO_LOSS,
    AL_ABORT_WATCHDOG,     // phase exceeded its max time budget
    AL_ABORT_CONFIG,       // master disabled mid-sequence
    AL_ABORT_OTHER,
} autolandAbortCause_e;

// Config snapshot taken at autolandRequestEntry() time. The PG-backed
// wingAutolandConfig can change mid-flight (user tweaks in configurator);
// freezing the subset we actually consume at entry prevents a live-edit
// from glitching an active sequence.
//
// max_bank_angle_deg, nav_p_gain, and orbit_radius_m are sourced from
// gpsRescueConfig at entry -- same field semantics, already tuned for
// the airframe by whichever pilot is flying the wing. Avoids a parallel
// tuning surface.
typedef struct autolandSequenceConfig_s {
    uint8_t  glide_pitch_deg;
    uint16_t throttle_cut_alt_cm;
    uint8_t  cruise_throttle_pct;
    uint16_t commit_altitude_cm;
    uint16_t flare_start_alt_cm;
    uint8_t  flare_pitch_deg;
    uint8_t  max_bank_angle_deg;     // roll clamp for waypoint controller
    uint8_t  nav_p_gain;             // P gain on heading error, hundredths

    // Pattern / orbit fields (used by Phase 5b path):
    uint16_t orbit_radius_m;         // from gpsRescueConfig->orbitRadiusM
    uint16_t loiter_timeout_s;       // fall back to STRAIGHT_IN if exceeded
    uint8_t  orbits_before_descent;  // orbit count before pattern handoff
    uint16_t downwind_distance_m;
    uint16_t base_radius_m;
    uint16_t final_distance_m;

    // Touchdown detection fields (used by Phase 6 path):
    uint16_t touchdown_alt_threshold_cm;   // AL_FLARE -> AL_TOUCHDOWN trigger
    uint8_t  touchdown_accel_threshold;    // 0.1g units; 30 = 3.0g impact
    uint16_t touchdown_quiescence_ms;      // disarm delay in AL_TOUCHDOWN
} autolandSequenceConfig_t;

// Entry mode picks which flight-phase path we take from AL_ENTRY.
// Trigger code (Phase 7) picks based on the trigger source:
//   STRAIGHT_IN    = manual AUX / low battery / failsafe  (emergency descend)
//   ORBIT_DESCENT  = RTH loiter timeout -- wing was already orbiting home
typedef enum {
    AL_ENTRY_STRAIGHT_IN = 0,
    AL_ENTRY_ORBIT_DESCENT,
} autolandEntryMode_e;

// Sensor snapshot pushed in on each autolandUpdate tick. Passive design
// mirrors nav_wind -- autoland doesn't reach out to read sensors, it
// just consumes whatever the scheduler gives it. Pass NULL in a tick
// to say "no new data"; the watchdogs still fire and the last-known
// values stay cached.
typedef struct autolandSensorSample_s {
    float baroCm;              // raw baro altitude, same frame as initialBaroCm
    bool  baroValid;
    float latDeg, lonDeg;      // wing position, decimal degrees
    float groundCourseRad;     // GPS course-over-ground, [0, 2*PI)
    float groundSpeedCmS;
    bool  gpsValid;
    float accelMagG;           // accelerometer vector magnitude (~1.0 at rest)
    float gyroRateDegS;        // 3-axis gyro magnitude, deg/s
    bool  imuValid;            // accel+gyro both fresh and trustworthy
} autolandSensorSample_t;

// ---- Lifecycle ----

// Resets the state machine to IDLE and clears the arm-session lock.
// Call once on every arm event so a fresh arm gets a fresh autoland
// opportunity. Also clears throttle override + baro reference latch.
void autolandInit(void);

// Flight-loop tick. Drives watchdog checks, cross-phase transitions,
// and recomputes controller outputs from the latest sensor sample
// against the snapshotted config. Safe to call at any rate; expected
// ~100 Hz when hooked to the scheduler.
//
// `sample` may be NULL for a "heartbeat" tick (watchdogs still run
// against the last good sample). Pass a real sample every ~10 Hz from
// the GPS tick; denser from the baro tick is fine too -- autoland
// just caches whatever it's handed.
void autolandUpdate(timeUs_t currentTimeUs,
                    const autolandSensorSample_t *sample);

// ---- Trigger / abort ----

// True if the state machine will accept a fresh entry request:
// phase == IDLE and the arm session hasn't already been consumed.
bool autolandCanEnter(void);

// Transition IDLE -> ENTRY if canEnter(). Returns true if accepted.
// Does NOT check the wingAutolandConfig master enable -- callers
// (triggers in Phase 7) are responsible for that.
//
// At entry we latch the initial baro reading as the ground reference
// (invariant #2), snapshot config, and remember the home coordinates
// as the autoland target. `cfg` must be non-NULL.
//
// `entryMode` picks the flight-phase path:
//   STRAIGHT_IN   -> AL_ENTRY (dwell) -> AL_STRAIGHT_IN -> AL_FLARE
//   ORBIT_DESCENT -> AL_ENTRY (dwell) -> AL_ORBIT_DESCENT ->
//                    (wind known) AL_DOWNWIND -> AL_BASE -> AL_FINAL -> AL_FLARE
//                    (wind times out) AL_STRAIGHT_IN -> AL_FLARE
bool autolandRequestEntry(timeUs_t currentTimeUs,
                          float initialBaroCm,
                          float homeLatDeg,
                          float homeLonDeg,
                          autolandEntryMode_e entryMode,
                          const autolandSequenceConfig_t *cfg);

// Transition current phase -> ABORT. Returns false if rejected by the
// commit latch (below commit_altitude_cm, external aborts are ignored
// per invariant #5; Phase 6 will start driving the latch -- in Phase 2
// it's always false, so this always succeeds).
bool autolandAbort(autolandAbortCause_e cause, timeUs_t currentTimeUs);

// Raw transition. Exposed so (a) Phase 4-6 phase handlers can advance
// the machine and (b) unit tests can exercise the transition graph.
// Performs the side effects of the target phase (stamping phaseStartUs,
// clearing throttle override on IDLE/ABORT/COMPLETE, flipping the
// arm-session lock when a sequence terminates).
void autolandTransition(autolandPhase_e next, timeUs_t currentTimeUs);

// ---- Queries ----

bool                  autolandIsActive(void);
autolandPhase_e       autolandGetPhase(void);
autolandAbortCause_e  autolandGetLastAbortCause(void);
uint32_t              autolandGetPhaseElapsedMs(timeUs_t currentTimeUs);
const char           *autolandPhaseName(autolandPhase_e phase);

// Single-reader entry point for mixer.c.
// Returns true and writes *outThrottle in [0.0, 1.0] when the override
// is active; returns false when the pilot's throttle command stands.
bool autolandGetThrottleOverride(float *outThrottle);

// Pitch attitude setpoint for the PID controller. Active during all
// flight phases (AL_ENTRY..AL_TOUCHDOWN). Returns glide_pitch_deg
// everywhere except AL_FLARE, which returns flare_pitch_deg. Units:
// degrees, nose-down convention matched to the PID consumer (pid.c
// treats `angleTarget` as positive = nose-down, so the consumer
// negates internally).
bool autolandGetPitchSetpoint(float *outDeg);

// Roll attitude setpoint. Phase 4 always returns 0 (wings-level); the
// pattern geometry in Phase 5 will drive actual turns.
bool autolandGetRollSetpoint(float *outDeg);

// Diagnostic: the baro reading latched at AL_ENTRY. Returns false if
// the sequence hasn't entered yet (no latch).
bool autolandGetBaroReferenceCm(float *outCm);

// Latches at AL_COMPLETE; stays true until autolandInit() clears it.
// Phase 7 trigger wiring polls this from the arming task and issues
// the actual disarm -- keeps autoland.c free of direct arming deps.
bool autolandShouldDisarm(void);

// Diagnostic/test: commit latch is set when the wing drops below
// commit_altitude_cm during a flight phase. Once latched, external
// aborts (stick, BOXAUTOLAND toggle) are refused; only internal
// watchdog/sensor-loss aborts bypass.
bool autolandIsCommitLatched(void);

#endif // USE_WING
