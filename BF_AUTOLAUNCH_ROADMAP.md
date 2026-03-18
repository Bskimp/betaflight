# BF Wing Autolaunch — Development Roadmap

## Repo Setup

Fork and clone the full betaflight repo. You need the complete tree because
the build system, headers, and #ifdef guards are deeply interconnected.

```bash
git clone https://github.com/YOUR_USERNAME/betaflight.git
cd betaflight
git checkout -b wing-autolaunch
git submodule update --init    # pulls betaflight/config for targets
```

### Build Environment

BF has a dev container. Easiest path:

```bash
# VS Code: Install "Dev Containers" extension, open folder, "Reopen in Container"

# Or CLI:
docker build -t betaflight-dev -f .devcontainer/containerfile .devcontainer/
docker run --rm -v "${PWD}:/workspace" -w /workspace betaflight-dev \
  make TARGET=SPEEDYBEEF405WING
```

Test that you can build a wing target before changing anything:
```bash
make TARGET=SPEEDYBEEF405WING
# Should produce betaflight_4.6_SPEEDYBEEF405WING.hex
```

### Claude Code Context Strategy

For Claude Code to be effective on the BF codebase, it needs focused context.
The full repo is ~50k+ lines across hundreds of files. Create a CLAUDE.md at
repo root that points to the key areas:

```markdown
# CLAUDE.md

## Project: Betaflight Wing Autolaunch

### What this is
Adding a hand-launch mode for fixed-wing aircraft to Betaflight firmware.
State machine: detect throw via accelerometer → motor spin-up → timed climb
at fixed pitch angle → transition to pilot control.

### Key constraint
All new code is wrapped in `#ifdef USE_WING` and `#ifdef USE_WING_LAUNCH`.
Must not affect quad/multirotor builds AT ALL.

### Build & test
make TARGET=SPEEDYBEEF405WING
make test (runs unit tests)

### Architecture rules
- No target-specific code outside src/platform/
- Use float math with f suffix (1.0f not 1.0)
- Use sin_approx/cos_approx from common/maths.h
- Static functions for module-internal, STATIC_UNIT_TESTED for testable
- Data flows right-to-left (dst, src pattern)
- No lazy init — use explicit init functions

### Files I'm working on (PRIMARY — read these first)
- src/main/flight/wing_launch.c     (NEW — the state machine)
- src/main/flight/wing_launch.h     (NEW — public interface)
- src/main/flight/pid.c             (integration point — launch overrides setpoints)
- src/main/flight/pid.h             (launch state types, config params)
- src/main/flight/mixer.c           (throttle override during launch)
- src/main/fc/core.c                (arming logic, mode activation)
- src/main/fc/rc_modes.h            (BOXLAUNCH mode flag)

### Files I need to modify (SECONDARY — read relevant sections)
- src/main/cli/settings.c           (CLI parameter definitions)
- src/main/cli/settings.h           (lookup table enum)
- src/main/fc/parameter_names.h     (CLI parameter name strings)
- src/main/blackbox/blackbox.c      (logging launch state)
- src/main/osd/osd_elements.c       (OSD launch status)
- src/main/pg/pg_ids.h              (parameter group ID if needed)

### Reference files (READ ONLY — understand patterns)
- src/main/flight/autopilot_wing.c  (limonspb's wing autopilot scaffold)
- src/main/flight/autopilot_wing.h
- src/main/flight/pid_init.c        (how wing TPA/SPA/S-term are initialized)
- src/main/fc/rc.c                  (how RC input flows to pid)
- src/main/flight/imu.c             (attitude estimation, accel access)
- src/main/sensors/acceleration.h   (raw accel data types)

### Coding style
- https://betaflight.com/docs/development/codingstyle
- K&R braces, 4-space indent, no tabs
- Comments: // style, not /* */
- No goto, no continue, minimize multiple returns
```

---

## File Map — What You're Touching

### NEW FILES (you create these)

```
src/main/flight/wing_launch.c      # State machine + throw detection + climb control
src/main/flight/wing_launch.h      # Public API: wingLaunchInit(), wingLaunchUpdate(), 
                                   # wingLaunchIsActive(), wingLaunchGetThrottle(),
                                   # wingLaunchGetPitchAngle(), etc.
src/test/unit/wing_launch_unittest.cc  # Unit tests for state machine
```

### MODIFIED FILES (you edit these)

```
src/main/flight/pid.c              # ~10 lines: call wingLaunchUpdate(), override 
                                   # setpoints when launch active
src/main/flight/pid.h              # ~15 lines: add launch config params to pidProfile_t
src/main/flight/pid_init.c         # ~5 lines: call wingLaunchInit() 
src/main/flight/mixer.c            # ~10 lines: override throttle when launch active
src/main/fc/core.c                 # ~15 lines: launch mode activation logic,
                                   # prevent mid-flight activation
src/main/fc/rc_modes.h             # ~2 lines: add BOXLAUNCH to mode enum
src/main/fc/rc_modes.c             # ~2 lines: register BOXLAUNCH mode
src/main/cli/settings.c            # ~20 lines: add CLI params
src/main/cli/settings.h            # ~1 line: lookup table entry if needed
src/main/fc/parameter_names.h      # ~12 lines: param name strings
src/main/blackbox/blackbox.c       # ~5 lines: log launch params
src/main/osd/osd_elements.c        # ~20 lines: launch status OSD element
src/main/build/debug.h             # ~1 line: DEBUG_WING_LAUNCH mode
src/test/unit/Makefile              # ~5 lines: add wing_launch test target
```

### REFERENCE FILES (read, don't modify)

```
src/main/flight/autopilot_wing.c   # Limonspb's scaffold — YOUR code lives next to this
src/main/flight/autopilot_wing.h   # Pattern for how wing-specific flight code interfaces
src/main/flight/imu.c              # attitude_t, getEstimatedAttitude()
src/main/flight/imu.h              # attitude.values.pitch/roll/yaw
src/main/sensors/acceleration.h    # acc.accADC[XYZ], accumulatedMeasurements
src/main/common/maths.h            # constrainf, sin_approx, cos_approx, DEGREES_TO_RADIANS
src/main/common/time.h             # timeUs_t, micros(), millis()
src/main/fc/runtime_config.h       # FLIGHT_MODE(), IS_RC_MODE_ACTIVE(), armingFlags
src/main/fc/rc.c                   # How stick input becomes rcCommand/setpoint
```

---

## Phase 1 — Scaffolding (Day 1)

**Goal:** New files exist, compile, mode flag registered, does nothing yet.

### Tasks

1. Create `wing_launch.h` with:
   - State enum (IDLE, DETECTED, MOTOR_DELAY, MOTOR_RAMP, CLIMBING, 
     TRANSITION, COMPLETE)
   - Config struct or fields in pidProfile_t
   - Public function declarations

2. Create `wing_launch.c` with:
   - Empty/stub implementations of all public functions
   - `wingLaunchInit()` — reads config, initializes state to IDLE
   - `wingLaunchUpdate()` — empty, returns immediately
   - `wingLaunchIsActive()` — returns false
   - Wrapped in `#ifdef USE_WING_LAUNCH`

3. Add `BOXLAUNCH` to rc_modes.h mode enum

4. Add CLI parameters to settings.c / parameter_names.h (all with defaults)

5. Add `#define USE_WING_LAUNCH` to the appropriate build config
   (probably in `common_pre.h` or `target.h` — needs to be gated on USE_WING)

6. Add to Makefile — wing_launch.c in the SRC list

7. **Verify:** `make TARGET=SPEEDYBEEF405WING` compiles clean with new files

### Acceptance
- Builds with zero warnings
- LAUNCH mode appears in `resource` / mode list
- CLI `set fw_launch_*` params work
- No behavioral change — everything still flies normally

---

## Phase 2 — Throw Detection (Day 2)

**Goal:** The FC detects a hand throw via accelerometer.

### Tasks

1. Implement accel monitoring in IDLE state:
   ```c
   // In wingLaunchUpdate(), IDLE state:
   // Read accel magnitude
   float accelMagnitude = sqrtf(sq(acc.accADC[X]) + sq(acc.accADC[Y]) + sq(acc.accADC[Z]));
   float accelG = accelMagnitude / acc.dev.acc_1G;
   
   // Check against threshold
   if (accelG > config->accelThresh) {
       detectCounter++;
       if (detectCounter >= config->detectSamples) {
           // LAUNCH DETECTED
           transitionTo(LAUNCH_STATE_DETECTED);
       }
   } else {
       detectCounter = 0;  // reset — must be sustained
   }
   ```

2. Add blackbox debug output for accel during detection:
   ```c
   DEBUG_SET(DEBUG_WING_LAUNCH, 0, lrintf(accelG * 100));
   DEBUG_SET(DEBUG_WING_LAUNCH, 1, launchState);
   ```

3. Wire into PID loop — call `wingLaunchUpdate()` from `pidController()` 
   when USE_WING_LAUNCH and LAUNCH mode active

4. **Safety:** Add mid-flight activation guard:
   ```c
   // In core.c mode activation:
   // Only allow LAUNCH if armed < 3 seconds ago
   if (IS_RC_MODE_ACTIVE(BOXLAUNCH) && 
       millis() - getArmTime() < 3000 &&
       !wingLaunchIsComplete()) {
       ENABLE_FLIGHT_MODE(LAUNCH_MODE);
   }
   ```

### Testing

- Blackbox with `debug_mode = WING_LAUNCH`
- Shake the FC on the bench, confirm detection triggers
- Confirm detection does NOT trigger from gentle handling
- Confirm mid-flight activation is blocked

### Acceptance
- Throw detection works reliably in bench tests
- No false triggers from normal handling  
- Debug logging shows accel values and state transitions
- Mid-flight switch flip does not activate launch

---

## Phase 3 — Motor Control + Climb (Day 3)

**Goal:** After detection, motor spins up and FC holds climb attitude.

### Tasks

1. Implement MOTOR_DELAY state:
   - Servos snap to climb position
   - Motor stays at idle
   - Timer counts down motor_delay ms

2. Implement MOTOR_RAMP state:
   - Linear ramp from idle_thr to launch_thr over ramp_time ms
   - `wingLaunchGetThrottle()` returns interpolated value

3. Implement CLIMBING state:
   - `wingLaunchGetThrottle()` returns launch_thr
   - `wingLaunchGetPitchAngle()` returns climb_angle
   - Roll axis: force wings-level (angle mode on roll)
   - Timer counts down launch_timeout ms
   - Monitor bank/pitch safety limits

4. Wire throttle override into mixer.c:
   ```c
   #ifdef USE_WING_LAUNCH
   if (wingLaunchIsActive() && wingLaunchGetThrottle() >= 0) {
       throttle = wingLaunchGetThrottle();
   }
   #endif
   ```

5. Wire pitch/roll override into pid.c:
   ```c
   #ifdef USE_WING_LAUNCH
   if (wingLaunchIsActive()) {
       // Override pitch setpoint to climb angle
       if (wingLaunchGetPitchAngle() != 0) {
           currentPidSetpoint = wingLaunchGetPitchAngle() * 10.0f; // decidegrees
       }
       // Force wings level on roll
       // (use angle mode logic on roll axis)
   }
   #endif
   ```

### Safety Limits in CLIMBING

```c
// Auto-abort conditions checked every loop iteration:
if (fabsf(attitude.values.roll) > config->maxBank * 10) {
    transitionTo(LAUNCH_STATE_ABORT);
}
if (attitude.values.pitch < -(config->maxDive * 10)) {
    transitionTo(LAUNCH_STATE_ABORT);
}
if (fabsf(attitude.values.roll) > 1500) {  // 150° = inverted
    transitionTo(LAUNCH_STATE_ABORT);
}
```

### Acceptance
- Motor starts after delay, ramps smoothly
- Wing holds configured pitch angle during climb  
- Wings stay level (roll stabilized)
- Auto-aborts on excessive bank
- Throttle override works correctly in mixer

---

## Phase 4 — Transition + Abort (Day 4)

**Goal:** Clean handoff to pilot, instant abort on switch-off.

### Tasks

1. Implement TRANSITION state:
   - Blend factor from 1.0 → 0.0 over end_time ms
   - Pitch override = climb_angle * blendFactor
   - Throttle transitions from launch_thr to pilot stick
   - Stick authority gradually restored: setpoint * (1.0 - blendFactor) 
     + launchSetpoint * blendFactor

2. Implement switch-off abort (ALL states):
   ```c
   // Top of wingLaunchUpdate():
   if (!IS_RC_MODE_ACTIVE(BOXLAUNCH) || !ARMING_FLAG(ARMED)) {
       if (launchState != LAUNCH_STATE_IDLE && 
           launchState != LAUNCH_STATE_COMPLETE) {
           // ABORT — instant return to pilot control
           launchState = LAUNCH_STATE_COMPLETE;
           // No transition blend — immediate
       }
       return;
   }
   ```

3. Implement ABORT state:
   - Same as switch-off: instant full control return
   - Throttle snaps to current stick position
   - All overrides removed in one cycle
   - Log abort reason in blackbox

4. Reset on disarm:
   ```c
   // Called from disarm handler in core.c:
   void wingLaunchReset(void) {
       launchState = LAUNCH_STATE_IDLE;
       detectCounter = 0;
       launchTimer = 0;
       // Ready for next arm cycle
   }
   ```

### Acceptance
- Flipping switch OFF mid-climb = instant pilot control
- Transition from climb to normal flight is smooth
- Disarm always works in every state
- Re-arm after failed launch resets clean
- Abort on bank angle works and logs reason

---

## Phase 5 — OSD + Polish (Day 5)

**Goal:** Pilot feedback, blackbox support, unit tests.

### Tasks

1. OSD element in osd_elements.c:
   ```c
   // OSD_WING_LAUNCH_STATUS
   case OSD_WING_LAUNCH_STATUS:
       switch (wingLaunchGetState()) {
       case LAUNCH_STATE_IDLE:       osdFormatCentiNumber(buff, 0, 0, 0, 0, 10, false); 
                                     tfp_sprintf(buff, "LNCH RDY"); break;
       case LAUNCH_STATE_DETECTED:   tfp_sprintf(buff, "THROW!"); break;
       case LAUNCH_STATE_MOTOR_DELAY:
       case LAUNCH_STATE_MOTOR_RAMP: tfp_sprintf(buff, "LNCH GO"); break;
       case LAUNCH_STATE_CLIMBING:   tfp_sprintf(buff, "CLIMB %d", 
                                     (int)(launchTimerRemaining/1000)); break;
       case LAUNCH_STATE_TRANSITION: tfp_sprintf(buff, "RECOVER"); break;
       default: buff[0] = '\0';
       }
       break;
   ```

2. Blackbox logging:
   - Log all fw_launch_* params in header
   - Log state transitions with timestamps
   - Debug mode WING_LAUNCH: accel magnitude, state, timer, throttle

3. Unit tests in wing_launch_unittest.cc:
   - Test state transitions (IDLE → DETECTED → DELAY → RAMP → CLIMB → TRANSITION → COMPLETE)
   - Test abort on switch-off from each state
   - Test abort on bank angle exceeded
   - Test mid-flight activation blocked
   - Test detection threshold (below/at/above)
   - Test motor ramp timing
   - Test transition blend math
   - Test disarm resets state

4. Code cleanup:
   - All functions properly documented
   - All magic numbers replaced with named constants
   - All #ifdef guards consistent
   - Coding style compliance

### Acceptance
- OSD shows correct state during launch sequence
- Blackbox captures full launch data
- All unit tests pass
- `make test` passes with new tests
- Code review ready

---

## Phase 6 — Testing Protocol

### Bench Testing (Before ANY flight)

1. **Detection test:** Arm with LAUNCH mode, hold wing, do practice throws 
   (catch immediately). Confirm detection triggers at desired force level.
   Adjust `fw_launch_accel_thresh`.

2. **Motor test:** WITH PROPS OFF. Arm with LAUNCH mode. Throw. Confirm 
   motor starts after delay, ramps up smoothly, runs for timeout duration,
   then stops (or transitions).

3. **Abort test:** WITH PROPS OFF. Arm, throw, confirm motor starts. 
   Flip LAUNCH switch off. Confirm instant stop of launch sequence.

4. **Attitude test:** WITH PROPS OFF. Mount FC on a board you can tilt.
   Arm, simulate throw (shake), confirm servos go to climb position.
   Tilt past bank limit — confirm abort.

### First Flight Testing

5. **Conservative settings first:**
   ```
   set fw_launch_thr = 1500          # lower than cruise, just enough to fly
   set fw_launch_climb_angle = 12    # gentle climb  
   set fw_launch_timeout = 3000      # short, 3 seconds
   set fw_launch_max_bank = 35       # tight safety margin
   ```

6. **Open field, low wind, have a spotter**

7. **First throw:** Gentle, straight, slightly nose up. Confirm motor starts,
   wing climbs, transitions to pilot control. Have finger on LAUNCH switch
   to abort if anything looks wrong.

8. **Iterate:** Gradually increase climb_angle, timeout, throttle as confidence grows.

---

## File Size Estimates

| File | New Lines | Modified Lines |
|------|-----------|----------------|
| wing_launch.c | ~350 | — |
| wing_launch.h | ~60 | — |
| wing_launch_unittest.cc | ~200 | — |
| pid.c | — | ~15 |
| pid.h | — | ~20 |
| pid_init.c | — | ~5 |
| mixer.c | — | ~10 |
| core.c | — | ~20 |
| rc_modes.h/c | — | ~5 |
| settings.c | — | ~25 |
| parameter_names.h | — | ~12 |
| blackbox.c | — | ~10 |
| osd_elements.c | — | ~25 |
| debug.h | — | ~2 |
| Makefile | — | ~5 |
| **Total** | **~610** | **~154** |

~750 lines total. Tight, focused, single-purpose PR.

---

## PR Strategy

### Title
`Wing autolaunch mode (for wings) #ifdef USE_WING_LAUNCH`

### Description Format
Follow limonspb's pattern — clear description, CLI params listed, testing notes,
safety considerations, wrapped in USE_WING ifdef.

### Review Path
1. Post in BF Discord #limon-wing-channel first — get limonspb's input
   on the approach before writing code
2. He may want it structured differently or integrated with his autopilot_wing 
   scaffold — better to know before than after
3. Submit PR against betaflight:master
4. Tag limonspb, haslinghuis, ledvinap as reviewers

### Dependencies
- PR #14108 (autopilot wing separation) should be merged first
- Your code goes in the wing autopilot area it creates
- If #14108 isn't merged yet, base your branch on limonspb's branch

---

## Context Mode Repo Setup

If you want a dedicated repo for BF wing development with Claude Code context:

```bash
# Fork betaflight
# Clone your fork
git clone https://github.com/Bskimp/betaflight.git
cd betaflight

# Create CLAUDE.md (contents above)

# Create .claude/ directory for skills/context
mkdir -p .claude

# Create the wing launch skill
cat > .claude/wing-launch.md << 'EOF'
# Wing Launch Development Context

## Current task
Implementing fw_launch mode for fixed-wing hand launch.

## Key patterns to follow
- Look at how SPA (PR #13719) was added for the pattern of:
  adding wing-specific PID modifications with CLI params
- Look at how autopilot_wing.c interfaces with pid.c
  for the pattern of overriding setpoints
- Look at failsafe in core.c for the pattern of
  overriding throttle in mixer.c

## Build command
make TARGET=SPEEDYBEEF405WING

## Test command  
make test

## Files to read before any change
Always read wing_launch.h first to understand the public API.
Then check pid.c for the integration points.
EOF

# Install Claude Code skills if desired
# Point roadmap-planning skill at this roadmap doc
```

This gives Claude Code focused context without needing to ingest the entire
50k+ line codebase every session.
