# Changelog — Betaflight VTOL & Wing Launch Fork

Based on Betaflight 4.6.0-alpha (2026.6.0). 45 files changed,
2160 insertions, 26 deletions across all custom commits.

---

## Section 1: Wing Autolaunch (`USE_WING_LAUNCH`)

Hand-launch state machine for fixed-wing aircraft. Detects a throw,
ramps motors, holds a fixed climb angle, then transitions to pilot
control.

### Commits (oldest first)

1. **`9160dc8`** — Add wing autolaunch mode for fixed-wing hand launch
   - New files: `wing_launch.c`, `wing_launch.h`, `wing_launch_unittest.cc`
   - 8-state machine: IDLE > DETECTED > MOTOR_DELAY > MOTOR_RAMP > CLIMBING > TRANSITION > COMPLETE / ABORT
   - BOXLAUNCH mode flag, 10 CLI parameters in `pidProfile_t`
   - `#ifdef USE_WING_LAUNCH` guard, enabled in `common_pre.h`

2. **`0e7d7cb`** — Switch autolaunch trigger from accel detection to arm event
   - Changed throw detection to trigger on arm with BOXLAUNCH active

3. **`05c7820`** — Fix elevator pre-deflection when AUTOLAUNCH switch is on while disarmed
   - Servo output correction for pre-launch elevator position

4. **`d9e6c98`** — Add elevon pre-deflection on AUTOLAUNCH switch via servo output path
   - Servos deflect to pitch-up position before throw for better launch angle

5. **`b71d654`** — Fix elevon pre-deflection sign to match PID pitch-up direction
   - Corrected servo direction for pre-deflection

6. **`855051a`** — Fix elevon pre-deflection sign and seamless handoff to PID on arm
   - Clean transition from pre-deflection to PID-controlled output

7. **`e81f3ba`** — Smooth throttle handoff during transition and extend max climb time
   - Throttle blends smoothly from launch target to pilot stick
   - `wing_launch_climb_time` max extended to 20000 ms

8. **`1611b69`** — Add throw detection, fix ANGLE_MODE bleed-through, and OSD/servo fixes
   - Re-added accelerometer-based throw detection
   - Fixed pitch setpoint leak from ANGLE mode during launch
   - OSD and servo integration fixes

9. **`0a41565`** — Add OSD climb countdown and expanded debug channels
   - `OSD_WING_LAUNCH_STATUS` element shows state and time remaining
   - `DEBUG_WING_LAUNCH` mode with 8 debug channels

10. **`3572ff3`** — Add stick override, throttle gate, and timer clamp
    - `wing_launch_stick_override`: pilot stick cancels launch (0=disabled)
    - Throttle gate: motors don't start until delay passes
    - Timer clamp: climb timer doesn't start until motors are running

### Files Modified

| File | Change |
|------|--------|
| `flight/wing_launch.c` | New — state machine, throw detection, climb control |
| `flight/wing_launch.h` | New — public API, state enum |
| `flight/pid.c` | Pitch/throttle override when launch active |
| `flight/pid.h` | 10 launch params in `pidProfile_t` |
| `flight/pid_init.c` | `wingLaunchInit()` call |
| `flight/mixer.c` | Throttle override during motor ramp/climb |
| `flight/servos.c` | Elevator pre-deflection, servo control during launch |
| `fc/core.c` | BOXLAUNCH mode activation, arming logic |
| `fc/rc_modes.h` / `rc_modes.c` | BOXLAUNCH mode registration |
| `cli/settings.c` | 10 CLI parameter definitions |
| `fc/parameter_names.h` | Parameter name macros |
| `blackbox/blackbox.c` | Launch parameter logging |
| `osd/osd_elements.c` | OSD launch status element |
| `build/debug.h` | `DEBUG_WING_LAUNCH` mode |
| `target/common_pre.h` | `#define USE_WING_LAUNCH` |
| `test/unit/wing_launch_unittest.cc` | New — 327-line unit test |

---

## Section 2: VTOL Mixer Profiles (`USE_VTOL`)

Switchable motor/servo mixer profiles for VTOL aircraft. Two
independent mixer configurations with smooth blended transitions,
per-profile IMU orientation, and failsafe integration.

### Commits (oldest first)

1. **`2c494bb`** — Add VTOL switchable mixer profiles (PR1: core plumbing)
   - New files: `mixer_profile.c`, `mixer_profile.h`
   - `mixerProfile_t` struct: per-profile `motorMix`, `servoMix`, `mixerMode`, `platformType`
   - PG registration with `MAX_MIXER_PROFILE_COUNT = 2`
   - Profile 0 defaults to `MIXER_CUSTOM` / `PLATFORM_MULTIROTOR`
   - Profile 1 defaults to `MIXER_CUSTOM_AIRPLANE` / `PLATFORM_AIRPLANE`
   - `mixerProfileSelect()` for disarmed switching
   - Superset calculation: `mixerProfileGetSuperset()` scans all profiles for max motor/servo count
   - Linked PID/rate profile switching
   - CLI settings: `mixer_profile_count`, `mixer_transition_time`, `mixer_linked_*`
   - `#define USE_VTOL` in `common_pre.h`

2. **`fc97c33`** — Add VTOL mixer profile transition state machine and AUX mode switching (PR2)
   - `BOXMIXERPROFILE` AUX mode for in-flight switching
   - Blended transition state machine: IDLE > BLENDING > finalize
   - `mixerProfileTransitionUpdate()` called from main loop
   - PID/rate profiles switch immediately at transition start
   - I-term reset on finalize
   - GPS rescue and failsafe mode blocking
   - Failsafe integration: `failsafe_mixer_profile`, `failsafe_mixer_action`
   - `mixerProfileOnFailsafe()` for immediate snap to safe profile

3. **`f331e92`** — Add VTOL motor/servo output blending, transition-only gating, and OSD element (PR3)
   - Motor output crossfade during transition (linear blend)
   - Servo output crossfade between profile rules
   - `motorTransitionOnly[]` flag: motors that only output during transitions
   - `BOXMIXERTRANSITION` servo box (box 4) for transition-only servo rules
   - `OSD_MIXER_PROFILE` element: shows "MP:0", "MP:0>1 50%"
   - `DEBUG_MIXER_PROFILE` mode with 4 debug channels

4. **`342e308`** — Make mmix/smix CLI commands profile-aware
   - `mixer_profile <n>` CLI command to switch profiles
   - `mmix` and `smix` read/write the active mixer profile's data
   - `dump` and `diff` show per-profile motor/servo mixes
   - Profile-aware display in CLI output

5. **`28b0d25`** — Add VTOL per-profile IMU rotation, servo fallbacks, and bench-test fixes
   - `boardAlignmentSetVtolRotation()` applies per-profile rotation matrix
   - 6 CLI settings: `mixer_imu_orientation_{roll,pitch,yaw}_{1,2}` (decidegrees)
   - Rotation applied after board alignment, affects gyro + accel
   - Servo system fallbacks: `servosInit()` always enables servos, `writeServos()` handles non-airplane platforms
   - Boot sequence: `mixerProfileInit()` + `mixerProfileApplyActive()` in `init.c`

### Files Modified

| File | Change |
|------|--------|
| `flight/mixer_profile.c` | New — profile state, transitions, blending helpers, failsafe |
| `flight/mixer_profile.h` | New — `mixerProfile_t`, transition API, superset API |
| `flight/mixer.c` | Motor output blending during transitions |
| `flight/mixer.h` | Superset motor/servo count declarations |
| `flight/mixer_init.c` | Superset allocation, default mixer config |
| `flight/servos.c` | Servo blending, transition-only box, fallbacks |
| `sensors/boardalignment.c` | VTOL rotation matrix, `boardAlignmentSetVtolRotation()` |
| `sensors/boardalignment.h` | Function declaration |
| `fc/init.c` | Mixer profile init/apply at boot |
| `fc/core.c` | `BOXMIXERTRANSITION` mode handling |
| `fc/rc.c` | Added mixer.h / mixer_profile.h includes |
| `cli/cli.c` | `mixer_profile` command, profile-aware mmix/smix |
| `cli/settings.c` | 12 VTOL CLI settings |
| `fc/parameter_names.h` | Parameter name macros |
| `msp/msp.c` | Superset motor/servo count in MSP responses |
| `msp/msp_box.c` | BOXMIXERPROFILE, BOXMIXERTRANSITION registration |
| `osd/osd_elements.c` | OSD mixer profile status element |
| `build/debug.h` | `DEBUG_MIXER_PROFILE` mode |
| `target/common_pre.h` | `#define USE_VTOL` |

---

## Section 3: Bug Fixes & Improvements (this session)

### SPA Mode — Changed from Global to Per-Profile

**File:** `cli/settings.c`

`spa_roll_mode`, `spa_pitch_mode`, `spa_yaw_mode` were declared as
`MASTER_VALUE` (global) despite being stored in `pidProfile_t`
(per-profile struct). This meant SPA mode couldn't differ between
PID profiles — critical for VTOL where hover needs SPA OFF and
airplane needs SPA active.

**Fix:** Changed all three from `MASTER_VALUE` to `PROFILE_VALUE`,
consistent with `spa_center` and `spa_width` which were already
per-profile.

### Known Limitation: `dump all` Does Not Include Mixer Profiles

The CLI `dump all` command iterates PID profiles and rate profiles
but does not yet dump per-mixer-profile mmix/smix entries. To view
them manually:

```
mixer_profile 0
mmix
smix
mixer_profile 1
mmix
smix
```

This could be fixed by extending the dump loop in `cli.c` to iterate
`mixer_profile_count` profiles.

---

## Build Targets Tested

| Target | MCU | Flash Usage | Status |
|--------|-----|-------------|--------|
| MICOAIR743 | STM32H743 | 30.5% | Bench tested, hover verified |
| SPEEDYBEEF405WING | STM32F405 | 50.4% | Builds clean |
