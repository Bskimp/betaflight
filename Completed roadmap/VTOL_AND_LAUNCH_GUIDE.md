# Betaflight VTOL Mixer Profiles & Wing Autolaunch Guide

This documents the custom features in the `Bskimp/betaflight` fork:
switchable mixer profiles for VTOL aircraft and hand-launch autolaunch
for fixed-wing. Both are fully `#ifdef` guarded and do not affect
quad/multirotor builds.

---

## Table of Contents

1. [VTOL Mixer Profiles](#vtol-mixer-profiles)
2. [Per-Profile IMU Orientation](#per-profile-imu-orientation)
3. [Transition Blending](#transition-blending)
4. [Failsafe Integration](#failsafe-integration)
5. [Wing Autolaunch](#wing-autolaunch)
6. [CLI Reference](#cli-reference)
7. [Example: Tailsitter Bicopter Config](#example-tailsitter-bicopter-config)
8. [Flashing & Applying Config](#flashing--applying-config)

---

## VTOL Mixer Profiles

Allows two independent motor/servo mixer configurations that can be
switched in flight via an AUX channel. Each profile defines its own
motor mix (`mmix`), servo mix (`smix`), mixer mode, and platform type.

### How It Works

- **Profile 0** defaults to `MIXER_CUSTOM` / `PLATFORM_MULTIROTOR`
- **Profile 1** defaults to `MIXER_CUSTOM_AIRPLANE` / `PLATFORM_AIRPLANE`
- A `BOXMIXERPROFILE` AUX mode switch selects the active profile
  (OFF = profile 0, ON = profile 1)
- Each mixer profile can be linked to a specific PID profile and rate
  profile, so tuning switches automatically with the flight mode

### Master Settings

These go in the master (global) section of your CLI config:

| Parameter | Range | Default | Description |
|-----------|-------|---------|-------------|
| `mixer_profile_count` | 1-2 | 1 | Set to 2 to enable VTOL profiles |
| `mixer_transition_time` | 0-10000 ms | 1500 | Duration of blended transition |
| `mixer_linked_pid_profile_1` | 0-N | 0 | PID profile for mixer profile 0 (1-based, 0=none) |
| `mixer_linked_pid_profile_2` | 0-N | 0 | PID profile for mixer profile 1 (1-based, 0=none) |
| `mixer_linked_rate_profile_1` | 0-N | 0 | Rate profile for mixer profile 0 (1-based, 0=none) |
| `mixer_linked_rate_profile_2` | 0-N | 0 | Rate profile for mixer profile 1 (1-based, 0=none) |

### CLI Commands

```
mixer_profile <index>    # Switch to mixer profile 0 or 1
mmix reset               # Reset motor mix for current mixer profile
mmix <i> <thr> <roll> <pitch> <yaw>   # Set motor mix row
smix reset               # Reset servo mix for current mixer profile
smix <rule> <servo> <source> <rate> <speed> <min> <max> <box>
```

**Important:** `mmix` and `smix` are profile-aware. You must first run
`mixer_profile <n>` before configuring that profile's mix.

### Servo Mix Source Channels

| Source | Axis |
|--------|------|
| 0 | Roll (stabilized) |
| 1 | Pitch (stabilized) |
| 2 | Yaw (stabilized) |
| 3 | Throttle |

### Servo Mix Box Values

| Box | Meaning |
|-----|---------|
| 0 | Always active |
| 1-3 | BOXSERVO1-3 |
| 4 | BOXMIXERTRANSITION (active only during profile transition) |

---

## Per-Profile IMU Orientation

Each mixer profile can apply an additional IMU rotation on top of the
board alignment. This is essential for tailsitters where the FC
orientation changes between hover (vertical) and forward flight
(horizontal).

| Parameter | Range | Unit | Description |
|-----------|-------|------|-------------|
| `mixer_imu_orientation_roll_1` | -1800 to 3600 | decidegrees | Roll rotation for profile 0 |
| `mixer_imu_orientation_pitch_1` | -1800 to 3600 | decidegrees | Pitch rotation for profile 0 |
| `mixer_imu_orientation_yaw_1` | -1800 to 3600 | decidegrees | Yaw rotation for profile 0 |
| `mixer_imu_orientation_roll_2` | -1800 to 3600 | decidegrees | Roll rotation for profile 1 |
| `mixer_imu_orientation_pitch_2` | -1800 to 3600 | decidegrees | Pitch rotation for profile 1 |
| `mixer_imu_orientation_yaw_2` | -1800 to 3600 | decidegrees | Yaw rotation for profile 1 |

Units are 0.1 degrees, so `900` = 90.0 degrees.

**Example:** A tailsitter with the FC mounted for forward flight needs
`mixer_imu_orientation_pitch_1 = 900` for hover mode (90 degree nose-up
rotation so the IMU treats vertical as "level").

---

## Transition Blending

When switching profiles while armed, the outputs blend smoothly over
`mixer_transition_time` milliseconds instead of snapping instantly.

- Motor outputs crossfade between the from-profile and to-profile mixes
- Servo outputs crossfade between the from-profile and to-profile rules
- PID/rate profiles switch immediately at transition start so the
  controller runs the target gains during the blend
- I-term is reset when the transition finalizes
- Progress is linear from 0.0 to 1.0

### Transition-Only Motors

The `motorTransitionOnly` flag in each mixer profile marks motors that
should only output during a transition (e.g., a pusher motor that only
runs while transitioning to forward flight). Set via the motor mix
configuration.

### OSD Element

`OSD_MIXER_PROFILE` shows the current state:
- `MP:0` or `MP:1` when idle
- `MP:0>1 50%` during transition (showing direction and progress)

### Debug Mode

`set debug_mode = MIXER_PROFILE` enables debug output:
- debug[0] = active mixer profile index
- debug[1] = target profile
- debug[2] = transition progress (0-1000)
- debug[3] = transition state

---

## Failsafe Integration

| Parameter | Range | Default | Description |
|-----------|-------|---------|-------------|
| `failsafe_mixer_profile` | 0-1 | 0 | Profile to switch to on failsafe |
| `failsafe_mixer_action` | 0-1 | 0 | 0 = immediate switch, 1 = hold current |

When failsafe activates and action is 0, any in-progress transition is
aborted and the mixer snaps immediately to `failsafe_mixer_profile`.
I-term is reset. This ensures the aircraft is in a known configuration
(typically hover) for failsafe landing.

---

## Wing Autolaunch

Hand-launch state machine for fixed-wing aircraft. Detects a throw via
the accelerometer, then automatically controls motor ramp-up and a timed
climb at a fixed pitch angle before handing control back to the pilot.

### State Machine

```
IDLE ──throw──> DETECTED ──> MOTOR_DELAY ──> MOTOR_RAMP ──> CLIMBING ──> TRANSITION ──> COMPLETE
                  │                                            │
                  └──────────── tilt/stick ──────────────> ABORT
```

1. **IDLE** — Armed with BOXLAUNCH active, waiting for throw
2. **DETECTED** — Accelerometer exceeded threshold
3. **MOTOR_DELAY** — Brief hold before motor starts (clear the hand)
4. **MOTOR_RAMP** — Throttle ramps from 0 to target
5. **CLIMBING** — Fixed pitch angle, target throttle, timed
6. **TRANSITION** — Blends from launch control to pilot input
7. **COMPLETE** — Normal flight, launch finished
8. **ABORT** — Roll exceeded limit or stick override triggered

### Activation

Assign `BOXLAUNCH` (mode ID 13) to an AUX channel in the Modes tab.
Arm with the launch switch ON. The state machine waits for throw
detection. After launch completes or is aborted, the mode can be
deactivated.

### Settings

All settings are per-PID-profile (set under `profile <n>`):

| Parameter | Range | Default | Description |
|-----------|-------|---------|-------------|
| `wing_launch_accel_thresh` | 10-100 | 25 | Throw detection in 0.1G (25 = 2.5G) |
| `wing_launch_motor_delay` | 0-500 ms | 100 | Delay after detection before motor starts |
| `wing_launch_motor_ramp` | 100-2000 ms | 500 | Time to ramp throttle to target |
| `wing_launch_throttle` | 25-100 % | 75 | Throttle during climb |
| `wing_launch_climb_time` | 1000-20000 ms | 3000 | Duration of climb phase |
| `wing_launch_climb_angle` | 10-60 deg | 45 | Fixed pitch angle during climb |
| `wing_launch_transition` | 200-3000 ms | 1000 | Blend time to pilot control |
| `wing_launch_max_tilt` | 5-90 deg | 45 | Max roll before abort |
| `wing_launch_idle_thr` | 0-25 % | 0 | Idle throttle while waiting for throw |
| `wing_launch_stick_override` | 0-100 % | 0 | Stick deflection % to cancel (0=disabled) |

### OSD Element

`OSD_WING_LAUNCH_STATUS` shows the current launch state and progress.

---

## CLI Reference

### SPA (Setpoint-dependent PID Attenuation) — Per-Profile

SPA mode settings are per-PID-profile (set under `profile <n>`).
This is important for VTOL: hover profiles typically want SPA OFF,
while airplane profiles use SPA to attenuate PID at high stick rates.

| Parameter | Values | Description |
|-----------|--------|-------------|
| `spa_roll_mode` | OFF, PD, PD_I_FREEZE | SPA mode for roll axis |
| `spa_pitch_mode` | OFF, PD, PD_I_FREEZE | SPA mode for pitch axis |
| `spa_yaw_mode` | OFF, PD, PD_I_FREEZE | SPA mode for yaw axis |
| `spa_roll_center` | 0-65535 | Roll rate center point (deg/s) |
| `spa_roll_width` | 0-65535 | Roll attenuation width |
| `spa_pitch_center` | 0-65535 | Pitch rate center point |
| `spa_pitch_width` | 0-65535 | Pitch attenuation width |
| `spa_yaw_center` | 0-65535 | Yaw rate center point |
| `spa_yaw_width` | 0-65535 | Yaw attenuation width |

**Warning:** With `spa_mode = PD_I_FREEZE` and `center = 0, width = 0`,
P and D terms will be multiplied by zero for any non-zero gyro rate.
This completely kills stabilization. Always set appropriate center/width
values when SPA is enabled, or set mode to OFF.

---

## Example: Tailsitter Bicopter Config

See `MICOAIR743_VTOL_DIFF.txt` for the complete working configuration.
Key concepts:

### Aircraft Layout
- 2 motors (differential thrust for yaw in hover, both forward in airplane)
- 3 servos (ailerons + elevator)
- FC mounted for forward flight, IMU rotation for hover

### Mixer Profile 0 — Bicopter Hover
```
mixer_profile 0
mmix reset
mmix 0  1.000  0.500  0.000  0.000   # left motor, roll authority
mmix 1  1.000 -0.500  0.000  0.000   # right motor, roll authority
smix reset
smix 0 3 2  100 0 0 100 0   # servo 3: yaw -> aileron L
smix 1 4 2 -100 0 0 100 0   # servo 4: yaw -> aileron R (reversed)
smix 2 2 1  100 0 0 100 0   # servo 2: pitch -> elevator
```

IMU rotation maps body axes for vertical hover:
- Virtual yaw = body roll (ailerons rotate around thrust axis)
- Virtual pitch = body pitch (elevator tilts nose)
- Virtual roll = body yaw (differential thrust)

### Mixer Profile 1 — Airplane Forward Flight
```
mixer_profile 1
mmix reset
mmix 0  1.000  0.000  0.000  0.300   # left motor, yaw via diff thrust
mmix 1  1.000  0.000  0.000 -0.300   # right motor

mixer_profile 1
smix reset
smix 0 3 0 -100 0 0 100 0   # servo 3: roll -> aileron L
smix 1 4 0 -100 0 0 100 0   # servo 4: roll -> aileron R
smix 2 2 1  100 0 0 100 0   # servo 2: pitch -> elevator
```

### Profile Linking
```
set mixer_profile_count = 2
set mixer_linked_pid_profile_1 = 1   # mixer 0 -> PID profile 0
set mixer_linked_pid_profile_2 = 2   # mixer 1 -> PID profile 1
set mixer_linked_rate_profile_1 = 1  # mixer 0 -> rate profile 0
set mixer_linked_rate_profile_2 = 2  # mixer 1 -> rate profile 1
```

### PID Profile Notes
- **Profile 0 (Hover):** `spa_*_mode = OFF`, `iterm_relax = RPY`,
  `yaw_type = DIFF_THRUST`
- **Profile 1 (Airplane):** `spa_*_mode = PD_I_FREEZE` with proper
  center/width values, wing launch settings configured

---

## Flashing & Applying Config

After building and flashing new firmware:

1. Connect to Configurator and open the CLI tab
2. Type `defaults` and press Enter — this resets all EEPROM settings
3. Paste your complete diff file (e.g., `MICOAIR743_VTOL_DIFF.txt`)
4. Type `save` and press Enter

**Why `defaults` first?** Flashing firmware does NOT reset EEPROM
settings. Old values persist and can conflict with new code. The
`defaults` command ensures a clean slate before applying your config.

**Backup first:** Before running `defaults`, you can save your current
config with `dump all` and save to a file. This gives you a complete
snapshot to diff against later.

### Known Limitation

`dump all` does not currently include per-mixer-profile mmix/smix
entries. To view them, manually run:
```
mixer_profile 0
mmix
smix
mixer_profile 1
mmix
smix
```
