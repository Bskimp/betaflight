# Betaflight VTOL Mixer Profiles Guide

Switchable motor/servo mixer profiles for VTOL aircraft in the
`Bskimp/betaflight` fork. Fully `#ifdef USE_VTOL` guarded — does not
affect quad/multirotor builds.

---

## Table of Contents

1. [Overview](#overview)
2. [Master Settings](#master-settings)
3. [CLI Commands](#cli-commands)
4. [Servo Mix Reference](#servo-mix-reference)
5. [Per-Profile IMU Orientation](#per-profile-imu-orientation)
6. [Transition Blending](#transition-blending)
7. [Failsafe Integration](#failsafe-integration)
8. [SPA (Per-Profile PID Attenuation)](#spa-per-profile-pid-attenuation)
9. [Example: Tailsitter Bicopter Config](#example-tailsitter-bicopter-config)
10. [Board-Specific Notes](#board-specific-notes)
11. [Flashing & Applying Config](#flashing--applying-config)
12. [Troubleshooting](#troubleshooting)

---

## Overview

Allows two independent motor/servo mixer configurations that can be
switched in flight via an AUX channel. Each profile defines its own
motor mix (`mmix`), servo mix (`smix`), mixer mode, and platform type.

- **Profile 0** defaults to `MIXER_CUSTOM` / `PLATFORM_MULTIROTOR`
- **Profile 1** defaults to `MIXER_CUSTOM_AIRPLANE` / `PLATFORM_AIRPLANE`
- A `BOXMIXERPROFILE` AUX mode switch selects the active profile
  (OFF = profile 0, ON = profile 1)
- Each mixer profile can be linked to a specific PID profile and rate
  profile, so tuning switches automatically with the flight mode

---

## Master Settings

These go in the master (global) section of your CLI config:

| Parameter | Range | Default | Description |
|-----------|-------|---------|-------------|
| `mixer_profile_count` | 1-2 | 1 | Set to 2 to enable VTOL profiles |
| `mixer_transition_time` | 0-10000 ms | 1500 | Duration of blended transition |
| `mixer_linked_pid_profile_1` | 0-N | 0 | PID profile for mixer profile 0 (1-based, 0=none) |
| `mixer_linked_pid_profile_2` | 0-N | 0 | PID profile for mixer profile 1 (1-based, 0=none) |
| `mixer_linked_rate_profile_1` | 0-N | 0 | Rate profile for mixer profile 0 (1-based, 0=none) |
| `mixer_linked_rate_profile_2` | 0-N | 0 | Rate profile for mixer profile 1 (1-based, 0=none) |

---

## CLI Commands

```
mixer_profile <index>    # Switch to mixer profile 0 or 1
mmix reset               # Reset motor mix for current mixer profile
mmix <i> <thr> <roll> <pitch> <yaw>   # Set motor mix row
smix reset               # Reset servo mix for current mixer profile
smix <rule> <servo> <source> <rate> <speed> <min> <max> <box>
```

**Important:** `mmix` and `smix` are profile-aware. You must first run
`mixer_profile <n>` before configuring that profile's mix.

---

## Servo Mix Reference

### Servo Index Mapping

**Critical:** The `<servo>` field in `smix` is an index into the
internal `servo[]` array, NOT a direct physical output number. For VTOL
and airplane modes, `writeServos()` maps physical outputs starting from
`SERVO_PLANE_INDEX_MIN` (= index 2):

| servo[] index | Enum name | Physical output |
|---------------|-----------|-----------------|
| 0 | SERVO_GIMBAL_PITCH | Not used in VTOL path |
| 1 | SERVO_GIMBAL_ROLL | Not used in VTOL path |
| **2** | **SERVO_FLAPS** | **Physical output 0 → SERVO1 pin** |
| **3** | **SERVO_FLAPPERON_1** | **Physical output 1 → SERVO2 pin** |
| **4** | **SERVO_FLAPPERON_2** | **Physical output 2 → SERVO3 pin** |
| 5 | SERVO_RUDDER | Physical output 3 → SERVO4 pin |
| 6 | SERVO_ELEVATOR | Physical output 4 → SERVO5 pin |
| 7 | SERVO_THROTTLE | Physical output 5 → SERVO6 pin |

**Use indices 2-7 in smix rules.** Indices 0-1 are gimbal servos and
will never reach physical outputs in the VTOL code path.

### Source Channels

| Source | Axis |
|--------|------|
| 0 | Roll (stabilized PID output) |
| 1 | Pitch (stabilized PID output) |
| 2 | Yaw (stabilized PID output) |
| 3 | Throttle |

### Box Values

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

### How It Works

The rotation chain for sensor data is:
1. **Sensor alignment** — compensates for gyro chip orientation on the PCB
   (defined in target config, e.g., `GYRO_1_ALIGN = CW270_DEG`)
2. **Board alignment** — compensates for FC mounting angle in the airframe
   (`align_board_roll/pitch/yaw`)
3. **VTOL rotation** — remaps axes for the current flight mode
   (`mixer_imu_orientation_*`)

The VTOL rotation operates on data that has **already been corrected**
by sensor and board alignment. This means the VTOL rotation values
should be the same regardless of your `align_board_yaw` setting.

**For a tailsitter (nose-up hover):**
```
set mixer_imu_orientation_roll_1 = 0
set mixer_imu_orientation_pitch_1 = 900
set mixer_imu_orientation_yaw_1 = 0
```

When all three values are zero (e.g., airplane profile), the VTOL
rotation is skipped entirely — no performance cost.

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
runs while transitioning to forward flight).

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

## SPA (Per-Profile PID Attenuation)

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

See `MICOAIR743_VTOL_DIFF.txt` and `SPEEDYBEEF405WING_VTOL_DIFF.txt`
for complete working configurations. Key concepts:

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
smix reset
smix 0 3 0  100 0 0 100 0   # servo 3: roll -> aileron L
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

## Board-Specific Notes

### Gyro Chip Alignment vs Board Alignment

Many FC boards have `GYRO_1_ALIGN` set in their target config (e.g.,
`CW270_DEG`) to compensate for the gyro chip's physical orientation on
the PCB. This is separate from `align_board_yaw` which compensates for
how the FC is mounted in the airframe.

**Do not double-rotate.** If your target already defines `GYRO_1_ALIGN`,
the chip orientation is already handled. Set `align_board_yaw` only if
you physically rotated the FC from its default mounting orientation.

To check your target's gyro alignment, look in:
```
src/config/configs/<BOARD_NAME>/config.h
```

| Board | GYRO_1_ALIGN | Default board_yaw needed |
|-------|-------------|--------------------------|
| MICOAIR743 | None (CW0_DEG) | Set to match physical mounting |
| SPEEDYBEEF405WING | CW270_DEG | 0 if default mounting |

### Verifying Axis Mapping

If disturbance response is wrong (servos correct the wrong axis):

1. **Test airplane mode first** — it has no VTOL rotation, so any axis
   errors are purely from board/sensor alignment
2. **Check `align_board_yaw`** — make sure you're not double-rotating
   with the target's `GYRO_1_ALIGN`
3. **Then test hover** — VTOL rotation should just be `pitch_1 = 900`
   for any tailsitter, regardless of board alignment

**Stick inputs vs disturbances:** In stabilized mode, sticks feed the
PID setpoint while disturbances come from gyro data. If sticks work but
disturbances are on the wrong axis, the problem is in the rotation chain
(sensor align → board align → VTOL rotation), not the smix mapping.

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
config with `dump all` and save to a file.

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

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Servos don't move at all | smix using indices 0-1 (gimbal range) | Use indices 2-7 for smix servo field |
| Sticks correct, disturbances on wrong axis | Board alignment double-rotation | Check `GYRO_1_ALIGN` in target config, adjust `align_board_yaw` |
| Hover yaw corrects wrong direction | smix aileron signs inverted | Swap signs on BOTH aileron smix rules |
| Elevator responds to roll in hover only | Wrong VTOL rotation | Use only `pitch_1 = 900`, set `roll_1 = 0` and `yaw_1 = 0` |
| SPA kills stabilization | center=0, width=0 with PD_I_FREEZE | Set proper center/width values, or use `spa_*_mode = OFF` |
| Servos appear on wrong Configurator bars | Normal — bars offset by SERVO_PLANE_INDEX_MIN | Check `channel_forwarding_start` and servo tab layout |
