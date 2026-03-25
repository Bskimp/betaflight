# Betaflight Wing Autolaunch Guide

Hand-launch state machine for fixed-wing aircraft in the
`Bskimp/betaflight` fork. Fully `#ifdef USE_WING_LAUNCH` guarded —
does not affect quad/multirotor builds.

---

## Overview

Detects a throw via the accelerometer, then automatically controls
motor ramp-up and a timed climb at a fixed pitch angle before handing
control back to the pilot.

---

## State Machine

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

---

## Activation

Assign `BOXLAUNCH` (mode ID 13) to an AUX channel in the Modes tab.
Arm with the launch switch ON. The state machine waits for throw
detection. After launch completes or is aborted, the mode can be
deactivated.

---

## Settings

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
| `wing_launch_stick_override` | 0-100 % | 0 | Stick deflection % to cancel launch (0=disabled) |

---

## Features

### Throw Detection
Accelerometer-based detection triggers when sustained acceleration
exceeds `wing_launch_accel_thresh` (in 0.1G units). The threshold
should be high enough to avoid false triggers from handling the
aircraft but low enough for a normal throw.

### Throttle Gate
Motors do not start until the motor delay timer expires after throw
detection. This ensures the propeller is clear of the pilot's hand.

### Timer Clamp
The climb timer does not start counting until motors are actually
running, ensuring the full climb duration is available.

### Stick Override
When `wing_launch_stick_override > 0`, the pilot can cancel the
launch at any time by deflecting any stick beyond the threshold
percentage. This immediately transitions to COMPLETE state, giving
the pilot full control.

### Elevon Pre-Deflection
When BOXLAUNCH is active while disarmed, the servos deflect to a
pitch-up position. This provides a better launch angle when the
aircraft is thrown. On arm, the servos smoothly transition from
pre-deflection to PID control.

### Smooth Throttle Handoff
During the TRANSITION phase, throttle blends smoothly from the
launch target to the pilot's stick position over the transition
time. This avoids abrupt throttle changes at handoff.

---

## OSD Element

`OSD_WING_LAUNCH_STATUS` shows the current launch state and countdown
timer during climb phase.

---

## Debug Mode

`set debug_mode = WING_LAUNCH` enables debug output with 8 channels
showing state, timers, accel data, and throttle values.

---

## Example Configuration

```
# Assign BOXLAUNCH to AUX channel
aux 3 13 5 1700 2100 0 0         # LAUNCH on AUX6

# Launch settings (in PID profile for airplane)
profile 1
set wing_launch_throttle = 35
set wing_launch_climb_time = 6000
set wing_launch_climb_angle = 15
set wing_launch_max_tilt = 60
set wing_launch_stick_override = 50
```

### Typical Workflow

1. Power on, confirm LAUNCH switch is OFF
2. Place aircraft in launch position, turn LAUNCH switch ON
3. Servos pre-deflect to pitch-up — ready for throw
4. Arm (switch or stick command)
5. Throw the aircraft
6. Autolaunch detects throw → motors ramp → climb → transition
7. Take over control after transition completes
8. Turn LAUNCH switch OFF (optional, mode auto-completes)
