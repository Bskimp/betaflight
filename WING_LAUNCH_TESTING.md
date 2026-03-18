# Wing Autolaunch — Testing Protocol (Phase 6)

## Prerequisites

- Firmware built with `USE_WING_LAUNCH` enabled (SPEEDYBEEF405WING or similar wing target)
- Betaflight Configurator connected
- AUTOLAUNCH mode assigned to a switch in the Modes tab
- OSD element `WING_LAUNCH_STATUS` positioned on screen
- Blackbox `debug_mode = WING_LAUNCH` set for debug logging

---

## CLI Parameters Reference

```
set wing_launch_accel_thresh = 25   # G-force threshold x10 (25 = 2.5G) [10-100]
set wing_launch_motor_delay  = 100  # ms before motor starts after throw [0-500]
set wing_launch_motor_ramp   = 500  # ms to ramp motor to full launch throttle [100-2000]
set wing_launch_throttle     = 75   # launch throttle percent [25-100]
set wing_launch_climb_time   = 3000 # ms of powered climb [1000-10000]
set wing_launch_climb_angle  = 45   # pitch angle in degrees during climb [10-60]
set wing_launch_transition   = 1000 # ms to blend back to pilot control [200-3000]
set wing_launch_max_tilt     = 45   # abort if roll or pitch exceeds this [5-90]
set wing_launch_idle_thr     = 0    # idle throttle percent while waiting [0-25]
```

---

## Part A — Bench Testing (PROPS OFF)

> **REMOVE ALL PROPELLERS before bench testing. No exceptions.**

### Test 1: Throw Detection

**Setup:** Arm with AUTOLAUNCH switch ON. Hold the wing in hand.

**Steps:**
1. Observe OSD shows `LNCH RDY`
2. Gently rock the wing — should NOT trigger (below 2.5G)
3. Do a practice throw motion (quick forward acceleration, catch immediately)
4. OSD should flash `THROW!` then `LNCH GO`

**Adjust:** If too sensitive, increase `wing_launch_accel_thresh`. If not triggering, decrease it. Default 25 (2.5G) is a good starting point — a real hand throw typically hits 3-5G.

**Pass criteria:**
- [ ] No false triggers from gentle handling
- [ ] Reliably detects a throw-strength acceleration
- [ ] OSD shows correct state transitions

### Test 2: Motor Sequence (PROPS OFF!)

**Setup:** Arm with AUTOLAUNCH switch ON.

**Steps:**
1. Trigger a throw (practice throw or shake firmly)
2. Listen/watch for motor response:
   - Motor should stay OFF during delay period (100ms default)
   - Motor should ramp up smoothly over ramp period (500ms default)
   - Motor should run at launch throttle for climb time (3000ms default)
   - Motor should wind down during transition (1000ms default)
3. After full sequence, OSD element should disappear (COMPLETE state)

**Pass criteria:**
- [ ] Motor delay is noticeable (no instant start)
- [ ] Motor ramp is smooth (not a sudden jump)
- [ ] Motor runs for approximately the climb time duration
- [ ] Motor stops after transition completes
- [ ] Total sequence timing feels right

### Test 3: Abort on Switch Off

**Setup:** Arm with AUTOLAUNCH switch ON.

**Steps:**
1. Trigger a throw to start the launch sequence
2. While motor is ramping or at full throttle, flip AUTOLAUNCH switch OFF
3. Motor should stop immediately — no delay, no ramp-down

**Pass criteria:**
- [ ] Instant motor stop on switch off during MOTOR_RAMP
- [ ] Instant motor stop on switch off during CLIMBING
- [ ] OSD shows `ABORT` briefly

### Test 4: Abort on Excessive Tilt

**Setup:** Mount FC on a board you can tilt. Arm with AUTOLAUNCH switch ON.

**Steps:**
1. Trigger detection by shaking
2. While in CLIMBING state, tilt past `wing_launch_max_tilt` (default 45 degrees)
3. Motor should stop, OSD should show abort

**Test both axes:**
- Roll past 45 degrees → should abort
- Pitch nose-down past -45 degrees → should abort

**Pass criteria:**
- [ ] Roll abort triggers correctly
- [ ] Pitch dive abort triggers correctly
- [ ] Abort is immediate (no delay)

### Test 5: Attitude Hold (Servo Response)

**Setup:** Wing with servos connected, PROPS OFF. Arm with AUTOLAUNCH switch ON.

**Steps:**
1. Trigger a throw
2. Observe servo positions:
   - Elevator should deflect to climb position
   - Ailerons should center (wings level)
3. Tilt the wing in roll — servos should fight to stay level
4. During TRANSITION state, servo authority should gradually return to sticks

**Pass criteria:**
- [ ] Elevator holds climb angle
- [ ] Roll is stabilized (wings level)
- [ ] Transition smoothly returns control to sticks

### Test 6: Mid-Flight Activation Block

**Setup:** Arm with AUTOLAUNCH switch OFF.

**Steps:**
1. Arm the aircraft
2. Wait more than 3 seconds
3. Flip AUTOLAUNCH switch ON
4. Shake/accelerate the wing — should NOT enter launch sequence

**Pass criteria:**
- [ ] Launch does not activate when armed > 3 seconds
- [ ] OSD does not show launch states

### Test 7: Disarm Reset

**Steps:**
1. Arm with AUTOLAUNCH ON, trigger a throw, let sequence run partway
2. Disarm
3. Re-arm with AUTOLAUNCH ON
4. System should be back in IDLE (`LNCH RDY`), ready for a new throw

**Pass criteria:**
- [ ] State resets to IDLE after disarm
- [ ] Fresh arm cycle works correctly

### Test 8: Blackbox Verification

**Steps:**
1. Run through a full launch sequence on the bench
2. Download blackbox log
3. Open in Blackbox Explorer, check:
   - Header contains all `wing_launch_*` parameters
   - `debug[0]` shows accel magnitude (should spike during throw)
   - `debug[1]` shows state transitions (0→1→2→3→4→5→6)

**Pass criteria:**
- [ ] All parameters logged in header
- [ ] Debug values show clear state machine progression
- [ ] Timing matches configured values

---

## Part B — First Flight Testing

> **Only proceed after ALL bench tests pass.**
> **Fly in an open field, low wind, with a spotter.**
> **Keep finger on AUTOLAUNCH switch to abort at any time.**

### Conservative First-Flight Settings

```
set wing_launch_accel_thresh = 25   # keep default
set wing_launch_motor_delay  = 100  # keep default
set wing_launch_motor_ramp   = 500  # keep default
set wing_launch_throttle     = 50   # START LOW — just enough to maintain flight
set wing_launch_climb_angle  = 15   # gentle climb, not aggressive
set wing_launch_climb_time   = 3000 # 3 seconds is plenty to start
set wing_launch_transition   = 1500 # generous transition time
set wing_launch_max_tilt     = 35   # tight safety margin
set wing_launch_idle_thr     = 0    # no idle throttle
```

### Flight 1: Gentle Throw

**Steps:**
1. Arm, confirm `LNCH RDY` on OSD
2. Hold wing overhead, slight nose-up attitude
3. Throw gently but firmly, straight ahead, into the wind
4. Watch for:
   - Motor starts after brief delay
   - Wing pitches up gently (15 degrees)
   - Wing climbs straight
   - After 3s, `RECOVER` appears and control returns
5. Fly normally, land, review blackbox

**What to watch for:**
- Does it climb straight or veer off? → adjust trims/PIDs
- Is 50% throttle enough? → increase `wing_launch_throttle`
- Does transition feel abrupt? → increase `wing_launch_transition`
- Does it feel too gentle? → increase `wing_launch_climb_angle` (later)

### Flight 2-5: Iterate Settings

Gradually adjust ONE parameter at a time:

1. **Throttle:** Increase `wing_launch_throttle` in steps of 10 (50→60→70→75)
2. **Climb angle:** Increase `wing_launch_climb_angle` in steps of 5 (15→20→25→30)
3. **Climb time:** Try longer climbs (3000→4000→5000)
4. **Safety margin:** Only loosen `wing_launch_max_tilt` if you're getting false aborts

### Flight Progression Checklist

- [ ] Flight 1: 50% throttle, 15 degree climb — confirms basic function
- [ ] Flight 2: 60% throttle, 20 degree climb — more authority
- [ ] Flight 3: 70% throttle, 25 degree climb — approaching normal settings
- [ ] Flight 4: 75% throttle, 30 degree climb — near-final tuning
- [ ] Flight 5: Final settings locked in, clean launch

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| No detection on throw | Threshold too high | Lower `wing_launch_accel_thresh` |
| False triggers on handling | Threshold too low | Raise `wing_launch_accel_thresh` |
| Motor starts too early | Delay too short | Increase `wing_launch_motor_delay` |
| Motor jump (no smooth ramp) | Ramp too short | Increase `wing_launch_motor_ramp` |
| Wing rolls during climb | PID tuning or CG issue | Check wing CG, increase roll PIDs |
| Climb too steep/shallow | Angle setting | Adjust `wing_launch_climb_angle` |
| Abrupt transition | Transition too short | Increase `wing_launch_transition` |
| False abort during climb | Max tilt too tight | Increase `wing_launch_max_tilt` |
| Won't activate after arm | Armed > 3 seconds | Must activate within 3s of arming |

---

## After Testing

Once you have reliable settings:
1. Save your tuned values: `diff` in CLI and record the `wing_launch_*` lines
2. Consider posting results in BF Discord #limon-wing-channel
3. Blackbox logs from successful launches are valuable for PR review evidence
