# Wing GPS Rescue: Heading Acquisition Strategy

**Status:** Draft heuristics, tuned from early flight data
**Date:** 2026-03-25

---

## The Problem

When GPS Rescue activates on a fixed wing, the IMU yaw estimate may be
unreliable. Unlike a multirotor that can rotate in place to align heading,
a wing must fly forward before GPS course-over-ground (COG) becomes a useful
heading source.

GPS COG is only meaningful when:
1. The aircraft has enough groundspeed
2. The aircraft is flying roughly straight
3. The COG has stayed stable long enough for the IMU to converge

## How Betaflight Uses COG During Rescue

From `imu.c`, rescue overrides the normal groundspeed-based COG gain and uses
`gpsRescueGetImuYawCogGain()` instead. That means wing rescue controls how
aggressively the IMU trusts GPS heading during each rescue phase.

- Higher gain: faster convergence, more sensitivity to noisy COG
- Lower gain: slower convergence, less noise sensitivity

The existing IMU code already does the quaternion update. Wing rescue only
needs to choose the right gain and decide when heading is trustworthy enough
to start navigating.

## Wing Heading Acquisition Strategy

### Phase: `RESCUE_WING_ACQUIRE_HEADING`

**Entry conditions:** Rescue just activated. Home point is frozen. Aircraft may
be at any attitude and heading.

**Behavior during this phase:**
1. **Wings level** - command 0 deg roll via `gpsRescueAngle[AI_ROLL]`
2. **Gentle climb pitch** - command a small negative Betaflight pitch value
   so the aircraft holds nose-up attitude, gains altitude, and keeps airspeed
3. **Cruise throttle** - maintain airspeed floor without aggressive thrust steps
4. **Do not turn** - keep flying the current track until COG is trustworthy
5. **High COG gain** - let the IMU lock onto GPS heading more quickly

In Betaflight pitch sign, negative pitch is nose-up and positive pitch is
nose-down. A climb command therefore appears as a negative rescue pitch value.

### COG Gain Schedule

| Rescue Phase | COG Gain | Rationale |
|-------------|----------|-----------|
| `ACQUIRE_HEADING` | 3.0 | Faster yaw convergence while flying straight |
| `CLIMB` | 2.5 | Moderate trust while climbing toward target altitude |
| `TURN_TO_HOME` | 1.5 | Lower trust while banking toward home |
| `CRUISE` | 2.5 | Moderate trust during straight navigation |
| `ORBIT` | 1.5 | Lower trust during continuous banking |
| `ABORT` / idle | 1.0 | Neutral fallback |

## Heading Validity Criteria

Heading is considered valid when two things are true for a sustained duration:

**Condition 1: Sufficient groundspeed**
```c
gpsSol.groundSpeed >= minHeadingSpeedCmS
```

**Condition 2: Stable COG**
```c
spread(cogHistory) < 150 decidegrees
```

Implementation summary:
1. Store recent `gpsSol.groundCourse` samples in a circular buffer
2. Compute wrapped angular spread across the buffer
3. Require speed and spread to stay good for about 1 second before declaring
   heading valid
4. Once valid, require about 2 seconds of sustained bad data before declaring
   it invalid again

That hysteresis avoids phase flapping from short GPS dropouts.

## Heading Validity Loss Mid-Rescue

Heading can become invalid after acquisition if:
- Speed drops below threshold
- COG becomes unstable because of turbulence or bad GPS velocity data

Current implementation behavior:
1. `CLIMB`, `TURN_TO_HOME`, and `CRUISE` recheck heading validity every tick
2. Brief bad data is tolerated through the invalidity dwell timer
3. If invalidity persists, rescue re-enters `ACQUIRE_HEADING`
4. On re-entry, smoothing is re-seeded and the aircraft levels the wings again
   while the IMU re-locks to GPS COG

## Timeout

If heading never becomes valid within `HEADING_ACQUIRE_TIMEOUT_S`
(currently 10 seconds), rescue transitions to `ABORT`.

That is intentionally generous. A healthy fixed wing should usually build
enough speed and COG stability well before that.

## Interaction with Existing IMU Code

The important design point is that wing rescue does not need to change
`imu.c`:

1. `gpsRescueGetImuYawCogGain()` provides the phase-specific gain
2. `gpsRescueDisableMag()` keeps rescue on COG-based heading instead of mag
3. The existing Mahony AHRS code handles the actual yaw convergence

## Orbit Direction Selection

On the first `CRUISE -> ORBIT` transition, the rescue picks whether to orbit clockwise or counter-clockwise based on the **sign of the current heading error** (target-to-home vs current COG). This picks the shorter turn into orbit rather than biasing toward whichever way the plane happened to be banked at the transition instant.

A previous implementation used the sign of `targetRollCd` (the currently commanded bank). Because approach geometry tends to produce the same residual bank flight after flight -- final turn-to-home is often the same direction given a consistent launch spot + prevailing wind -- orbit would lock into the same direction every RTH, even when the shorter turn was the other way. Using `headingErr` sign removes that approach-geometry bias.

The chosen direction is then **sticky** for the remainder of the rescue: subsequent `ORBIT -> CRUISE -> ORBIT` cycles keep the original direction so the orbit doesn't flip if the plane jitters across the re-entry threshold.

## Constants Summary

| Constant | Value | Purpose |
|----------|-------|---------|
| `minHeadingSpeedCmS` | 400 cm/s default | Minimum groundspeed for valid heading |
| `COG_HISTORY_SIZE` | 10 samples | Sliding window for COG stability |
| `COG_STABILITY_THRESHOLD` | 150 decideg | Max wrapped spread for stable heading |
| `HEADING_VALID_DWELL_TICKS` | 10 ticks | About 1 second of good heading before valid |
| `HEADING_INVALID_DWELL_TICKS` | 20 ticks | About 2 seconds of bad heading before invalid |
| `HEADING_ACQUIRE_TIMEOUT_S` | 10 s | Maximum time allowed to acquire heading |

## Flight-Test Questions

1. Is 4 m/s the right minimum speed for a wide range of wings?
2. Is 15 deg of wrapped COG spread the right stability threshold in wind?
3. Does the current COG gain schedule converge quickly without overreacting
   during turns?
4. Should orbit eventually gain the same heading-loss reacquire behavior as
   climb/turn/cruise, or is the current orbit handling sufficient?
