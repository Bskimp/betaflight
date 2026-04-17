# Wing GPS Rescue (Return to Home)

## What it does

When GPS Rescue activates on a fixed-wing aircraft, the flight controller takes over and flies the plane home autonomously. The sequence is:

1. **Acquire heading** -- the aircraft levels out and builds enough airspeed for GPS course-over-ground (COG) to provide a reliable heading estimate.
2. **Climb** -- if below the return altitude, the aircraft commands nose-up pitch and climbs while maintaining heading toward home.
3. **Cruise** -- the aircraft navigates toward home at the configured return altitude using bank-to-turn steering.
4. **Orbit** -- once near home, the aircraft enters a circular loiter pattern at the orbit radius until the pilot regains control.

There is **no autonomous descent or landing**. The aircraft will orbit indefinitely until the pilot takes back control or the battery runs out. This is intentional -- autonomous landing for fixed-wing is risky and out of scope.

GPS Rescue can be triggered two ways:
- **Manual**: Assign the GPS Rescue mode to an AUX switch in the Modes tab.
- **Failsafe**: Set `failsafe_procedure = GPS-RESCUE` so it activates automatically on signal loss.

## Prerequisites

- A GPS module configured and getting a fix
- Accelerometer enabled
- `USE_WING` target (wing flight controller firmware)
- Sufficient satellites before arming (default: 8)
- A valid home point before GPS Rescue can activate

## CLI Parameters

All parameters are set via the CLI with `set <param> = <value>`.

### Navigation

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `gps_rescue_nav_p` | 30 | 0 -- 200 | Heading P gain. Higher = sharper turns toward home. Controls how aggressively the aircraft banks to correct heading error. |
| `gps_rescue_max_bank_angle` | 25 | 10 -- 45 | Maximum roll angle (degrees) the rescue will command during turns. Limits how steeply the aircraft banks. |
| `gps_rescue_min_heading_speed` | 400 | 100 -- 1000 | Minimum groundspeed (cm/s) for GPS course-over-ground to be considered a valid heading source. Below this speed, heading data is unreliable. |
| `gps_rescue_min_start_dist` | 30 | 10 -- 500 | Minimum distance from home (meters) to attempt a rescue. If closer than this, rescue will abort since there isn't enough room to navigate. |

### Altitude

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `gps_rescue_return_alt` | 50 | 10 -- 500 | Target cruise altitude (meters) during rescue. If the aircraft is below this altitude, rescue commands negative Betaflight pitch (nose-up) until it climbs close to the target. |
| `gps_rescue_min_loiter_alt` | 25 | 10 -- 500 | Minimum safe altitude (meters) for the orbit/loiter phase. The aircraft won't descend below this during orbit. |
| `gps_rescue_alt_p` | 30 | 0 -- 200 | Altitude P gain. Controls how aggressively pitch adjusts to correct altitude error. In Betaflight sign, negative pitch is nose-up and positive pitch is nose-down. |
| `gps_rescue_turn_compensation` | 50 | 0 -- 100 | Nose-up pitch added during banked turns to compensate for lift loss. At 0 there is no compensation; at 100 the full physics-based correction is applied. Higher values help maintain altitude in turns at the cost of slightly lower airspeed. |

### Throttle

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `gps_rescue_cruise_throttle` | 50 | 10 -- 90 | Throttle percentage during normal cruise flight. Set this to whatever keeps your aircraft at a comfortable cruise speed in level flight. |
| `gps_rescue_min_throttle` | 30 | 10 -- 80 | Minimum throttle percentage (stall prevention floor). The rescue will never command throttle below this value. Set above your aircraft's stall threshold. |
| `gps_rescue_abort_throttle` | 45 | 10 -- 80 | Throttle percentage during abort/recovery. Used when rescue encounters a failure and needs to maintain safe flight while the pilot takes over. |

### Orbit

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `gps_rescue_orbit_radius` | 50 | 20 -- 500 | Orbit radius (meters) for the loiter pattern at home. Larger values make a wider, gentler circle. Smaller values keep the aircraft closer but require tighter turns. Must be achievable at your max bank angle and airspeed. |

### Safety

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `gps_rescue_sanity_checks` | ON | OFF / ON / FS_ONLY | Enable sanity checks during rescue. ON monitors for stalls, fly-aways, and GPS loss and aborts on any failure. FS_ONLY only aborts when RX signal is also lost (hard failsafe). OFF disables abort routing. Strongly recommended to leave ON. |
| `gps_rescue_stall_speed` | 200 | 50 -- 1000 | Groundspeed threshold (cm/s) below which the rescue declares a stall. Must be sustained for ~5s before aborting. Set this above your wing's measured min-sink groundspeed but below its cruise speed. Default 200 cm/s (2 m/s) is conservative. |
| `gps_rescue_min_sats` | 8 | 5 -- 50 | Minimum satellite count required for rescue to be available. Below this count, GPS Rescue will show as unavailable in the OSD. |
| `gps_rescue_allow_arming_without_fix` | OFF | OFF / ON | Allow arming without a GPS fix. This only affects arming policy; GPS Rescue still remains unavailable until there is a current GPS fix, enough satellites, and a valid home point. |

### Sanity check behavior

When `gps_rescue_sanity_checks = ON`, four detectors run during active rescue and abort to recovery throttle + wings-level if any of them trip:

| Detector | Trigger | Dwell before abort |
|---|---|---|
| **GPS fix lost** | `GPS_FIX` state cleared | immediate (1 sanity tick, ~1s) |
| **Low satellite count** | `numSat < gps_rescue_min_sats` | 10s sustained |
| **Stall** | `groundSpeed < gps_rescue_stall_speed` | 5s sustained, evaluated every tick (phase-independent) |
| **No progress** | distance-to-home doesn't close by at least 5m | 12s sustained (CRUISE phase only) |

The stall counter is **phase-independent**: it accumulates in any active rescue phase rather than resetting on phase transitions. This matters because a real stall drops groundspeed below both the stall threshold and the heading-validity threshold, which would otherwise cause a CRUISE <-> ACQUIRE_HEADING revert cycle that resets the counter every loop and masks the stall indefinitely.

`FS_ONLY` is useful on wings where you want rescue to continue fighting bad conditions as long as you still have RX link; only when you lose both does the rescue give up.

## Recommended Starting Configuration

These defaults should work for most medium-sized fixed-wing aircraft (800mm--1500mm wingspan):

```
set gps_rescue_return_alt = 50
set gps_rescue_cruise_throttle = 50
set gps_rescue_min_throttle = 30
set gps_rescue_max_bank_angle = 25
set gps_rescue_orbit_radius = 50
set gps_rescue_nav_p = 30
set gps_rescue_alt_p = 30
set gps_rescue_turn_compensation = 50
set gps_rescue_sanity_checks = ON
set gps_rescue_stall_speed = 200
save
```

## Tuning Guide

### First flight checklist

1. Set `debug_mode = GPS_RESCUE_WING` and enable blackbox logging.
2. Fly to at least 100m away and 30m altitude before testing.
3. Activate GPS Rescue via AUX switch -- do NOT test via failsafe on the first flight.
4. Let the aircraft fly the full sequence (acquire heading, cruise home, orbit) at least once.
5. Deactivate the switch to regain control. Be ready to take over immediately if anything looks wrong.
6. Review the blackbox log before adjusting any parameters.

### Tuning by symptom

#### Aircraft won't turn hard enough toward home
The aircraft flies a very wide arc and takes a long time to point at home.
- Increase `gps_rescue_nav_p` (try 40-50).
- Increase `gps_rescue_max_bank_angle` (try 30-35). Steeper bank turns tighter but costs more lift.
- In blackbox, check `debug[2]` (heading error) -- if it stays large for many seconds, nav_p is too low.

#### Aircraft oscillates or S-turns during cruise
The aircraft weaves left-right instead of tracking a straight line home.
- Decrease `gps_rescue_nav_p` (try 20-25).
- This means the heading P gain is overcorrecting. Small heading errors cause large roll commands that overshoot.

#### Aircraft loses altitude during orbit / turns
The aircraft sinks during the loiter pattern or during the turn-to-home phase.
- Increase `gps_rescue_turn_compensation` (try 65-80). This adds more nose-up pitch during banked turns to offset the lift lost from banking. The physics: at 25 deg bank you lose ~9% of lift; at 35 deg you lose ~18%.
- Increase `gps_rescue_alt_p` (try 40-50). This makes the altitude correction loop react faster to errors.
- Increase `gps_rescue_cruise_throttle`. More power helps maintain altitude in turns.
- As a last resort, decrease `gps_rescue_max_bank_angle` to reduce the lift loss -- but this makes orbits wider.
- In blackbox, check `debug[4]` (pitch command). If it's going strongly negative (nose-up) but altitude still drops, the aircraft may need more throttle rather than more pitch.

#### Aircraft climbs or descends too aggressively
Large altitude overshoots above or below the target.
- Decrease `gps_rescue_alt_p` (try 15-20).
- The altitude D term (internal, not CLI-exposed) provides some damping, but if the P gain is too high the aircraft will porpoise around the target altitude.

#### Aircraft stalls during rescue
The aircraft runs out of airspeed and enters an uncontrolled descent.
- Increase `gps_rescue_min_throttle`. This is the absolute floor -- rescue will never go below this.
- Increase `gps_rescue_cruise_throttle`. If the aircraft is too slow at cruise power, it can't sustain turns.
- Decrease `gps_rescue_max_bank_angle`. Steep bank + low throttle = stall risk.
- In blackbox, check `debug[6]` (groundspeed). If it drops below ~800 cm/s (8 m/s) the aircraft is likely close to stall.
- Raise `gps_rescue_stall_speed` closer to your observed min-sink groundspeed so the rescue aborts sooner on stall. The default 200 cm/s is deliberately conservative to avoid false positives; many wings can safely detect stall at 300-500 cm/s.

#### Orbit is too tight or the aircraft can't complete the circle
The aircraft spirals inward or can't maintain the orbit distance.
- Increase `gps_rescue_orbit_radius`. The minimum achievable radius depends on speed and bank angle. At 15 m/s and 25 deg bank the physics minimum is ~44m. The firmware adds a 20% safety margin and will widen the orbit automatically if needed, but if the config radius is too small relative to airspeed the aircraft may struggle.
- Decrease `gps_rescue_max_bank_angle` if the orbit radius needs to stay small.

#### Aircraft bounces between orbit and cruise
The aircraft enters orbit, drifts outward, re-enters cruise, flies back in, enters orbit again -- repeating this cycle.
- This is **normal behavior**. The orbit uses hysteresis: it enters at the orbit radius and exits back to cruise at 1.5x the orbit radius. Some cycling is expected, especially in wind.
- To reduce the cycling: increase `gps_rescue_orbit_radius` so the orbit circle is larger and the aircraft has more room, or decrease `gps_rescue_max_bank_angle` so the orbit is gentler and the aircraft tracks the circle more precisely.

#### Heading is unreliable / aircraft wanders before turning home
The heading acquisition phase takes too long or the aircraft turns the wrong way initially.
- Increase `gps_rescue_min_heading_speed` (try 500-600). The aircraft needs to be moving fast enough for GPS course-over-ground to be accurate. Slow-flying wings or high-wind conditions need a higher threshold.
- Make sure the aircraft has airspeed before activating rescue. The heading acquisition phase will wait until groundspeed exceeds this threshold before trusting the heading.

#### Rescue aborts immediately after activation
The aircraft goes straight to ABORT phase.
- Check distance to home. If you're closer than `gps_rescue_min_start_dist` (default 30m), rescue will abort because there isn't enough room to navigate.
- Check satellite count. If below `gps_rescue_min_sats`, rescue isn't available.
- In blackbox, check `debug[0]` (phase). Phase 7 = ABORT. Check `debug[5]` (distance) to see how far from home you were.

### Parameter interaction summary

Some parameters interact and should be tuned together:

- **Bank angle + orbit radius + speed**: Steeper banks allow tighter orbits but cost more altitude. The firmware auto-widens the orbit if the physics minimum radius exceeds your config. Check `debug[5]` during orbit to see actual distances.
- **Alt P + turn compensation**: Both fight altitude loss in turns. Turn compensation is feedforward (prevents the drop); alt_p is feedback (corrects after the drop). Increase turn_compensation first since it's proactive.
- **Cruise throttle + min throttle**: Cruise throttle sets the baseline; the firmware adds extra throttle automatically during climbs. Min throttle is the absolute safety floor for stall prevention.
- **Nav P + max bank angle**: Nav P controls how aggressively heading error maps to roll. Max bank angle caps the roll. If nav_p is high but max bank is low, the system will hit the bank limit frequently -- which is fine but means turns are rate-limited by the bank cap.

## Blackbox Debug Channels

Set `debug_mode = GPS_RESCUE_WING` to log rescue internals. The 8 debug fields are:

| Channel | Field | Units | Notes |
|---------|-------|-------|-------|
| `debug[0]` | Phase | 0-8 | 0=IDLE, 1=INIT, 2=ACQ_HDG, 3=CLIMB, 4=TURN_HOME, 5=CRUISE, 6=ORBIT, 7=ABORT, 8=COMPLETE |
| `debug[1]` | Heading valid | 0/1 | 1 when GPS COG heading is trusted |
| `debug[2]` | Heading error | decidegrees | Signed error between current heading and target |
| `debug[3]` | Roll command | decidegrees | Commanded roll / 10. Negative = left bank |
| `debug[4]` | Pitch command | decidegrees | Commanded pitch / 10. Negative = nose-up (climb), positive = nose-down |
| `debug[5]` | Distance to home | meters | Straight-line distance to home point |
| `debug[6]` | Ground speed | cm/s | GPS ground speed |
| `debug[7]` | Throttle | 0-1000 | Commanded throttle x 1000 (e.g. 500 = 50%) |

### Reading the blackbox

- **Phase progression**: A healthy rescue shows 0 → 2 → 3 → 4 → 5 → 6 (IDLE → ACQ_HDG → CLIMB → TURN → CRUISE → ORBIT). If you see phase 7 (ABORT), something went wrong.
- **Heading error** should decrease during TURN_HOME and CRUISE as the aircraft converges on the home bearing.
- **Distance** should decrease steadily during CRUISE and stabilize around the orbit radius during ORBIT.
- **Pitch command**: Watch for sustained large negative values (nose-up) -- this means the aircraft is fighting to maintain altitude. Consider increasing turn_compensation or cruise_throttle.
- **Throttle**: If it's pegged near 1000 during cruise, the aircraft is underpowered for the configured return altitude and bank angle.

## OSD

The OSD shows the current rescue phase as a 4-character code when GPS Rescue is active:

`IDLE` / `INIT` / `ACQH` / `CLMB` / `TURN` / `CRSE` / `ORBT` / `ABRT` / `DONE`

Enable the **GPS Rescue Phase** OSD element in the configurator to see this in flight.

Additional OSD warnings:
- **RESCUE N/A** -- GPS rescue is configured but not currently available (insufficient sats, no current GPS fix, or no home point).
- **RESCUE OFF** -- GPS rescue is not configured (missing sensors).
