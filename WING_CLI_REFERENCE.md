# Wing Feature CLI Reference

All CLI parameters introduced by the three wing features on the `wing-combined` branch.
Gated behind `USE_WING` (and `USE_GPS_RESCUE` / `USE_WING_LAUNCH` where noted).

---

## 1. Servo Autotrim (`USE_WING`)

Activated via mode switch: `BOXAUTOTRIM`

No CLI parameters. All thresholds are compile-time constants:

| Constant                   | Value    | Description                              |
|----------------------------|----------|------------------------------------------|
| AUTOTRIM_MAX_OFFSET        | 100 us   | Max servo offset applied per channel     |
| AUTOTRIM_MIN_SAMPLES       | 200      | Min samples before applying (~2s)        |
| AUTOTRIM_STICK_DEADBAND    | 50       | ~5% stick deadband for sampling          |
| AUTOTRIM_MAX_ATT_DECI      | 300      | Max attitude for sampling (30 deg)       |
| AUTOTRIM_MAX_GYRO_RATE     | 15.0 d/s | Max gyro rate for sampling               |
| AUTOTRIM_RESULT_DISPLAY_MS | 3000 ms  | OSD result display duration              |

---

## 2. Wing Autolaunch (`USE_WING_LAUNCH`)

Profile-scoped parameters (`set` in CLI, stored per PID profile).

| CLI Parameter                  | Type   | Min  | Max   | Default | Unit / Notes                        |
|--------------------------------|--------|------|-------|---------|-------------------------------------|
| wing_launch_accel_thresh       | uint8  | 10   | 100   | 25      | 0.1G units (25 = 2.5G throw detect)|
| wing_launch_motor_delay        | uint16 | 0    | 500   | 100     | ms, delay before motor start        |
| wing_launch_motor_ramp         | uint16 | 100  | 2000  | 500     | ms, motor ramp-up time              |
| wing_launch_throttle           | uint8  | 25   | 100   | 75      | %, climb throttle                   |
| wing_launch_climb_time         | uint16 | 1000 | 20000 | 3000    | ms, climb phase duration            |
| wing_launch_climb_angle        | int16  | 10   | 60    | 45      | degrees, pitch angle during climb   |
| wing_launch_transition         | uint16 | 200  | 3000  | 1000    | ms, blend time to pilot control     |
| wing_launch_max_tilt           | uint8  | 5    | 90    | 45      | degrees, abort if roll exceeds this |
| wing_launch_idle_thr           | uint8  | 0    | 25    | 0       | %, throttle while awaiting throw    |
| wing_launch_stick_override     | uint8  | 0    | 100   | 0       | %, stick deflection to abort (0=off)|

---

## 3. GPS Rescue - Wing (`USE_WING` + `USE_GPS_RESCUE`)

Master-scoped parameters (PG_GPS_RESCUE, version 9).
These replace the multirotor GPS rescue params when `USE_WING` is defined.

| CLI Parameter                       | Type   | Min | Max  | Default | Unit / Notes                          |
|-------------------------------------|--------|-----|------|---------|---------------------------------------|
| gps_rescue_min_start_dist           | uint16 | 10  | 500  | 30      | meters, min distance to attempt rescue|
| gps_rescue_return_alt               | uint16 | 10  | 500  | 50      | meters, cruise altitude               |
| gps_rescue_min_loiter_alt           | uint16 | 10  | 500  | 25      | meters, minimum safe orbit altitude   |
| gps_rescue_max_bank_angle           | uint8  | 10  | 45   | 25      | degrees, max roll in rescue turns     |
| gps_rescue_orbit_radius             | uint16 | 20  | 500  | 50      | meters, orbit radius at home          |
| gps_rescue_cruise_throttle          | uint8  | 10  | 90   | 50      | %, cruise throttle                    |
| gps_rescue_min_throttle             | uint8  | 10  | 80   | 30      | %, stall prevention floor             |
| gps_rescue_abort_throttle           | uint8  | 10  | 80   | 45      | %, abort/recovery throttle            |
| gps_rescue_nav_p                    | uint8  | 0   | 200  | 30      | heading P gain                        |
| gps_rescue_alt_p                    | uint8  | 0   | 200  | 30      | altitude P gain                       |
| gps_rescue_turn_compensation        | uint8  | 0   | 100  | 50      | %, nose-up pitch in banked turns      |
| gps_rescue_min_heading_speed        | uint16 | 100 | 1000 | 400     | cm/s, min groundspeed for valid COG   |
| gps_rescue_sanity_checks            | uint8  | —   | —    | ON      | lookup: OFF / ON                      |
| gps_rescue_min_sats                 | uint8  | 5   | 50   | 8       | minimum satellites                    |
| gps_rescue_allow_arming_without_fix | uint8  | —   | —    | OFF     | lookup: OFF / ON                      |

---

## Quick-Set Examples

```
# Autolaunch — gentle launch, lower climb angle
set wing_launch_accel_thresh = 20
set wing_launch_climb_angle = 30
set wing_launch_throttle = 60
set wing_launch_climb_time = 4000

# GPS Rescue — conservative settings for small field
set gps_rescue_return_alt = 40
set gps_rescue_orbit_radius = 30
set gps_rescue_max_bank_angle = 20
set gps_rescue_cruise_throttle = 45

# Save
save
```

---

**Total: 25 tunable parameters** (10 autolaunch + 15 GPS rescue) plus servo autotrim (mode-only, no CLI params).
