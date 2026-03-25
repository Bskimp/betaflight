# Betaflight Mixer Profile Roadmap

**Feature:** In-flight switchable mixer profiles for VTOL transition  
**Target branch:** `master` → 2026.6.x alpha  
**Author:** Brian  
**Status:** Planning — locked for Phase 0 execution  
**Last updated:** 2026-03-20 (rev 5 — tailsitter orientation offset)  

---

## Summary

Add switchable mixer profiles to Betaflight so a single AUX channel can transition a VTOL between multirotor hover and fixed-wing forward flight. Each profile carries its own motor mix rules, servo mix rules, mixer mode, and platform type. A transition state machine blends outputs during the switch. PID and rate profiles optionally follow the mixer profile.

The entire feature is gated behind `USE_MIXER_PROFILE` and compiles out on flash-constrained targets. When enabled but only one profile is configured, runtime behavior is identical to current Betaflight.

**Prior art:** iNav `mixer_profile` (PR #8555, merged ~7.x), ArduPilot QuadPlane (`ArduPlane/quadplane.cpp`).

**Scope boundary:** This roadmap covers firmware only. The Betaflight Configurator (App) is a separate PR that depends on the MSP protocol finalized in Phase 5. However, App paging/UX compatibility must be validated before the firmware PR merges — the App is a merge-time dependency, not a deferred follow-on.

**Upstream PR strategy:** The feature ships as 3 sequential PRs to reduce review burden:

| PR | Content | Phases | Standalone value |
|----|---------|--------|-----------------|
| **PR 1** | Core profile plumbing | 1 + 2 + partial 4 (arming) | PG, CLI, superset allocation, immediate profile switch, arming checks. Testable end-to-end via CLI. |
| **PR 2** | Safe transition behavior | 3 + rest of 4 | Blend state machine, AUX modes, failsafe, transition-only motors, linked profile switching. |
| **PR 3** | Observability + polish | 5 | MSP2 protocol, OSD, blackbox, debug mode, docs. |

Each PR stands alone — PR 1 is useful without PR 2 (disarmed profile switching, immediate armed switching for testing). PR 2 adds the smooth transition. PR 3 is pure observability.

**What PR 1 explicitly does NOT include:** No transition blending, no AUX mode switching, no failsafe profile logic, no transition-only motor rules, no linked PID/rate profiles, no MSP/OSD/blackbox. PR 1 is pure plumbing: data structures, CLI, superset allocation, and immediate profile swap. If a reviewer sees transition logic in PR 1, the scope has crept.

### Canonical example: Quad+Pusher VTOL

A concrete end-to-end config to anchor the feature. 4 hover motors (quad X), 1 pusher motor, 2 elevon servos:

```
# Profile 0: Multirotor hover
mixer_profile 0
mixer CUSTOM
set platform_type = MULTIROTOR
mmix 0  1.0 -1.0  1.0 -1.0   # front right
mmix 1  1.0 -1.0 -1.0  1.0   # rear right
mmix 2  1.0  1.0  1.0  1.0   # front left
mmix 3  1.0  1.0 -1.0 -1.0   # rear left
mmix 4 -1.200 0.0 0.0 0.0    # pusher: transition-only, 20%
set transition_timer_ms = 2000
set linked_pid_profile = 1
set linked_rate_profile = 1

# Profile 1: Fixed-wing forward flight
mixer_profile 1
mixer CUSTOM_AIRPLANE
set platform_type = AIRPLANE
mmix 0  1.0  0.0  0.0  0.0   # pusher: full authority
smix 0  3  0  100  0  0  100  0   # elevon L ← roll
smix 1  3  1  100  0  0  100  0   # elevon L ← pitch
smix 2  4  0 -100  0  0  100  0   # elevon R ← roll (reversed)
smix 3  4  1  100  0  0  100  0   # elevon R ← pitch
set transition_timer_ms = 1500
set linked_pid_profile = 2
set linked_rate_profile = 2

# Global
set mixer_profile_count = 2
set failsafe_mixer_profile = 0
set failsafe_mixer_action = SWITCH_IMMEDIATE
```

---

## Code Evidence

### Boot / Init sequence (confirmed from `src/main/fc/init.c`)
```
init()
  → mixerInit(mixerConfig()->mixerMode)   // sets motorCount from mixerMode
  → motorDevInit(getMotorCount())          // claims timers + DMA per motor
  → servoDevInit()                         // claims PWM timers per servo (ifdef USE_SERVOS)
  → ppmRxInit / pwmRxInit                  // must follow motors (timer clash avoidance)
```

**Key finding:** `motorDevInit()` is called with `getMotorCount()` which returns the GLOBAL motor count set by `mixerInit()`. There is no per-profile concept. Timer allocation in `timerAllocate()` is one-shot — no `timerFree()` exists. This confirms the superset-at-boot strategy is the only safe path.

### Current mixer data flow
```
mixerConfig_t (PG)
  → mixerConfig()->mixerMode
  → mixerInit() picks from mixers[] table → sets motorCount
  → customMotorMixer[] (PG_DECLARE_ARRAY) for MIXER_CUSTOM*
  → mixTable() reads currentMixer[i].throttle/roll/pitch/yaw per motor
  → motor[i] computed → writeMotors()
```


### mixerRuntime derived state (critical for Phase 2)
```
mixerRuntime contains (from mixer_init.c / mixer.c):
  .motorCount              — used in all runtime loops
  .motorOutputHigh         — DShot/PWM max output value
  .motorOutputLow          — DShot/PWM min output value  
  .disarmMotorOutput       — value sent when disarmed
  .motorOutputRange        — high - low
  .deadbandMotor3dHigh/Low — 3D mode deadband
  .throttleAngleCorrection — tilt compensation
  .crashflip state         — crashflip mode tracking
  .rpmLimiter state        — RPM limiter PID + filters
  .ezLanding state         — EZ landing attenuation
  .vbatSagCompensation*    — battery voltage compensation fields
  .feature3dEnabled        — 3D mode flag
```

These are all computed at init from global config and assumed static at runtime. Profile switching must reconcile them.

### Servo config vs servo mixing (two separate layers)
```
servoParams[] (PG) — per-servo: min, max, middle, rate, forwardFromChannel
  → defines physical calibration and endpoints

servoMixer[] / customServoMixers[] — per-rule: target, source, rate, speed, min, max, box
  → defines how FC inputs map to servo movement

mixTable() → servo computation uses BOTH layers
```

**Key finding:** Switching only servoMixer rules (smix) without switching servoParams would produce correct mixing logic driving incorrectly-calibrated outputs if a VTOL needs different servo endpoints between profiles. Decision L9 addresses this explicitly.

### Files and approximate sizes (from GitHub master, 2025.12.x)

| File | Est. Lines | Role | Modification type |
|------|-----------|------|-------------------|
| `src/main/flight/mixer.h` | ~155 | Types, externs | Modify — add profile externs |
| `src/main/flight/mixer.c` | ~600 | mixTable, RPM limiter, crashflip | Modify — pointer indirection + blend path |
| `src/main/flight/mixer_init.c` | ~450 | mixerInit, motorDevInit call setup | Modify — superset scan |
| `src/main/flight/servos.h` | ~120 | Servo types | Minor modify |
| `src/main/flight/servos.c` | ~400 | Servo mixing, servoDevInit | Modify — superset servo init |
| `src/main/fc/init.c` | ~800 | Boot sequence | Modify — call mixerProfileInit |
| `src/main/fc/rc_modes.h` | ~180 | boxId_e enum | Modify — add BOXMIXERPROFILE |
| `src/main/fc/rc_modes.c` | ~300 | Mode processing | Modify — add profile switch hook |
| `src/main/fc/core.c` | ~1200 | Main task loop, arming | Modify — arming checks |
| `src/main/msp/msp.c` | ~3500 | MSP command handling | Modify — add MSP2 handlers |
| `src/main/msp/msp_box.c` | ~300 | Box/mode table | Modify — add entries |
| `src/main/msp/msp_protocol_v2_betaflight.h` | ~80 | MSP2 command IDs | Modify — add IDs |
| `src/main/cli/cli.c` | ~4000 | CLI commands | Modify — mixer_profile context |
| `src/main/cli/settings.c` | ~2000 | Parameter tables | Modify — PG registration |
| `src/main/config/config.c` | ~1500 | EEPROM save/load, migration | Modify — migration fn |
| `src/main/flight/failsafe.c` | ~500 | Failsafe state machine | Modify — profile force-switch |
| `src/main/flight/pid.c` | ~1200 | PID controller | Modify — I-term reset hook |
| `src/main/osd/osd.h` | ~200 | OSD element IDs | Modify — add element |
| `src/main/osd/osd_elements.c` | ~2500 | OSD rendering | Modify — add element render |
| `src/main/blackbox/blackbox.c` | ~2000 | Blackbox logging | Modify — add fields |
| `src/main/target/common_pre.h` | ~300 | Feature gates | Modify — USE_MIXER_PROFILE |

### New files
```
src/main/flight/mixer_profile.h       — ~120 lines — types, API declarations
src/main/flight/mixer_profile.c       — ~500 lines — PG, init, switch, transition SM
src/main/pg/mixer_profile.h           — ~30 lines  — PG ID (if separate from flight/)
test/unit/mixer_profile_unittest.cc   — ~300 lines — unit tests
docs/MixerProfile.md                  — ~200 lines — user documentation
```

---

## Locked Decisions

| # | Decision | Date | Reversal condition |
|---|----------|------|--------------------|
| L1 | **2 mixer profiles, not 3.** Saves ~400 bytes flash vs 3. Sufficient for all VTOL topologies (hover + FW). | 2026-03-20 | A concrete use case requires 3 simultaneous mixer configs on a single airframe. |
| L2 | **Superset allocation at boot.** No runtime timer/DMA reallocation. Both profiles' outputs claimed at init. | 2026-03-20 | Someone implements a safe, tested `timerFree()` + `timerAllocate()` hot-swap path. |
| L3 | **Feature gated behind `USE_MIXER_PROFILE`.** Compiles out on F411 and flash-constrained targets. | 2026-03-20 | F411 gains enough flash headroom (unlikely with current chip). |
| L4 | **iNav-style negative throttle encoding for transition-only motors.** Values -1.0 to -2.0 mean "spin at abs(throttle)-1.0 only during MIXER TRANSITION." Matches iNav's proven pattern — reduces friction for users migrating from iNav VTOL setups. | 2026-03-20 | BF maintainers reject this in review. Fallback: add `uint8_t transitionOnlyMask` bitmask to `mixerProfile_t` — small PG change, cleaner API. |
| L5 | **Profile coupling is opt-in.** `linkedPidProfile` and `linkedRateProfile` default to 0 (no link). User configures explicitly. | 2026-03-20 | Maintainers demand auto-coupling as default for safety. |
| L6 | **Failsafe behavior is configurable via `failsafe_mixer_action` enum.** Default `SWITCH_IMMEDIATE` to failsafe profile (hover). Other options: `HOLD_CURRENT` (tailsitters), `FAST_BLEND` (50ms). Target profile set by `failsafe_mixer_profile`. | 2026-03-20 | Maintainers want a single hardcoded behavior for simplicity. |
| L7 | **MSP2 (not MSP v1) for all new commands.** MSP v1 ID space is nearly exhausted. | 2026-03-20 | BF maintainers allocate v1 IDs (unlikely). |
| L8 | **Firmware PRs first, Configurator PR separate.** CLI-only configuration sufficient for testing. App paging/UX compatibility validated before firmware merge. | 2026-03-20 | BF App team wants to co-develop in parallel (would be welcome). |
| L9 | **Servo params (min/max/middle/rate) are GLOBAL in v1 — not per-profile.** Profiles switch servo MIX rules (smix) only, not servo calibration. Most VTOL builds use same servo endpoints in both modes; tilt servos are driven by smix weights. | 2026-03-20 | A concrete VTOL topology requires different servo min/max/middle between profiles that cannot be handled via smix rate/weight. Would add `servoParam_t servoParams[]` to `mixerProfile_t`. |
| L10 | **v1 supports only manual pilot-controlled transitions.** Profile switch blocked when any autonomous/navigation mode is active. No mode-aware automatic transitions. | 2026-03-20 | Demand for GPS-rescue-aware or RTH-triggered automatic transitions (v2 feature). |
| L11 | **Mid-transition switch requests are dropped.** A new transition cannot start until the current one reaches IDLE. Pilot must wait for completion then toggle again. No latching, no mid-blend reversal. | 2026-03-20 | A real flight scenario where mid-transition reversal (e.g., abort-to-hover below progress threshold) is safer than completion. Would add a `TRANSITION_REVERSING` state. |
| L12 | **Tailsitter orientation offset is a per-profile enum applied in the IMU/attitude path, not the mixer.** When `orientationOffset = TAILSITTER`, the attitude reference is rotated 90° on the pitch axis before PID computation. This matches iNav's `tailsitter_orientation_offset` approach. Default `NONE` — zero behavior change for non-tailsitter builds. | 2026-03-20 | A VTOL topology requires an arbitrary rotation angle rather than a fixed 90° offset. Would replace the enum with a per-profile `int16_t orientationOffsetDegrees`. |

---

## v1 Airframe Support Matrix

Defines which VTOL topologies are supported, caveated, or out of scope. Prevents scope creep during review.

| Airframe type | v1 status | Notes |
|--------------|-----------|-------|
| Quadplane 4+1 (4 hover motors, 1 pusher) | **Supported** | Primary design target. iNav-proven topology. |
| Elevon quadplane (4 hover + elevon wing) | **Supported** | Elevon servos driven by smix, same endpoints both modes. |
| Single-pusher + control surfaces | **Supported** | Standard FW profile with pusher motor + aileron/elevator/rudder servos. |
| Tailsitter (2-4 motors, rotates 90°) | **Supported** | Set `orientation_offset = TAILSITTER` on the hover profile (L12). Use `HOLD_CURRENT` failsafe (L6). Configure smix for rotated axis mapping. |
| Tilt-rotor, shared servo endpoints | **Supported** | Tilt servo position controlled via smix weight/rate. Same servo min/max/middle in both modes. |
| Tailsitter with tilting motors | **Supported** | Combines orientation offset (L12) with tilt servo smix rules. Most complex topology — exercise all code paths. |
| Tilt-rotor, different servo endpoints per mode | **Out of scope v1** | Requires per-profile `servoParams` (L9 reversal). Workaround: set endpoints wide enough for both modes, control travel via smix rate. |
| 3D / reversible motor mode | **Blocked** | Arming rejected when `FEATURE_3D` + `mixer_profile_count = 2`. |
| Helicopter / CCPM | **Out of scope v1** | Heli mixing is a separate subsystem not addressed by this feature. |

---

## Phase 0: Pre-Implementation (Discord Recon + Fork Setup)

**Goal:** Get informal architecture buy-in before writing code. Reserve ID ranges.  
**Duration:** 1 week  
**Deliverable:** Written confirmation on Discord or GitHub Discussion  

### Tasks

- [ ] **0.1** Post architecture summary to Betaflight Discord `#development` channel
  - Superset allocation approach
  - 2 profiles
  - Feature gate naming (`USE_MIXER_PROFILE` vs `USE_VTOL`)
  - Link to this roadmap for full context
- [ ] **0.2** Request `permanentId` allocation for `BOXMIXERPROFILE` and `BOXMIXERTRANSITION`
  - Check current highest permanentId in `msp_box.c` on master at time of request
  - Need 2 consecutive IDs
- [ ] **0.3** Request MSP2 command ID allocation
  - Need 3 IDs: `MSP2_MIXER_PROFILE`, `MSP2_SET_MIXER_PROFILE`, `MSP2_MIXER_PROFILE_STATUS`
  - Check `msp_protocol_v2_betaflight.h` for next available
- [ ] **0.4** Timer/protocol compatibility spot-check
  - On at least one F405, one F7, and one H7 target: configure max superset (4 DShot motors + 1 DShot pusher + 4 PWM servos) via resource remapping on stock BF
  - Confirm all 9 outputs initialize without timer bank conflicts
  - Document any targets where mixed DShot + slow-rate servo PWM on same timer bank fails
- [ ] **0.5** Fork Betaflight master, create `feature/mixer-profile` branch
- [ ] **0.6** Set up CI: ensure `make TARGET=STM32F411` and `make TARGET=STM32H743` both pass on clean fork

### No-Go Gate
Cannot proceed to Phase 1 if:
- Maintainers explicitly reject the superset allocation approach
- Maintainers want a fundamentally different architecture (e.g., full output abstraction layer rewrite)
- `permanentId` or MSP2 ID allocation is refused
- Timer spot-check reveals fundamental incompatibility on common targets for typical VTOL superset

**Fallback:** If gated, maintain as personal fork without mainline PR ambition. Simplifies scope significantly (skip MSP, OSD, Configurator).

---

## Phase 1: Parameter Group, Types, and CLI Foundation

**Goal:** Define the data structures, register the PG, add CLI commands. Zero runtime behavior change.  
**Duration:** 1-2 weeks  
**Dependencies:** Phase 0 complete  
**Estimated new/modified lines:** ~600  

### 1.1 New file: `src/main/flight/mixer_profile.h`

```c
#pragma once

#include "flight/mixer.h"
#include "flight/servos.h"
#include "pg/pg.h"

#define MAX_MIXER_PROFILE_COUNT 2

typedef enum {
    PLATFORM_MULTIROTOR = 0,
    PLATFORM_AIRPLANE   = 1,
} platformType_e;

typedef enum {
    TRANSITION_CURVE_LINEAR = 0,
    TRANSITION_CURVE_SCURVE = 1,
} transitionCurve_e;

typedef enum {
    FAILSAFE_MIXER_SWITCH_IMMEDIATE = 0,
    FAILSAFE_MIXER_HOLD_CURRENT     = 1,
    FAILSAFE_MIXER_FAST_BLEND       = 2,
} failsafeMixerAction_e;

typedef enum {
    ORIENTATION_OFFSET_NONE       = 0,
    ORIENTATION_OFFSET_TAILSITTER = 1,  // 90° pitch rotation — "nose up" in hover becomes "nose forward" in FW
} orientationOffset_e;

typedef struct mixerProfile_s {
    motorMixer_t motorMix[MAX_SUPPORTED_MOTORS];
    uint8_t motorCount;

    servoMixer_t servoMix[MAX_SUPPORTED_SERVOS];
    uint8_t servoCount;

    uint8_t mixerMode;              // mixerMode_e
    uint8_t platformType;           // platformType_e
    uint8_t orientationOffset;      // orientationOffset_e — IMU frame rotation for this profile

    uint16_t transitionTimerMs;     // blend duration when switching TO this profile
    uint8_t  transitionCurve;       // transitionCurve_e

    uint8_t linkedPidProfile;       // 0=no link, 1-3=auto-switch PID profile index
    uint8_t linkedRateProfile;      // 0=no link, 1-3=auto-switch rate profile index
} mixerProfile_t;

// NOTE: servoParams (min/max/middle/rate) are GLOBAL, not per-profile (L9).
// Profiles switch only motor mix rules (mmix) and servo mix rules (smix).

PG_DECLARE_ARRAY(mixerProfile_t, MAX_MIXER_PROFILE_COUNT, mixerProfiles);

typedef struct mixerProfileConfig_s {
    uint8_t profileCount;           // 1 or 2 (default 1 = feature effectively disabled)
    uint8_t failsafeProfile;        // 0-indexed, default 0
    uint8_t failsafeAction;         // failsafeMixerAction_e
    uint16_t airspeedGateCms;       // 0=disabled, >0 = cm/s for FW transition complete
} mixerProfileConfig_t;

PG_DECLARE(mixerProfileConfig_t, mixerProfileConfig);

// Runtime API
void mixerProfileInit(void);
uint8_t mixerProfileGetActiveIndex(void);
void mixerProfileSwitch(uint8_t profileIndex, bool withTransition);
bool mixerProfileTransitionInProgress(void);
float mixerProfileTransitionProgress(void);
```

### 1.2 New file: `src/main/flight/mixer_profile.c` (Phase 1 subset)

Phase 1 implements ONLY:
- PG registration with defaults
- `mixerProfileInit()` — loads profile 0, no superset logic yet
- CLI parameter table entries
- EEPROM migration function

Runtime switch, transition SM, and blend are Phase 2-3 stubs.

### 1.3 CLI commands

```
# Context selection (like 'profile' for PID)
mixer_profile <0|1>           Switch CLI editing context to profile N

# Per-profile commands (scoped to current mixer_profile context)
mmix <rule> <thr> <roll> <pitch> <yaw>     Motor mix rule
smix <rule> <target> <src> <rate> <speed> <min> <max> <box>  Servo mix rule
mixer <mode>                   Set mixerMode for current profile
set platform_type = MULTIROTOR|AIRPLANE
set orientation_offset = NONE|TAILSITTER

# Per-profile transition settings
set transition_timer_ms = 1500
set transition_curve = LINEAR|SCURVE
set linked_pid_profile = 0
set linked_rate_profile = 0

# Global settings
set mixer_profile_count = 1      # 1=disabled, 2=enabled
set failsafe_mixer_profile = 0
set failsafe_mixer_action = SWITCH_IMMEDIATE
set mixer_airspeed_gate = 0      # cm/s, 0=disabled
```

### 1.4 EEPROM Migration

In `config.c`, add migration function:
- Copy existing `customMotorMixer[]` → `mixerProfiles(0)->motorMix[]`
- Copy existing `mixerConfig()->mixerMode` → `mixerProfiles(0)->mixerMode`
- Set `mixerProfiles(0)->motorCount` from existing `getMotorCount()` logic
- Initialize `mixerProfiles(1)` to zeroed/defaults
- Set `mixerProfileConfig()->profileCount = 1` (disabled by default)
- Bump PG version

**Rollback / disable path:** The migration is fully reversible:
- Setting `mixer_profile_count = 1` at any time makes the feature behave exactly like stock Betaflight. Profile 0 becomes the canonical config; profile 1 is ignored.
- Users who flash firmware with this feature and never touch `mixer_profile_count` see zero behavior change — profile 0 contains their migrated config, and the feature is dormant.
- Downgrading to firmware without `USE_MIXER_PROFILE` drops the PG cleanly; Betaflight's PG system handles unknown PGs by ignoring them on load.

### 1.5 Feature gate

```c
// src/main/target/common_pre.h — enable on all targets with sufficient flash
#if !defined(STM32F411xE)  // F411 too constrained
#define USE_MIXER_PROFILE
#endif
```

All new code wrapped in `#ifdef USE_MIXER_PROFILE`.

### 1.6 Files touched

| File | Change |
|------|--------|
| `src/main/flight/mixer_profile.h` | **NEW** |
| `src/main/flight/mixer_profile.c` | **NEW** (Phase 1 subset: PG, CLI glue, migration) |
| `src/main/cli/cli.c` | Add `mixer_profile` context command, per-profile mmix/smix |
| `src/main/cli/settings.c` | Register mixerProfile PG, add parameter entries |
| `src/main/config/config.c` | EEPROM migration function |
| `src/main/target/common_pre.h` | `USE_MIXER_PROFILE` define |
| `CMakeLists.txt` / `Makefile` | Add new source file |

### 1.7 Acceptance Criteria

- [ ] `make TARGET=STM32F405` builds clean with `USE_MIXER_PROFILE` defined
- [ ] `make TARGET=STM32F411` builds clean with `USE_MIXER_PROFILE` NOT defined — zero size increase
- [ ] `make TARGET=STM32H743` builds clean
- [ ] CLI: `mixer_profile 0` and `mixer_profile 1` switch context
- [ ] CLI: `mmix` rules saved per-profile, survive power cycle
- [ ] CLI: `diff all` dumps both profiles when `mixer_profile_count = 2`
- [ ] CLI: `diff all` output is identical to stock when `mixer_profile_count = 1`
- [ ] EEPROM migration: existing quad config loads into profile 0 without data loss
- [ ] Unit test: PG defaults are sane, migration produces expected output

### No-Go Gate
Cannot proceed to Phase 2 if:
- `diff all` output breaks backwards compatibility for users not using mixer profiles
- Flash size increase on F405 exceeds ~2KB for Phase 1 alone
- CI fails on any supported target

---

## Phase 2: Superset Allocation and Runtime Profile Switch

**Goal:** Boot allocates max outputs across profiles. CLI can switch active profile with immediate effect. No transition blend yet.  
**Duration:** 1-2 weeks  
**Dependencies:** Phase 1 merged to feature branch  
**Estimated new/modified lines:** ~450  

### 2.1 Superset scan in `mixerProfileInit()`

```c
void mixerProfileInit(void) {
    uint8_t maxMotors = 0;
    uint8_t maxServos = 0;

    const int count = mixerProfileConfig()->profileCount;
    for (int i = 0; i < count; i++) {
        // Compute motor count from mixerMode + custom rules for each profile
        uint8_t mc = computeMotorCountForProfile(i);
        uint8_t sc = computeServoCountForProfile(i);
        mixerProfilesMutable(i)->motorCount = mc;
        mixerProfilesMutable(i)->servoCount = sc;
        maxMotors = MAX(maxMotors, mc);
        maxServos = MAX(maxServos, sc);
    }

    supersetMotorCount = maxMotors;
    supersetServoCount = maxServos;

    // Activate profile 0 as default (immediate, no transition)
    mixerProfileActivate(0);
}
```

### 2.2 Modify boot sequence in `init.c`

```c
// Before (current):
mixerInit(mixerConfig()->mixerMode);
motorDevInit(getMotorCount());

// After:
mixerInit(mixerConfig()->mixerMode);  // still needed for legacy path
#ifdef USE_MIXER_PROFILE
if (mixerProfileConfig()->profileCount > 1) {
    mixerProfileInit();  // computes superset, activates profile 0
    motorDevInit(mixerProfileGetSupersetMotorCount());
} else
#endif
{
    motorDevInit(getMotorCount());  // legacy path unchanged
}
```

**Critical constraint from code evidence:** `ppmRxInit()` must follow `motorDevInit()` due to timer clash avoidance (`ppmAvoidPWMTimerClash`). The superset approach doesn't change this ordering — we just pass a larger count to `motorDevInit()`.

### 2.3 Hot-path pointer swap in mixer

```c
// In mixer_profile.c:
static const motorMixer_t *activeMix;
static uint8_t activeMotorCount;
static uint8_t activeMixerProfile;

void mixerProfileActivate(uint8_t index) {
    activeMix = mixerProfiles(index)->motorMix;
    activeMotorCount = mixerProfiles(index)->motorCount;
    activeMixerProfile = index;
    // ... servo equivalent
}
```

In `mixer.c`, `mixTable()` changes from reading `currentMixer[i]` to `activeMix[i]`. This is a single-pointer indirection — zero measurable CPU cost.

### 2.4 mixerRuntime state reconciliation

This is the highest-risk part of Phase 2. The `mixerRuntime` struct contains derived state computed once at init and assumed static. Profile switching must update the subset that depends on profile-specific config.

**Field-by-field disposition:**

| mixerRuntime field | Category | Action on profile switch |
|-------------------|----------|--------------------------|
| `motorCount` | **Profile-local** | Set to `activeMotorCount`. Runtime loops use this, not superset count. |
| `motorOutputHigh` | **Profile-local** | Recompute from active profile's motor protocol + config. |
| `motorOutputLow` | **Profile-local** | Recompute. Differs between DShot (MOTOR_STOP) and PWM (`mincommand`). |
| `motorOutputRange` | **Profile-local** | Recompute as `high - low`. |
| `disarmMotorOutput` | **Profile-local** | Recompute. **Safety critical** — wrong value means motors won't stop. |
| `deadbandMotor3dHigh/Low` | **Forbidden in v1** | 3D mode blocked with mixer profiles at arming. |
| `feature3dEnabled` | **Forbidden in v1** | Same — block at arming. |
| `throttleAngleCorrection` | **Superset-global** | Stays at boot value. Tilt compensation from IMU, not profile-specific. |
| `crashflip state` | **Reset on switch** | Clear. Crashflip makes no sense mid-transition. |
| `rpmLimiter state` | **Reset on switch** | Reset PID integral and filters. RPM characteristics change between profiles. |
| `ezLanding state` | **Reset on switch** | Clear EZ landing attenuation. |
| `vbatSagCompensation*` | **Superset-global** | Battery state is physical, not profile-dependent. |

```c
static void mixerProfileReconcileRuntime(uint8_t profileIndex) {
    const mixerProfile_t *profile = mixerProfiles(profileIndex);

    // Recompute motor output range for active profile
    mixerRuntime.motorCount = profile->motorCount;
    initEscEndpointsForProfile(profileIndex);  // sets motorOutputHigh/Low/Range/disarm

    // Reset stateful subsystems
    mixerResetRpmLimiter();
    mixerResetCrashflipState();
    mixerResetEzLandingState();

    mixerRuntime.servoCount = profile->servoCount;
}
```

`initEscEndpointsForProfile()` is a refactored `initEscEndpoints()` that accepts a profile index. The existing function becomes a wrapper calling it with profile 0.

### 2.5 Tailsitter orientation offset (L12)

When a profile has `orientationOffset = TAILSITTER`, the attitude reference frame is rotated 90° around the pitch axis before the PID controller sees it. This means the PID controller always works in the "correct" frame for the current flight mode — it doesn't need to know it's a tailsitter.

**Implementation in `imu.c`:**

```c
// Applied after attitude estimation, before PID input
void imuApplyOrientationOffset(orientationOffset_e offset) {
    if (offset == ORIENTATION_OFFSET_NONE) return;

    if (offset == ORIENTATION_OFFSET_TAILSITTER) {
        // Rotate attitude 90° around pitch axis
        // In hover: nose pointing up, "forward" is the aircraft belly direction
        // Effect: PID roll axis → geographic yaw, PID yaw axis → geographic roll
        const float tempRoll = attitude.values.roll;
        attitude.values.roll = attitude.values.yaw;
        attitude.values.yaw = -tempRoll;
        // Pitch stays pitch (rotation is around this axis)
    }
}
```

**Called from `mixerProfileActivate()`:**

```c
// Update the active orientation offset for the IMU path
activeOrientationOffset = mixerProfiles(index)->orientationOffset;
```

The actual rotation is applied every IMU cycle, not just on switch — the IMU continuously provides attitude in the global frame, and the offset continuously remaps it to the profile's body frame.

**iNav precedent:** iNav's `tailsitter_orientation_offset = ON` does exactly this. When the tailsitter MC mixer profile is active, a 90° offset is applied so that the multirotor PID controller sees the correct orientation even though the aircraft is physically nose-up. iNav also adds a 45° offset to the target angle when MIXER TRANSITION is active, helping the aircraft pitch forward to gain airspeed before switching to FW mode. We defer the 45° transition angle to v2 — v1 handles the full 90° rotation only.

**Cost:** ~20 lines in `imu.c`, 1 byte per profile in the PG. Zero cost when `ORIENTATION_OFFSET_NONE` (a single branch that returns immediately).

### 2.6 Idle output management

After `mixTable()` computes outputs for active motors:
```c
for (int i = activeMotorCount; i < supersetMotorCount; i++) {
    motor[i] = mixerRuntime.disarmMotorOutput;
}
for (int i = activeServoCount; i < supersetServoCount; i++) {
    servo[i] = servoParams(i)->middle;
}
```

### 2.7 Coupled PID/rate switch

On profile activate:
```c
if (mixerProfiles(index)->linkedPidProfile > 0) {
    changePidProfile(mixerProfiles(index)->linkedPidProfile - 1);
}
if (mixerProfiles(index)->linkedRateProfile > 0) {
    changeControlRateProfile(mixerProfiles(index)->linkedRateProfile - 1);
}
pidInitConfig(currentPidProfile);  // re-init PID for new platform
```

### 2.8 Files touched

| File | Change |
|------|--------|
| `src/main/flight/mixer_profile.c` | Superset scan, activate, runtime reconciliation, idle management |
| `src/main/flight/mixer.c` | `mixTable()` → read from `activeMix` pointer |
| `src/main/flight/mixer.h` | Extern for `activeMix`, superset counts |
| `src/main/flight/mixer_init.c` | Accept superset motor count, refactor `initEscEndpoints()` → `initEscEndpointsForProfile()` |
| `src/main/flight/servos.c` | Accept superset servo count |
| `src/main/fc/init.c` | Boot sequence modification |
| `src/main/flight/imu.c` | `imuApplyOrientationOffset()` — 90° attitude rotation for tailsitter profiles |
| `src/main/flight/pid.c` | `pidInitConfig()` call on profile switch |

### 2.9 Acceptance Criteria

- [ ] Boot with 2 profiles: `getMotorCount()` returns superset count
- [ ] Logic analyzer: all superset outputs get DShot/PWM init at boot
- [ ] CLI `mixer_profile_switch 1` → active motors change, idle motors get correct disarm output
- [ ] Logic analyzer: no output glitch (no rogue pulse) on switch
- [ ] `disarmMotorOutput` correct per profile (verified via debug mode)
- [ ] `motorOutputHigh`/`Low` recomputed on switch (verified via debug mode)
- [ ] RPM limiter state reset on switch — no stale integral carryover
- [ ] Crashflip state cleared on switch
- [ ] Servos: inactive servos hold `middle` position, don't jitter
- [ ] PID profile auto-switches when linked
- [ ] Rate profile auto-switches when linked
- [ ] Single-profile config (`profileCount=1`): behavior identical to stock Betaflight
- [ ] Orientation offset `NONE`: attitude unchanged on profile switch
- [ ] Orientation offset `TAILSITTER`: attitude roll/yaw axes swap correctly on profile switch (verify via OSD or debug attitude values)
- [ ] 3D mode: arming blocked when `mixer_profile_count = 2` and 3D feature enabled
- [ ] Flash size: superset path adds < 1.5KB over Phase 1

### No-Go Gate
Cannot proceed to Phase 3 if:
- DMA allocation fails on F405 with superset count > 4 motors + 4 servos
- Timer clash with PPM/PWM Rx when superset claims additional timers
- `disarmMotorOutput` incorrect for either profile (safety critical)
- Any output glitch visible on logic analyzer during profile switch
- `mixerRuntime` fields produce incorrect behavior after switch

**Fallback:** If DMA constrained on F4, limit superset to max 6 motors on F4 targets (sufficient for most VTOL — 4 hover + 1 pusher + 1 spare). H7 has abundant DMA.

---

## Phase 3: Transition State Machine

**Goal:** AUX-triggered blend between profiles over configurable duration. Linear and S-curve options.  
**Duration:** 1-2 weeks  
**Dependencies:** Phase 2 passing all acceptance criteria  
**Estimated new/modified lines:** ~400  

### 3.1 Transition state

```c
typedef enum {
    MIXER_TRANSITION_IDLE,
    MIXER_TRANSITION_BLENDING,
    MIXER_TRANSITION_AIRSPEED_WAIT,   // optional, ifdef USE_PITOT
} mixerTransitionState_e;

typedef struct {
    mixerTransitionState_e state;
    timeMs_t startTimeMs;
    uint16_t durationMs;
    float progress;                   // 0.0 = fromProfile, 1.0 = toProfile
    uint8_t fromProfile;
    uint8_t toProfile;
    uint8_t curve;
} mixerTransition_t;
```

### 3.2 Start transition

```c
void mixerProfileTransitionStart(uint8_t fromProfile, uint8_t toProfile) {
    if (transition.state != MIXER_TRANSITION_IDLE) return;  // already transitioning
    if (fromProfile == toProfile) return;

    transition.state = MIXER_TRANSITION_BLENDING;
    transition.startTimeMs = millis();
    transition.durationMs = mixerProfiles(toProfile)->transitionTimerMs;
    transition.curve = mixerProfiles(toProfile)->transitionCurve;
    transition.fromProfile = fromProfile;
    transition.toProfile = toProfile;
    transition.progress = 0.0f;
}
```

### 3.3 Progress computation

```c
void mixerProfileTransitionUpdate(timeMs_t currentTimeMs) {
    if (transition.state == MIXER_TRANSITION_IDLE) return;

    timeMs_t elapsed = currentTimeMs - transition.startTimeMs;

    if (transition.state == MIXER_TRANSITION_BLENDING) {
        float linear = constrainf((float)elapsed / transition.durationMs, 0.0f, 1.0f);

        switch (transition.curve) {
            case TRANSITION_CURVE_SCURVE:
                transition.progress = linear * linear * (3.0f - 2.0f * linear);
                break;
            default:
                transition.progress = linear;
                break;
        }

        if (linear >= 1.0f) {
#ifdef USE_PITOT
            if (mixerProfileConfig()->airspeedGateCms > 0
                && mixerProfiles(transition.toProfile)->platformType == PLATFORM_AIRPLANE) {
                transition.state = MIXER_TRANSITION_AIRSPEED_WAIT;
            } else
#endif
            {
                mixerProfileTransitionFinalize();
            }
        }
    }

#ifdef USE_PITOT
    if (transition.state == MIXER_TRANSITION_AIRSPEED_WAIT) {
        if (getAirspeedEstimateCms() >= mixerProfileConfig()->airspeedGateCms) {
            mixerProfileTransitionFinalize();
        } else if (elapsed > transition.durationMs + AIRSPEED_GATE_TIMEOUT_MS) {
            mixerProfileTransitionFinalize();  // safety timeout
        }
        // During airspeed wait, progress stays at 1.0 (fully in target profile outputs)
        transition.progress = 1.0f;
    }
#endif
}

static void mixerProfileTransitionFinalize(void) {
    mixerProfileActivate(transition.toProfile);
    transition.state = MIXER_TRANSITION_IDLE;
    pidResetIterm();  // prevent integral windup carryover
}
```

### 3.4 Blended output in `mixTable()`

During `MIXER_TRANSITION_BLENDING` or `MIXER_TRANSITION_AIRSPEED_WAIT`:

```c
if (mixerProfileTransitionInProgress()) {
    float p = mixerProfileTransitionProgress();
    
    // Evaluate both profiles
    float motorFrom[MAX_SUPPORTED_MOTORS];
    float motorTo[MAX_SUPPORTED_MOTORS];
    evaluateMixProfile(transition.fromProfile, rpyMixInput, throttle, motorFrom);
    evaluateMixProfile(transition.toProfile,   rpyMixInput, throttle, motorTo);

    // Blend
    for (int i = 0; i < supersetMotorCount; i++) {
        motor[i] = motorFrom[i] * (1.0f - p) + motorTo[i] * p;
    }
    for (int i = 0; i < supersetServoCount; i++) {
        servo[i] = servoFrom[i] * (1.0f - p) + servoTo[i] * p;
    }
} else {
    // Normal single-profile path (existing code, pointer indirection from Phase 2)
}
```

**CPU cost note:** During transition only, the mix computation runs 2x. At 8KHz PID loop, each `evaluateMixProfile` is ~50 multiply-adds (8 motors × 4 axes + overhead). Total additional cost during transition: ~400 FLOPs per loop iteration. Negligible on F4/F7/H7.

### 3.5 I-term and D-term handling

- **I-term:** Reset on `transitionFinalize()`, NOT during blend. During blend, the PID controller keeps running normally against the from-profile's platform type. The I-term accumulation during the ~1-2s transition is small and gets wiped on finalize.
- **D-term:** No reset. Gyro signal is continuous through transition. D-term filters should not be disturbed.
- **Output limits:** During blend, use `MAX(fromProfile.motorOutputLow, toProfile.motorOutputLow)` as floor and `MIN(fromProfile.motorOutputHigh, toProfile.motorOutputHigh)` as ceiling. Conservative limits prevent output spikes.

### 3.6 Files touched

| File | Change |
|------|--------|
| `src/main/flight/mixer_profile.c` | Transition SM: start, update, finalize, progress |
| `src/main/flight/mixer_profile.h` | Transition types, API |
| `src/main/flight/mixer.c` | Blended output path in `mixTable()` |
| `src/main/flight/pid.c` | `pidResetIterm()` hook on transition finalize |
| `src/main/sensors/pitot.c` | Airspeed getter for gate (ifdef `USE_PITOT`) |

### 3.7 Acceptance Criteria

- [ ] CLI trigger → outputs blend smoothly over configured `transition_timer_ms`
- [ ] Logic analyzer: no discontinuity at blend start (progress=0) or end (progress=1)
- [ ] S-curve: visually smooth acceleration and deceleration on analyzer
- [ ] Asymmetric timing: hover→FW can be 2000ms while FW→hover is 1000ms
- [ ] I-term resets only on finalize, not during blend
- [ ] D-term filters undisturbed through transition
- [ ] Airspeed gate holds if `USE_PITOT` and `airspeedGateCms > 0` and airspeed below threshold
- [ ] Airspeed gate safety timeout fires after 10s regardless
- [ ] Transition blocked if already in progress (no re-entrant triggering)
- [ ] CPU load: < 2% increase during transition on F405 at 8KHz

### No-Go Gate
Cannot proceed to Phase 4 if:
- Output discontinuity visible at blend boundaries
- PID controller produces oscillation during or immediately after transition
- CPU load exceeds headroom on F405 during blend

**Fallback:** If blend too expensive on F4, degrade to immediate switch (no blend) on F4 targets, blend only on F7/H7.

---

## Phase 4: AUX Mode Integration, Failsafe, and Arming

**Goal:** Wire the transition to actual AUX channel control. Integrate with failsafe and arming systems.  
**Duration:** 1 week  
**Dependencies:** Phase 3 passing all acceptance criteria  
**Estimated new/modified lines:** ~350  

### 4.1 New BOX modes

In `rc_modes.h` — add to `boxId_e` enum:
```c
BOXMIXERPROFILE,          // when active: profile 2; when inactive: profile 1
BOXMIXERTRANSITION,       // enables transition-only motor rules
```

In `msp_box.c` — add to box table:
```c
{ .boxId = BOXMIXERPROFILE,    .boxName = "MIXER PROFILE 2", .permanentId = XX },
{ .boxId = BOXMIXERTRANSITION, .boxName = "MIXER TRANSITION", .permanentId = XX+1 },
```

(XX = allocated IDs from Phase 0.)

### 4.2 Mode processing

In `rc_modes.c` or called from it:
```c
void mixerProfileUpdateFromRcModes(void) {
#ifdef USE_MIXER_PROFILE
    if (mixerProfileConfig()->profileCount < 2) return;

    static bool lastState = false;
    bool currentState = IS_RC_MODE_ACTIVE(BOXMIXERPROFILE);

    if (currentState != lastState) {
        lastState = currentState;
        uint8_t target = currentState ? 1 : 0;
        if (target != mixerProfileGetActiveIndex()) {
            if (ARMING_FLAG(ARMED)) {
                mixerProfileSwitch(target, true);   // armed: transition blend
            } else {
                mixerProfileSwitch(target, false);  // disarmed: immediate switch
            }
        }
    }

    // Update transition mixing flag
    mixerTransitionMixingActive = IS_RC_MODE_ACTIVE(BOXMIXERTRANSITION);
#endif
}
```

### 4.3 Transition-only motor rules

In `mixTable()`, per motor in the active profile:
```c
if (activeMix[i].throttle <= -1.05f && activeMix[i].throttle >= -2.0f) {
    if (mixerTransitionMixingActive) {
        float transitionThrottle = fabsf(activeMix[i].throttle) - 1.0f;
        motor[i] = motorOutputLow + transitionThrottle * (motorOutputHigh - motorOutputLow);
    } else {
        motor[i] = getMotorOutputLow();
    }
    continue;  // skip normal mix computation for this motor
}
```

This enables the iNav pattern: define a pusher motor in the hover profile with `mmix 4 -1.200 0.0 0.0 0.0` → spins at 20% only when MIXER TRANSITION is active.

### 4.4 Blocked modes policy (L10)

**v1 rule: Profile switch is blocked when ANY autonomous or navigation mode is active.** Only manual pilot-controlled transitions are supported.

| Mode | Switch allowed? | Rationale |
|------|----------------|-----------|
| Acro / manual | **Yes** | Pilot has full control |
| Angle | **Yes** | Self-leveling doesn't conflict with mix rules |
| Horizon | **Yes** | Same as Angle |
| Alt Hold | **BLOCKED** | Altitude controller state is profile-dependent |
| Position Hold | **BLOCKED** | Position controller assumes single platform type |
| GPS Rescue | **BLOCKED** | Rescue logic assumes single platform type throughout |
| Launch Control | **BLOCKED** | Launch sequence is mode-specific |
| Crashflip | **BLOCKED** | Turtle mode is MC-only |
| MSP Override | **BLOCKED** | External control source may not expect profile change |
| Passthrough | **Yes** | Pure RC forwarding, no FC control loop involvement |
| Failsafe | Special | Failsafe itself triggers the switch per `failsafe_mixer_action` (4.5) |

```c
bool mixerProfileSwitchAllowed(void) {
    if (mixerProfileTransitionInProgress()) return false;
    if (millis() - getArmingTimeMs() < 2000) return false;

    // Block during any autonomous/navigation mode
    if (FLIGHT_MODE(ALT_HOLD_MODE))       return false;
    if (FLIGHT_MODE(POS_HOLD_MODE))       return false;
    if (FLIGHT_MODE(GPS_RESCUE_MODE))     return false;
    if (IS_RC_MODE_ACTIVE(BOXLAUNCHCONTROL)) return false;
    if (IS_RC_MODE_ACTIVE(BOXMSPOVERRIDE))  return false;
    if (isCrashFlipModeActive())          return false;

    return true;
}
```

**If a blocked mode activates DURING a transition:** The transition runs to completion. Aborting mid-blend is more dangerous than completing it. The blocked-mode check only gates *initiation*.

### 4.5 Failsafe integration

```c
#ifdef USE_MIXER_PROFILE
if (mixerProfileConfig()->profileCount > 1) {
    const uint8_t targetProfile = mixerProfileConfig()->failsafeProfile;
    const uint8_t action = mixerProfileConfig()->failsafeAction;

    if (mixerProfileGetActiveIndex() != targetProfile) {
        switch (action) {
            case FAILSAFE_MIXER_SWITCH_IMMEDIATE:
                mixerProfileSwitch(targetProfile, false);  // no blend
                break;
            case FAILSAFE_MIXER_FAST_BLEND:
                mixerProfileTransitionStartForced(targetProfile, 50); // 50ms
                break;
            case FAILSAFE_MIXER_HOLD_CURRENT:
                break;  // stay put — for tailsitters or FW-safe configs
        }
    }

    // If transition was in progress, abort or force-complete per action
    if (mixerProfileTransitionInProgress()
        && action == FAILSAFE_MIXER_SWITCH_IMMEDIATE) {
        mixerProfileTransitionAbort(targetProfile);
    }
}
#endif
```

**Default `SWITCH_IMMEDIATE` rationale:** In failsafe, hover control authority is needed NOW. A 2-second blend could be fatal at low altitude. The brief output discontinuity is acceptable.

**`HOLD_CURRENT` use case:** Tailsitters or airframes where FW glide is safer than hover in wind.

**`FAST_BLEND` use case:** Compromise — 50ms is fast enough for safety but avoids the harshest output discontinuity.

### 4.6 Arming checks

In `core.c` arming validation:
```c
#ifdef USE_MIXER_PROFILE
if (mixerProfileConfig()->profileCount > 1) {
    // Active profile must have motors
    if (mixerProfiles(mixerProfileGetActiveIndex())->motorCount == 0) {
        setArmingDisabled(ARMING_DISABLED_MIXER_PROFILE);
    }
    // Both profiles must have valid pin assignments for their motor/servo counts
    for (int p = 0; p < mixerProfileConfig()->profileCount; p++) {
        if (!validateMixerProfileOutputs(p)) {
            setArmingDisabled(ARMING_DISABLED_MIXER_PROFILE);
        }
    }
    // Block 3D mode with mixer profiles in v1
    if (featureIsEnabled(FEATURE_3D)) {
        setArmingDisabled(ARMING_DISABLED_MIXER_PROFILE);
    }
}
#endif
```

New arming disable flag: `ARMING_DISABLED_MIXER_PROFILE` added to the enum.

### 4.7 Files touched

| File | Change |
|------|--------|
| `src/main/fc/rc_modes.h` | Add `BOXMIXERPROFILE`, `BOXMIXERTRANSITION` to enum |
| `src/main/fc/rc_modes.c` | Mode processing hook |
| `src/main/msp/msp_box.c` | Box table entries |
| `src/main/flight/mixer_profile.c` | Switch lockout logic |
| `src/main/flight/mixer.c` | Transition-only motor rule handling |
| `src/main/flight/failsafe.c` | Failsafe profile force-switch |
| `src/main/fc/core.c` | Arming checks, disable flag |
| `src/main/fc/runtime_config.h` | `ARMING_DISABLED_MIXER_PROFILE` flag |

### 4.8 Acceptance Criteria

- [ ] AUX switch toggles between profiles with transition blend (when armed)
- [ ] AUX switch toggles immediately (when disarmed)
- [ ] MIXER TRANSITION mode spins transition-only motors at configured speed
- [ ] Transition-only motors stop when MIXER TRANSITION deactivated
- [ ] Failsafe `SWITCH_IMMEDIATE`: immediate switch, no blend
- [ ] Failsafe `FAST_BLEND`: 50ms blend to target profile
- [ ] Failsafe `HOLD_CURRENT`: stays in current profile
- [ ] Failsafe during active transition: handled per action setting
- [ ] Alt Hold active → switch blocked
- [ ] Position Hold active → switch blocked
- [ ] GPS Rescue active → switch blocked
- [ ] Launch Control active → switch blocked
- [ ] Crashflip active → switch blocked
- [ ] MSP Override active → switch blocked
- [ ] First 2s after arm: switch blocked
- [ ] Arming blocked if active profile has 0 motors
- [ ] Arming blocked if either profile has invalid output pin assignments
- [ ] Arming blocked if 3D mode enabled with `mixer_profile_count = 2`
- [ ] Rapid AUX toggling: no crash, no re-entrant transition (debounced)

### No-Go Gate
Cannot proceed to Phase 5 if:
- Failsafe switch produces motor output spike > 10% above expected
- AUX toggling during transition causes undefined behavior
- Arming checks have false positives on valid single-profile configs

**Fallback:** If failsafe immediate-switch output spike is unacceptable, add a very fast blend (50ms) for failsafe transitions specifically.

---

## Phase 5: MSP Protocol, OSD, and Blackbox

**Goal:** Enable Configurator readiness and flight data analysis.  
**Duration:** 1 week (can partially parallel with Phase 4)  
**Dependencies:** Phase 3 types finalized (transition state struct), Phase 4 box IDs allocated  
**Estimated new/modified lines:** ~350  

### 5.1 MSP2 commands

| Command | ID | Direction | Payload |
|---------|----|-----------|---------|
| `MSP2_BETAFLIGHT_MIXER_PROFILE` | TBD | GET | Profile index + full profile data |
| `MSP2_BETAFLIGHT_SET_MIXER_PROFILE` | TBD | SET | Profile index + full profile data |
| `MSP2_BETAFLIGHT_MIXER_PROFILE_STATUS` | TBD | GET | Active profile, transition state, progress |

GET payload for `MSP2_BETAFLIGHT_MIXER_PROFILE`:
```
Request:  [0] uint8 profileIndex
Response: [0]    uint8  profileIndex
          [1]    uint8  motorCount
          [2]    uint8  servoCount
          [3]    uint8  mixerMode
          [4]    uint8  platformType
          [5-6]  uint16 transitionTimerMs
          [7]    uint8  transitionCurve
          [8]    uint8  linkedPidProfile
          [9]    uint8  linkedRateProfile
          [10..] motorMixer_t × motorCount (16 bytes each: 4 × float32)
          [..]   servoMixer_t × servoCount
```

STATUS payload:
```
Response: [0]   uint8  activeProfile
          [1]   uint8  transitionState (enum)
          [2-3] uint16 transitionProgress (0-10000, scaled float×10000)
          [4]   uint8  profileCount
```

### 5.2 OSD elements

```c
OSD_MIXER_PROFILE    — renders "M1", "M2", or "M1→2" / "M2→1" during transition
```

Single element, compact. Uses existing OSD element infrastructure. Conditionally compiled under `USE_MIXER_PROFILE && USE_OSD`.

### 5.3 Blackbox fields

Add to slow frame:
```c
{"mixerProfile",          UNSIGNED, PREDICT(0), ENCODING(UNSIGNED_VB), CONDITION(MIXER_PROFILE)}
{"mixerTransitionState",  UNSIGNED, PREDICT(0), ENCODING(UNSIGNED_VB), CONDITION(MIXER_PROFILE)}
```

Add to blackbox header:
```
mixer_profile_count
failsafe_mixer_profile
transition_timer_ms (both profiles)
```

### 5.4 Debug mode

Add `DEBUG_MIXER_PROFILE` to debug modes:
```
[0] = active profile index
[1] = transition state
[2] = transition progress × 1000
[3] = switch blocked reason (0=allowed, 1=in_transition, 2=arm_lockout, 3=alt_hold, 4=pos_hold, 5=gps_rescue, 6=launch_ctrl, 7=crashflip, 8=msp_override)
```

The blocked reason enum doubles as the source for the OSD switch-blocked warning. When `mixerProfileSwitchAllowed()` returns false, it sets a `lastBlockedReason` that debug and OSD can read. This costs ~10 lines and saves significant bench debugging time — the pilot immediately sees WHY a switch was rejected.

### 5.5 OSD switch-blocked warning

When the pilot toggles `BOXMIXERPROFILE` and the switch is rejected, briefly flash a warning on the OSD warnings line:

```
MIX SW BLOCKED: POSHOLD
MIX SW BLOCKED: GPS RESCUE
MIX SW BLOCKED: 2S ARM LOCK
```

Uses the existing OSD warnings infrastructure. Displayed for 2s after a rejected switch attempt, then clears. ~20 lines in `osd_warnings.c`.

### 5.6 Files touched

| File | Change |
|------|--------|
| `src/main/msp/msp.c` | MSP2 handlers (GET/SET profile, GET status) |
| `src/main/msp/msp_protocol_v2_betaflight.h` | Command ID defines |
| `src/main/osd/osd.h` | `OSD_MIXER_PROFILE` element ID |
| `src/main/osd/osd_elements.c` | Element rendering function |
| `src/main/osd/osd_warnings.c` | Switch-blocked warning |
| `src/main/blackbox/blackbox.c` | Slow frame fields, header entries |
| `src/main/blackbox/blackbox_fielddefs.h` | Field condition |
| `src/main/build/debug.h` | `DEBUG_MIXER_PROFILE` mode |

### 5.7 Acceptance Criteria

- [ ] MSP2 GET returns correct profile data for both profiles
- [ ] MSP2 SET writes profile data, survives reboot
- [ ] MSP2 STATUS reflects real-time active profile and transition state
- [ ] Round-trip test: SET then GET produces identical data
- [ ] OSD: displays "M1" when stable in profile 1
- [ ] OSD: displays "M1→2" during transition
- [ ] OSD: switch-blocked warning appears with correct reason when switch rejected
- [ ] Blackbox: profile switch events visible in log viewer
- [ ] Debug mode: all 4 values update correctly (including blocked reason)
- [ ] No MSP handler crashes with malformed packets (bounds checking)

### No-Go Gate
Cannot proceed to bench/flight testing if:
- MSP round-trip fails (data corruption)
- OSD rendering causes frame time overrun

---

## Phase 6: Testing and Validation

**Goal:** Systematic bench and flight validation before PR submission.  
**Duration:** 3-6 weeks (hardware dependent)  
**Dependencies:** All firmware phases (1-5) complete  

### 6.1 Unit tests (`test/unit/mixer_profile_unittest.cc`)

```
Core tests:
- PG defaults: motorCount=0, transitionTimerMs=1500, curve=LINEAR
- Superset computation: max(4,1)=4 motors, max(0,4)=4 servos
- Transition progress: linear 0→1 over duration
- Transition progress: S-curve smooth at endpoints
- Transition progress: clamped to [0,1]
- Profile switch: activeMix pointer updated
- Profile switch: idle outputs set for inactive motors
- Profile switch: linked PID/rate profiles applied
- Switch lockout: rejected during transition
- Switch lockout: rejected during first 2s after arm
- Single profile (count=1): all mixer_profile code effectively no-op

Runtime reconciliation tests:
- motorOutputHigh/Low recomputed on switch
- disarmMotorOutput correct for each profile's protocol
- motorCount updated to active profile count (not superset)
- RPM limiter state reset on switch
- Crashflip state cleared on switch

Orientation offset tests:
- ORIENTATION_OFFSET_NONE: attitude values unchanged after profile switch
- ORIENTATION_OFFSET_TAILSITTER: roll/yaw axes correctly swapped
- ORIENTATION_OFFSET_TAILSITTER: pitch axis unchanged (rotation is around pitch)
- Offset applied continuously (every IMU cycle), not just on switch event
- Default (NONE): zero overhead — early return before any math

Failsafe tests:
- SWITCH_IMMEDIATE: immediate switch, no transition
- FAST_BLEND: 50ms blend initiated
- HOLD_CURRENT: no switch occurs
- Failsafe aborts in-progress transition

Blocked mode tests:
- Switch rejected during Alt Hold
- Switch rejected during Position Hold
- Switch rejected during GPS Rescue
- Switch rejected during Launch Control
- Switch rejected during Crashflip
- Switch rejected during MSP Override
- Switch allowed during Acro, Angle, Passthrough

Transition-only motor tests:
- Spin only when MIXER_TRANSITION active
- Stopped when MIXER_TRANSITION inactive

Arming tests:
- Blocked with 0 motors in active profile
- Blocked with 3D feature enabled and profileCount=2
- Passes with valid single-profile config

Migration tests:
- Quad custom mixer → profile 0
- Custom airplane → profile 0
- Flying wing → profile 0
- Profile 1 initialized to clean defaults
```

### 6.2 SITL validation

- 1000× automated hover↔FW transition cycles, monitor for output anomalies
- Failsafe injection at random points during transition
- Rapid AUX toggling stress test (10Hz switch rate)
- Profile switch with all combinations of linked PID/rate profiles

### 6.3 Bench test protocol (props off, logic analyzer on all outputs)

| Test | Method | Pass criteria |
|------|--------|---------------|
| Boot superset allocation | Power on, observe init pulses | All superset outputs get init pulse |
| Profile 1 outputs | Arm, raise throttle | Only profile 1 motors spin, others at MOTOR_STOP |
| Profile switch (disarmed) | Toggle AUX | Immediate switch, no glitch |
| Profile switch (armed) | Toggle AUX | Smooth blend over configured time |
| S-curve blend | Toggle AUX, capture waveform | Smooth acceleration/deceleration |
| Transition-only motor | Enable MIXER TRANSITION | Motor spins at configured %, others unaffected |
| Failsafe | Cut radio link | Immediate switch to failsafe profile |
| Rapid toggle | Toggle AUX 5× in 1s | Only first switch processes, no crash |
| Power cycle | Toggle profile, power cycle, reboot | Active profile reverts to 0 on boot |
| Runtime reconciliation | Switch, read debug values | motorOutputHigh/Low match active profile |
| Blocked mode enforcement | Activate Alt Hold, attempt switch | Switch rejected |
| Servo config consistency | Different smix per profile, same servo params | Servo endpoints consistent across switch |
| 3D mode rejection | Enable 3D + profileCount=2, attempt arm | Arming blocked |
| Tailsitter orientation offset | Set `orientation_offset = TAILSITTER`, switch profiles, read attitude debug | Roll/yaw axes swap, pitch unchanged |

### 6.4 Flight test protocol

**Prerequisites:**
- Bench test 100% pass
- VTOL airframe with known-good hover and FW modes (tested separately on stock BF)
- Recovery plan for each test (altitude, bailout switch, failsafe config)

| # | Test | Altitude | Risk | Recovery |
|---|------|----------|------|----------|
| F1 | Hover only, profile 1 | >20m | Low | Standard failsafe |
| F2 | FW only, profile 2 (hand launch) | >30m | Medium | FW failsafe or manual |
| F3 | Hover → FW transition, 5s timer | >50m | High | Abort to hover via AUX |
| F4 | FW → hover transition, 3s timer | >50m | High | Maintain throttle, manual recovery |
| F5 | Full circuit: hover→FW→hover | >50m | High | Abort to hover at any point |
| F6 | Failsafe during hover | >30m | Medium | Let failsafe do its thing |
| F7 | Failsafe during FW | >50m | High | Failsafe → hover → land |
| F8 | Failsafe during transition | >50m | High | Should abort transition, switch to hover |
| F9 | Transition-only motor buildup | >30m | Medium | Manual abort |
| F10 | Airspeed gate (if pitot equipped) | >50m | High | Timer safety timeout as backup |

### 6.5 Blackbox analysis checklist

After each flight test:
- [ ] Profile switch event timestamps match AUX channel transitions
- [ ] Motor outputs during blend show smooth interpolation (no spikes)
- [ ] PID I-term: verify reset on transition finalize
- [ ] PID D-term: verify continuous (no reset)
- [ ] Motor outputs: verify idle motors at MOTOR_STOP during stable profiles
- [ ] Failsafe event: verify immediate switch (no blend artifacts)

---

## Phase 7: PR Preparation and Submission

**Goal:** Clean, reviewable PR with passing CI and comprehensive documentation.  
**Duration:** 1-2 weeks  
**Dependencies:** Phase 6 flight testing complete, no blocking issues  

### 7.1 PR 1 commit structure (core plumbing)

```
1. feat(mixer): add USE_MIXER_PROFILE gate and mixerProfile_t PG definition
2. feat(cli): add mixer_profile context and per-profile mmix/smix commands
3. feat(config): EEPROM migration for mixer profile data
4. feat(mixer): superset motor/servo allocation at boot
5. feat(mixer): runtime profile switching with state reconciliation
6. feat(mixer): arming checks for mixer profiles
7. test(mixer): mixer profile unit tests (PG, superset, switch, arming)
```

### 7.2 PR 2 commit structure (transition behavior)

```
1. feat(mixer): transition state machine with linear and S-curve blending
2. feat(modes): add BOXMIXERPROFILE and BOXMIXERTRANSITION AUX modes
3. feat(mixer): transition-only motor rules
4. feat(mixer): blocked mode enforcement for profile switching
5. feat(failsafe): configurable mixer profile failsafe action
6. feat(mixer): linked PID/rate profile auto-switching
7. test(mixer): transition, failsafe, blocked mode unit tests
```

### 7.3 PR 3 commit structure (observability)

```
1. feat(msp): MSP2 mixer profile read/write/status protocol
2. feat(osd): mixer profile OSD element and switch-blocked warning
3. feat(blackbox): mixer profile logging fields and header
4. feat(debug): DEBUG_MIXER_PROFILE debug mode with blocked-switch reason
5. docs: mixer profile user guide
```

Each commit must:
- Build clean on all CI targets
- Not break existing functionality (bisectable)
- Have a clear, descriptive message following BF conventions

### 7.4 Documentation deliverable

`docs/MixerProfile.md`:
- Overview and VTOL use case
- v1 airframe support matrix (from roadmap)
- CLI setup walkthrough (quad+pusher VTOL example)
- Parameter reference table
- Transition-only motor rules explanation
- Transition command semantics (L11: mid-transition requests dropped)
- Failsafe behavior (all three modes)
- v1 limitations: global servo params (L9), no 3D mode, no auto-transition (L10), manual only
- Timer/protocol compatibility note
- Blocked modes list with user-visible reasons
- Example `diff all` for a complete VTOL config

### 7.5 Reviewer evidence pack (per PR)

Each PR submission includes concrete artifacts so maintainers can reason from real data, not just roadmap text:

| Artifact | PR 1 | PR 2 | PR 3 |
|----------|------|------|------|
| Working `diff all` (quad+pusher VTOL) | ✓ | ✓ (updated with transition config) | ✓ (final) |
| Intentionally-rejected config (e.g., 3D + profiles) | ✓ | ✓ (blocked mode rejection) | — |
| Logic analyzer screenshots (boot init, profile switch) | ✓ | ✓ (transition blend waveform) | — |
| Blackbox transition clip | — | ✓ | ✓ (with OSD overlay) |
| Flash size delta table (F405, F411, H7) | ✓ | ✓ | ✓ |

### 7.6 PR descriptions

Each PR includes:
- Architecture summary (superset allocation, pointer swap, transition blend)
- Prior art references (iNav PR #8555, ArduPilot QuadPlane)
- Testing evidence (linked from reviewer evidence pack)
- Flash size impact table (per target)
- Configurator PR link (or "planned separately")
- v1 airframe support matrix (from roadmap)

### 7.7 Acceptance Criteria (overall)

- [ ] All CI targets pass
- [ ] Flash size increase < 4KB on F405 (entire feature across all 3 PRs)
- [ ] Zero size increase on F411 (feature compiled out)
- [ ] `diff all` backwards compatible (no output change for non-mixer-profile users)
- [ ] At least 3 successful VTOL transition flights logged
- [ ] Blackbox logs attached showing clean transitions
- [ ] Documentation complete (including airframe support matrix and blocked modes list)
- [ ] Unit tests pass
- [ ] 2 new BOX modes visible and functional in Betaflight App Modes tab (verified on current App release)
- [ ] Reviewer evidence pack complete for each PR

---

## Assumptions

Things that would invalidate this plan if they turn out to be false:

| # | Assumption | Validation method |
|---|-----------|-------------------|
| A1 | `timerAllocate()` can handle superset motor count without exhausting DMA on F4 | Phase 2 bench test on actual F405 hardware |
| A2 | PPM Rx timer clash avoidance works with larger motor count | Phase 2 bench test with PPM receiver |
| A3 | `mixTable()` double-evaluation during blend fits in PID loop budget on F4 at 8KHz | Phase 3 CPU load measurement |
| A4 | BF maintainers accept the superset allocation architecture | Phase 0 Discord discussion |
| A5 | permanentId and MSP2 command IDs are available for allocation | Phase 0 coordination |
| A6 | `pidInitConfig()` re-init is sufficient for platform type change (no deeper PID state issues) | Phase 2 bench test, Phase 6 flight test |
| A7 | Existing ESCs handle DSHOT_CMD_MOTOR_STOP on previously-spinning outputs without desync | Phase 2 bench test with actual ESCs |
| A8 | Global servo params (L9) are sufficient for common VTOL topologies | Phase 6 flight test with tilt-rotor or elevon VTOL |
| A9 | `mixerRuntime` reconciliation list (Phase 2.4) is complete — no other derived fields affect output correctness | Phase 2 bench test + code audit of `mixer_init.c` |
| A10 | Mixed DShot + PWM servo outputs on typical VTOL targets don't have timer bank conflicts | Phase 0 timer spot-check |
| A11 | Betaflight App Modes tab can discover and display 2 additional BOX modes without UX breakage | Phase 7 merge gate — verify on current App release before firmware PR merges |

---

## Handoff Packet (Phase 1 Start)

When ready to begin implementation after Phase 0 clears:

### Pre-flight checklist
1. Fork is on latest `master`
2. `make TARGET=STM32F405` passes clean
3. `make TARGET=STM32F411` passes clean
4. `make TARGET=STM32H743` passes clean
5. permanentId values confirmed
6. MSP2 command IDs confirmed
7. Next available PG ID confirmed (check `pg/pg_ids.h`)

### First implementation task
Create `src/main/flight/mixer_profile.h` and `mixer_profile.c` with:
- PG definition and registration
- Default values (profile 0 = empty, will be populated by migration)
- Stub functions for runtime API (return immediately, no-op)
- Feature gate in `common_pre.h`
- Add source file to build system

### Verification command
```bash
make TARGET=STM32F405 && make TARGET=STM32F411 && make TARGET=STM32H743
# All three must pass
# F411 binary size must not increase
```


---

## Revision History

| Date | Rev | Changes |
|------|-----|---------|
| 2026-03-20 | 1 | Initial roadmap |
| 2026-03-20 | 2 | Incorporated external review: added L9 (global servo params), L10 (manual transitions only); added mixerRuntime reconciliation section (2.4) with field disposition table; added blocked modes policy table (4.4); changed failsafe from hardcoded to configurable enum (L6, 4.5); added timer compatibility spot-check to Phase 0; adopted 3-PR split strategy; added 3D mode arming block; expanded unit/bench test cases; added migration tests; added A8-A10 assumptions. |
| 2026-03-20 | 3 | Second review round: added L11 (mid-transition requests dropped); added v1 airframe support matrix; added switch-blocked OSD warning and debug reason enum (5.4, 5.5); added reviewer evidence pack per PR (7.5); updated Phase 7 acceptance criteria with App mode visibility gate; updated docs deliverable to include airframe matrix and transition semantics. |
| 2026-03-20 | 4 | Final polish: added canonical quad+pusher example config in summary; added "PR 1 does not" exclusion note under PR table; added explicit rollback/disable path in migration (1.4); added A11 assumption (App mode visibility). Roadmap locked for Phase 0 execution. |
| 2026-03-20 | 5 | Added tailsitter orientation offset: L12 locked decision, `orientationOffset_e` enum + field in `mixerProfile_t` (Phase 1), Phase 2.5 IMU rotation implementation in `imu.c`, CLI `set orientation_offset`, updated airframe support matrix (tailsitter now fully supported, added tailsitter-with-tilt-motors row), added orientation test cases to Phase 2 acceptance criteria and Phase 6 unit/bench tests. Removed Carlson `vtol-motor-mix` fork check from Phase 0 (5-year-old dead branch, iNav mixer_profile is the authoritative reference). |
