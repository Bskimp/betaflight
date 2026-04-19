# Wing Target Pattern

How to make a Betaflight board's config wing-capable. This is the
firmware-side counterpart to the configurator's Wing Hardware sub-tab;
together they cover "which boards can fly fixed-wing" and "how users
configure their pads for wing use."

See [wing_targets.md](wing_targets.md) for the current inventory of
wing-capable boards shipping in `wing-main` releases.

## When to add a board to the wing list

Most F405/F722/H7 boards already used for quads have enough resources
to host a simple fixed-wing setup (1 motor + 2-4 servos + 2-3 UARTs).
The firmware is **lazy about output init** —
[src/main/fc/init.c:554](../src/main/fc/init.c#L554)
calls `motorDevInit(getMotorCount())` which only claims timer/DMA for
motors the mixer actually drives, and the servo init path is similar.

Practical consequence: you don't need to rewrite a board's
`TIMER_PIN_MAP` for wing use. Declaring 8 motor pads on a board where
a wing pilot only uses 1-2 costs zero extra resources at runtime — the
unused slots stay idle.

Cases where wing-enablement **isn't** appropriate:
- F411 boards with already-tight DMA/timer budgets. The wing use case
  (RPM filter on 2 motors + PWM on 4 servos + CRSF full-duplex) can
  exceed what the MCU has.
- Boards where physical pad routing puts all motor outputs on a
  timer group that conflicts with any available servo PWM timer.
  Rare on F405+, possible on tightly packed AIOs.
- Boards where the `TIMER_PIN_MAP` was authored to cram quad-only
  peripherals across every timer — no free PWM capacity for wing
  use. Phase 3 (`CubeIDE-driven target generation`) is the long-term
  answer here; for now, skip these boards.

## The two-line change

For a board that qualifies, adding wing support is two `#define`s in
the target's `config.h` (in the `Bskimp/config:wing-main` submodule):

```c
#define BOARD_NAME        MY_BOARD
#define MANUFACTURER_ID   MANU

#ifndef USE_WING
#define USE_WING
#endif

#ifndef USE_SERVOS
#define USE_SERVOS
#endif

// ... rest of config.h unchanged
```

Place the defines after `BOARD_NAME` / `MANUFACTURER_ID` and before
`USE_ACC` / other feature defines. Using `#ifndef` guards keeps the
file safe if upstream `betaflight/config` ever adds these defines
directly.

**Don't** declare `SERVO1_PIN`, `SERVO2_PIN`, ... in the config. They'd
collide with the existing `MOTOR*_PIN` declarations (same pads can't
be two roles at compile time). The runtime `resource SERVO N <pad>`
path — which the configurator's Wing Hardware sub-tab issues
automatically on preset apply — handles servo pad binding without
requiring pin declarations.

## Worked example: FLYWOOF405S_AIO

Before (stock quad config):
```c
#define BOARD_NAME        FLYWOOF405S_AIO
#define MANUFACTURER_ID   FLWO

#define USE_ACC
// ... 8 motor pads declared, 0 servo pads
```

After (wing-enabled):
```c
#define BOARD_NAME        FLYWOOF405S_AIO
#define MANUFACTURER_ID   FLWO

#ifndef USE_WING
#define USE_WING
#endif

#ifndef USE_SERVOS
#define USE_SERVOS
#endif

#define USE_ACC
// ... same 8 motor pads, 0 servo pads — unchanged
```

On flash: board still boots with 4 motors claimed (stock quad mixer).
User connects the wing configurator, picks a wing preset (e.g. Flying
Wing = 1 motor + 2 servos), the Hardware sub-tab's AIO-mode remap
flow releases M3+M4+M5+... and binds SERVO 1+2 to free PWM pads
(declared-but-unclaimed `TIMER_PIN_MAP` entries, or the LED_STRIP pad
if the user opts in via the Release LED_STRIP checkbox). Save +
reboot. Wing is live.

## Two-tier audit gate

Before adding a board to [wing_targets.md](wing_targets.md), confirm:

**Tier 1 — Simple wing (minimum bar):**
- 1 motor with bidir DSHOT + RPM filter working
- 2-4 servo outputs driveable concurrently with the motor (via
  the runtime remap flow, not necessarily in config.h)
- 2 UARTs free: CRSF full-duplex + one of {GPS, VTX, OSD}

**Tier 2 — Max wing (badged full-featured):**
- 2 motors with bidir DSHOT + RPM filter on BOTH
- 4-7 servo outputs driveable concurrently
- 3+ UARTs free at wing-typical rates:
  - CRSF full-duplex @ 420k (RX + TX)
  - GPS @ 115k
  - VTX (SmartAudio / Tramp) on TX, or MSP-DisplayPort on RX for DJI/HDZero
- Analog OSD path (MAX7456) works if the board has one

Tier 1 is the bar for the inventory table. Tier 2 is a "full-featured"
badge in the Status column notes.

## Static-analysis checklist

When hardware isn't available, you can still validate a config by
reading its `TIMER_PIN_MAP` + motor/servo pin declarations and
checking against the MCU's timer+DMA capability:

1. Count the motor pads declared. On F4/F7 boards with 4+ motor
   declarations, the wing use case is covered (user releases extras
   via runtime remap).
2. Check that `TIMER_PIN_MAP` has at least N+2 entries where N is
   the motor count. Extras become servo candidates.
3. Verify at least 2 UARTs are declared (UART1 TX/RX + at least one
   other pair). Wing minimum needs 2 working UARTs.
4. Check for `USE_MAX7456` or an OSD-related define if the board has
   an analog OSD. Not strictly required for wing but nice to confirm.
5. Confirm `USE_GPS` is either defined or available via build option —
   wing users almost always want GPS rescue.

A board passing all 5 is safe to add at `static` status. Flag as
`bench` once someone flies it.

## Runtime remap flow summary

Users don't edit `config.h` — they use the configurator. Typical flow:

1. Flash wing-main hex for their board (from the release matrix)
2. Open wing configurator → Wing Tuning tab → Hardware sub-tab
3. Inspect what firmware has claimed (motors, servos, LED_STRIP, UARTs)
4. Go to Mixer sub-tab, pick an airframe preset (Standard Plane,
   Flying Wing, etc.)
5. Apply preset → configurator computes resource remap using the
   board's `TIMER_PIN_MAP` (live via `timer` CLI dump) + preset's
   motor/servo needs. Sends resource + mmix + smix + save in one
   combined CLI+MSP batch. FC reboots.
6. Hardware sub-tab refreshes post-reboot, shows the new claim state.

For boards where step 5 doesn't work cleanly (AIO with non-exposed
pads, or a config whose `TIMER_PIN_MAP` boxes out servo options), the
Hardware sub-tab offers opt-in LED_STRIP release and spare-UART
release to expand the pad candidate pool.

## Critical files

Firmware-side:
- Submodule root: `src/config/configs/<TARGET>/config.h` (in
  `Bskimp/config:wing-main`)
- Lazy init paths this pattern relies on:
  - [src/main/fc/init.c](../src/main/fc/init.c) — motor + servo init
  - [src/main/flight/mixer_init.c](../src/main/flight/mixer_init.c) —
    `getMotorCount()` driving `motorDevInit`
  - [src/main/drivers/timer_common.c](../src/main/drivers/timer_common.c) —
    `timerGetAllocatedByNumberAndChannel()` enforces collision
    detection at runtime

Configurator-side (in `Bskimp/betaflight-configurator:wing-main`):
- `src/js/utils/wingResourceAnalyzer.js` — parses runtime state
- `src/js/utils/wingRemapRecommender.js` — computes per-board remap
- `src/js/utils/cliOneShot.js` — CLI reader/parser for `timer show` etc.
- `src/components/tabs/WingTuningTab.vue` — the UI that drives it all
