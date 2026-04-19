# Wing-capable Targets

Board configs in the `wing-main` firmware fork that have `USE_WING`
and `USE_SERVOS` enabled. These are the targets whose hex builds are
usable with the wing configurator's Wing Tuning tab (tuning, mixer,
auto-launch, GPS rescue, hardware remap).

See [wing_target_pattern.md](wing_target_pattern.md) for how to add
a new board to this list.

## Status legend

- **bench** — physically flashed + tested by the author on the listed
  airframe class. Known to boot, arm, respond to preset-apply flow,
  and drive motors/servos correctly.
- **static** — config.h present in `Bskimp/config:wing-main` with
  `USE_WING` defined; compile-tested by the CI matrix but not yet
  validated on hardware. Community bench reports welcomed.
- **community** — verified by a user other than the author. Reported
  via Discord / RCGroups / PR description; status line should include
  who confirmed.

## Inventory

### F405 (STM32F405)

| Target              | Motors | Servos | UARTs | LED_STRIP | Status   | Notes |
| ------------------- | ------ | ------ | ----- | --------- | -------- | ----- |
| SPEEDYBEEF405WING   | 2      | 7      | 6     | yes       | bench    | Reference wing target. Bidir DSHOT via TIMUP burst. |
| MATEKF405WINGV2     | 2      | 7      | 6     | yes       | static   | Wing-native; widely deployed in the community. |
| BOTWINGF405         | 2      | 7      | 6     | yes       | static   | Wing-native. |
| JHEF405WING         | 2      | 7      | 6     | yes       | static   | Wing-native. |
| KAKUTEF4WING        | 2      | 7      | 6     | yes       | static   | Wing-native. |
| IFLIGHT_F405_TWING  | 2      | 7      | 6     | yes       | static   | Wing-native; iFlight Twinboom variant. |
| FURYF4OSD           | 4      | 0      | 3     | yes       | bench    | **Quad-converted.** `USE_WING` added in wing-main fork of config; motor-to-servo remap via Hardware sub-tab (discrete mode). |
| FLYWOOF405S_AIO     | 8      | 0      | 6     | yes       | bench    | **Quad-converted AIO.** Motor pads are ESC-soldered; use AIO mode in remap tool + opt-in LED_STRIP release for extra servo pad. |

### F722 (STM32F722)

| Target              | Motors | Servos | UARTs | LED_STRIP | Status   | Notes |
| ------------------- | ------ | ------ | ----- | --------- | -------- | ----- |
| BOTWINGF722         | 2      | 7      | 6     | yes       | static   | Wing-native. |
| IFLIGHT_F722_TWING  | 2      | 7      | 6     | yes       | static   | Wing-native. |
| VWING_F722AIO       | 2      | 7      | 6     | yes       | static   | Wing-native AIO. |

### H7 (STM32H743/H7A3)

| Target              | Motors | Servos | UARTs | LED_STRIP | Status   | Notes |
| ------------------- | ------ | ------ | ----- | --------- | -------- | ----- |
| KAKUTEH7WING        | 2      | 7      | 8     | yes       | static   | Wing-native H7. |
| IFLIGHT_H7_TWING    | 2      | 7      | 8     | yes       | static   | Wing-native H7. |

## Release build matrix

Every target in the inventory above is built as part of the
`wing-<YYYY>.<MM>.<N>` firmware release tag. Hex files follow the
BF convention: `betaflight_<version>_<MCU>_<TARGET>.hex`.

CI runs `make CONFIG=<target>` for each inventory entry on every
wing-main push; any target that fails to compile blocks the release.

## Contributing a board

Add `USE_WING` + `USE_SERVOS` to a board's `config.h` in
`Bskimp/config:wing-main`, build-test, add a row to this table,
PR against `Bskimp/config` (submodule) + `Bskimp/betaflight` (this
repo, for the pointer bump + doc update). See
[wing_target_pattern.md](wing_target_pattern.md) for the full flow.

Community bench reports graduate a row from `static` to `community`
or `bench` — file a report via GitHub issue on this repo with:
- Target name + MCU
- Airframe class flown (elevon, standard plane, twin, V-tail)
- Motor + servo count actually used
- Any quirks encountered (DMA clashes, pads not broken out, etc.)
