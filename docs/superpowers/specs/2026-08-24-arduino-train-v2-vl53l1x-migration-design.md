# Arduino Train v2: VL53L0X to VL53L1X Migration

Date: 2026-08-24
Status: Approved

## Problem

The train's time-of-flight distance sensor is being replaced with a VL53L1X. The sketch currently
contains a hand-written register-level VL53L0X driver, so this is a hardware migration rather than a
rename. The two chips are not protocol compatible: the VL53L0X uses 8-bit register addresses, while
the VL53L1X uses 16-bit register addresses. Every I2C access primitive changes, not only the names.

The XSHUT wiring also moves from D3 to A3 on the new module.

The sketch targets an Arduino Nano (ATmega328P), so flash and SRAM are scarce. The migration must not
increase memory use, which rules out a full third-party VL53L1X library.

## Goals

- Drive a VL53L1X over the shared I2C bus at address `0x2A`.
- Keep flash and SRAM use at roughly current levels.
- Preserve the existing distance behaviour, error codes, and sleep/wake power handling.
- Expose the VL53L1X detection cone as builder-tunable settings in `config.h`.
- Retain the old VL53L0X driver as reference code that costs nothing when disabled.

## Non-goals

- No changes to `libraries/`, the standalone `time-of-flight*` sketches, or historical documents
  under `docs/superpowers/`.
- No new distance features such as long-range mode, interrupt-driven ranging, or calibration
  routines.
- No runtime switching between VL53L0X and VL53L1X hardware.

## Design

### Configuration and wiring

In `config.h`:

- `constexpr uint8_t vl53l0xAddress = 0x2A;` becomes `vl53l1xAddress`, keeping the value `0x2A`.
- `constexpr int pinVL53L0X_XSHUT = 3;` becomes `constexpr int pinVL53L1X_XSHUT = A3;`.
- The `distanceTofTimeoutMs`, `distanceTofTimingBudgetUs`, and `distanceTofContinuousPeriodMs`
  comments are updated to say VL53L1X. Their values do not change.
- A new feature switch is added beside the other switches:
  `#define ENABLE_VL53L0X_LEGACY_DRIVER 0`.
- New detection-cone settings are added, described below.

A3 is free: A0 is battery sense, and A4/A5 are the I2C bus.

The sensor still boots at its `0x29` default, which collides with the TCS34725. The existing XSHUT
startup sequence, which holds the sensor in reset and then reassigns it to `0x2A`, is unchanged
apart from the renamed pin constant.

### Detection cone settings

The VL53L1X aims and narrows its detection cone by selecting a region of interest (ROI) on its
16x16 SPAD array. A smaller ROI narrows the cone, which rejects floor and side-wall reflections at
the cost of maximum range. This is a genuine VL53L1X capability with no VL53L0X equivalent, so these
settings are new rather than renamed.

`config.h` gains a dedicated block next to the existing distance-sensor timing settings:

```cpp
// Distance-sensor detection cone (VL53L1X region of interest).
constexpr uint8_t distanceTofRoiWidth = 4;         // SPAD columns, 4..16. Smaller = narrower cone.
constexpr uint8_t distanceTofRoiHeight = 4;        // SPAD rows, 4..16. Smaller = narrower cone.
constexpr uint8_t distanceTofRoiCenterSpad = 199;  // Aims the cone. 199 is the optical centre.
```

The shipped default is the minimum 4x4 window, giving the narrowest cone of roughly 15 degrees
versus the 27 degrees of the full 16x16 array. The train points its sensor forward along the track,
where a tight cone is wanted so the floor and passing scenery do not register as obstacles.

`distanceTofRoiCenterSpad` lets a builder aim the cone up, down, or sideways to compensate for how
the sensor sits in the locomotive body without physically remounting it. `199` is the array's
optical centre and is the correct value for a squarely mounted sensor.

The driver applies these through a `setRoi(width, height, centerSpad)` call after `init()` and
before `startContinuous()`. It writes the centre SPAD to register `0x007F` and the packed size to
register `0x0080` as `((height - 1) << 4) | (width - 1)`. Widths and heights are clamped to the 4..16
range the hardware accepts, so an out-of-range value in `config.h` degrades to the nearest legal cone
instead of leaving the sensor misconfigured.

A 4x4 ROI roughly halves maximum range, but the existing thresholds are unaffected:
`AUTO_DISTANCE_MAX_SPEED` at 50 cm is the longest distance the auto-drive logic considers, and that
stays comfortably inside the narrowed cone's reach. No threshold retuning is required.

### Driver replacement

`TrainDistanceSensorVL53L0X` in `40-sensors.ino` is replaced by `TrainDistanceSensorVL53L1X`. The
existing public API is preserved so no current call site changes shape. The only addition is
`setRoi()`, which exposes the new detection-cone capability:

| Member | Purpose |
| --- | --- |
| `setTimeout(uint16_t)` | I/O timeout in milliseconds |
| `setAddress(uint8_t)` | Reassign the I2C address after XSHUT release |
| `init(bool io2v8 = true)` | Identify and configure the sensor |
| `setMeasurementTimingBudget(uint32_t)` | Per-measurement time budget |
| `setRoi(uint8_t, uint8_t, uint8_t)` | Detection cone size and aim (new for VL53L1X) |
| `startContinuous(uint32_t periodMs = 0)` | Begin continuous ranging |
| `readRangeContinuousMillimeters()` | Latest distance in millimetres |
| `timeoutOccurred()` | Sticky I/O timeout flag |
| `lastRangeReadValid() const` | Whether the last reading was usable |
| `logStartupI2cState(const __FlashStringHelper*)` | Debug-only bus trace |
| `logModelId()` | Debug-only identity trace |

The retained instance fields are `address`, `ioTimeoutMs`, `didTimeout`, `lastReadValid`, and
`timeoutStartMs`. The VL53L0X-specific `stopVariable` and `measurementTimingBudgetUs` fields are
dropped, which offsets the slightly larger 16-bit register helpers and keeps SRAM flat.

The internals become a minimal VL53L1X implementation:

- 16-bit register read and write helpers replacing the 8-bit ones.
- Identity check reading `0x010F`, expecting `0xEACC`.
- Address assignment through register `0x0001`.
- ST's compact default configuration block written at boot, stored in `PROGMEM` so it occupies flash
  rather than SRAM.
- Distance mode and timing budget applied from the existing `config.h` values.
- Detection cone applied from the new `config.h` ROI values.
- Continuous ranging that reads range status from `0x0089` and the final crosstalk-corrected
  distance in millimetres from `0x0096`.

Only readings whose range status indicates a valid measurement set `lastReadValid`. All other
statuses are treated the same way the current driver treats a failed read, so the existing fault and
recovery path is reused unchanged.

### Retained legacy driver

A new sketch tab `arduino-train-v2/90-legacy-vl53l0x.ino` holds the current
`TrainDistanceSensorVL53L0X` struct verbatim, preceded by a comment explaining that it is retired
reference code for the superseded sensor.

Arduino concatenates every `.ino` in the sketch root into one translation unit, so the file needs an
explicit guard:

```cpp
#include "config.h"
#if ENABLE_VL53L0X_LEGACY_DRIVER
  // ...entire old driver...
#endif
```

`config.h` is safe to include here because it uses `#pragma once`. With the switch at `0`, the
preprocessor strips the whole body, so the file costs zero flash and zero SRAM while remaining
visible in the sketch.

The legacy tab is fully disconnected: nothing references it, and `distanceTof` is an instance of the
new `TrainDistanceSensorVL53L1X`. As a frozen snapshot it deliberately keeps its own `0x29` and
`DefaultAddress` constants and does not track `vl53l1xAddress` or `pinVL53L1X_XSHUT`. Re-enabling it
would require reconnecting the call sites by hand.

### Call sites

The renamed pin, address, and log strings ripple through:

- `40-sensors.ino`: driver struct, `initDistanceSensorHardware()`, and the ranging path.
- `20-motor.ino` and `30-lights-and-sounds.ino`: XSHUT wake and shutdown writes.
- `50-power-management.ino`: sleep/wake XSHUT writes and the boot-error report string.
- `arduino-train-v2.ino`: wiring comments and the error-bit documentation block.

`ERR_DISTANCE_TOF` keeps its value of `0x04` and its meaning. Only the human-readable text changes
from VL53L0X to VL53L1X, so previously logged EEPROM boot errors stay comparable.

## Testing

Compile `arduino-train-v2` with `arduino-cli` and confirm the build succeeds with flash and SRAM at
or below the pre-migration figures. Repeat the compile with `ENABLE_VL53L0X_LEGACY_DRIVER` set to
`1` to confirm the legacy tab still parses, then restore it to `0` and confirm the memory figures are
identical to the guarded build.

Behavioural verification beyond compilation requires the physical VL53L1X module, so on-hardware
checks of ranging accuracy, address reassignment, cone width, and sleep/wake recovery are left to the
operator.

## Risks

The VL53L1X configuration block is the main risk: an incorrect or truncated sequence leaves the
sensor responding on I2C but returning unusable distances. The debug-only `logModelId()` and
`logStartupI2cState()` helpers are retained specifically to diagnose this on hardware.

The 4x4 cone is the second risk. It is deliberately the narrowest the hardware allows, so a sensor
mounted even slightly off-axis may aim past an obstacle. `distanceTofRoiCenterSpad` is the intended
remedy, and widening to 8x8 remains available if aiming proves fiddly on the real locomotive.
