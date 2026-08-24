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
meaningfully increase memory use, which rules out a full third-party VL53L1X library.

## Goals

- Drive a VL53L1X over the shared I2C bus at address `0x2A`.
- Keep flash and SRAM use close to current levels.
- Preserve the existing distance behaviour, error codes, and sleep/wake power handling.
- Expose the VL53L1X detection cone as builder-tunable settings in `config.h`.
- Retain the old VL53L0X driver as reference code that costs nothing when disabled.

## Non-goals

- No changes to `libraries/`, the standalone `time-of-flight*` sketches, or historical documents
  under `docs/superpowers/`.
- No calibration routines, interrupt-driven ranging, or multi-zone ROI scanning.
- No runtime switching between VL53L0X and VL53L1X hardware.

## Source of truth for register values

This driver is a size-reduced reimplementation of ST's VL53L1X Ultra Lite Driver (ULD). The VL53L1X
has no published full register map, and its working configuration is a long block of undocumented
magic values.

Every literal register address, configuration byte, and status mapping must be transcribed from ST's
ULD sources (`VL53L1X_api.c` and its default configuration array), or from Pololu's VL53L1X Arduino
library which mirrors them. They must not be reconstructed from memory during implementation.

The register addresses named in this document identify *which* values to transcribe. They are not a
substitute for the ULD listing.

## Design

### Configuration and wiring

In `config.h`:

- `constexpr uint8_t vl53l0xAddress = 0x2A;` (line 91) is renamed `vl53l1xAddress`. The value `0x2A`
  is unchanged.
- `constexpr int pinVL53L0X_XSHUT = A3;` (line 104) is renamed `pinVL53L1X_XSHUT`. The pin value is
  already `A3` in the checked-in file, so only the symbol name changes here.
- The `distanceTofTimeoutMs`, `distanceTofTimingBudgetUs`, and `distanceTofContinuousPeriodMs`
  comments are updated to say VL53L1X. Their values do not change.
- A new feature switch is added beside the other switches, following the existing override-safe
  pattern used by `DEBUG_DISTANCE_SENSOR` and friends:

  ```cpp
  #ifndef ENABLE_VL53L0X_LEGACY_DRIVER
  #define ENABLE_VL53L0X_LEGACY_DRIVER 0
  #endif
  ```

- New detection-cone settings are added, described below.

A3 is free: A0 is battery sense, and A4/A5 are the I2C bus.

### Startup, address assignment, and reset ordering

The sensor boots at its `0x29` default, which collides with the TCS34725 colour sensor. The TCS34725
is physically present and answering at `0x29`, so the VL53L1X must never be *configured* while still
at the default address. Only the address-assignment write itself happens there.

This constrains the ordering, and the constraint is the single most important correctness rule in
this design:

1. Drive XSHUT low, then high, to reset the sensor.
2. Wait a fixed, conservative boot delay. **Do not read any register while the sensor is still at
   `0x29`.** Both the VL53L1X and the TCS34725 acknowledge that address and would both drive SDA
   during a read, so a firmware-status byte read there is not trustworthy and could time out even
   with healthy hardware.
3. Write the 7-bit address `0x2A` to `I2C_SLAVE__DEVICE_ADDRESS` (`0x0001`). This is a write-only
   transaction, which is safe despite the shared address: it is exactly what the current VL53L0X
   code already does at `40-sensors.ino:986`, and the TCS34725 ignores it because its own register
   protocol requires the command bit `0x80` to be set on the first byte.
4. Poll `FIRMWARE__SYSTEM_STATUS` (`0x00E5`) at the new address `0x2A` until the low bit reads `1`,
   subject to `distanceTofTimeoutMs`. Now that the sensor is alone on `0x2A`, this read is reliable.
   It replaces a fixed delay as the real readiness gate, because the VL53L1X does not behave
   meaningfully before firmware boot completes.
5. Run initialisation at `0x2A`.

This same ordering applies on every sleep/wake XSHUT cycle in `50-power-management.ino`, not only at
cold boot, since XSHUT returns the sensor to `0x29` each time.

**`init()` must not perform a software reset.** Writing `SOFT_RESET` (`0x0000`), which the Pololu
library and several other VL53L1X implementations do as their first initialisation step, returns the
sensor to address `0x29` and silently recreates the TCS34725 collision. Any reset must happen only
in step 1 via XSHUT, before the address is assigned.

The existing code already assigns the address before calling `init()` (`40-sensors.ino:986`), so this
ordering matches the current structure.

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

The driver applies these through `setRoi(width, height, centerSpad)`, called after `init()` and
before `startContinuous()`. It writes the packed size to
`ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE` (`0x0080`) as `((height - 1) << 4) | (width - 1)`,
then the centre to `ROI_CONFIG__USER_ROI_CENTRE_SPAD` (`0x007F`).

ST's ULD `VL53L1X_SetROI()` writes these in the opposite order, centre first. Size-then-centre is
chosen deliberately so the centre is applied against an already-narrowed window, and either order
works because neither register takes effect until ranging starts.

Width and height are clamped to the 4..16 range the hardware accepts, so an out-of-range value in
`config.h` degrades to the nearest legal cone rather than leaving the sensor misconfigured.

Clamping the dimensions is not by itself sufficient. ST warns that a centre and size combination
whose window falls outside the SPAD array boundary produces range status 13 and no usable readings.
The driver does not attempt to validate this, because a correct check needs the SPAD location map
and that costs flash the sketch cannot spare. Instead the constraint is documented at the setting,
and the failure is diagnosable: status 13 surfaces through the normal invalid-reading path and, with
`DEBUG_DISTANCE_SENSOR` enabled, through the startup logs. A builder who moves the centre far from
199 with a small ROI is the only one exposed to this.

A 4x4 ROI roughly halves maximum range, consistent with ST's approximate 4 m to 2 m guidance. The
existing thresholds sit well inside that: `AUTO_DISTANCE_MAX_SPEED` at 50 cm is the longest distance
the auto-drive logic considers. This makes threshold retuning unlikely to be needed, but it is not a
guarantee, since real-world range also depends on target reflectance, ambient light, alignment, and
timing budget. Confirm on hardware.

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
dropped.

`io2v8` is kept in the signature for call-site compatibility but has no VL53L1X equivalent, since
2.8 V mode is part of the default configuration block. The implementation ignores it.

#### Access primitives and identity

- 16-bit register read and write helpers replace the 8-bit ones, with 8-, 16-, and 32-bit value
  variants.
- Identity check reads 16 bits at `IDENTIFICATION__MODEL_ID` (`0x010F`), expecting `0xEACC`.
  Bytewise this is `0x010F = 0xEA` and `0x0110 = 0xCC`.

#### Initialisation sequence

Executed at `0x2A`, after the boot poll and address assignment described above:

1. Write ST's default configuration block, registers `0x002D` through `0x0087`, from a `PROGMEM`
   array so it occupies flash rather than SRAM.
2. Perform the ULD's post-configuration calibration, which is required and is not part of the
   configuration block: start ranging, wait for data ready, clear the interrupt, stop ranging, then
   write `VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND` (`0x0008`) = `0x09` and register `0x000B` = `0x00`.
   Skipping this leaves a sensor that acknowledges on I2C but never returns usable distances.
3. Select **short distance mode**. The default configuration block is long mode, so this is an
   explicit change, not a default. Short mode is correct here: it is more robust to ambient light
   and its roughly 1.3 m ceiling still covers the 50 cm the auto-drive logic uses.
4. Apply the timing budget, then the ROI, then the intermeasurement period.

#### Timing budget and intermeasurement period

The VL53L1X accepts only discrete timing budgets: 15, 20, 33, 50, 100, 200, and 500 ms. The existing
`distanceTofTimingBudgetUs` of 33000 us is a supported value and is kept.

`setMeasurementTimingBudget` writes the ULD register pair for the requested budget. Unlike the ULD,
which rejects unsupported budgets outright, it snaps an unsupported request to the nearest supported
value and still returns success. This matches how the ROI dimensions degrade gracefully rather than
failing, and keeps a mistyped `config.h` value from disabling the distance sensor entirely. The
policy is documented at the setting so the behaviour is not surprising. It does not affect the
shipped configuration, which is already an exact match.

The intermeasurement period uses the VL53L1X formula, which is also not the VL53L0X one:

```text
value = (readReg16(0x00DE) & 0x03FF) * periodMs * 1.075
```

written as a 32-bit value to `SYSTEM__INTERMEASUREMENT_PERIOD` (`0x006C`).

The `1.075` factor is computed in integer arithmetic as `* 43 / 40` rather than in floating point,
which would otherwise link AVR floating-point support into flash for a single multiply. Overflow is
not a concern at the values involved: the oscillator reading is at most 1023 and `periodMs` is 50,
so the widest intermediate is about 2.2 million and fits comfortably in `uint32_t`.

The existing `distanceTofContinuousPeriodMs` of 50 ms is retained and is valid, since it exceeds the
33 ms budget.

#### Continuous read path

`readRangeContinuousMillimeters()` mirrors what the current VL53L0X read does at
`40-sensors.ino:407`, adapted to VL53L1X semantics:

1. Poll data ready via `GPIO__TIO_HV_STATUS` (`0x0031`) against the interrupt polarity derived from
   `GPIO_HV_MUX__CTRL` (`0x0030`). Note the inversion, which is easy to get backwards and yields the
   exact opposite result if dropped:

   ```cpp
   const uint8_t polarity = !((readReg8(0x0030) >> 4) & 0x01);
   const bool ready = (readReg8(0x0031) & 0x01) == polarity;
   ```

   Honour `ioTimeoutMs` and set `didTimeout` on expiry, as today.
2. Read `RESULT__RANGE_STATUS` (`0x0089`).
3. Read the distance in millimetres as a 16-bit value from
   `RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0` (`0x0096`).
4. Clear the interrupt by writing `0x01` to `SYSTEM__INTERRUPT_CLEAR` (`0x0086`). Without this the
   sensor never signals a new measurement and ranging stalls after the first reading.

The low five bits of `0x0089` are an internal raw status, not the public status code. They must be
masked with `0x1F` and interpreted through ST's mapping, in which **raw status 9 is the valid
result**. Testing the register against `0` is wrong and would reject every good measurement.

Since the train only needs valid-versus-invalid, the driver deliberately omits the full 24-entry
translation table and treats masked raw status 9 as valid and everything else as invalid. This is a
conscious size trade-off; the cost is that debug logs show raw rather than public status codes.

Only valid readings set `lastReadValid`. Every other status is treated exactly as the current driver
treats a failed read, so `maxConsecutiveTofFailures`, the latched distance fault, and the existing
recovery path are reused unchanged.

### Retained legacy driver

A new sketch tab `arduino-train-v2/90-legacy-vl53l0x.ino` holds the current
`TrainDistanceSensorVL53L0X` struct, preceded by a comment explaining that it is retired reference
code for the superseded sensor.

Arduino concatenates every `.ino` in the sketch root into one translation unit, primary sketch first
and remaining tabs alphabetically, so the file needs an explicit guard:

```cpp
#include "config.h"
#if ENABLE_VL53L0X_LEGACY_DRIVER
  // ...old driver...
#endif
```

`config.h` is safe to include here because it uses `#pragma once`. With the switch at `0`, the
preprocessor strips the whole body, so the file costs zero flash and zero SRAM while remaining
visible in the sketch.

The snapshot is not quite verbatim. The old struct's debug helper references `vl53l0xAddress`
(`40-sensors.ino:238`), a symbol this migration renames, so enabling the legacy tab together with
`DEBUG_DISTANCE_SENSOR` would fail to compile. To keep the tab self-contained, a legacy-local
constant is declared inside the guard:

```cpp
constexpr uint8_t vl53l0xAddress = 0x2A;
```

This is safe because it only exists when the guard is enabled, and the live code now uses
`vl53l1xAddress`.

The legacy tab is otherwise fully disconnected: nothing references it, and `distanceTof` is an
instance of `TrainDistanceSensorVL53L1X`. As a frozen snapshot it keeps its own `0x29` and
`DefaultAddress` constants and does not track `pinVL53L1X_XSHUT`. Re-enabling it would require
reconnecting the call sites by hand.

### Call sites

The renamed pin, address, and log strings ripple through:

- `40-sensors.ino`: driver struct, `initDistanceSensorHardware()`, and the ranging path.
- `20-motor.ino` and `30-lights-and-sounds.ino`: XSHUT wake and shutdown writes.
- `50-power-management.ino`: sleep/wake XSHUT writes and the boot-error report string.
- `arduino-train-v2.ino`: the error-bit documentation block, the wiring comment at line 367 which
  still describes XSHUT on D3 and is now doubly stale, and the I2C bus comments at lines 382 and 383
  which name the VL53L0X as a bus participant.
- `README.md` lines 41 and 60, which name the VL53L0X in the hardware list and the degraded-mode
  notes.

`ERR_DISTANCE_TOF` keeps its value of `0x04` and its meaning. Only the human-readable text changes
from VL53L0X to VL53L1X, so previously logged EEPROM boot errors stay comparable.

## Memory expectations

Dropping `stopVariable` and `measurementTimingBudgetUs` removes about five bytes of persistent SRAM
from the driver object, so static SRAM should decrease slightly.

Flash is the uncertain direction. The 16-bit access helpers and the `PROGMEM` configuration block are
code and constant data, and the VL53L1X configuration block is larger than the VL53L0X setup it
replaces. Stack use is a third, separate figure driven by local transaction buffers.

These are independent budgets and one does not offset another. The intent is that total usage stays
close to current, and the actual figures must be read from real builds rather than argued from the
struct layout.

## Testing

Compile `arduino-train-v2` with `arduino-cli` and record flash and SRAM. Compare against a
pre-migration baseline build of the same sketch, and confirm the change is small and explained.

Compile again with `ENABLE_VL53L0X_LEGACY_DRIVER` set to `1`, and separately with both that switch
and `DEBUG_DISTANCE_SENSOR` set to `1`, to confirm the legacy tab still parses in the configuration
that previously would have failed. Restore the switch to `0` and confirm the memory figures return
to the guarded build's numbers exactly.

Behavioural verification requires the physical VL53L1X module and is left to the operator: ranging
accuracy at the auto-drive thresholds, address reassignment with the TCS34725 present on the same
bus, effective cone width, and sleep/wake recovery.

## Risks

The configuration block and the calibration step in initialisation are the main risk. An incorrect,
truncated, or misordered sequence leaves a sensor that acknowledges on I2C and passes the model-ID
check while returning unusable distances. The debug-only `logModelId()` and `logStartupI2cState()`
helpers are retained specifically to diagnose this on hardware.

The address-and-reset interaction is the sharpest failure mode. A stray software reset inside
`init()` drops the sensor back to `0x29` on top of the TCS34725, which presents as intermittent
corruption on both devices rather than as a clean distance-sensor fault. The related trap is reading
any register while the sensor is still at `0x29`: two devices then drive the bus and the returned
byte is meaningless, which would show up as a sensor that "fails to boot" despite being healthy.

The 4x4 cone is the third risk. It is deliberately the narrowest the hardware allows, so a sensor
mounted even slightly off-axis may aim past an obstacle, and an unwise `distanceTofRoiCenterSpad`
can push the window off the SPAD array entirely. Widening to 8x8 remains available if aiming proves
fiddly on the real locomotive.
