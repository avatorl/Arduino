# VL53L1X Migration Implementation Plan

> **For agentic workers:** Steps use checkbox (`- [ ]`) syntax for tracking. Execute task-by-task,
> compiling after each task because flash headroom is only ~1.9 KB.

**Goal:** Replace the hand-written VL53L0X driver with a VL53L1X driver, move XSHUT to A3, and fix
the I2C address defects that prevent devices on the shared bus from working.

**Architecture:** A size-reduced reimplementation of ST's VL53L1X Ultra Lite Driver (ULD) as a
struct in `40-sensors.ino`, preserving the existing public API so call sites are unchanged. The old
VL53L0X driver moves to a new tab guarded out of compilation.

**Tech Stack:** Arduino Nano (ATmega328P), `arduino-cli` 1.5.1, `Wire` library.

**Spec:** `docs/superpowers/specs/2026-08-24-arduino-train-v2-vl53l1x-migration-design.md`

**Baseline (pre-migration, measured):** flash 28746 bytes (93%), SRAM 1219 bytes (59%).

---

## Build and verify command

Used throughout. There is no unit-test harness for driver code; compilation plus memory measurement
is the verification gate, and behavioural checks need hardware.

```powershell
cd D:\GITHUB\Arduino
arduino-cli compile --fqbn arduino:avr:nano --libraries D:\GITHUB\Arduino\libraries arduino-train-v2 2>&1 | Select-String "Sketch uses|Global variables|error:"
```

---

## File Structure

| File | Responsibility | Change |
| --- | --- | --- |
| `arduino-train-v2/config.h` | All builder-tunable settings | Rename ToF symbols, fix MCP address, add cone settings and legacy switch |
| `arduino-train-v2/40-sensors.ino` | Sensor drivers and sensor logic | Replace ToF driver struct and its startup path |
| `arduino-train-v2/90-legacy-vl53l0x.ino` | Retired VL53L0X driver, not compiled | Create |
| `arduino-train-v2/20-motor.ino` | Motor control | XSHUT symbol rename |
| `arduino-train-v2/30-lights-and-sounds.ino` | LEDs and sound | XSHUT symbol rename |
| `arduino-train-v2/50-power-management.ino` | Sleep/wake and boot errors | XSHUT rename, log text |
| `arduino-train-v2/arduino-train-v2.ino` | Main sketch, wiring docs | Comment updates |
| `arduino-train-v2/README.md` | User-facing docs | Sensor name updates |

---

### Task 0: Fix pre-existing I2C address defects

Two defects currently prevent devices on the shared bus from working. Both are prerequisites,
because nothing else can be validated while the sketch does not compile.

**Files:**
- Modify: `arduino-train-v2/config.h`

- [ ] **Step 1: Confirm the sketch compiles**

`mpu6050Address` was defined twice (I2C block and accelerometer block); the duplicate in the
accelerometer block has been removed. Run the build command. Expected: success, 28746 bytes.

- [ ] **Step 2: Fix the MCP23008 address**

`config.h` line 89 reads `0x20`, but its own comment and the physical wiring (A0 LOW, A1 LOW,
A2 HIGH) mean the expander answers at `0x24`. At `0x20` the LED expander never acknowledges and all
train lights silently fail with `ERR_LED_EXPANDER`.

```cpp
constexpr uint8_t mcp23008Address = 0x24; // MCP23008 LED expander = 0x24 (A0 LOW, A1 LOW, A2 HIGH)
```

- [ ] **Step 3: Rebuild and commit**

Run the build command. Expected: success, memory unchanged.

```bash
git add arduino-train-v2/config.h
git commit -m "Fix duplicate MPU6050 and wrong MCP23008 I2C addresses"
```

---

### Task 1: Rename configuration symbols and add cone settings

**Files:**
- Modify: `arduino-train-v2/config.h`

- [ ] **Step 1: Rename the ToF symbols**

Line 91: `vl53l0xAddress` becomes `vl53l1xAddress` (value `0x2A` unchanged).
Line 104: `pinVL53L0X_XSHUT` becomes `pinVL53L1X_XSHUT` (value already `A3`).
Update the VL53L0X mentions in the `distanceTof*` comments to VL53L1X.

- [ ] **Step 2: Add the detection-cone settings**

Place beside the existing distance-sensor timing settings:

```cpp
// Distance-sensor detection cone (VL53L1X region of interest).
// Smaller ROI = narrower cone = better floor/wall rejection, but shorter range.
// Width/height are clamped to 4..16 by the driver. 4x4 is roughly a 15 degree cone.
// A centre far from 199 combined with a small ROI can push the window off the SPAD
// array, which yields range status 13 and no readings.
constexpr uint8_t distanceTofRoiWidth = 4;
constexpr uint8_t distanceTofRoiHeight = 4;
constexpr uint8_t distanceTofRoiCenterSpad = 199;
```

- [ ] **Step 3: Add the legacy-driver switch**

With the other feature switches, using the override-safe pattern:

```cpp
#ifndef ENABLE_VL53L0X_LEGACY_DRIVER
#define ENABLE_VL53L0X_LEGACY_DRIVER 0
#endif
```

- [ ] **Step 4: Update the remaining call sites**

Rename `pinVL53L0X_XSHUT` in `20-motor.ino:323`, `30-lights-and-sounds.ino:350`,
`50-power-management.ino:387,448,467,510,546`, and `40-sensors.ino:966-972`.
Rename `vl53l0xAddress` in `40-sensors.ino:238,986`.

- [ ] **Step 5: Build and commit**

Expected: success, memory essentially unchanged.

```bash
git add arduino-train-v2
git commit -m "Rename ToF config symbols and add VL53L1X cone settings"
```

---

### Tasks 2-7 execution note (revised after plan review)

The original split compiled a broken tree at several commit points, because `distanceTof`'s declared
type would have referred to a struct that no longer existed. Tasks 2 through 7 are therefore executed
as **one atomic change**: move the old struct to the legacy tab, add the new struct, and repoint the
instance and startup path together, then compile once.

Two further corrections from review apply throughout:

- The struct needs an explicit `public:` after the initial members, otherwise `setAddress`,
  `waitForBoot`, `init`, `setMeasurementTimingBudget`, `setRoi`, `startContinuous`,
  `readRangeContinuousMillimeters`, and the debug helpers all end up private and no call site
  compiles.
- `logI2cAddress()`, `probeAddress()`, and `printHexByte()` live in the old struct and must be
  ported to the new one, or the `DEBUG_DISTANCE_SENSOR=1` build fails.
- `init()` must temporarily raise `ioTimeoutMs`. Its one-off calibration ranges using the default
  block's long-mode ~100 ms budget, so the normal 50 ms `distanceTofTimeoutMs` would abort a
  perfectly healthy sensor.

---

### Task 2: Move the VL53L0X driver to a guarded legacy tab

Executed together with Tasks 3-7 as described above.

**Files:**
- Create: `arduino-train-v2/90-legacy-vl53l0x.ino`
- Modify: `arduino-train-v2/40-sensors.ino`

- [ ] **Step 1: Create the legacy tab**

Move the entire `TrainDistanceSensorVL53L0X` struct out of `40-sensors.ino` into the new file:

```cpp
// ================================================================================================
// File description
// ================================================================================================
// Retired VL53L0X distance-sensor driver, kept for reference only.
// The train now uses a VL53L1X (see TrainDistanceSensorVL53L1X in 40-sensors.ino). This tab is
// excluded from compilation by ENABLE_VL53L0X_LEGACY_DRIVER in config.h, so it costs zero flash
// and zero SRAM while switched off.
// Re-enabling it only makes this struct compile again; the call sites in 40-sensors.ino would
// still have to be reconnected by hand.

#include "config.h"
#if ENABLE_VL53L0X_LEGACY_DRIVER

// The live code renamed this symbol to vl53l1xAddress, so the snapshot carries its own copy to
// stay self-contained. Only exists while the legacy switch is on.
constexpr uint8_t vl53l0xAddress = 0x2A;

  struct TrainDistanceSensorVL53L0X {
    // ...moved verbatim...
  };

#endif
```

- [ ] **Step 2: Build with the switch off**

Expected: success. Flash must drop noticeably now that the old driver is gone and no replacement
exists yet. Record the figure; it is the floor the new driver builds on.

- [ ] **Step 3: Build with the switch on**

```powershell
arduino-cli compile --fqbn arduino:avr:nano --libraries D:\GITHUB\Arduino\libraries --build-property "compiler.cpp.extra_flags=-DENABLE_VL53L0X_LEGACY_DRIVER=1 -DDEBUG_DISTANCE_SENSOR=1" arduino-train-v2 2>&1 | Select-String "Sketch uses|error:"
```

Expected: compiles. This is the combination that would have failed without the legacy-local
`vl53l0xAddress`. Flash will be large; that is fine and expected.

- [ ] **Step 4: Commit**

```bash
git add arduino-train-v2
git commit -m "Move VL53L0X driver to guarded legacy tab"
```

---

### Task 3: VL53L1X access primitives and identity

**Files:**
- Modify: `arduino-train-v2/40-sensors.ino`

- [ ] **Step 1: Add the struct skeleton with 16-bit register helpers**

The VL53L1X uses 16-bit register addresses, so every helper sends a high byte then a low byte.

```cpp
  struct TrainDistanceSensorVL53L1X {
    static const uint8_t DefaultAddress = 0x29;

    uint8_t address = DefaultAddress;
    uint16_t ioTimeoutMs = 0;
    bool didTimeout = false;
    bool lastReadValid = false;
    unsigned long timeoutStartMs = 0;

    void setTimeout(uint16_t timeoutMs) { ioTimeoutMs = timeoutMs; }
    bool timeoutOccurred() { const bool t = didTimeout; didTimeout = false; return t; }
    bool lastRangeReadValid() const { return lastReadValid; }

   private:
    bool writeRegAt(uint8_t targetAddress, uint16_t reg, uint8_t value) {
      Wire.beginTransmission(targetAddress);
      Wire.write((uint8_t)(reg >> 8));
      Wire.write((uint8_t)(reg & 0xFF));
      Wire.write(value);
      return Wire.endTransmission() == 0;
    }

    bool writeReg(uint16_t reg, uint8_t value) { return writeRegAt(address, reg, value); }

    bool writeReg16(uint16_t reg, uint16_t value) {
      Wire.beginTransmission(address);
      Wire.write((uint8_t)(reg >> 8));
      Wire.write((uint8_t)(reg & 0xFF));
      Wire.write((uint8_t)(value >> 8));
      Wire.write((uint8_t)(value & 0xFF));
      return Wire.endTransmission() == 0;
    }

    bool writeReg32(uint16_t reg, uint32_t value) {
      Wire.beginTransmission(address);
      Wire.write((uint8_t)(reg >> 8));
      Wire.write((uint8_t)(reg & 0xFF));
      for (int8_t shift = 24; shift >= 0; shift -= 8) {
        Wire.write((uint8_t)(value >> shift));
      }
      return Wire.endTransmission() == 0;
    }

    bool readRegsAt(uint8_t targetAddress, uint16_t reg, uint8_t* buffer, uint8_t length) {
      Wire.beginTransmission(targetAddress);
      Wire.write((uint8_t)(reg >> 8));
      Wire.write((uint8_t)(reg & 0xFF));
      if (Wire.endTransmission(false) != 0) return false;
      if (Wire.requestFrom((int)targetAddress, (int)length) != length) return false;
      for (uint8_t i = 0; i < length; ++i) buffer[i] = (uint8_t)Wire.read();
      return true;
    }

    uint8_t readReg(uint16_t reg) {
      uint8_t v = 0;
      if (!readRegsAt(address, reg, &v, 1)) { didTimeout = true; return 0; }
      return v;
    }

    uint16_t readReg16(uint16_t reg) {
      uint8_t v[2] = { 0, 0 };
      if (!readRegsAt(address, reg, v, 2)) { didTimeout = true; return 0; }
      return ((uint16_t)v[0] << 8) | v[1];
    }
  };
```

- [ ] **Step 2: Build**

Expected: success. The struct is unused so far, so flash barely moves.

- [ ] **Step 3: Commit**

```bash
git add arduino-train-v2/40-sensors.ino
git commit -m "Add VL53L1X register access primitives"
```

---

### Task 4: Address assignment and boot handling

This is the task that keeps the shared bus working. Get the ordering wrong and the VL53L1X collides
with the TCS34725 at `0x29`.

**Files:**
- Modify: `arduino-train-v2/40-sensors.ino`

- [ ] **Step 1: Add address assignment**

Public members. Writes the 7-bit value, matching Pololu's convention (ST's ULD writes
`new_address >> 1` only because its `dev` handle is 8-bit).

```cpp
    // Assign the runtime I2C address. This is a WRITE-ONLY transaction, which matters: the
    // TCS34725 also answers at 0x29, so reads there are ambiguous. On the wire this is
    // [0x00, 0x01, 0x2A], and the TCS34725 discards it because its command byte must have bit
    // 0x80 set.
    bool setAddress(uint8_t newAddress) {
      if (!writeRegAt(address, 0x0001, (uint8_t)(newAddress & 0x7F))) {
        if (address == DefaultAddress) return false;
        if (!writeRegAt(DefaultAddress, 0x0001, (uint8_t)(newAddress & 0x7F))) return false;
      }
      address = newAddress;
      return true;
    }

    // Wait for the firmware to finish booting. Only safe once the sensor is alone on its address.
    bool waitForBoot() {
      startTimeout();
      while ((readReg(0x00E5) & 0x01) == 0) {
        if (ioTimeoutMs > 0 && hasTimedOut()) { didTimeout = true; return false; }
      }
      return true;
    }
```

- [ ] **Step 2: Add the timeout helpers**

```cpp
    void startTimeout() { timeoutStartMs = millis(); }
    bool hasTimedOut() const {
      return ioTimeoutMs > 0 && (millis() - timeoutStartMs) > ioTimeoutMs;
    }
```

- [ ] **Step 3: Build and commit**

```bash
git add arduino-train-v2/40-sensors.ino
git commit -m "Add VL53L1X address assignment and boot wait"
```

---

### Task 5: Initialisation

**Files:**
- Modify: `arduino-train-v2/40-sensors.ino`

- [ ] **Step 1: Add ST's default configuration block in PROGMEM**

Registers `0x2D` through `0x87`, 91 bytes, stored in flash. Transcribe exactly; these are
undocumented magic values.

```cpp
  const uint8_t kVl53l1xDefaultConfig[] PROGMEM = {
    0x00, 0x01, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01,
    0x01, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x20, 0x0B, 0x00, 0x00, 0x02, 0x0A, 0x21, 0x00, 0x00, 0x05, 0x00,
    0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00,
    0x00, 0x01, 0xCC, 0x0F, 0x01, 0xF1, 0x0D, 0x01, 0x68, 0x00, 0x80, 0x08,
    0xB8, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x0F, 0x0D, 0x0E, 0x0E, 0x00, 0x00, 0x02, 0xC7, 0xFF,
    0x9B, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00
  };
```

Index 1 (register `0x002E`) is `0x01`, not `0x00`: that bit selects AVDD pull-ups, which is what
enables 2.8 V I/O and is why `init()` can ignore its `io2v8` argument.
Index 50 (register `0x005F`) is `0xCC`, completing the block's default long-mode `0x01CC` timing
budget at `0x005E`/`0x005F`.

- [ ] **Step 2: Add `init()`**

`io2v8` is kept for call-site compatibility and ignored; 2.8 V mode is already in the block.
Note there is deliberately **no software reset** here: writing `0x0000` would return the sensor to
`0x29` on top of the TCS34725.

```cpp
    bool init(bool io2v8 = true) {
      (void)io2v8;
      didTimeout = false;
      lastReadValid = false;

      if (readReg16(0x010F) != 0xEACC) return false;

      for (uint8_t i = 0; i < sizeof(kVl53l1xDefaultConfig); ++i) {
        if (!writeReg((uint16_t)(0x002D + i), pgm_read_byte(&kVl53l1xDefaultConfig[i]))) {
          return false;
        }
      }

      // Required one-off calibration. Without it the sensor answers on I2C but never returns
      // usable distances.
      if (!writeReg(0x0087, 0x40)) return false;
      if (!waitForDataReady()) return false;
      if (!clearInterrupt()) return false;
      if (!writeReg(0x0087, 0x00)) return false;
      if (!writeReg(0x0008, 0x09)) return false;
      if (!writeReg(0x000B, 0x00)) return false;

      return setDistanceModeShort();
    }

    // Short mode: more robust against ambient light, ~1.3 m ceiling, which covers the 50 cm the
    // auto-drive logic uses. The default config block is long mode, so this is an explicit change.
    bool setDistanceModeShort() {
      if (!writeReg(0x004B, 0x14)) return false;
      if (!writeReg(0x0060, 0x07)) return false;
      if (!writeReg(0x0063, 0x05)) return false;
      if (!writeReg(0x0069, 0x38)) return false;
      if (!writeReg16(0x0078, 0x0705)) return false;
      if (!writeReg16(0x007A, 0x0606)) return false;
      return true;
    }
```

- [ ] **Step 3: Build and commit**

```bash
git add arduino-train-v2/40-sensors.ino
git commit -m "Add VL53L1X initialisation and short distance mode"
```

---

### Task 6: Timing budget, ROI, and continuous ranging

**Files:**
- Modify: `arduino-train-v2/40-sensors.ino`

- [ ] **Step 1: Add the timing budget**

Only discrete budgets exist. Unsupported requests snap to the nearest supported value rather than
failing, so a mistyped `config.h` cannot disable the sensor. Values are the short-mode pairs.

```cpp
    bool setMeasurementTimingBudget(uint32_t budgetUs) {
      const uint16_t ms = (uint16_t)(budgetUs / 1000UL);
      uint16_t a, b;
      if (ms < 18)       { a = 0x001D; b = 0x0027; }  // 15 ms
      else if (ms < 27)  { a = 0x0051; b = 0x006E; }  // 20 ms
      else if (ms < 42)  { a = 0x00D6; b = 0x006E; }  // 33 ms
      else if (ms < 75)  { a = 0x01AE; b = 0x01E8; }  // 50 ms
      else if (ms < 150) { a = 0x02E1; b = 0x0388; }  // 100 ms
      else if (ms < 350) { a = 0x03E1; b = 0x0496; }  // 200 ms
      else               { a = 0x0591; b = 0x05C1; }  // 500 ms
      if (!writeReg16(0x005E, a)) return false;
      return writeReg16(0x0061, b);
    }
```

These are the **short-mode** pairs. The configured 33000 us selects `0x00D6`/`0x006E`; note the B
value is shared with the 20 ms entry, which is correct and not a copy-paste slip.

- [ ] **Step 2: Add the ROI**

```cpp
    bool setRoi(uint8_t width, uint8_t height, uint8_t centerSpad) {
      if (width < 4) width = 4;
      if (width > 16) width = 16;
      if (height < 4) height = 4;
      if (height > 16) height = 16;
      if (!writeReg(0x0080, (uint8_t)(((height - 1) << 4) | (width - 1)))) return false;
      return writeReg(0x007F, centerSpad);
    }
```

- [ ] **Step 3: Add continuous ranging**

The `* 43 / 40` is the integer form of ST's `* 1.075`, avoiding linking AVR floating-point support.
Maximum intermediate is `1023 * 50 * 43 = 2199450`, well inside `uint32_t`.

```cpp
    void startContinuous(uint32_t periodMs = 0) {
      if (periodMs > 0) {
        const uint32_t clockPll = (uint32_t)(readReg16(0x00DE) & 0x03FF);
        writeReg32(0x006C, clockPll * periodMs * 43UL / 40UL);
      }
      writeReg(0x0087, 0x40);
    }
```

- [ ] **Step 4: Add the read path**

Raw status `9` is the only value ST maps to "valid". Testing the register against `0` would reject
every good reading. Clearing the interrupt is mandatory or ranging stalls after one measurement.

```cpp
    uint16_t readRangeContinuousMillimeters() {
      lastReadValid = false;
      if (!waitForDataReady()) return 0;

      const uint8_t status = (uint8_t)(readReg(0x0089) & 0x1F);
      const uint16_t mm = readReg16(0x0096);
      clearInterrupt();

      if (status != 9) return 0;
      lastReadValid = true;
      return mm;
    }

   private:
    bool clearInterrupt() { return writeReg(0x0086, 0x01); }

    // Note the inversion: dropping the "!" gives exactly the wrong answer and the poll never ends.
    bool waitForDataReady() {
      const uint8_t polarity = (uint8_t)(!((readReg(0x0030) >> 4) & 0x01));
      startTimeout();
      while ((readReg(0x0031) & 0x01) != polarity) {
        if (ioTimeoutMs > 0 && hasTimedOut()) { didTimeout = true; return false; }
      }
      return true;
    }
```

- [ ] **Step 5: Build and commit**

```bash
git add arduino-train-v2/40-sensors.ino
git commit -m "Add VL53L1X ranging, timing budget, and ROI"
```

---

### Task 7: Debug helpers and startup wiring

**Files:**
- Modify: `arduino-train-v2/40-sensors.ino`

- [ ] **Step 1: Port the debug helpers**

Keep them behind `DEBUG_DISTANCE_SENSOR`. `logModelId()` now reads a 16-bit ID.

```cpp
    #if DEBUG_DISTANCE_SENSOR
    void logStartupI2cState(const __FlashStringHelper* checkpoint) {
      DBGLN_DISTANCE_SENSOR(checkpoint);
      logI2cAddress(mcp23008Address);
      logI2cAddress(DefaultAddress);   // ambiguous: TCS34725 also answers here
      logI2cAddress(vl53l1xAddress);
    }

    void logModelId() {
      DBG_DISTANCE_SENSOR(F("VL53L1X model ID at 0x"));
      printHexByte(address);
      DBG_DISTANCE_SENSOR(F(": 0x"));
      const uint16_t id = readReg16(0x010F);
      printHexByte((uint8_t)(id >> 8));
      printHexByte((uint8_t)(id & 0xFF));
      DBGLN_DISTANCE_SENSOR(F(""));
    }
    #endif
```

- [ ] **Step 2: Update `initDistanceSensorHardware()`**

Keep `delay(10)` after XSHUT release. The datasheet allows ~1.2 ms for boot, so 10 ms is a
comfortable margin, and it is the only guarantee the sensor is awake before the address write.

```cpp
  void initDistanceSensorHardware() {
    pinMode(pinVL53L1X_XSHUT, OUTPUT);
    digitalWrite(pinVL53L1X_XSHUT, LOW);
    delay(10);
    digitalWrite(pinVL53L1X_XSHUT, HIGH);
    delay(10);   // sensor is at 0x29 here, shared with the TCS34725: do not read
    startDistanceSensorRanging(true);
  }
```

- [ ] **Step 3: Update `startDistanceSensorRanging()`**

Ordering is the correctness rule: assign the address first, then confirm boot at the new address.
Step 4 is what catches a failed assignment, since the TCS34725 would have acknowledged it anyway.

```cpp
    distanceTof.setTimeout(distanceTofTimeoutMs);

    if (!distanceTof.setAddress(vl53l1xAddress)) { /* existing failure path */ }
    if (!distanceTof.waitForBoot())              { /* existing failure path */ }
    if (reinitializeSensor && !distanceTof.init()) { /* existing failure path */ }
    if (!distanceTof.setMeasurementTimingBudget(distanceTofTimingBudgetUs)) { /* existing */ }
    distanceTof.setRoi(distanceTofRoiWidth, distanceTofRoiHeight, distanceTofRoiCenterSpad);
    distanceTof.startContinuous(distanceTofContinuousPeriodMs);
```

- [ ] **Step 4: Rename the instance**

`TrainDistanceSensorVL53L0X distanceTof;` becomes `TrainDistanceSensorVL53L1X distanceTof;`
(`40-sensors.ino:618`).

- [ ] **Step 5: Build, including the debug configuration**

```powershell
arduino-cli compile --fqbn arduino:avr:nano --libraries D:\GITHUB\Arduino\libraries --build-property "compiler.cpp.extra_flags=-DDEBUG_DISTANCE_SENSOR=1" arduino-train-v2 2>&1 | Select-String "Sketch uses|error:"
```

Expected: both configurations compile. **Flash must stay under 30720 bytes.** If the default build
exceeds roughly 29500, stop and report before continuing.

- [ ] **Step 6: Commit**

```bash
git add arduino-train-v2/40-sensors.ino
git commit -m "Wire VL53L1X into the distance-sensor startup path"
```

---

### Task 8: Documentation and final verification

**Files:**
- Modify: `arduino-train-v2/arduino-train-v2.ino`, `arduino-train-v2/50-power-management.ino`,
  `arduino-train-v2/README.md`

- [ ] **Step 1: Update comments and log text**

- `arduino-train-v2.ino:367` — XSHUT is on A3, not D3.
- `arduino-train-v2.ino:382-383` — I2C bus participants list.
- `arduino-train-v2.ino` error-bit block — `ERR_DISTANCE_TOF` text.
- `50-power-management.ino:265` — boot-error report string.
- `README.md:41` and `README.md:60` — sensor name.

`ERR_DISTANCE_TOF` keeps the value `0x04` so previously logged EEPROM boot errors stay comparable.

- [ ] **Step 2: Verify all four build configurations**

| Configuration | Expectation |
| --- | --- |
| default | compiles, flash < 30720 |
| `-DDEBUG_DISTANCE_SENSOR=1` | compiles |
| `-DENABLE_VL53L0X_LEGACY_DRIVER=1` | compiles |
| `-DENABLE_VL53L0X_LEGACY_DRIVER=1 -DDEBUG_DISTANCE_SENSOR=1` | compiles |

- [ ] **Step 3: Confirm the guard is free**

Default build and legacy-disabled build must report byte-identical flash and SRAM, proving the
guarded tab costs nothing.

- [ ] **Step 4: Record final memory against the 28746 / 1219 baseline and commit**

```bash
git add arduino-train-v2
git commit -m "Update docs for VL53L1X migration"
```

---

## Hardware verification (operator, not automatable)

- All four I2C devices respond: MCP23008 at `0x24`, TCS34725 at `0x29`, VL53L1X at `0x2A`,
  MPU6050 at `0x68`.
- Train lights work, confirming the MCP23008 address fix.
- Distance readings are sane at the auto-drive thresholds (stop 8 cm, restart 11 cm, full speed
  50 cm).
- Sleep/wake cycles recover ranging, since XSHUT returns the sensor to `0x29` each time.
- The 4x4 cone actually sees obstacles ahead; if aiming proves fiddly, widen to 8x8 or adjust
  `distanceTofRoiCenterSpad`.
