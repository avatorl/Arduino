# Reading Boot Error Log from EEPROM

On every `setup()` call the train sketch appends one record to a **16-entry ring buffer** in EEPROM.
Each record captures:
1. **2-byte boot sequence number** (used as a timestamp — no RTC is present)
2. **Sensor error flags** (bitmask)
3. **Battery voltage** (in tenths of a volt)
4. **Runtime DRV8833 motor fault count** (`0` written on setup, then updated in `loop()` on each verified fault event, **capped at 16 writes max per power-on cycle**)

---

## EEPROM layout (83 bytes total; Nano has 1024)

| Address | Size | Field | Notes |
|---------|------|-------|-------|
| `0x00..0x01` | 2 bytes (`uint16_t`) | Boot counter | Incremented each boot; wraps 1–65535 then back to 1 |
| `0x02` | 1 byte (`uint8_t`) | Log head | Next-write slot index (0–15) |
| `0x03 + i×5` | 2 bytes (`uint16_t`) | `boot_seq` | 2-byte counter value at boot *i* — acts as timestamp |
| `0x05 + i×5` | 1 byte (`uint8_t`) | `flags` | Sensor error bitmask (0x00 = all OK) |
| `0x06 + i×5` | 1 byte (`uint8_t`) | `battery` | Pack voltage × 10 as uint8_t (e.g. 74 = 7.4 V); 0xFF = not measured |
| `0x07 + i×5` | 1 byte (`uint8_t`) | `faults` | DRV8833 motor fault events count for this session (0..16) |

### Sensor error flag bit map (`flags` byte)

| Bit | Hex mask | Meaning |
|-----|----------|---------|
| 0   | `0x01`   | MCP23008 LED expander not detected on I2C |
| 1   | `0x02`   | TCS34725 color sensor not detected on I2C |
| 2   | `0x04`   | VL53L0X distance sensor not detected on I2C |

Multiple faults are OR-ed (e.g. `0x06` = color sensor + distance sensor both missing).

---

## How to read the log

### Option A — Upload a reader sketch (recommended)

Paste this into a **new Arduino IDE sketch**, upload to the Nano, then open **Serial Monitor at 9600 baud**:

```cpp
#include <EEPROM.h>

const uint8_t LOG_SIZE = 16;
const uint8_t LOG_BASE = 0x03;

void decodeFlags(uint8_t f) {
  if (f == 0x00) { Serial.print(F("OK")); return; }
  if (f & 0x01) Serial.print(F("LED-EXP "));
  if (f & 0x02) Serial.print(F("COLOR-SENSOR "));
  if (f & 0x04) Serial.print(F("DISTANCE-TOF "));
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {}

  uint16_t bootCount = 0;
  EEPROM.get(0x00, bootCount);
  uint8_t logHead = EEPROM.read(0x02);
  if (logHead >= LOG_SIZE) logHead = 0;

  Serial.print(F("Global boot counter (uint16_t): ")); Serial.println(bootCount);
  Serial.print(F("Log head (next write slot)    : ")); Serial.println(logHead);
  Serial.println(F("--- Boot log (oldest -> newest, max 16 timestamps) ---"));
  Serial.println(F("Slot | Boot#   | Flags | Battery | Faults"));

  // Print entries in chronological order (oldest first)
  for (uint8_t i = 0; i < LOG_SIZE; i++) {
    uint8_t slot = (logHead + i) % LOG_SIZE;          // oldest slot = current head
    uint16_t addr = LOG_BASE + (uint16_t)slot * 5;
    uint16_t seq = 0;
    EEPROM.get(addr, seq);
    uint8_t flags  = EEPROM.read(addr + 2);
    uint8_t batt   = EEPROM.read(addr + 3);
    uint8_t faults = EEPROM.read(addr + 4);

    if (seq == 0xFFFF) continue;  // unwritten slot (fresh EEPROM)

    Serial.print(F("  ")); Serial.print(slot);
    if (slot < 10) Serial.print(' ');
    Serial.print(F(" | #")); Serial.print(seq);
    if (seq < 10) Serial.print(F("     "));
    else if (seq < 100) Serial.print(F("    "));
    else if (seq < 1000) Serial.print(F("   "));
    else if (seq < 10000) Serial.print(F("  "));
    else Serial.print(F(" "));
    Serial.print(F("| 0x")); if (flags < 0x10) Serial.print('0');
    Serial.print(flags, HEX);
    Serial.print(F(" ("));  decodeFlags(flags); Serial.print(F(")"));
    Serial.print(F(" | "));
    if (batt == 0xFF) { Serial.print(F("n/a ")); }
    else { Serial.print(batt / 10); Serial.print('.'); Serial.print(batt % 10); Serial.print(F("V")); }
    Serial.print(F("    | "));
    Serial.println(faults);
  }
}

void loop() {}
```

**Example output:**
```
Global boot counter (uint16_t): 1042
Log head (next write slot)    : 3
--- Boot log (oldest -> newest, max 16 timestamps) ---
Slot  | Boot#   | Flags | Battery | Faults
  3   | #1023   | 0x04 (DISTANCE-TOF ) | 7.4V    | 0
  4   | #1024   | 0x00 (OK)            | 7.4V    | 2
  5   | #1025   | 0x00 (OK)            | 7.3V    | 0
```

---

### Option B — `avrdude` raw dump (no sketch upload needed)

Replace `COM3` / `/dev/ttyUSB0` with the port shown in **Arduino IDE → Tools → Port**.

**Windows (PowerShell):**
```powershell
# Dump first 83 bytes of EEPROM as hex (2-byte boot counter + log head + 16 entries × 5 bytes)
avrdude -c arduino -p atmega328p -P COM3 -b 115200 -U eeprom:r:eeprom_dump.hex:i
Get-Content eeprom_dump.hex | Select-Object -First 6
```

**Linux / macOS (bash):**
```bash
avrdude -c arduino -p atmega328p -P /dev/ttyUSB0 -b 115200 \
  -U eeprom:r:eeprom_dump.hex:i 2>/dev/null
head -6 eeprom_dump.hex
```

---

## Enabling verbose serial output from the train sketch itself

In `arduino-train-v2.ino`, set the debug flag before uploading:

```cpp
#define DEBUG_LEDS 1
```

Open Serial Monitor at **115200 baud**. On each boot you will see:

```
Boot #1042  flags=0x02  batt=7.4
  ERR 0x02: TCS34725 color sensor not detected
```

---

## Clearing the log

To reset the entire log (e.g. after fixing a wiring fault), upload this once:

```cpp
#include <EEPROM.h>
void setup() {
  for (uint16_t i = 0; i < 90; i++) EEPROM.update(i, 0xFF);
}
void loop() {}
```

---

## Enabling verbose serial output from the train sketch itself

In `arduino-train-v2.ino`, set the debug flag before uploading:

```cpp
#define DEBUG_LEDS 1
```

Open Serial Monitor at **115200 baud**. On each boot you will see:

```
Boot #1042  flags=0x02  batt=7.4
  ERR 0x02: TCS34725 color sensor not detected
```

---

## Clearing the log

To reset the entire log (e.g. after fixing a wiring fault), upload this once:

```cpp
#include <EEPROM.h>
void setup() {
  for (uint16_t i = 0; i < 110; i++) EEPROM.update(i, 0xFF);
}
void loop() {}
```
avrdude -c arduino -p atmega328p -P /dev/ttyUSB0 -b 115200 \
  -U eeprom:r:eeprom_dump.hex:i 2>/dev/null
head -6 eeprom_dump.hex
```

---

## Enabling verbose serial output from the train sketch itself

In `arduino-train-v2.ino`, set the debug flag before uploading:

```cpp
#define DEBUG_LEDS 1
```

Open Serial Monitor at **115200 baud**. On each boot you will see:

```
Boot #1042  flags=0x02  batt=7.4
  ERR 0x02: TCS34725 color sensor not detected
```

---

## Clearing the log

To reset the entire log (e.g. after fixing a wiring fault), upload this once:

```cpp
#include <EEPROM.h>
void setup() {
  for (uint16_t i = 0; i < 100; i++) EEPROM.update(i, 0xFF);
}
void loop() {}
```

---

## Enabling verbose serial output from the train sketch itself

In `arduino-train-v2.ino`, set the debug flag before uploading:

```cpp
#define DEBUG_LEDS 1
```

Open Serial Monitor at **115200 baud**. On each boot you will see:

```
Boot #5  flags=0x02  batt=7.4
  ERR 0x02: TCS34725 color sensor not detected
```

---

## Clearing the log

To reset the entire log (e.g. after fixing a wiring fault), upload this once:

```cpp
#include <EEPROM.h>
void setup() {
  for (uint16_t i = 0; i < 64; i++) EEPROM.update(i, 0xFF);
}
void loop() {}
```
