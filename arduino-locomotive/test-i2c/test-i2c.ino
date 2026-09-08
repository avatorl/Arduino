#include <Wire.h>

// ================================================================================================
// I2C BUS DIAGNOSTIC SKETCH
// ================================================================================================
// What this sketch is for:
// - This is a standalone hardware-communication test for the train's I2C bus.
// - It checks whether three devices can be reached reliably over I2C:
//     1. MCP23008  GPIO expander
//     2. TCS34725  color sensor
//     3. VL53L0X   distance sensor
// - It tests the bus at two common I2C speeds: 100 kHz and 400 kHz.
//
// What this sketch does NOT test:
// - It does not test whether the color sensor reads correct colors.
// - It does not test whether the distance sensor measures correct distance.
// - It does not drive motors, lights, or other train outputs.
// - It only checks "Can we talk to each I2C device cleanly and repeatedly?"
//
// Why this sketch exists separately from the main train program:
// - The full train sketch does many things at once, which can hide the root cause of I2C problems.
// - This sketch strips the problem down to the bus itself, making debugging much easier.
// - If this sketch passes, the wiring and low-level I2C communication are probably healthy.
// - If it fails, the printed counters help show whether the problem is missing devices, bad IDs,
//   short reads, timeouts, or a stuck bus.
//
// High-level test flow (per selected test length, per speed profile):
//  1. Wait for you to select a short (1,000 reads) or long (10,000 reads) test.
//  2. Measure the idle voltage of SDA and SCL with the ADC (checks pull-up strength).
//  3. Recover the I2C bus if needed and confirm SDA/SCL are idle HIGH.
//  4. Scan every I2C address (0x08-0x77) and report exactly which devices answer.
//  5. Hold the VL53L0X in reset so it cannot conflict with the TCS34725 at address 0x29.
//  6. Read one known register from the MCP23008 and TCS34725.
//  7. Write test patterns (0x00/0xFF/0x55/0xAA) to a safe MCP23008 register and read them
//     back, to prove data integrity in BOTH directions (reads alone cannot catch stuck bits).
//  8. Stress each device with repeated reads; every 10th read is a multi-byte burst to
//     exercise sustained transfers and register auto-increment like real drivers do.
//  9. Release VL53L0X reset, move it to address 0x2A, and confirm its model ID.
// 10. Run an interleaved round-robin phase that alternates rapidly between all working
//     devices, because some bus faults only appear when switching targets quickly.
// 11. Check whether SDA and SCL return to idle HIGH after the run.
// 12. Print per-device results (including min/avg/max transaction times) and a PASS/FAIL
//     summary.
//
// What software still CANNOT verify (needs an oscilloscope or logic analyzer):
// - signal rise times, ringing, and electrical noise margins. The idle-voltage measurement
//   and timing statistics below are the closest software approximations.
//
// Reading the output:
// - totalAttempts       = all setup + stress transactions for that device
// - stressAttempts      = only the repeated-test transactions; this matches the selected test size
// - successes           = how many completed successfully
// - transmissionErrors  = address/register phase failed (often NACK / device missing)
// - shortReads          = fewer bytes came back than expected
// - mismatches          = the register returned an unexpected value
// - timeouts            = Wire detected a bus timeout
//
// Important hardware detail:
// - The TCS34725 and VL53L0X both use default I2C address 0x29.
// - To avoid that collision, this sketch holds VL53L0X XSHUT LOW while testing the color sensor,
//   then re-enables VL53L0X and moves it to 0x2A before testing it.
// ================================================================================================

constexpr uint8_t kVlXshutPin = A3;
constexpr uint8_t kSdaPin = A4;
constexpr uint8_t kSclPin = A5;

constexpr uint8_t kMcp23008Address = 0x20;
constexpr uint8_t kTcs34725Address = 0x29;
constexpr uint8_t kVl53l0xDefaultAddress = 0x29;
constexpr uint8_t kVl53l0xAddress = 0x2A;

constexpr uint8_t kMcp23008IodirRegister = 0x00;
// DEFVAL (0x03) is the "default compare value" for the MCP23008 interrupt-on-change feature.
// Because this sketch never enables interrupts (GPINTEN stays 0), writing DEFVAL has no visible
// effect on any pin, which makes it a safe scratch register for write/read-back testing.
constexpr uint8_t kMcp23008DefvalRegister = 0x03;
constexpr uint8_t kTcs34725IdRegister = 0x92;
// Command byte for a multi-byte auto-increment read of the TCS34725 RGBC data block:
// 0x80 (command bit) | 0x20 (auto-increment) | 0x14 (clear-data-low register).
constexpr uint8_t kTcs34725RgbcBlockCommand = 0xB4;
constexpr uint8_t kTcs34725ExpectedId = 0x4D;
constexpr uint8_t kVl53l0xModelIdRegister = 0xC0;

constexpr uint32_t kStandardSpeed = 100000UL;
constexpr uint32_t kFastSpeed = 400000UL;
constexpr uint16_t kShortReadsPerDevice = 1000;
constexpr uint16_t kLongReadsPerDevice = 10000;
constexpr uint32_t kWireTimeoutUs = 3000UL;

// Every Nth stress read is a multi-byte burst instead of a single-byte read.
constexpr uint8_t kBurstEveryNthRead = 10;

// Idle-voltage thresholds. The ATmega328P at 5 V needs at least 0.6 * VCC = 3.0 V to reliably
// see a HIGH. A healthy 5 V bus with pull-ups idles near 5 V; ~3.3 V suggests the pull-ups go to
// a 3.3 V rail (works, but with little noise margin); below 3.0 V is out of spec.
constexpr uint16_t kIdleVoltageGoodMv = 4000;
constexpr uint16_t kIdleVoltageMinMv = 3000;

// Patterns for the MCP23008 write/read-back test: all-zeros, all-ones, and both alternating
// bit patterns. Together they detect stuck-at-0, stuck-at-1, and adjacent-bit-short faults.
constexpr uint8_t kWritePatterns[] = {0x00, 0xFF, 0x55, 0xAA};
constexpr uint8_t kWritePatternCount = sizeof(kWritePatterns);

// If a stress loop keeps failing, printing every single failure would flood the serial monitor
// (potentially 10,000 lines per device) and slow the whole test down at 115200 baud. To keep the
// output useful we print detailed information for at most the first few failures per phase, and
// then just print a single "N more errors suppressed" line at the end. The counters in
// DeviceResult still track every failure, so PASS/FAIL is unaffected.
constexpr uint8_t kMaxDetailedErrorsPerPhase = 5;

// Per-device counters. These are printed at the end of each speed profile so we can tell exactly
// how communication failed instead of just seeing a single generic PASS/FAIL result.
// The timing fields track how long each complete transaction took (in microseconds); a rising
// max or average can reveal clock stretching or a struggling device even when every read passes.
struct DeviceResult {
  uint16_t totalAttempts;
  uint16_t stressAttempts;
  uint16_t successes;
  uint16_t transmissionErrors;
  uint16_t shortReads;
  uint16_t mismatches;
  uint16_t timeouts;
  uint16_t timedTransactions;
  uint32_t minTimeUs;
  uint32_t maxTimeUs;
  uint32_t sumTimeUs;
};

enum FailureKind {
  FailureNone,
  FailureRegisterPointerWrite,
  FailureEndTransmissionStatus,
  FailureShortRead,
  FailureTimeout,
  FailureAddressMoveWrite
};

struct FailureDetails {
  FailureKind kind;
  uint8_t address;
  uint8_t reg;
  uint8_t transmissionStatus;
  uint8_t receivedBytes;
  uint8_t expectedBytes;
};

// A small "print budget" that lets each stress loop print detailed messages for the first few
// failures and then stay quiet. "used" counts how many detailed lines we have already emitted;
// "suppressed" counts how many extra failures got hidden so we can report the total at the end.
struct ErrorPrintBudget {
  uint16_t used;
  uint16_t suppressed;
};

// Sketch-wide state used by the low-level helpers:
// - wireInitialized tracks whether Wire.begin() has already been called so recovery can safely
//   shut Wire down before taking direct control of SDA/SCL pins.
// - lastTransactionTimedOut lets the caller know that the most recent failure was specifically a
//   timeout, which is important because the bus may need recovery afterward.
// - activeResult is a "sentinel" pointer. Before every I2C helper call we set it to the current
//   device's DeviceResult, so the shared low-level helpers know which counters to increment.
//   After the call, we set it back to NULL so an accidental call outside a device context is
//   detected immediately (readRegister8 refuses to run when activeResult is NULL).
bool wireInitialized = false;
bool lastTransactionTimedOut = false;
DeviceResult* activeResult = NULL;
FailureDetails lastFailure = {FailureNone, 0, 0, 0, 0, 0};

// Start every profile from a clean slate. minTimeUs starts at the largest possible value so the
// first measured transaction always replaces it.
void initDeviceResult(DeviceResult* result) {
  result->totalAttempts = 0;
  result->stressAttempts = 0;
  result->successes = 0;
  result->transmissionErrors = 0;
  result->shortReads = 0;
  result->mismatches = 0;
  result->timeouts = 0;
  result->timedTransactions = 0;
  result->minTimeUs = 0xFFFFFFFFUL;
  result->maxTimeUs = 0;
  result->sumTimeUs = 0;
}

// Fold one measured transaction duration into the device's timing statistics.
void recordTransactionTime(DeviceResult* result, uint32_t elapsedUs) {
  result->timedTransactions++;
  result->sumTimeUs += elapsedUs;
  if (elapsedUs < result->minTimeUs) {
    result->minTimeUs = elapsedUs;
  }
  if (elapsedUs > result->maxTimeUs) {
    result->maxTimeUs = elapsedUs;
  }
}

// Clear any timeout flag left behind by a previous transaction.
void clearWireTimeout() {
#if defined(WIRE_HAS_TIMEOUT)
  Wire.clearWireTimeoutFlag();
#endif
}

// Read and clear the timeout flag. Returning true means the most recent Wire operation timed out.
bool consumeWireTimeout() {
#if defined(WIRE_HAS_TIMEOUT)
  if (Wire.getWireTimeoutFlag()) {
    Wire.clearWireTimeoutFlag();
    return true;
  }
#endif
  return false;
}

// Start Wire for one specific test speed and apply the same timeout settings every time Wire is
// restarted. Recovery code reuses this so each profile begins from a known bus configuration.
void configureWire(uint32_t speed) {
  Wire.begin();
  wireInitialized = true;
  Wire.setClock(speed);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(kWireTimeoutUs, true);
#endif
  clearWireTimeout();
}

// "Release" a bus line by letting the external pull-up resistor bring it HIGH.
// I2C is an "open-drain" bus: devices can only actively pull lines LOW. HIGH is achieved by
// simply not driving the line - the pull-up resistor on the module (or on the shared VIN rail)
// gently pulls it up. Using INPUT_PULLUP here (and HIGH first, to avoid a stray LOW pulse) is
// exactly this "let go of the line" behavior.
void releaseLine(uint8_t pin) {
  digitalWrite(pin, HIGH);
  pinMode(pin, INPUT_PULLUP);
}

// Force a bus line LOW by driving it as a digital output. Used only during manual bus recovery
// clock pulses and to fabricate a STOP condition (SDA LOW->HIGH while SCL is HIGH).
void driveLineLow(uint8_t pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
}

// A healthy idle I2C bus sits HIGH on both SDA and SCL because pull-up resistors hold the lines up
// when no device is actively pulling them down.
bool isBusIdleHigh() {
  return digitalRead(kSdaPin) == HIGH && digitalRead(kSclPin) == HIGH;
}

// Perform the standard "9 clock pulses + STOP" I2C recovery routine.
// This helps when a device got interrupted mid-transaction and is still holding SDA low waiting for
// extra clocks. The serial print shows line states before and after recovery for easier debugging.
bool recoverI2cBus() {
  releaseLine(kSdaPin);
  releaseLine(kSclPin);
  delayMicroseconds(5);

  const bool sdaBefore = digitalRead(kSdaPin) == HIGH;
  const bool sclBefore = digitalRead(kSclPin) == HIGH;

  if (!sdaBefore) {
    for (uint8_t clock = 0; clock < 9 && digitalRead(kSdaPin) == LOW; ++clock) {
      driveLineLow(kSclPin);
      delayMicroseconds(5);
      releaseLine(kSclPin);
      delayMicroseconds(5);
    }

    driveLineLow(kSdaPin);
    delayMicroseconds(5);
    releaseLine(kSclPin);
    delayMicroseconds(5);
    releaseLine(kSdaPin);
    delayMicroseconds(5);
  }

  const bool sdaAfter = digitalRead(kSdaPin) == HIGH;
  const bool sclAfter = digitalRead(kSclPin) == HIGH;
  const bool recovered = sdaAfter && sclAfter;

  Serial.print(F("RECOVERY: SDA="));
  Serial.print(sdaBefore ? F("HIGH") : F("LOW"));
  Serial.print(F(" SCL="));
  Serial.print(sclBefore ? F("HIGH") : F("LOW"));
  Serial.print(F(" -> SDA="));
  Serial.print(sdaAfter ? F("HIGH") : F("LOW"));
  Serial.print(F(" SCL="));
  Serial.print(sclAfter ? F("HIGH") : F("LOW"));
  Serial.print(F(" "));
  Serial.println(recovered ? F("PASS") : F("FAIL"));

  return recovered;
}

// When Wire already owns the I2C hardware, stop it before manually toggling SDA/SCL pins for
// recovery. Then restart Wire with the same speed profile afterwards.
bool restartWireAfterRecovery(uint32_t speed) {
  if (wireInitialized) {
    Wire.end();
    wireInitialized = false;
  }

  const bool recovered = recoverI2cBus();
  configureWire(speed);
  return recovered;
}

// Measure the Nano's actual supply voltage (VCC) in millivolts using the ATmega328P's internal
// 1.1 V bandgap reference. Trick: the ADC normally measures "pin voltage as a fraction of VCC".
// Here we flip it around and measure the KNOWN 1.1 V reference as a fraction of VCC — from that
// ratio we can back-calculate what VCC really is. A USB-powered Nano typically shows ~4.6-4.9 V
// because of cable losses and the board's protection diode, not a clean 5.00 V.
// Accuracy note: the bandgap is specified as 1.0-1.2 V per chip (calibrated once at the factory),
// so the result can be off by up to ~10%, but it is stable and far better than assuming 5.0 V.
uint16_t readVccMillivolts() {
  // Select the internal 1.1 V bandgap as the ADC input, with AVcc (= VCC) as the reference.
  // Arduino's analogRead() cannot select this channel, so we program the ADC registers directly.
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);  // Give the bandgap reference time to settle after being connected to the ADC.

  uint16_t raw = 0;
  for (uint8_t i = 0; i < 2; ++i) {  // First conversion after a channel switch is inaccurate.
    ADCSRA |= _BV(ADSC);             // Start a conversion.
    while (ADCSRA & _BV(ADSC)) {}    // Wait for it to finish (~104 us).
    raw = ADC;
  }

  if (raw == 0) {
    return 0;  // Should never happen; avoid dividing by zero.
  }

  // raw = 1100 mV * 1023 / VCC  =>  VCC = 1100 mV * 1023 / raw
  return static_cast<uint16_t>((1100UL * 1023UL) / raw);
}

// Measure the idle DC voltage on SDA and SCL using the ADC (A4/A5 are analog-capable pins).
// This is something digitalRead() cannot do: it only says HIGH/LOW, while the ADC shows HOW HIGH
// the pull-ups actually lift the lines. On a healthy bus the lines should idle at VCC; ~3300 mV
// means the pull-ups go to a 3.3 V rail (workable but with a thin noise margin on a 5 V
// ATmega328P, whose minimum reliable HIGH is 0.6 * VCC = 3000 mV); below 3000 mV is out of spec.
// Wire must NOT be active during this measurement, so it runs before the profile starts I2C.
// The pin readings are scaled by the real measured VCC (see readVccMillivolts), so the printed
// millivolts match what a multimeter shows instead of assuming a perfect 5.00 V supply.
bool measureIdleVoltages(bool* profilePassed) {
  if (wireInitialized) {
    Wire.end();
    wireInitialized = false;
  }

  // Plain INPUT (not INPUT_PULLUP): the internal pull-up would add its own ~35k resistor to the
  // line and distort the measurement of the external pull-ups.
  pinMode(kSdaPin, INPUT);
  pinMode(kSclPin, INPUT);
  delayMicroseconds(50);

  const uint16_t vccMv = readVccMillivolts();

  bool bothAboveMinimum = true;
  bool marginal = false;
  const uint8_t pins[2] = {kSdaPin, kSclPin};
  const __FlashStringHelper* names[2] = {F("SDA"), F("SCL")};

  Serial.print(F("IDLE VOLTAGE: VCC="));
  Serial.print(vccMv);
  Serial.print(F("mV"));
  for (uint8_t i = 0; i < 2; ++i) {
    analogRead(pins[i]);  // First conversion after switching channels is less accurate; discard.
    const uint16_t raw = analogRead(pins[i]);
    // The ADC reports the pin as a fraction of VCC (raw/1023); multiply by real VCC to get mV.
    const uint16_t millivolts = static_cast<uint16_t>((static_cast<uint32_t>(raw) * vccMv) / 1023UL);

    Serial.print(F(" "));
    Serial.print(names[i]);
    Serial.print(F("="));
    Serial.print(millivolts);
    Serial.print(F("mV"));

    if (millivolts < kIdleVoltageMinMv) {
      bothAboveMinimum = false;
    }
    if (millivolts < kIdleVoltageGoodMv) {
      marginal = true;
    }
  }

  if (!bothAboveMinimum) {
    Serial.println(F(" FAIL (below 3000 mV minimum HIGH level)"));
    *profilePassed = false;
  } else {
    // Report marginal levels without failing: a 3.3 V pull-up rail is common on breakout
    // modules and usually works, but the user should know the margin is thin.
    Serial.println(marginal ? F(" MARGINAL (pull-ups likely go to 3.3 V; works but thin noise margin)")
                            : F(" GOOD"));
  }

  return bothAboveMinimum;
}

// Scan every legal 7-bit I2C address and report which ones acknowledge.
// A professional bus test always starts with a scan because it immediately reveals:
// - missing devices (expected address does not ACK),
// - address conflicts or wrong solder-jumper settings (device on an unexpected address),
// - a shorted SDA/SCL pair (many/all addresses appear to ACK, which is physically impossible
//   on a healthy bus).
// The probe is a write-only transaction with no data bytes: the device just ACKs its own
// address, nothing inside it is read or changed. VL53L0X is held in reset during the scan, so
// with this train's wiring only 0x20 (MCP23008) and 0x29 (TCS34725) should answer.
void scanBus(const __FlashStringHelper* heading, uint32_t speed, bool* profilePassed) {
  uint8_t devicesFound = 0;

  Serial.print(heading);
  for (uint8_t address = 0x08; address <= 0x77; ++address) {
    Wire.beginTransmission(address);
    const uint8_t status = Wire.endTransmission();
    if (consumeWireTimeout()) {
      Serial.println();
      Serial.println(F("BUS SCAN: Wire timeout during scan; recovering. Profile FAILED."));
      *profilePassed = false;
      restartWireAfterRecovery(speed);
      return;
    }
    if (status != 0) {
      continue;
    }

    devicesFound++;
    Serial.print(F(" "));
    printHexByte(address);
    if (address == kMcp23008Address) {
      Serial.print(F("(MCP23008)"));
    } else if (address == kTcs34725Address) {
      Serial.print(F("(TCS34725)"));
    } else if (address == kVl53l0xAddress) {
      Serial.print(F("(VL53L0X?)"));  // Should not answer here: it is held in reset.
    } else {
      Serial.print(F("(UNEXPECTED)"));
    }
  }
  Serial.println();

  // Far more ACKs than physically present devices means the ACK bit is being faked by a stuck
  // or shorted SDA line, not by real devices.
  if (devicesFound > 16) {
    Serial.println(F("BUS SCAN: too many addresses ACK; SDA may be shorted or stuck. FAIL."));
    *profilePassed = false;
    return;
  }

  if (devicesFound == 0) {
    Serial.println(F("BUS SCAN: no devices answered at all; check power and wiring."));
  }
}

// Record a timeout against the active device and remember that the caller should consider recovery.
void recordTimeout() {
  activeResult->timeouts++;
  lastTransactionTimedOut = true;
}

// If a requestFrom() returned too few bytes, empty any partial bytes so the next transaction starts
// with a clean receive buffer.
void drainWireReceiveBuffer() {
  while (Wire.available() > 0) {
    Wire.read();
  }
}

// Helper for readable serial diagnostics like "0x44" or "0x0A".
void printHexByte(uint8_t value) {
  Serial.print(F("0x"));
  if (value < 0x10) {
    Serial.print(F("0"));
  }
  Serial.print(value, HEX);
}

// Reset the print budget at the start of each phase (setup / stress / confirmation).
void resetErrorBudget(ErrorPrintBudget* budget) {
  budget->used = 0;
  budget->suppressed = 0;
}

// Ask the budget "may I print this error?". Returns true for the first few failures, then false.
// The counters in DeviceResult still count every failure regardless; only the printing is capped.
bool shouldPrintError(ErrorPrintBudget* budget) {
  if (budget->used < kMaxDetailedErrorsPerPhase) {
    budget->used++;
    return true;
  }
  budget->suppressed++;
  return false;
}

// Print a single summary line if any error messages were suppressed during a phase.
void reportSuppressedErrors(const __FlashStringHelper* deviceName,
                            const __FlashStringHelper* phase,
                            const ErrorPrintBudget& budget) {
  if (budget.suppressed == 0) {
    return;
  }
  Serial.print(deviceName);
  Serial.print(F(" "));
  Serial.print(phase);
  Serial.print(F(": "));
  Serial.print(budget.suppressed);
  Serial.println(F(" additional error line(s) suppressed."));
}

void resetFailureDetails(uint8_t address, uint8_t reg) {
  lastFailure.kind = FailureNone;
  lastFailure.address = address;
  lastFailure.reg = reg;
  lastFailure.transmissionStatus = 0;
  lastFailure.receivedBytes = 0;
  lastFailure.expectedBytes = 0;
}

const __FlashStringHelper* transmissionStatusLabel(uint8_t status) {
  switch (status) {
    case 0: return F("success");
    case 1: return F("data-too-long");
    case 2: return F("address-nack");
    case 3: return F("data-nack");
    case 4: return F("other-error");
    case 5: return F("timeout");
    default: return F("unknown");
  }
}

void printAttemptLabel(const __FlashStringHelper* phase, uint16_t attemptNumber) {
  Serial.print(phase);
  Serial.print(F(" attempt "));
  Serial.print(attemptNumber);
}

void printCommonFailurePrefix(const __FlashStringHelper* deviceName,
                              const __FlashStringHelper* phase,
                              uint16_t attemptNumber) {
  Serial.print(deviceName);
  Serial.print(F(" "));
  printAttemptLabel(phase, attemptNumber);
  Serial.print(F(": "));
}

void printReadFailureDetails(const __FlashStringHelper* deviceName,
                             const __FlashStringHelper* phase,
                             uint16_t attemptNumber) {
  printCommonFailurePrefix(deviceName, phase, attemptNumber);

  switch (lastFailure.kind) {
    case FailureRegisterPointerWrite:
      Serial.print(F("failed to queue register pointer "));
      printHexByte(lastFailure.reg);
      Serial.print(F(" for address "));
      printHexByte(lastFailure.address);
      Serial.println(F("."));
      break;

    case FailureEndTransmissionStatus:
      Serial.print(F("register pointer write to address "));
      printHexByte(lastFailure.address);
      Serial.print(F(" failed with Wire.endTransmission status "));
      Serial.print(lastFailure.transmissionStatus);
      Serial.print(F(" ("));
      Serial.print(transmissionStatusLabel(lastFailure.transmissionStatus));
      Serial.print(F(") while selecting register "));
      printHexByte(lastFailure.reg);
      Serial.println(F("."));
      break;

    case FailureShortRead:
      Serial.print(F("short read from address "));
      printHexByte(lastFailure.address);
      Serial.print(F(" register "));
      printHexByte(lastFailure.reg);
      Serial.print(F(": expected "));
      Serial.print(lastFailure.expectedBytes);
      Serial.print(F(" byte, received "));
      Serial.print(lastFailure.receivedBytes);
      Serial.println(F("."));
      break;

    case FailureTimeout:
      Serial.print(F("Wire timeout while accessing address "));
      printHexByte(lastFailure.address);
      Serial.print(F(" register "));
      printHexByte(lastFailure.reg);
      Serial.println(F("."));
      break;

    default:
      Serial.print(F("read failed at address "));
      printHexByte(lastFailure.address);
      Serial.print(F(" register "));
      printHexByte(lastFailure.reg);
      Serial.println(F("."));
      break;
  }
}

void printMismatchDetails(const __FlashStringHelper* deviceName,
                          const __FlashStringHelper* phase,
                          uint16_t attemptNumber,
                          uint8_t expectedValue,
                          uint8_t actualValue) {
  printCommonFailurePrefix(deviceName, phase, attemptNumber);
  Serial.print(F("value mismatch at address "));
  printHexByte(lastFailure.address);
  Serial.print(F(" register "));
  printHexByte(lastFailure.reg);
  Serial.print(F(": expected "));
  printHexByte(expectedValue);
  Serial.print(F(", got "));
  printHexByte(actualValue);
  Serial.println(F("."));
}

void printVl53l0xMoveFailureDetails(uint16_t attemptNumber) {
  printCommonFailurePrefix(F("VL53L0X"), F("address-move"), attemptNumber);

  switch (lastFailure.kind) {
    case FailureAddressMoveWrite:
      Serial.print(F("failed to queue address-change bytes [0x8A, "));
      printHexByte(kVl53l0xAddress);
      Serial.println(F("]."));
      break;

    case FailureEndTransmissionStatus:
      Serial.print(F("address-change write to default address "));
      printHexByte(kVl53l0xDefaultAddress);
      Serial.print(F(" failed with Wire.endTransmission status "));
      Serial.print(lastFailure.transmissionStatus);
      Serial.print(F(" ("));
      Serial.print(transmissionStatusLabel(lastFailure.transmissionStatus));
      Serial.println(F(")."));
      break;

    case FailureTimeout:
      Serial.print(F("Wire timeout while moving sensor from "));
      printHexByte(kVl53l0xDefaultAddress);
      Serial.print(F(" to "));
      printHexByte(kVl53l0xAddress);
      Serial.println(F("."));
      break;

    default:
      Serial.println(F("address move failed."));
      break;
  }
}

// Read "length" bytes from a device register using the standard I2C register-read pattern:
// 1. Master sends: START, slave address+W, register number.
// 2. Master sends a REPEATED START (endTransmission(false)) instead of releasing the bus with a
//    STOP. A repeated start keeps the bus busy so no other master can jump in between the
//    "select register" step and the "read the bytes" step.
// 3. Master sends: slave address+R, reads exactly "length" bytes, then STOP.
//
// This function is the core of the sketch. It counts every failure type precisely (transmission
// error, short read, timeout) so the summary can distinguish "device missing" from "wrong data"
// from "bus timed out". The specific failure is also stored in lastFailure so the caller can
// print a human-readable diagnostic line.
bool readRegisterBytes(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t length) {
  if (activeResult == NULL) {
    return false;
  }

  activeResult->totalAttempts++;
  lastTransactionTimedOut = false;
  resetFailureDetails(address, reg);
  clearWireTimeout();

  Wire.beginTransmission(address);
  if (Wire.write(reg) != 1) {
    lastFailure.kind = FailureRegisterPointerWrite;
    activeResult->transmissionErrors++;
    return false;
  }

  const uint8_t transmissionStatus = Wire.endTransmission(false);
  if (consumeWireTimeout()) {
    lastFailure.kind = FailureTimeout;
    recordTimeout();
    return false;
  }
  if (transmissionStatus != 0) {
    lastFailure.kind = FailureEndTransmissionStatus;
    lastFailure.transmissionStatus = transmissionStatus;
    activeResult->transmissionErrors++;
    return false;
  }

  const uint8_t received = Wire.requestFrom(address, length,
                                             static_cast<uint8_t>(true));
  if (consumeWireTimeout()) {
    drainWireReceiveBuffer();
    lastFailure.kind = FailureTimeout;
    recordTimeout();
    return false;
  }
  if (received != length || Wire.available() != length) {
    drainWireReceiveBuffer();
    lastFailure.kind = FailureShortRead;
    lastFailure.receivedBytes = received;
    lastFailure.expectedBytes = length;
    activeResult->shortReads++;
    return false;
  }

  for (uint8_t i = 0; i < length; ++i) {
    buffer[i] = static_cast<uint8_t>(Wire.read());
  }
  activeResult->successes++;
  return true;
}

// Single-byte convenience wrapper around readRegisterBytes().
bool readRegister8(uint8_t address, uint8_t reg, uint8_t* value) {
  return readRegisterBytes(address, reg, value, 1);
}

// Write one byte to a device register: START, address+W, register, value, STOP.
// Used by the MCP23008 write/read-back test. Counts failures with the same counters as reads.
bool writeRegister8(uint8_t address, uint8_t reg, uint8_t value) {
  if (activeResult == NULL) {
    return false;
  }

  activeResult->totalAttempts++;
  lastTransactionTimedOut = false;
  resetFailureDetails(address, reg);
  clearWireTimeout();

  Wire.beginTransmission(address);
  if (Wire.write(reg) != 1 || Wire.write(value) != 1) {
    lastFailure.kind = FailureRegisterPointerWrite;
    activeResult->transmissionErrors++;
    return false;
  }

  const uint8_t transmissionStatus = Wire.endTransmission();
  if (consumeWireTimeout()) {
    lastFailure.kind = FailureTimeout;
    recordTimeout();
    return false;
  }
  if (transmissionStatus != 0) {
    lastFailure.kind = FailureEndTransmissionStatus;
    lastFailure.transmissionStatus = transmissionStatus;
    activeResult->transmissionErrors++;
    return false;
  }

  activeResult->successes++;
  return true;
}

// Wrapper that selects the device's counters and measures how long the whole transaction took.
bool readDeviceRegister(DeviceResult* result, uint8_t address, uint8_t reg,
                        uint8_t* value) {
  activeResult = result;
  const uint32_t startedUs = micros();
  const bool success = readRegister8(address, reg, value);
  recordTransactionTime(result, micros() - startedUs);
  activeResult = NULL;
  return success;
}

// Multi-byte version of readDeviceRegister(), used by the burst-read parts of the stress loops.
bool readDeviceRegisters(DeviceResult* result, uint8_t address, uint8_t reg,
                         uint8_t* buffer, uint8_t length) {
  activeResult = result;
  const uint32_t startedUs = micros();
  const bool success = readRegisterBytes(address, reg, buffer, length);
  recordTransactionTime(result, micros() - startedUs);
  activeResult = NULL;
  return success;
}

// Write version of readDeviceRegister(), used by the MCP23008 write/read-back test.
bool writeDeviceRegister(DeviceResult* result, uint8_t address, uint8_t reg,
                         uint8_t value) {
  activeResult = result;
  const uint32_t startedUs = micros();
  const bool success = writeRegister8(address, reg, value);
  recordTransactionTime(result, micros() - startedUs);
  activeResult = NULL;
  return success;
}

// VL53L0X powers up at 0x29, which clashes with the TCS34725 color sensor.
// This write changes VL53L0X to 0x2A so both devices can coexist on the bus during the test.
bool moveVl53l0x(DeviceResult* result) {
  activeResult = result;
  activeResult->totalAttempts++;
  lastTransactionTimedOut = false;
  resetFailureDetails(kVl53l0xDefaultAddress, 0x8A);
  clearWireTimeout();

  Wire.beginTransmission(kVl53l0xDefaultAddress);
  const size_t pointerWritten = Wire.write(0x8A);
  const size_t addressWritten = Wire.write(kVl53l0xAddress);
  if (pointerWritten != 1 || addressWritten != 1) {
    lastFailure.kind = FailureAddressMoveWrite;
    activeResult->transmissionErrors++;
    activeResult = NULL;
    return false;
  }

  const uint8_t transmissionStatus = Wire.endTransmission();
  if (consumeWireTimeout()) {
    lastFailure.kind = FailureTimeout;
    recordTimeout();
    activeResult = NULL;
    return false;
  }
  if (transmissionStatus != 0) {
    lastFailure.kind = FailureEndTransmissionStatus;
    lastFailure.transmissionStatus = transmissionStatus;
    activeResult->transmissionErrors++;
    activeResult = NULL;
    return false;
  }

  activeResult->successes++;
  activeResult = NULL;
  return true;
}

// If a transaction timed out, this profile is already considered failed. Still, try bus recovery so
// later transactions can continue and provide more information instead of stopping at the first fault.
void handleTimeoutIfNeeded(uint32_t speed, const __FlashStringHelper* source,
                           bool* profilePassed) {
  if (!lastTransactionTimedOut) {
    return;
  }

  *profilePassed = false;
  Serial.print(F("TIMEOUT: "));
  Serial.println(source);
  Serial.println(F("Recovering and restarting Wire; this profile remains failed."));
  if (!restartWireAfterRecovery(speed)) {
    Serial.println(F("RECOVERY AFTER TIMEOUT: FAIL"));
  } else {
    Serial.println(F("RECOVERY AFTER TIMEOUT: PASS"));
  }
}

// A device has a clean result only when every error counter stayed at zero.
bool resultHasNoErrors(const DeviceResult& result) {
  return result.transmissionErrors == 0 && result.shortReads == 0 &&
         result.mismatches == 0 && result.timeouts == 0;
}

// Print one beginner-readable summary line per device at the end of each speed profile.
void printDeviceResult(const __FlashStringHelper* name, const DeviceResult& result,
                       bool setupPassed, bool stressCompleted) {
  const bool passed = setupPassed && stressCompleted && resultHasNoErrors(result);

  Serial.print(name);
  Serial.print(F(": totalAttempts="));
  Serial.print(result.totalAttempts);
  Serial.print(F(" stressAttempts="));
  Serial.print(result.stressAttempts);
  Serial.print(F(" successes="));
  Serial.print(result.successes);
  Serial.print(F(" transmissionErrors="));
  Serial.print(result.transmissionErrors);
  Serial.print(F(" shortReads="));
  Serial.print(result.shortReads);
  Serial.print(F(" mismatches="));
  Serial.print(result.mismatches);
  Serial.print(F(" timeouts="));
  Serial.print(result.timeouts);
  if (result.timedTransactions > 0) {
    Serial.print(F(" usMin/avg/max="));
    Serial.print(result.minTimeUs);
    Serial.print(F("/"));
    Serial.print(result.sumTimeUs / result.timedTransactions);
    Serial.print(F("/"));
    Serial.print(result.maxTimeUs);
  }
  Serial.print(F(" setup="));
  Serial.print(setupPassed ? F("PASS") : F("FAIL"));
  Serial.print(F(" stress="));
  Serial.print(stressCompleted ? F("COMPLETE") : F("SKIPPED"));
  Serial.print(F(" "));
  Serial.println(passed ? F("PASS") : F("FAIL"));
}

// Write/read-back data-integrity test on the MCP23008 DEFVAL scratch register.
// Read-only tests can never prove that WRITES reach the device intact, and a constant ID value
// can hide a stuck data bit (if the stuck level happens to match the ID). Writing all four
// patterns (0x00, 0xFF, 0x55, 0xAA) and reading each back exercises every bit in both states
// and both transfer directions. The original register value is restored afterwards.
void runMcpWriteReadback(uint32_t speed, uint16_t readsPerDevice, DeviceResult* result,
                         bool* profilePassed) {
  ErrorPrintBudget budget;
  resetErrorBudget(&budget);

  uint8_t originalValue = 0;
  if (!readDeviceRegister(result, kMcp23008Address, kMcp23008DefvalRegister,
                          &originalValue)) {
    *profilePassed = false;
    printReadFailureDetails(F("MCP23008"), F("write-readback"), 1);
    handleTimeoutIfNeeded(speed, F("MCP23008 DEFVAL original read"), profilePassed);
    Serial.println(F("MCP23008 WRITE-READBACK: skipped (could not read original DEFVAL)."));
    return;
  }

  // Scale the number of cycles with the selected test length: 10 cycles for the short test,
  // 100 for the long test. Each cycle writes and verifies all four patterns.
  const uint16_t cycles = readsPerDevice / 100;
  for (uint16_t cycle = 0; cycle < cycles; ++cycle) {
    for (uint8_t p = 0; p < kWritePatternCount; ++p) {
      const uint8_t pattern = kWritePatterns[p];
      const uint16_t attemptNumber = cycle * kWritePatternCount + p + 1;

      if (!writeDeviceRegister(result, kMcp23008Address, kMcp23008DefvalRegister,
                               pattern)) {
        *profilePassed = false;
        if (shouldPrintError(&budget)) {
          printReadFailureDetails(F("MCP23008"), F("write-readback"), attemptNumber);
        }
        handleTimeoutIfNeeded(speed, F("MCP23008 DEFVAL write"), profilePassed);
        continue;
      }

      uint8_t readBack = 0;
      if (!readDeviceRegister(result, kMcp23008Address, kMcp23008DefvalRegister,
                              &readBack)) {
        *profilePassed = false;
        if (shouldPrintError(&budget)) {
          printReadFailureDetails(F("MCP23008"), F("write-readback"), attemptNumber);
        }
        handleTimeoutIfNeeded(speed, F("MCP23008 DEFVAL read-back"), profilePassed);
      } else if (readBack != pattern) {
        result->mismatches++;
        *profilePassed = false;
        if (shouldPrintError(&budget)) {
          printMismatchDetails(F("MCP23008"), F("write-readback"), attemptNumber,
                               pattern, readBack);
        }
      }
    }
  }
  reportSuppressedErrors(F("MCP23008"), F("write-readback"), budget);

  // Restore the register so the test leaves the expander exactly as it found it.
  if (!writeDeviceRegister(result, kMcp23008Address, kMcp23008DefvalRegister,
                           originalValue)) {
    *profilePassed = false;
    Serial.println(F("MCP23008 WRITE-READBACK: failed to restore original DEFVAL."));
    handleTimeoutIfNeeded(speed, F("MCP23008 DEFVAL restore"), profilePassed);
  }
}

// Stress-test MCP23008 with repeated reads. Most iterations read the single IODIR byte; every
// 10th iteration instead reads a 4-byte block (IODIR..DEFVAL) using the expander's sequential
// address-pointer mode, so sustained multi-byte transfers get exercised too.
// The first successful setup read becomes the "baseline" IODIR value; every later read of that
// register must match it. Detailed failure lines are throttled by kMaxDetailedErrorsPerPhase to
// avoid serial flooding; the DeviceResult counters still track every failure.
void runMcpStress(uint32_t speed, uint16_t readsPerDevice, DeviceResult* result, uint8_t baseline,
                  bool* profilePassed) {
  ErrorPrintBudget budget;
  resetErrorBudget(&budget);
  for (uint16_t read = 0; read < readsPerDevice; ++read) {
    result->stressAttempts++;
    const bool burst = ((read + 1) % kBurstEveryNthRead) == 0;
    uint8_t buffer[4] = {0, 0, 0, 0};
    const bool ok = burst
        ? readDeviceRegisters(result, kMcp23008Address, kMcp23008IodirRegister,
                              buffer, sizeof(buffer))
        : readDeviceRegister(result, kMcp23008Address, kMcp23008IodirRegister,
                             &buffer[0]);
    if (!ok) {
      *profilePassed = false;
      if (shouldPrintError(&budget)) {
        printReadFailureDetails(F("MCP23008"), F("stress"), read + 1);
      }
      handleTimeoutIfNeeded(speed, F("MCP23008 stress read"), profilePassed);
    } else if (buffer[0] != baseline) {
      result->mismatches++;
      *profilePassed = false;
      if (shouldPrintError(&budget)) {
        printMismatchDetails(F("MCP23008"), F("stress"), read + 1, baseline, buffer[0]);
      }
    }
  }
  reportSuppressedErrors(F("MCP23008"), F("stress"), budget);
}

// Stress-test the color sensor. Most iterations re-read the 1-byte ID; every 10th iteration
// instead reads the full 8-byte RGBC data block with auto-increment, exactly like the real
// color-sensor driver does. The RGBC values themselves are not checked (the sensor ADC is off
// during this test), only that the 8-byte transfer completes cleanly.
// Detailed failure lines are throttled to avoid a flood when the sensor misbehaves.
void runTcsStress(uint32_t speed, uint16_t readsPerDevice, DeviceResult* result,
                  uint8_t expectedId,
                  bool* profilePassed) {
  ErrorPrintBudget budget;
  resetErrorBudget(&budget);
  for (uint16_t read = 0; read < readsPerDevice; ++read) {
    result->stressAttempts++;
    const bool burst = ((read + 1) % kBurstEveryNthRead) == 0;
    if (burst) {
      uint8_t block[8];
      if (!readDeviceRegisters(result, kTcs34725Address, kTcs34725RgbcBlockCommand,
                               block, sizeof(block))) {
        *profilePassed = false;
        if (shouldPrintError(&budget)) {
          printReadFailureDetails(F("TCS34725"), F("stress"), read + 1);
        }
        handleTimeoutIfNeeded(speed, F("TCS34725 stress burst read"), profilePassed);
      }
      continue;
    }

    uint8_t value = 0;
    if (!readDeviceRegister(result, kTcs34725Address, kTcs34725IdRegister,
                            &value)) {
      *profilePassed = false;
      if (shouldPrintError(&budget)) {
        printReadFailureDetails(F("TCS34725"), F("stress"), read + 1);
      }
      handleTimeoutIfNeeded(speed, F("TCS34725 stress read"), profilePassed);
    } else if (value != expectedId) {
      result->mismatches++;
      *profilePassed = false;
      if (shouldPrintError(&budget)) {
        printMismatchDetails(F("TCS34725"), F("stress"), read + 1, expectedId, value);
      }
    }
  }
  reportSuppressedErrors(F("TCS34725"), F("stress"), budget);
}

// Stress-test VL53L0X. Most iterations read the 1-byte model ID; every 10th iteration reads the
// 3-byte identification block starting at 0xC0. The first byte must always be 0xEE.
// Detailed failure lines are throttled to avoid a flood when the sensor misbehaves.
void runVlStress(uint32_t speed, uint16_t readsPerDevice, DeviceResult* result,
                 bool* profilePassed) {
  ErrorPrintBudget budget;
  resetErrorBudget(&budget);
  for (uint16_t read = 0; read < readsPerDevice; ++read) {
    result->stressAttempts++;
    const bool burst = ((read + 1) % kBurstEveryNthRead) == 0;
    uint8_t buffer[3] = {0, 0, 0};
    const bool ok = burst
        ? readDeviceRegisters(result, kVl53l0xAddress, kVl53l0xModelIdRegister,
                              buffer, sizeof(buffer))
        : readDeviceRegister(result, kVl53l0xAddress, kVl53l0xModelIdRegister,
                             &buffer[0]);
    if (!ok) {
      *profilePassed = false;
      if (shouldPrintError(&budget)) {
        printReadFailureDetails(F("VL53L0X"), F("stress"), read + 1);
      }
      handleTimeoutIfNeeded(speed, F("VL53L0X stress read"), profilePassed);
    } else if (buffer[0] != 0xEE) {
      result->mismatches++;
      *profilePassed = false;
      if (shouldPrintError(&budget)) {
        printMismatchDetails(F("VL53L0X"), F("stress"), read + 1, 0xEE, buffer[0]);
      }
    }
  }
  reportSuppressedErrors(F("VL53L0X"), F("stress"), budget);
}

// Interleaved round-robin stress: alternate rapidly between all devices that passed setup.
// The per-device stress loops above test each device in isolation, but some real bus faults
// only show up when the master switches between targets quickly (for example, a device that is
// slow to release SDA after its STOP, corrupting the START of the next transaction). This phase
// distributes readsPerDevice transactions across the participating devices in rotation.
void runInterleavedStress(uint32_t speed, uint16_t readsPerDevice,
                          DeviceResult* mcp, uint8_t mcpBaseline, bool mcpActive,
                          DeviceResult* tcs, bool tcsActive,
                          DeviceResult* vl, bool vlActive,
                          bool* profilePassed) {
  const uint8_t participants = (mcpActive ? 1 : 0) + (tcsActive ? 1 : 0) + (vlActive ? 1 : 0);
  if (participants < 2) {
    Serial.println(F("INTERLEAVED: skipped (needs at least two working devices)."));
    return;
  }

  ErrorPrintBudget budget;
  resetErrorBudget(&budget);
  uint8_t turn = 0;

  for (uint16_t i = 0; i < readsPerDevice; ++i) {
    // Pick the next active device in rotation.
    uint8_t device = turn % 3;
    turn++;
    if (device == 0 && !mcpActive) device = tcsActive ? 1 : 2;
    if (device == 1 && !tcsActive) device = vlActive ? 2 : 0;
    if (device == 2 && !vlActive) device = mcpActive ? 0 : 1;

    uint8_t value = 0;
    if (device == 0) {
      mcp->stressAttempts++;
      if (!readDeviceRegister(mcp, kMcp23008Address, kMcp23008IodirRegister, &value)) {
        *profilePassed = false;
        if (shouldPrintError(&budget)) {
          printReadFailureDetails(F("MCP23008"), F("interleaved"), i + 1);
        }
        handleTimeoutIfNeeded(speed, F("interleaved MCP23008 read"), profilePassed);
      } else if (value != mcpBaseline) {
        mcp->mismatches++;
        *profilePassed = false;
        if (shouldPrintError(&budget)) {
          printMismatchDetails(F("MCP23008"), F("interleaved"), i + 1, mcpBaseline, value);
        }
      }
    } else if (device == 1) {
      tcs->stressAttempts++;
      if (!readDeviceRegister(tcs, kTcs34725Address, kTcs34725IdRegister, &value)) {
        *profilePassed = false;
        if (shouldPrintError(&budget)) {
          printReadFailureDetails(F("TCS34725"), F("interleaved"), i + 1);
        }
        handleTimeoutIfNeeded(speed, F("interleaved TCS34725 read"), profilePassed);
      } else if (value != kTcs34725ExpectedId) {
        tcs->mismatches++;
        *profilePassed = false;
        if (shouldPrintError(&budget)) {
          printMismatchDetails(F("TCS34725"), F("interleaved"), i + 1,
                               kTcs34725ExpectedId, value);
        }
      }
    } else {
      vl->stressAttempts++;
      if (!readDeviceRegister(vl, kVl53l0xAddress, kVl53l0xModelIdRegister, &value)) {
        *profilePassed = false;
        if (shouldPrintError(&budget)) {
          printReadFailureDetails(F("VL53L0X"), F("interleaved"), i + 1);
        }
        handleTimeoutIfNeeded(speed, F("interleaved VL53L0X read"), profilePassed);
      } else if (value != 0xEE) {
        vl->mismatches++;
        *profilePassed = false;
        if (shouldPrintError(&budget)) {
          printMismatchDetails(F("VL53L0X"), F("interleaved"), i + 1, 0xEE, value);
        }
      }
    }
  }
  reportSuppressedErrors(F("BUS"), F("interleaved"), budget);
}

// Run the full diagnostic once at one bus speed.
// Each speed profile is self-contained so the 100 kHz and 400 kHz results can be compared directly.
bool runProfile(uint32_t speed, uint16_t readsPerDevice) {
  DeviceResult mcp;
  DeviceResult tcs;
  DeviceResult vl;
  initDeviceResult(&mcp);
  initDeviceResult(&tcs);
  initDeviceResult(&vl);
  bool profilePassed = true;
  bool mcpSetupPassed = false;
  bool tcsSetupPassed = false;
  bool vlSetupPassed = false;
  bool mcpStressCompleted = false;
  bool tcsStressCompleted = false;
  bool vlStressCompleted = false;

  Serial.println();
  Serial.print(F("===== "));
  Serial.print(speed / 1000UL);
  Serial.print(F(" kHz PROFILE ("));
  Serial.print(readsPerDevice);
  Serial.println(F(" stress reads/device) ====="));

  // Step 1: measure the raw idle voltage on both lines before any I2C traffic. This must run
  // while Wire is off, so it happens before recovery/configuration.
  measureIdleVoltages(&profilePassed);

  if (!restartWireAfterRecovery(speed)) {
    profilePassed = false;
    Serial.println(F("PROFILE SETUP: bus recovery failed."));
  }

  // Keep VL53L0X disabled at first so the TCS34725 is the only device answering at 0x29.
  digitalWrite(kVlXshutPin, LOW);
  delay(10);

  // Step 2: scan the whole address range so missing, extra, or ghost devices are visible
  // before any register-level testing starts.
  scanBus(F("BUS SCAN BEFORE VL53L0X MOVE:"), speed, &profilePassed);

  uint8_t mcpBaseline = 0;
  if (readDeviceRegister(&mcp, kMcp23008Address, kMcp23008IodirRegister,
                         &mcpBaseline)) {
    mcpSetupPassed = true;
    Serial.print(F("MCP23008 SETUP: baseline IODIR = "));
    printHexByte(mcpBaseline);
    Serial.println(F("."));
  } else {
    profilePassed = false;
    Serial.println(F("MCP23008 SETUP: IODIR read failed."));
    printReadFailureDetails(F("MCP23008"), F("setup"), 1);
    handleTimeoutIfNeeded(speed, F("MCP23008 setup read"), &profilePassed);
  }

  uint8_t tcsId = 0;
  if (readDeviceRegister(&tcs, kTcs34725Address, kTcs34725IdRegister, &tcsId)) {
    if (tcsId == kTcs34725ExpectedId) {
      tcsSetupPassed = true;
      Serial.print(F("TCS34725 SETUP: observed expected ID "));
      printHexByte(tcsId);
      Serial.println(F("."));
    } else {
      tcs.mismatches++;
      profilePassed = false;
      Serial.print(F("TCS34725 SETUP: expected ID "));
      printHexByte(kTcs34725ExpectedId);
      Serial.print(F(", got "));
      printHexByte(tcsId);
      Serial.println(F("."));
    }
  } else {
    profilePassed = false;
    Serial.println(F("TCS34725 SETUP: ID read failed."));
    printReadFailureDetails(F("TCS34725"), F("setup"), 1);
    handleTimeoutIfNeeded(speed, F("TCS34725 setup read"), &profilePassed);
  }

  if (mcpSetupPassed) {
    // Data-integrity check first (writes + read-backs), then the read-only stress loop.
    runMcpWriteReadback(speed, readsPerDevice, &mcp, &profilePassed);
    runMcpStress(speed, readsPerDevice, &mcp, mcpBaseline, &profilePassed);
    mcpStressCompleted = true;
  }
  if (tcsSetupPassed) {
    runTcsStress(speed, readsPerDevice, &tcs, kTcs34725ExpectedId, &profilePassed);
    tcsStressCompleted = true;
  }

  // Re-enable the distance sensor only after the color-sensor checks are done.
  digitalWrite(kVlXshutPin, HIGH);
  delay(10);

  if (!moveVl53l0x(&vl)) {
    profilePassed = false;
    Serial.println(F("VL53L0X SETUP: address move write failed."));
    printVl53l0xMoveFailureDetails(1);
    handleTimeoutIfNeeded(speed, F("VL53L0X address move"), &profilePassed);
  } else {
    // After releasing XSHUT the sensor boots at 0x29, we move it to 0x2A, then re-read its
    // model ID from the new address to prove the move actually worked. We give it up to 10
    // tries because the sensor firmware takes a moment to finish booting after the address
    // change. Errors are throttled the same way as the stress loops.
    bool modelConfirmed = false;
    ErrorPrintBudget confirmBudget;
    resetErrorBudget(&confirmBudget);
    for (uint8_t attempt = 0; attempt < 10 && !modelConfirmed; ++attempt) {
      uint8_t modelId = 0;
      if (readDeviceRegister(&vl, kVl53l0xAddress, kVl53l0xModelIdRegister,
                             &modelId)) {
        if (modelId == 0xEE) {
          modelConfirmed = true;
        } else {
          vl.mismatches++;
          profilePassed = false;
          if (shouldPrintError(&confirmBudget)) {
            printMismatchDetails(F("VL53L0X"), F("setup-confirm"), attempt + 1, 0xEE, modelId);
          }
        }
      } else {
        profilePassed = false;
        if (shouldPrintError(&confirmBudget)) {
          printReadFailureDetails(F("VL53L0X"), F("setup-confirm"), attempt + 1);
        }
        handleTimeoutIfNeeded(speed, F("VL53L0X model-ID confirmation"),
                              &profilePassed);
      }

      if (!modelConfirmed && attempt < 9) {
        delay(2);
      }
    }
    reportSuppressedErrors(F("VL53L0X"), F("setup-confirm"), confirmBudget);

    if (modelConfirmed) {
      vlSetupPassed = true;
      // The VL53L0X is now active at 0x2A, so this second scan confirms that all three
      // expected devices acknowledge simultaneously without the original 0x29 collision.
      scanBus(F("BUS SCAN AFTER VL53L0X MOVE:"), speed, &profilePassed);
    } else {
      profilePassed = false;
      Serial.println(F("VL53L0X SETUP: model ID 0xEE was not confirmed."));
    }
  }

  if (vlSetupPassed) {
    runVlStress(speed, readsPerDevice, &vl, &profilePassed);
    vlStressCompleted = true;
  }

  // Final stress phase: rapid alternation between all working devices. At this point the
  // VL53L0X is still enabled at 0x2A, so all three can share the bus without collisions.
  runInterleavedStress(speed, readsPerDevice,
                       &mcp, mcpBaseline, mcpSetupPassed,
                       &tcs, tcsSetupPassed,
                       &vl, vlSetupPassed,
                       &profilePassed);

  // Leave VL53L0X shut down so the next speed profile starts from the default-address power-up state.
  digitalWrite(kVlXshutPin, LOW);
  delay(2);

  const bool busIdleAfterRun = isBusIdleHigh();
  Serial.print(F("BUS AFTER RUN: SDA="));
  Serial.print(digitalRead(kSdaPin) == HIGH ? F("HIGH") : F("LOW"));
  Serial.print(F(" SCL="));
  Serial.println(digitalRead(kSclPin) == HIGH ? F("HIGH") : F("LOW"));
  if (!busIdleAfterRun) {
    profilePassed = false;
    Serial.println(F("POST-RUN BUS STATE: FAIL; recovering and restarting Wire."));
    if (!restartWireAfterRecovery(speed)) {
      Serial.println(F("RECOVERY AFTER NON-IDLE: FAIL"));
    } else {
      Serial.println(F("RECOVERY AFTER NON-IDLE: PASS"));
    }
  }

  printDeviceResult(F("MCP23008"), mcp, mcpSetupPassed, mcpStressCompleted);
  printDeviceResult(F("TCS34725"), tcs, tcsSetupPassed, tcsStressCompleted);
  printDeviceResult(F("VL53L0X"), vl, vlSetupPassed, vlStressCompleted);

  // profilePassed has already been set to false by every failure branch above, so we do not
  // need a final re-check. Just print and return.
  Serial.print(F("PROFILE "));
  Serial.print(speed / 1000UL);
  Serial.print(F(" kHz: "));
  Serial.println(profilePassed ? F("PASS") : F("FAIL"));
  return profilePassed;
}

// Display the menu and wait until the user selects a test length.
// Nothing on the I2C bus is tested before this function returns with a valid choice.
uint16_t waitForTestSelection() {
  Serial.println();
  Serial.println(F("SELECT I2C TEST LENGTH:"));
  Serial.print(F("  1 = SHORT: "));
  Serial.print(kShortReadsPerDevice);
  Serial.println(F(" stress reads per device"));
  Serial.print(F("  2 = LONG:  "));
  Serial.print(kLongReadsPerDevice);
  Serial.println(F(" stress reads per device"));
  Serial.println(F("Enter 1 or 2 in the Serial Monitor, then press Send."));

  while (true) {
    if (Serial.available() == 0) {
      delay(10);
      continue;
    }

    const char selection = static_cast<char>(Serial.read());
    if (selection == '1') {
      Serial.println(F("SHORT test selected."));
      return kShortReadsPerDevice;
    }
    if (selection == '2') {
      Serial.println(F("LONG test selected."));
      return kLongReadsPerDevice;
    }

    // Serial Monitor commonly sends a newline after the selected character. Ignore whitespace
    // quietly; report any other character so the user knows why the test did not start.
    if (selection != '\r' && selection != '\n' && selection != ' ' && selection != '\t') {
      Serial.print(F("Invalid selection '"));
      Serial.print(selection);
      Serial.println(F("'. Enter 1 for SHORT or 2 for LONG."));
    }
  }
}

// Run both I2C speeds for the user-selected stress length and print one overall result.
void runDiagnostic(uint16_t readsPerDevice) {
  Serial.println();
  Serial.print(F("STARTING I2C DIAGNOSTIC: "));
  Serial.print(readsPerDevice);
  Serial.println(F(" stress reads per device, per speed."));

  const bool standardPassed = runProfile(kStandardSpeed, readsPerDevice);
  const bool fastPassed = runProfile(kFastSpeed, readsPerDevice);

  Serial.println();
  Serial.print(F("OVERALL: "));
  Serial.println(standardPassed && fastPassed ? F("PASS") : F("FAIL"));
}

// Arduino entry point. setup() prepares the serial monitor and leaves the test selection to loop().
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(kVlXshutPin, OUTPUT);
  digitalWrite(kVlXshutPin, LOW);
  delay(10);

  Serial.println(F("I2C BUS DIAGNOSTIC"));
  Serial.println(F("Nano ATmega328P: MCP23008, TCS34725, VL53L0X"));
}

// Return to the selection menu after every completed run. This allows a short and a long test to
// be compared without resetting the Nano or uploading the sketch again.
void loop() {
  const uint16_t readsPerDevice = waitForTestSelection();
  runDiagnostic(readsPerDevice);
}
