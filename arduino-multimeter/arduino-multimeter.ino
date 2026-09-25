#include <Arduino.h>
#include <math.h>

/* Capacitance Meter Example

 *  Demonstrates use of RC time constants to measure the value of a capacitor

 * Theory   A capacitor will charge, through a resistor, in one time constant,
 defined as T seconds where

 *    TC = R * C
 *    TC = time constant period in seconds
 *    R = resistance in ohms
 *    C = capacitance in farads (1 microfarad (ufd) = .000001 farad = 10^-6
 farads )

 *    The capacitor's voltage at one time constant is defined as 63.2% of the
 charging voltage.

 *  Hardware setup:

 *  Test Capacitor between common point and ground (positive side of an
 electrolytic capacitor  to common)
 *  Test Resistor between chargePin and common point (10k ohm for small
 capacitors, 1k ohm for large capacitors)
 *  220 ohm resistor between dischargePin and common point (limits discharge
 current)
 *  Wire between common point and analogPin (A/D input)

 */

// https://docs.arduino.cc/tutorials/generic/capacitance-meter/
// https://www.instructables.com/Measure-Capacitance-with-Arduino/

// The following code has been updated for Dual-Resistor Auto-Ranging (10k ohm
// and 1k ohm)

// ===== Pin assignment =====
// Each analog pin measures a different circuit. Keep these assignments in
// sync with the physical wiring; changing a pin here does not rewire hardware.
const uint8_t VOLTAGE_ANALOG_PIN = A0;
const uint8_t CAPACITANCE_ANALOG_PIN = A1;
// Resistance-meter wiring:
// 1. Connect each D2-D6 pin through its own reference resistor to one shared
//    point: D2--221R, D3--1k, D4--9.84k, D5--92k, and D6--970k.
// 2. Connect the shared point to one end of the unknown resistor (Rx) and to
//    A2 for the temporary Uno test (A6 in the final Nano meter).
// 3. Connect Rx's other end to both Arduino GND and A3 for the temporary Uno
//    test (A7 in the final Nano meter). A3/A7 only senses this ground point.
//
// Only one D2-D6 pin is OUTPUT/HIGH at a time; every other range pin is high
// impedance. No D2-D6 pin connects directly to A2/A6 or another range pin.
//
// TEMPORARY UNO TEST WIRING: A2 replaces final-meter A6 and A3 replaces
// final-meter A7. Restore these constants to A6 and A7 for the Nano meter.
const uint8_t RESISTANCE_JUNCTION_ANALOG_PIN = A2;
const uint8_t RESISTANCE_GROUND_REFERENCE_ANALOG_PIN = A3;
const uint8_t CHARGE_PIN_10K = 13;
const uint8_t CHARGE_PIN_1K = 7;
const uint8_t DISCHARGE_PIN = 11;
const uint8_t RESISTANCE_FIRST_PIN = 2;
const uint8_t RESISTANCE_LAST_PIN = 6;

// Serial Monitor speed. Set the Serial Monitor to the same value.
const unsigned long SERIAL_BAUD_RATE = 115200UL;

// ===== Voltage meter calibration =====
// A0 sees the lower resistor of a 100k/10k divider, so DIVIDER_RATIO converts
// the safe ADC voltage back to the voltage at the divider input.
const float DIVIDER_R1 = 100000.0F;
const float DIVIDER_R2 = 10000.0F;
const float DIVIDER_RATIO = (DIVIDER_R1 + DIVIDER_R2) / DIVIDER_R2;
// Fine adjustment for measured resistor/reference tolerances.
const float VOLTAGE_CALIBRATION = 1.01F;
// ADC reference voltages used by the Nano/Uno. The 1.1 V reference improves
// resolution for small inputs, while the 5 V reference handles larger inputs.
const float DEFAULT_REFERENCE_VOLTAGE = 5.0F;
const float INTERNAL_REFERENCE_VOLTAGE = 1.1F;
// Divider-input voltage where the program changes to the internal reference.
const float VOLTAGE_REF_SWITCH_THRESHOLD = 11.80F;

// ===== Capacitance meter settings =====
// Each timeout prevents a missing, very large, or miswired capacitor from
// trapping the meter in a charge/discharge loop forever.
const unsigned long CAP_10K_TIMEOUT_US = 100000UL;
const unsigned long CAP_1K_TIMEOUT_US = 1000000UL;
const unsigned long CAP_DISCHARGE_TIMEOUT_US = 1000000UL;
// A short 1k charge time means a small capacitor; repeat it through the 10k
// resistor to obtain more timing resolution.
const unsigned long CAP_FAST_PROBE_MIN_US = 1500UL;
// Values below this are displayed but marked as outside the reliable range.
const float CAP_RELIABLE_MIN_NF = 10.0F;
// Show values at or above 0.75 uF in uF; smaller values are shown in nF.
const float CAP_MICROFARAD_DISPLAY_THRESHOLD = 0.75F;
// 648 is about 63.2% of a 10-bit ADC's 1023 maximum: one RC time constant.
const int CAP_CHARGE_THRESHOLD = 648;
// ADC reading considered sufficiently close to zero after discharging.
const int CAP_DISCHARGE_LIMIT = 5;
// Measured, rather than nominal, charging-resistor values improve accuracy.
const float CAP_RESISTOR_10K = 9900.0F;
const float CAP_RESISTOR_1K = 1018.0F;
// Percentage bands used only to explain how closely a reading matches common
// capacitor values; they do not change the measured capacitance.
const float CAP_NOMINAL_DIFFERENCE_LIMIT_PERCENT = 20.0F;
const float CAP_NOMINAL_NOT_FOUND_LIMIT_PERCENT = 50.0F;
// Common nominal capacitor values, stored in nF for one consistent unit.
const float CAP_NOMINAL_VALUES_NF[] = {
    100.0F, 1000.0F, 2200.0F, 3300.0F, 4700.0F, 10000.0F, 22000.0F,
    33000.0F, 47000.0F, 100000.0F, 220000.0F, 330000.0F, 470000.0F};
const int CAP_NOMINAL_VALUE_COUNT = sizeof(CAP_NOMINAL_VALUES_NF) / sizeof(CAP_NOMINAL_VALUES_NF[0]);

// ===== Resistance meter settings =====
// Raw ADC limits distinguish a shorted input, a valid divider voltage, and
// an open input close to the ADC's 1023 maximum.
const float RESISTANCE_MIN_VALID_RAW = 5.0F;
const float RESISTANCE_OPEN_INPUT_MIN_RAW = 1018.0F;
// D4 is temporarily driven LOW to discharge the 100 nF input capacitor.
const uint8_t RESISTANCE_DISCHARGE_PIN = 4;
// Allow +20% on the installed 100 nF capacitor, then wait nine RC time
// constants (<0.013% remaining voltage error) before checking stability.
const float RESISTANCE_SETTLING_CAPACITANCE_F = 120.0e-9F;
const float RESISTANCE_SETTLING_TIME_CONSTANTS = 9.0F;
// Three 32-pair windows are needed: one baseline plus two matching windows.
// Each pair includes four 1 ms ADC-settling delays, so 500 ms leaves margin
// for ADC conversion time while still reporting genuinely unstable inputs.
const unsigned long RESISTANCE_STABILITY_TIMEOUT_MS = 500UL;
// Consecutive 32-sample medians may differ by one ADC count. At low divider
// voltages, such as D6 measuring a low-ohm resistor, ADC quantization alone
// can move the median between adjacent whole counts without indicating drift.
const float RESISTANCE_STABLE_MEDIAN_CHANGE_RAW = 1.0F;
// Noise/spread limits reject measurements that are still moving or noisy.
const float RESISTANCE_MAX_STDDEV_RAW = 2.0F;
const int RESISTANCE_MAX_SPREAD_RAW = 8;
const int RESISTANCE_STABLE_COMPARISONS = 2;
// The ADC has unavoidable quantization/noise; this floor prevents falsely
// claiming unrealistic precision during range quality comparison.
const float RESISTANCE_ADC_ERROR_FLOOR_RAW = 2.0F;
// A consistency screen, not a claimed accuracy or calibration correction.
const float RESISTANCE_AGREEMENT_FRACTION = 0.05F;
const float RESISTANCE_COMPARISON_QUALITY_FACTOR = 2.0F;

struct ResistanceRange {
  // GPIO drives the top of this reference resistor; value is its calibrated
  // resistance in ohms.
  uint8_t pin;
  float referenceOhms;
};

// Auto-ranging reference resistors. The program enables exactly one at once.
const ResistanceRange RESISTANCE_RANGES[] = {
    {2, 221.0F},
    {3, 1000.0F},
    {4, 9840.0F},
    {5, 92000.0F},
    {6, 970000.0F},
};
const int RESISTANCE_RANGE_COUNT = sizeof(RESISTANCE_RANGES) / sizeof(RESISTANCE_RANGES[0]);
// Separate counts allow each physical measurement method to trade speed for
// noise rejection independently.
const int RESISTANCE_SAMPLES = 32;
const int CAPACITANCE_SAMPLES = 3;

// Zero-offset values measured with an empty capacitance socket. They remove
// stray capacitance from the wiring only after the user runs command 'z'.
float capacitanceBaseline1kNanoFarads = 0.0F;
float capacitanceBaseline10kNanoFarads = 0.0F;
bool capacitanceBaselinesCalibrated = false;

// Voltage needs fewer samples because each ADC read is fast and the divider is
// low impedance. ADC delays allow the sample-and-hold capacitor to settle.
const int VOLTAGE_SAMPLES = 10;
const unsigned long ADC_SETTLE_DELAY_MS = 25UL;
const unsigned long ADC_SAMPLE_SPACING_MS = 1UL;
const int ADC_REFERENCE_DUMMY_READS = 5;
const int OSCILLOGRAPH_SAMPLES = 128;
const uint8_t OSCILLOGRAPH_ADC_PRESCALER_BITS = 0x05; // /32 for faster buffered capture
const uint8_t ADC_DEFAULT_PRESCALER_BITS = 0x07;      // /128 for normal Arduino reads

// Statistics calculated from one mode's temporary sample array. The median is
// the robust value used by voltage/capacitance/resistance; mean and deviation
// remain useful diagnostics, and resistance uses deviation for range quality.
struct SampleStats {
  float medianRaw;
  float meanRaw;
  float stdDevRaw;
  float minRaw;
  float maxRaw;
};

// Public result types separate the measurement work from Serial presentation.
struct VoltageResult {
  bool ok;
  bool usedInternalReference;
  float voltage;
  float stdDev;
};

struct CapacitanceResult {
  // value is expressed by unit ("nF" or "uF"); usedPrecisionRange records
  // whether the slower 10k timing range produced this reading.
  bool ok;
  bool usedPrecisionRange;
  float value;
  const char *unit;
};

struct OscillographResult {
  // min/max are raw 0..1023 ADC bounds of one waveform capture.
  bool ok;
  bool usedInternalReference;
  int minRaw;
  int maxRaw;
};

enum CapRangeStatus {
  // Internal outcome for one charge/discharge timing attempt.
  CAP_RANGE_SUCCESS,
  CAP_RANGE_TIMEOUT,
  CAP_RANGE_DISCHARGE_FAILED
};

enum ResistanceStatus {
  // User-facing state explaining a completed resistance range scan.
  RESISTANCE_SUCCESS,
  RESISTANCE_RANGE_WARNING,
  RESISTANCE_TOO_LOW,
  RESISTANCE_TOO_HIGH,
  RESISTANCE_UNSTABLE,
  RESISTANCE_INCONSISTENT,
  RESISTANCE_MEASUREMENT_FAILED
};

struct ResistanceResult {
  // selectedRangeIndex identifies the D2-D6 reference resistor used for ohms.
  bool ok;
  ResistanceStatus status;
  int selectedRangeIndex;
  float ohms;
};

void printHelp();
void handleSerialCommands();
void printUnknownCommand(char command);
void selectAnalogReference(uint8_t reference);
void setAdcPrescalerBits(uint8_t bits);
void settleAfterReferenceChange();
SampleStats calculateSampleStats(float values[], int count);
void printSampleStats(const __FlashStringHelper *label, const SampleStats &stats);
SampleStats readSettledSamples(uint8_t pin);
SampleStats readSettledResistanceSamples();
VoltageResult measureVoltage();
CapacitanceResult measureCapacitance();
CapacitanceResult measureCapacitanceSample(bool reportStatus);
OscillographResult runOscillograph();
ResistanceResult measureResistance();
ResistanceResult scanResistanceRanges();
ResistanceResult selectResistanceRange(const SampleStats *samples);
bool dischargeCapacitor();
void releaseCapacitanceCircuit();
void releaseResistanceCircuit();
unsigned long resistanceSettlingDelayMs(float referenceOhms);
void dischargeResistanceInput();
bool readStableResistanceSamples(SampleStats &stats);
CapRangeStatus measureCapacitanceRange(float resistorOhms, unsigned long timeoutUs, unsigned long &elapsedUs);
bool calibrateCapacitanceBaselineRange(float resistorOhms, unsigned long timeoutUs, float &baselineNanoFarads);
void calibrateCapacitanceZero();
void printVoltageResult(const VoltageResult &result);
void printCapacitanceResult(const CapacitanceResult &result);
void printOscillographResult(const OscillographResult &result);
void printResistanceResult(const ResistanceResult &result);
float findClosestNominalCapacitance(float measuredNanoFarads);
void printCapacitanceValue(float nanoFarads);
void printResistanceValue(float ohms);
float calculateCapacitanceDifferencePercent(float measuredNanoFarads, float nominalNanoFarads);
float capacitanceNanoFarads(const CapacitanceResult &result);
int decimalPlacesForTenthPercent(float value);

// Arduino runs setup once after reset. Put every controllable circuit into a
// safe, disconnected state before accepting commands from the Serial Monitor.
void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  releaseCapacitanceCircuit();
  releaseResistanceCircuit();

  selectAnalogReference(DEFAULT);

  printHelp();
}

// Arduino repeatedly calls loop. This meter is command-driven, so it only
// checks whether the user sent a complete command character.
void loop() {
  handleSerialCommands();
}

// Prints the short command reference shown at startup and after 'h'.
void printHelp() {
  Serial.println();
  Serial.println(F("\033[0;92;49m=== STARTUP ===\033[0m"));
  Serial.println(F("Arduino multimeter ready"));
  Serial.println(F("Commands:"));
  Serial.println(F("  v - measure voltage"));
  Serial.println(F("  c - measure capacitance"));
  Serial.println(F("  r - measure resistance"));
  Serial.println(F("  z - zero capacitance meter (leave capacitor socket empty)"));
  Serial.println(F("  o - capture oscillograph waveform"));
  Serial.println(F("  h - show this help"));
}

// Reads all waiting Serial characters, ignores whitespace, and dispatches the
// remaining command to the corresponding measurement function.
void handleSerialCommands() {
  while (Serial.available() > 0) {
    const char command = (char)Serial.read();

    if (command == '\r' || command == '\n' || command == ' ' || command == '\t') {
      continue;
    }

    switch (command) {
      case 'v':
      case 'V':
        printVoltageResult(measureVoltage());
        break;
      case 'c':
      case 'C':
        printCapacitanceResult(measureCapacitance());
        break;
      case 'r':
      case 'R':
        printResistanceResult(measureResistance());
        break;
      case 'z':
      case 'Z':
        calibrateCapacitanceZero();
        break;
      case 'o':
      case 'O':
        printOscillographResult(runOscillograph());
        break;
      case 'h':
      case 'H':
        printHelp();
        break;
      default:
        printUnknownCommand(command);
        break;
    }
  }
}

// Explains invalid input instead of silently ignoring a possible typo.
void printUnknownCommand(char command) {
  Serial.print(F("ERROR: unknown command '"));
  Serial.print(command);
  Serial.println(F("'"));
}

// Changes the ADC voltage reference and performs the required settling reads.
// Without settling, the first readings after DEFAULT/INTERNAL selection drift.
void selectAnalogReference(uint8_t reference) {
  analogReference(reference);
  settleAfterReferenceChange();
}

// Changes only the three ADC clock-divider bits, preserving all other ADC
// control settings. The oscillograph uses a faster divider for waveform speed.
void setAdcPrescalerBits(uint8_t bits) {
  ADCSRA = (ADCSRA & 0xF8) | (bits & 0x07);
}

void settleAfterReferenceChange() {
  // The internal reference becomes active on the first conversion, so trigger
  // it before waiting for the voltage to stabilize.
  (void)analogRead(VOLTAGE_ANALOG_PIN);
  delay(ADC_SETTLE_DELAY_MS);

  for (int i = 0; i < ADC_REFERENCE_DUMMY_READS; i++) {
    (void)analogRead(VOLTAGE_ANALOG_PIN);
    delay(ADC_SAMPLE_SPACING_MS);
  }
}

// Computes shared statistics for a non-empty float array. It first calculates
// mean/deviation without storing another buffer, then insertion-sorts the
// caller's temporary array to find the median. Odd counts select one centre
// value; even counts average the two centre values.
SampleStats calculateSampleStats(float values[], int count) {
  SampleStats stats;
  stats.medianRaw = 0.0F;
  stats.meanRaw = values[0];
  stats.stdDevRaw = 0.0F;
  stats.minRaw = values[0];
  stats.maxRaw = values[0];

  float m2 = 0.0F;
  for (int i = 0; i < count; ++i) {
    const float sample = values[i];
    const float delta = sample - stats.meanRaw;
    stats.meanRaw += delta / (float)(i + 1);
    m2 += delta * (sample - stats.meanRaw);

    if (sample < stats.minRaw) {
      stats.minRaw = sample;
    }
    if (sample > stats.maxRaw) {
      stats.maxRaw = sample;
    }
  }

  for (int i = 1; i < count; ++i) {
    const float value = values[i];
    int j = i - 1;
    while (j >= 0 && values[j] > value) {
      values[j + 1] = values[j];
      --j;
    }
    values[j + 1] = value;
  }

  const int upperMiddle = count / 2;
  stats.medianRaw = count % 2 == 0
                        ? (values[upperMiddle - 1] + values[upperMiddle]) / 2.0F
                        : values[upperMiddle];
  stats.stdDevRaw = sqrt(m2 / (float)count);
  return stats;
}

// Prints diagnostic statistics. F() labels reside in flash, preserving scarce
// AVR RAM; these values help diagnose noise without changing the result.
void printSampleStats(const __FlashStringHelper *label, const SampleStats &stats) {
  Serial.print(F("STATUS: "));
  Serial.print(label);
  Serial.print(F(" median="));
  Serial.print(stats.medianRaw, 2);
  Serial.print(F(" mean="));
  Serial.print(stats.meanRaw, 2);
  Serial.print(F(" sd="));
  Serial.println(stats.stdDevRaw, 2);
}

// Takes VOLTAGE_SAMPLES settled ADC readings from one analog pin. A discarded
// first conversion lets the ADC's internal sample capacitor adopt this pin.
SampleStats readSettledSamples(uint8_t pin) {
  float readings[VOLTAGE_SAMPLES];

  (void)analogRead(pin);
  delay(ADC_SAMPLE_SPACING_MS);

  for (int i = 0; i < VOLTAGE_SAMPLES; ++i) {
    readings[i] = (float)analogRead(pin);
    delay(ADC_SAMPLE_SPACING_MS);
  }

  return calculateSampleStats(readings, VOLTAGE_SAMPLES);
}

// Takes paired resistance readings. Subtracting the ground-reference ADC
// reading from the junction ADC reading removes a ground-lead offset before
// the common statistics helper calculates the 32-sample median.
SampleStats readSettledResistanceSamples() {
  float correctedReadings[RESISTANCE_SAMPLES];

  for (int i = 0; i < RESISTANCE_SAMPLES; ++i) {
    (void)analogRead(RESISTANCE_JUNCTION_ANALOG_PIN);
    delay(ADC_SAMPLE_SPACING_MS);
    const int rawA6 = analogRead(RESISTANCE_JUNCTION_ANALOG_PIN);
    delay(ADC_SAMPLE_SPACING_MS);

    (void)analogRead(RESISTANCE_GROUND_REFERENCE_ANALOG_PIN);
    delay(ADC_SAMPLE_SPACING_MS);
    const int rawA7 = analogRead(RESISTANCE_GROUND_REFERENCE_ANALOG_PIN);
    delay(ADC_SAMPLE_SPACING_MS);

    const int correctedJunctionRaw = rawA6 - rawA7;
    correctedReadings[i] = (float)correctedJunctionRaw;
  }

  return calculateSampleStats(correctedReadings, RESISTANCE_SAMPLES);
}

// Measures the divider input voltage. Start at the 5 V ADC reference; if the
// result is small, repeat at 1.1 V for finer ADC resolution.
VoltageResult measureVoltage() {
  VoltageResult result;
  result.ok = false;
  result.usedInternalReference = false;
  result.voltage = 0.0F;
  result.stdDev = 0.0F;

  // Keep the capacitance resistors from loading the shared A0 divider node.
  releaseCapacitanceCircuit();
  selectAnalogReference(DEFAULT);

  Serial.println(F("STATUS: voltage reference = 5V"));
  SampleStats defaultStats = readSettledSamples(VOLTAGE_ANALOG_PIN);
  printSampleStats(F("voltage raw (5V reference)"), defaultStats);
  float measuredVoltage = (defaultStats.medianRaw * DEFAULT_REFERENCE_VOLTAGE / 1023.0F) *
                          DIVIDER_RATIO * VOLTAGE_CALIBRATION;

  if (measuredVoltage <= VOLTAGE_REF_SWITCH_THRESHOLD) {
    Serial.print(F("Less or equal to "));
    Serial.print(VOLTAGE_REF_SWITCH_THRESHOLD);
    Serial.println(F(" V, switching to internal reference"));
    selectAnalogReference(INTERNAL);
    Serial.println(F("STATUS: voltage reference = 1.1V"));

    SampleStats internalStats = readSettledSamples(VOLTAGE_ANALOG_PIN);
    printSampleStats(F("voltage raw (1.1V reference)"), internalStats);
    if (internalStats.maxRaw == 1023) {
      selectAnalogReference(DEFAULT);
      Serial.println(F("ERROR: voltage out of range on 1.1V reference"));
      return result;
    }

    measuredVoltage = (internalStats.medianRaw * INTERNAL_REFERENCE_VOLTAGE / 1023.0F) *
                      DIVIDER_RATIO * VOLTAGE_CALIBRATION;
    result.stdDev = (internalStats.stdDevRaw * INTERNAL_REFERENCE_VOLTAGE / 1023.0F) *
                    DIVIDER_RATIO * VOLTAGE_CALIBRATION;
    result.usedInternalReference = true;
  } else {
    if (defaultStats.maxRaw == 1023) {
      selectAnalogReference(DEFAULT);
      Serial.println(F("ERROR: voltage out of range on 5V reference"));
      return result;
    }

    result.stdDev = (defaultStats.stdDevRaw * DEFAULT_REFERENCE_VOLTAGE / 1023.0F) *
                    DIVIDER_RATIO * VOLTAGE_CALIBRATION;
  }

  selectAnalogReference(DEFAULT);

  result.ok = true;
  result.voltage = measuredVoltage;
  return result;
}

// Drives the capacitor node LOW through the discharge resistor and waits until
// the ADC confirms it is empty. Returns false instead of waiting indefinitely.
bool dischargeCapacitor() {
  pinMode(DISCHARGE_PIN, OUTPUT);
  digitalWrite(DISCHARGE_PIN, LOW);

  const unsigned long startTime = micros();
  while (analogRead(CAPACITANCE_ANALOG_PIN) > CAP_DISCHARGE_LIMIT) {
    if ((micros() - startTime) > CAP_DISCHARGE_TIMEOUT_US) {
      pinMode(DISCHARGE_PIN, INPUT);
      return false;
    }
  }

  pinMode(DISCHARGE_PIN, INPUT);
  return true;
}

// Makes all capacitance-control pins high impedance so they cannot load A0/A1
// while voltage, resistance, or another capacitance step is running.
void releaseCapacitanceCircuit() {
  digitalWrite(CHARGE_PIN_10K, LOW);
  digitalWrite(CHARGE_PIN_1K, LOW);
  pinMode(CHARGE_PIN_10K, INPUT);
  pinMode(CHARGE_PIN_1K, INPUT);
  pinMode(DISCHARGE_PIN, INPUT);
}

// Removes every resistance-range drive. INPUT mode is high impedance, which
// prevents inactive reference resistors from affecting the selected range.
void releaseResistanceCircuit() {
  for (uint8_t pin = RESISTANCE_FIRST_PIN; pin <= RESISTANCE_LAST_PIN; ++pin) {
    digitalWrite(pin, LOW);
    pinMode(pin, INPUT);
  }
}

unsigned long resistanceSettlingDelayMs(float referenceOhms) {
  // Rref alone is the worst-case charging resistance (an open test socket).
  // Calculate from calibrated values so edits cannot leave stale delays.
  return (unsigned long)ceil(referenceOhms * RESISTANCE_SETTLING_CAPACITANCE_F *
                             RESISTANCE_SETTLING_TIME_CONSTANTS * 1000.0F);
}

// Discharges the 100 nF resistance-input capacitor through D4's known
// reference resistor before testing the next range, avoiding carry-over charge.
void dischargeResistanceInput() {
  releaseResistanceCircuit();
  float dischargeOhms = 0.0F;
  for (int i = 0; i < RESISTANCE_RANGE_COUNT; ++i) {
    if (RESISTANCE_RANGES[i].pin == RESISTANCE_DISCHARGE_PIN) {
      dischargeOhms = RESISTANCE_RANGES[i].referenceOhms;
      break;
    }
  }
  if (dischargeOhms <= 0.0F) {
    return;
  }

  pinMode(RESISTANCE_DISCHARGE_PIN, OUTPUT);
  digitalWrite(RESISTANCE_DISCHARGE_PIN, LOW);
  delay(resistanceSettlingDelayMs(dischargeOhms));
  releaseResistanceCircuit();
}

// Repeats full 32-reading windows until both the within-window noise and the
// change between medians are small enough, or until the safety timeout expires.
bool readStableResistanceSamples(SampleStats &stats) {
  // The excitation stays HIGH throughout these windows. Stable readings
  // after the RC delay are required; averaging a charging ramp is invalid.
  const unsigned long startMs = millis();
  SampleStats previous = readSettledResistanceSamples();
  int stableComparisons = 0;
  do {
    stats = readSettledResistanceSamples();
    const bool quiet = stats.stdDevRaw <= RESISTANCE_MAX_STDDEV_RAW &&
                       previous.stdDevRaw <= RESISTANCE_MAX_STDDEV_RAW &&
                       stats.maxRaw - stats.minRaw <= RESISTANCE_MAX_SPREAD_RAW &&
                       previous.maxRaw - previous.minRaw <= RESISTANCE_MAX_SPREAD_RAW;
    if (quiet && fabs(stats.medianRaw - previous.medianRaw) <= RESISTANCE_STABLE_MEDIAN_CHANGE_RAW) {
      ++stableComparisons;
      if (stableComparisons >= RESISTANCE_STABLE_COMPARISONS) {
        return true;
      }
    } else {
      stableComparisons = 0;
    }
    previous = stats;
  } while (millis() - startMs < RESISTANCE_STABILITY_TIMEOUT_MS);
  return false;
}

// Top-level resistance measurement: disconnect other paths, scan all ranges,
// then always restore safe high-impedance GPIO and the normal ADC reference.
ResistanceResult measureResistance() {
  releaseResistanceCircuit();
  selectAnalogReference(DEFAULT);

  ResistanceResult result = scanResistanceRanges();

  releaseResistanceCircuit();
  selectAnalogReference(DEFAULT);
  return result;
}

// Excites each calibrated reference resistor in turn, records stable divider
// statistics, and passes all candidates to the range-selection calculation.
ResistanceResult scanResistanceRanges() {
  ResistanceResult result;
  result.ok = false;
  result.status = RESISTANCE_MEASUREMENT_FAILED;
  result.selectedRangeIndex = -1;
  result.ohms = 0.0F;

  SampleStats samples[RESISTANCE_RANGE_COUNT];

  for (int i = 0; i < RESISTANCE_RANGE_COUNT; ++i) {
    dischargeResistanceInput();
    pinMode(RESISTANCE_RANGES[i].pin, OUTPUT);
    digitalWrite(RESISTANCE_RANGES[i].pin, HIGH);
    delay(resistanceSettlingDelayMs(RESISTANCE_RANGES[i].referenceOhms));

    const bool stable = readStableResistanceSamples(samples[i]);
    releaseResistanceCircuit();
    const SampleStats &stats = samples[i];

    const float medianRaw = stats.medianRaw;
    Serial.print(F("STATUS: resistance D"));
    Serial.print(RESISTANCE_RANGES[i].pin);
    Serial.print(F(" ("));
    printResistanceValue(RESISTANCE_RANGES[i].referenceOhms);
    Serial.print(F(")"));
    Serial.print(F(" median_raw="));
    Serial.print(medianRaw, 1);
    Serial.print(F(" mean_raw="));
    Serial.print(stats.meanRaw, 1);
    Serial.print(F(" min="));
    Serial.print(stats.minRaw);
    Serial.print(F(" max="));
    Serial.print(stats.maxRaw);
    Serial.print(F(" sd="));
    Serial.print(stats.stdDevRaw, 2);
    Serial.print(F(" junction_raw="));
    Serial.print(stats.medianRaw, 1);
    Serial.print(F(" | EST: "));
    if (!stable) {
      Serial.println(F("UNSTABLE"));
      result.status = RESISTANCE_UNSTABLE;
      return result;
    } else if (medianRaw <= RESISTANCE_MIN_VALID_RAW) {
      Serial.println(F("SHORT/BELOW RANGE"));
    } else if (medianRaw < RESISTANCE_OPEN_INPUT_MIN_RAW) {
      const float estimatedOhms = RESISTANCE_RANGES[i].referenceOhms * medianRaw /
                                  (1023.0F - medianRaw);
      printResistanceValue(estimatedOhms);
      Serial.println();
    } else {
      Serial.println(F("OPEN"));
    }

  }

  result = selectResistanceRange(samples);
  releaseResistanceCircuit();
  selectAnalogReference(DEFAULT);
  return result;
}

// Selects the most precise valid range. Rx = Rref * raw / (1023 - raw), so
// ranges near either ADC rail lose resolution. Measured noise also lowers a
// range's quality score and helps detect contradictory range results.
ResistanceResult selectResistanceRange(const SampleStats *samples) {
  ResistanceResult result = {false, RESISTANCE_INCONSISTENT, -1, 0.0F};
  bool allMediansTooLow = true;
  bool allMediansTooHigh = true;
  float bestRelativeError = 1.0e9F;
  int validRangeCount = 0;

  for (int i = 0; i < RESISTANCE_RANGE_COUNT; ++i) {
    const float raw = samples[i].medianRaw;
    allMediansTooLow = allMediansTooLow && raw <= RESISTANCE_MIN_VALID_RAW;
    allMediansTooHigh = allMediansTooHigh && raw >= RESISTANCE_OPEN_INPUT_MIN_RAW;
    if (raw <= RESISTANCE_MIN_VALID_RAW || raw >= RESISTANCE_OPEN_INPUT_MIN_RAW) {
      continue;
    }
    ++validRangeCount;
    // The 100 nF capacitor supplies charge to the ADC sample-and-hold input.
    // Score every settled range by propagated ADC resolution and measured
    // noise, rather than rejecting high-impedance ranges outright.
    const float rawError = RESISTANCE_ADC_ERROR_FLOOR_RAW + 3.0F * samples[i].stdDevRaw;
    const float relativeError = rawError * 1023.0F / (raw * (1023.0F - raw));
    if (result.selectedRangeIndex < 0 || relativeError < bestRelativeError) {
      bestRelativeError = relativeError;
      result.selectedRangeIndex = i;
    }
  }

  if (allMediansTooLow) {
    result.status = RESISTANCE_TOO_LOW;
  } else if (allMediansTooHigh) {
    result.status = RESISTANCE_TOO_HIGH;
  } else if (result.selectedRangeIndex >= 0) {
    const int selected = result.selectedRangeIndex;
    const float raw = samples[selected].medianRaw;
    const float ohms = RESISTANCE_RANGES[selected].referenceOhms * raw / (1023.0F - raw);
    bool rangesDisagree = false;
    // Check even saturated ranges: a real high resistance can be measurable
    // only on D6 while lower references legitimately saturate. Conversely,
    // one spurious mid-scale reading cannot agree with all-open ranges.
    for (int i = 0; i < RESISTANCE_RANGE_COUNT; ++i) {
      const float expectedRaw = 1023.0F * ohms /
                                (RESISTANCE_RANGES[i].referenceOhms + ohms);
      const float slope = expectedRaw * (1023.0F - expectedRaw) / 1023.0F;
      const float raw = samples[i].medianRaw;
      if (raw <= RESISTANCE_MIN_VALID_RAW || raw >= RESISTANCE_OPEN_INPUT_MIN_RAW) {
        continue;
      }

      const float candidateRawError = RESISTANCE_ADC_ERROR_FLOOR_RAW +
                                      3.0F * samples[i].stdDevRaw;
      const float candidateRelativeError = candidateRawError * 1023.0F /
                                           (raw * (1023.0F - raw));
      // Near-rail estimates have much poorer resolution. Keep showing them
      // for diagnosis, but only use similarly precise ranges for warnings.
      if (candidateRelativeError >
          bestRelativeError * RESISTANCE_COMPARISON_QUALITY_FACTOR) {
        continue;
      }

      const float toleranceRaw = RESISTANCE_ADC_ERROR_FLOOR_RAW +
                                 3.0F * samples[i].stdDevRaw +
                                 slope * (RESISTANCE_AGREEMENT_FRACTION + bestRelativeError);
      if (fabs(samples[i].medianRaw - expectedRaw) > toleranceRaw) {
        rangesDisagree = true;
      }
    }
    // Keep rejecting a lone contradictory reading (e.g. faulty D3 with an
    // open socket), but do not suppress a stable multi-range measurement
    // merely because systematic errors differ between reference paths.
    if (rangesDisagree && validRangeCount < 2) {
      return result;
    }
    result.ok = true;
    result.status = rangesDisagree ? RESISTANCE_RANGE_WARNING : RESISTANCE_SUCCESS;
    result.ohms = ohms;
  }

  return result;
}

// Performs one RC timing trial with the specified charge resistor. The ADC
// threshold is one time constant (about 63.2% of supply), so C = time / R.
// elapsedUs is returned by reference only when the trial succeeds.
CapRangeStatus measureCapacitanceRange(float resistorOhms, unsigned long timeoutUs, unsigned long &elapsedUs) {
  releaseCapacitanceCircuit();
  selectAnalogReference(DEFAULT);
  (void)analogRead(CAPACITANCE_ANALOG_PIN);

  if (!dischargeCapacitor()) {
    return CAP_RANGE_DISCHARGE_FAILED;
  }

  if (resistorOhms >= CAP_RESISTOR_10K) {
    pinMode(CHARGE_PIN_10K, OUTPUT);
    digitalWrite(CHARGE_PIN_10K, HIGH);
  } else {
    pinMode(CHARGE_PIN_1K, OUTPUT);
    digitalWrite(CHARGE_PIN_1K, HIGH);
  }

  const unsigned long startTime = micros();
  while (analogRead(CAPACITANCE_ANALOG_PIN) < CAP_CHARGE_THRESHOLD) {
    if ((micros() - startTime) > timeoutUs) {
      releaseCapacitanceCircuit();
      elapsedUs = 0;
      return CAP_RANGE_TIMEOUT;
    }
  }

  elapsedUs = micros() - startTime;
  releaseCapacitanceCircuit();
  return CAP_RANGE_SUCCESS;
}

// Performs one complete capacitance reading. A fast 1k probe chooses between
// the 1k range for large capacitors and the 10k range for small capacitors.
// The stored zero baseline is then subtracted in the matching range.
CapacitanceResult measureCapacitanceSample(bool reportStatus) {
  CapacitanceResult result;
  result.ok = false;
  result.usedPrecisionRange = false;
  result.value = 0.0F;
  result.unit = "nF";

  if (reportStatus) {
    Serial.println(F("STATUS: capacitance range = 1k (fast probe)"));
  }

  unsigned long elapsedUs = 0;
  CapRangeStatus status = measureCapacitanceRange(CAP_RESISTOR_1K, CAP_1K_TIMEOUT_US, elapsedUs);
  if (status != CAP_RANGE_SUCCESS) {
    if (reportStatus) {
      if (status == CAP_RANGE_DISCHARGE_FAILED) {
        Serial.println(F("ERROR: capacitor did not discharge"));
      } else {
        Serial.println(F("ERROR: capacitance timeout on 1k fast range"));
      }
    }
    return result;
  }

  if (elapsedUs < CAP_FAST_PROBE_MIN_US) {
    if (reportStatus) {
      Serial.print(F("STATUS: 1k probe = "));
      Serial.print(elapsedUs);
      Serial.print(F(" us; below "));
      Serial.print(CAP_FAST_PROBE_MIN_US);
      Serial.println(F(" us precision threshold, switching to 10k"));
    }
    result.usedPrecisionRange = true;

    status = measureCapacitanceRange(CAP_RESISTOR_10K, CAP_10K_TIMEOUT_US, elapsedUs);
    if (status != CAP_RANGE_SUCCESS) {
      if (reportStatus) {
        if (status == CAP_RANGE_DISCHARGE_FAILED) {
          Serial.println(F("ERROR: capacitor did not discharge"));
        } else {
          Serial.println(F("ERROR: capacitance timeout on 10k precision range"));
        }
      }
      return result;
    }

    result.value = (float)elapsedUs / CAP_RESISTOR_10K;
  } else {
    result.value = (float)elapsedUs / CAP_RESISTOR_1K;
  }

  const float baselineNanoFarads = capacitanceBaselinesCalibrated
                                       ? (result.usedPrecisionRange
                                              ? capacitanceBaseline10kNanoFarads
                                              : capacitanceBaseline1kNanoFarads)
                                       : 0.0F;
  result.value -= baselineNanoFarads / 1000.0F;
  if (result.value < 0.0F) {
    result.value = 0.0F;
  }

  if (result.value >= CAP_MICROFARAD_DISPLAY_THRESHOLD) {
    result.unit = "uF";
  } else {
    result.value *= 1000.0F;
    result.unit = "nF";
  }

  result.ok = true;
  return result;
}

// Warms up the capacitance circuit once, then collects CAPACITANCE_SAMPLES
// complete readings in nF. The shared helper returns their median, protecting
// the displayed capacitance from one timing outlier.
CapacitanceResult measureCapacitance() {
  CapacitanceResult result = measureCapacitanceSample(false);
  if (!result.ok) {
    Serial.println(F("ERROR: capacitance warm-up measurement failed"));
    return result;
  }

  CapacitanceResult samples[CAPACITANCE_SAMPLES];
  float readingsNanoFarads[CAPACITANCE_SAMPLES];
  for (int i = 0; i < CAPACITANCE_SAMPLES; i++) {
    Serial.print(F("STATUS: capacitance sample "));
    Serial.print(i + 1);
    Serial.print(F(" of "));
    Serial.println(CAPACITANCE_SAMPLES);

    samples[i] = measureCapacitanceSample(true);
    if (!samples[i].ok) {
      return samples[i];
    }
    readingsNanoFarads[i] = capacitanceNanoFarads(samples[i]);
  }

  const SampleStats stats = calculateSampleStats(readingsNanoFarads, CAPACITANCE_SAMPLES);
  printSampleStats(F("capacitance nF"), stats);

  int medianSampleIndex = 0;
  for (int i = 0; i < CAPACITANCE_SAMPLES; ++i) {
    if (capacitanceNanoFarads(samples[i]) == stats.medianRaw) {
      medianSampleIndex = i;
      break;
    }
  }

  result = samples[medianSampleIndex];
  result.value = stats.medianRaw / 1000.0F;
  if (result.value >= CAP_MICROFARAD_DISPLAY_THRESHOLD) {
    result.unit = "uF";
  } else {
    result.value *= 1000.0F;
    result.unit = "nF";
  }

  Serial.print(F("STATUS: reporting median of "));
  Serial.println(CAPACITANCE_SAMPLES);
  return result;
}

// Normalizes either display unit to nF so samples can be compared and sorted.
float capacitanceNanoFarads(const CapacitanceResult &result) {
  return result.unit[0] == 'u' ? result.value * 1000.0F : result.value;
}

// Measures the empty-socket timing offset for one charge range. The median
// baseline rejects a stray timing outlier before future readings subtract it.
bool calibrateCapacitanceBaselineRange(float resistorOhms, unsigned long timeoutUs, float &baselineNanoFarads) {
  unsigned long elapsedUs = 0;
  CapRangeStatus status = measureCapacitanceRange(resistorOhms, timeoutUs, elapsedUs);
  if (status != CAP_RANGE_SUCCESS) {
    return false;
  }

  float readings[CAPACITANCE_SAMPLES];
  for (int i = 0; i < CAPACITANCE_SAMPLES; i++) {
    status = measureCapacitanceRange(resistorOhms, timeoutUs, elapsedUs);
    if (status != CAP_RANGE_SUCCESS) {
      return false;
    }
    readings[i] = ((float)elapsedUs / resistorOhms) * 1000.0F;
  }

  const SampleStats stats = calculateSampleStats(readings, CAPACITANCE_SAMPLES);
  printSampleStats(F("capacitance baseline nF"), stats);
  baselineNanoFarads = stats.medianRaw;
  return true;
}

// Handles command 'z': measure both empty-socket baselines. The user must
// remove the capacitor, otherwise its capacitance would be subtracted later.
void calibrateCapacitanceZero() {
  Serial.println(F("STATUS: capacitance zero calibration; leave socket empty"));

  Serial.println(F("STATUS: calibrating 1k baseline"));
  if (!calibrateCapacitanceBaselineRange(CAP_RESISTOR_1K, CAP_1K_TIMEOUT_US, capacitanceBaseline1kNanoFarads)) {
    Serial.println(F("ERROR: could not calibrate 1k baseline"));
    return;
  }

  Serial.println(F("STATUS: calibrating 10k baseline"));
  if (!calibrateCapacitanceBaselineRange(CAP_RESISTOR_10K, CAP_10K_TIMEOUT_US, capacitanceBaseline10kNanoFarads)) {
    Serial.println(F("ERROR: could not calibrate 10k baseline"));
    return;
  }

  capacitanceBaselinesCalibrated = true;
  Serial.print(F("STATUS: 1k baseline = "));
  Serial.print(capacitanceBaseline1kNanoFarads, 2);
  Serial.println(F(" nF"));
  Serial.print(F("STATUS: 10k baseline = "));
  Serial.print(capacitanceBaseline10kNanoFarads, 2);
  Serial.println(F(" nF"));
  Serial.println(F("STATUS: capacitance zero calibration complete"));
}

// Captures a fast block of raw ADC values and prints a simple waveform stream.
// Its preview intentionally uses the arithmetic mean to preserve prior
// reference-switch behaviour; the voltage meter itself uses the median.
OscillographResult runOscillograph() {
  OscillographResult result;
  result.ok = false;
  result.usedInternalReference = false;
  result.minRaw = 1023;
  result.maxRaw = 0;
  uint16_t samples[OSCILLOGRAPH_SAMPLES];

  releaseCapacitanceCircuit();
  selectAnalogReference(DEFAULT);

  Serial.println(F("STATUS: oscillograph preview = 5V"));
  SampleStats previewStats = readSettledSamples(VOLTAGE_ANALOG_PIN);
  float previewVoltage = (previewStats.meanRaw * DEFAULT_REFERENCE_VOLTAGE / 1023.0F) *
                         DIVIDER_RATIO * VOLTAGE_CALIBRATION;

  if (previewVoltage <= VOLTAGE_REF_SWITCH_THRESHOLD) {
    selectAnalogReference(INTERNAL);
    Serial.println(F("STATUS: oscillograph preview = 1.1V"));
    result.usedInternalReference = true;
  }

  Serial.println(F("OSCILLOGRAPH: start"));
  setAdcPrescalerBits(OSCILLOGRAPH_ADC_PRESCALER_BITS);
  const unsigned long captureStartUs = micros();
  for (int i = 0; i < OSCILLOGRAPH_SAMPLES; i++) {
    ADCSRA |= _BV(ADSC);
    while (ADCSRA & _BV(ADSC)) {
    }

    const int rawValue = ADC;
    samples[i] = (uint16_t)rawValue;
    if (rawValue < result.minRaw) {
      result.minRaw = rawValue;
    }
    if (rawValue > result.maxRaw) {
      result.maxRaw = rawValue;
    }

  }
  const unsigned long captureDurationUs = micros() - captureStartUs;
  setAdcPrescalerBits(ADC_DEFAULT_PRESCALER_BITS);
  selectAnalogReference(DEFAULT);

  for (int i = 0; i < OSCILLOGRAPH_SAMPLES; i++) {
    const int rawValue = samples[i];
    float voltage = (rawValue * (result.usedInternalReference ? INTERNAL_REFERENCE_VOLTAGE : DEFAULT_REFERENCE_VOLTAGE) / 1023.0F) *
                    DIVIDER_RATIO * VOLTAGE_CALIBRATION;

    Serial.print(F("OSC:"));
    Serial.print(i);
    Serial.print(F(",RAW:"));
    Serial.print(rawValue);
    Serial.print(F(",V:"));
    Serial.println(voltage, 2);
  }
  Serial.println(F("OSCILLOGRAPH: end"));
  Serial.print(F("OSCILLOGRAPH: capture_us="));
  Serial.println(captureDurationUs);

  result.ok = true;
  return result;
}

// Formats a successful voltage reading and its diagnostic standard deviation.
void printVoltageResult(const VoltageResult &result) {
  if (!result.ok) {
    return;
  }

  // White distinguishes a displayed 0.00 V from a non-zero successful value.
  const bool displaysZeroVolts = result.voltage < 0.005F;
  Serial.print(displaysZeroVolts ? F("\033[1;97m") : F("\033[0;92m"));
  Serial.print(F("VOLTAGE: "));
  Serial.print(result.voltage, 2);
  Serial.print(F(" V | REF: "));
  Serial.print(result.usedInternalReference ? F("1.1V") : F("5V"));
  Serial.print(F(" | STDDEV: "));
  Serial.print(result.stdDev, 2);
  Serial.println(F(" V\033[0m"));
}

// Formats a capacitance reading, finds the closest common nominal value, and
// uses colours/warnings to explain when that comparison is not trustworthy.
void printCapacitanceResult(const CapacitanceResult &result) {
  if (!result.ok) {
    return;
  }

  const float measuredNanoFarads = result.unit[0] == 'u' ? result.value * 1000.0F : result.value;
  const bool hasMeasuredCapacitance = measuredNanoFarads > 0.0F;
  const float nominalNanoFarads = hasMeasuredCapacitance ? findClosestNominalCapacitance(measuredNanoFarads) : 0.0F;
  const float differencePercent = hasMeasuredCapacitance
                                      ? calculateCapacitanceDifferencePercent(measuredNanoFarads, nominalNanoFarads)
                                      : 0.0F;
  const bool differenceExceedsLimit = hasMeasuredCapacitance &&
                                      fabs(differencePercent) > CAP_NOMINAL_DIFFERENCE_LIMIT_PERCENT;
  const bool nominalNotFound = hasMeasuredCapacitance &&
                               fabs(differencePercent) > CAP_NOMINAL_NOT_FOUND_LIMIT_PERCENT;

  Serial.print(!hasMeasuredCapacitance ? F("\033[1;97m")
                                       : (nominalNotFound ? F("\033[1;91m")
                                                          : (differenceExceedsLimit ? F("\033[38;5;208m")
                                                                                    : F("\033[0;92m"))));
  Serial.print(F("CAPACITANCE: "));
  printCapacitanceValue(measuredNanoFarads);
  if (hasMeasuredCapacitance) {
    Serial.print(F(" | Nearest NOMINAL: "));
    printCapacitanceValue(nominalNanoFarads);
    Serial.print(F(" | DIFF: "));
    if (differencePercent >= 0.0F) {
      Serial.print(F("+"));
    }
    Serial.print(differencePercent, 1);
    Serial.print(F("%"));
  }
  Serial.print(F("\033[0m"));

  if (!hasMeasuredCapacitance) {
    Serial.println(F(" (NO CAPACITOR DETECTED)"));
  } else if (result.unit[0] == 'n' && result.value < CAP_RELIABLE_MIN_NF) {
    Serial.print(F(" | BELOW RELIABLE RANGE (<"));
    Serial.print(CAP_RELIABLE_MIN_NF, 0);
    Serial.println(F(" nF; actual value shown, not reliable)"));
  } else if (nominalNotFound) {
    Serial.print(F(" | WARNING: no matching nominal capacitance within ±"));
    Serial.print(CAP_NOMINAL_NOT_FOUND_LIMIT_PERCENT, 0);
    Serial.println(F("% tolerance"));
  } else if (differenceExceedsLimit) {
    Serial.print(F(" | WARNING: exceeds ±"));
    Serial.print(CAP_NOMINAL_DIFFERENCE_LIMIT_PERCENT, 0);
    Serial.println(F("% nominal tolerance"));
  } else {
    Serial.println();
  }
}

// Formats resistance success, warning, and error states from the range scan.
void printResistanceResult(const ResistanceResult &result) {
  if (!result.ok) {
    if (result.status == RESISTANCE_TOO_HIGH) {
      Serial.println(F("\033[1;97mERROR: resistance is too high or input is open\033[0m"));
      return;
    }

    Serial.print(F("ERROR: "));
    switch (result.status) {
      case RESISTANCE_TOO_LOW:
        Serial.println(F("resistance is too low or input is shorted"));
        break;
      case RESISTANCE_UNSTABLE:
        Serial.println(F("resistance readings did not stabilize"));
        break;
      case RESISTANCE_INCONSISTENT:
        Serial.println(F("resistance ranges disagree; check connections and calibration"));
        break;
      default:
        Serial.println(F("resistance measurement failed"));
        break;
    }
    return;
  }

  if (result.status == RESISTANCE_RANGE_WARNING) {
    Serial.println(F("WARNING: range estimates disagree; selected result may be inaccurate"));
  }
  Serial.print(F("\033[0;92m"));
  Serial.print(F("RESISTANCE: "));
  printResistanceValue(result.ohms);
  Serial.print(F(" | REF: D"));
  Serial.print(RESISTANCE_RANGES[result.selectedRangeIndex].pin);
  Serial.print(F(" ("));
  printResistanceValue(RESISTANCE_RANGES[result.selectedRangeIndex].referenceOhms);
  Serial.println(F(")\033[0m"));
}

// Finds the nearest entry in the common-value table by absolute difference.
float findClosestNominalCapacitance(float measuredNanoFarads) {
  float closestValue = CAP_NOMINAL_VALUES_NF[0];
  float smallestDifference = fabs(measuredNanoFarads - closestValue);

  for (int i = 1; i < CAP_NOMINAL_VALUE_COUNT; i++) {
    const float difference = fabs(measuredNanoFarads - CAP_NOMINAL_VALUES_NF[i]);
    if (difference < smallestDifference) {
      smallestDifference = difference;
      closestValue = CAP_NOMINAL_VALUES_NF[i];
    }
  }

  return closestValue;
}

// Selects nF or uF for a readable capacitance value without changing it.
void printCapacitanceValue(float nanoFarads) {
  if (nanoFarads >= CAP_MICROFARAD_DISPLAY_THRESHOLD * 1000.0F) {
    const float microFarads = nanoFarads / 1000.0F;
    Serial.print(microFarads, decimalPlacesForTenthPercent(microFarads));
    Serial.print(F(" uF"));
  } else {
    Serial.print(nanoFarads, decimalPlacesForTenthPercent(nanoFarads));
    Serial.print(F(" nF"));
  }
}

// Selects ohms, kilohms, or megohms for readable Serial output.
void printResistanceValue(float ohms) {
  if (ohms < 1000.0F) {
    Serial.print(ohms, decimalPlacesForTenthPercent(ohms));
    Serial.print(F(" \xCE\xA9"));
  } else if (ohms < 1000000.0F) {
    const float kiloOhms = ohms / 1000.0F;
    Serial.print(kiloOhms, decimalPlacesForTenthPercent(kiloOhms));
    Serial.print(F(" k\xCE\xA9"));
  } else {
    const float megaOhms = ohms / 1000000.0F;
    Serial.print(megaOhms, decimalPlacesForTenthPercent(megaOhms));
    Serial.print(F(" M\xCE\xA9"));
  }
}

// Expresses measured-minus-nominal error as a percentage of the nominal value.
float calculateCapacitanceDifferencePercent(float measuredNanoFarads, float nominalNanoFarads) {
  return ((measuredNanoFarads - nominalNanoFarads) / nominalNanoFarads) * 100.0F;
}

// Chooses up to two decimal places so the displayed step is roughly 0.1%.
int decimalPlacesForTenthPercent(float value) {
  float displayIncrement = value / 1000.0F;
  int decimalPlaces = 0;

  while (displayIncrement < 1.0F && decimalPlaces < 2) {
    displayIncrement *= 10.0F;
    decimalPlaces++;
  }

  return decimalPlaces;
}

void printOscillographResult(const OscillographResult &result) {
  if (!result.ok) {
    return;
  }

  Serial.print(F("OSCILLOGRAPH: min RAW="));
  Serial.print(result.minRaw);
  Serial.print(F(" max RAW="));
  Serial.print(result.maxRaw);
  Serial.print(F(" | REF: "));
  Serial.println(result.usedInternalReference ? F("1.1V") : F("5V"));
}
