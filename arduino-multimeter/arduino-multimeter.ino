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

// Shared wiring from the source sketches.
const uint8_t VOLTAGE_ANALOG_PIN = A0;
const uint8_t CAPACITANCE_ANALOG_PIN = A1;
const uint8_t CHARGE_PIN_10K = 13;
const uint8_t CHARGE_PIN_1K = 7;
const uint8_t DISCHARGE_PIN = 11;

const unsigned long SERIAL_BAUD_RATE = 115200UL;

// Voltage divider constants copied from the high-precision voltage meter.
const float DIVIDER_R1 = 100000.0F;
const float DIVIDER_R2 = 10000.0F;
const float DIVIDER_RATIO = (DIVIDER_R1 + DIVIDER_R2) / DIVIDER_R2;
const float VOLTAGE_CALIBRATION = 1.01F;
const float DEFAULT_REFERENCE_VOLTAGE = 5.0F;
const float INTERNAL_REFERENCE_VOLTAGE = 1.1F;
const float VOLTAGE_REF_SWITCH_THRESHOLD = 11.80F;

// Capacitance meter constants copied from the dual-resistor sketch.
const unsigned long CAP_10K_TIMEOUT_US = 100000UL;
const unsigned long CAP_1K_TIMEOUT_US = 1000000UL;
const unsigned long CAP_DISCHARGE_TIMEOUT_US = 1000000UL;
const unsigned long CAP_FAST_PROBE_MIN_US = 1500UL;
const float CAP_RELIABLE_MIN_NF = 20.0F;
const float CAP_MICROFARAD_DISPLAY_THRESHOLD = 0.75F;
const int CAP_CHARGE_THRESHOLD = 648;
const int CAP_DISCHARGE_LIMIT = 5;
const float CAP_RESISTOR_10K = 9900.0F;
const float CAP_RESISTOR_1K = 1018.0F;
const float CAP_NOMINAL_DIFFERENCE_LIMIT_PERCENT = 20.0F;
const float CAP_NOMINAL_VALUES_NF[] = {
    100.0F, 1000.0F, 2200.0F, 3300.0F, 4700.0F, 10000.0F, 22000.0F,
    33000.0F, 47000.0F, 100000.0F, 220000.0F, 330000.0F, 470000.0F};
const int CAP_NOMINAL_VALUE_COUNT = sizeof(CAP_NOMINAL_VALUES_NF) / sizeof(CAP_NOMINAL_VALUES_NF[0]);

float capacitanceBaseline1kNanoFarads = 0.0F;
float capacitanceBaseline10kNanoFarads = 0.0F;
bool capacitanceBaselinesCalibrated = false;

const int VOLTAGE_SAMPLES = 10;
const unsigned long ADC_SETTLE_DELAY_MS = 25UL;
const unsigned long ADC_SAMPLE_SPACING_MS = 1UL;
const int ADC_REFERENCE_DUMMY_READS = 5;
const int OSCILLOGRAPH_SAMPLES = 128;
const uint8_t OSCILLOGRAPH_ADC_PRESCALER_BITS = 0x05; // /32 for faster buffered capture
const uint8_t ADC_DEFAULT_PRESCALER_BITS = 0x07;      // /128 for normal Arduino reads

struct SampleStats {
  float meanRaw;
  float stdDevRaw;
  int minRaw;
  int maxRaw;
};

struct VoltageResult {
  bool ok;
  bool usedInternalReference;
  float voltage;
  float stdDev;
};

struct CapacitanceResult {
  bool ok;
  bool usedPrecisionRange;
  float value;
  const char *unit;
};

struct OscillographResult {
  bool ok;
  bool usedInternalReference;
  int minRaw;
  int maxRaw;
};

enum CapRangeStatus {
  CAP_RANGE_SUCCESS,
  CAP_RANGE_TIMEOUT,
  CAP_RANGE_DISCHARGE_FAILED
};

void printHelp();
void handleSerialCommands();
void printUnknownCommand(char command);
void selectAnalogReference(uint8_t reference);
void setAdcPrescalerBits(uint8_t bits);
void settleAfterReferenceChange();
SampleStats readSettledSamples(uint8_t pin);
VoltageResult measureVoltage();
CapacitanceResult measureCapacitance();
CapacitanceResult measureCapacitanceSample(bool reportStatus);
OscillographResult runOscillograph();
bool dischargeCapacitor();
void releaseCapacitanceCircuit();
CapRangeStatus measureCapacitanceRange(float resistorOhms, unsigned long timeoutUs, unsigned long &elapsedUs);
bool calibrateCapacitanceBaselineRange(float resistorOhms, unsigned long timeoutUs, float &baselineNanoFarads);
void calibrateCapacitanceZero();
void printVoltageResult(const VoltageResult &result);
void printCapacitanceResult(const CapacitanceResult &result);
void printOscillographResult(const OscillographResult &result);
float findClosestNominalCapacitance(float measuredNanoFarads);
void printCapacitanceValue(float nanoFarads);
float calculateCapacitanceDifferencePercent(float measuredNanoFarads, float nominalNanoFarads);
float capacitanceNanoFarads(const CapacitanceResult &result);
int decimalPlacesForTenthPercent(float value);

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  releaseCapacitanceCircuit();

  selectAnalogReference(DEFAULT);

  printHelp();
}

void loop() {
  handleSerialCommands();
}

void printHelp() {
  Serial.println(F("Arduino multimeter ready"));
  Serial.println(F("Commands:"));
  Serial.println(F("  v - measure voltage"));
  Serial.println(F("  c - measure capacitance"));
  Serial.println(F("  z - zero capacitance meter (leave capacitor socket empty)"));
  Serial.println(F("  o - capture oscillograph waveform"));
  Serial.println(F("  h - show this help"));
}

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

void printUnknownCommand(char command) {
  Serial.print(F("ERROR: unknown command '"));
  Serial.print(command);
  Serial.println(F("'"));
}

void selectAnalogReference(uint8_t reference) {
  analogReference(reference);
  settleAfterReferenceChange();
}

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

SampleStats readSettledSamples(uint8_t pin) {
  SampleStats stats;
  stats.meanRaw = 0.0F;
  stats.stdDevRaw = 0.0F;
  stats.minRaw = 1023;
  stats.maxRaw = 0;

  float m2 = 0.0F;

  (void)analogRead(pin);
  delay(ADC_SAMPLE_SPACING_MS);

  for (int i = 0; i < VOLTAGE_SAMPLES; i++) {
    const int rawValue = analogRead(pin);
    const float sample = (float)rawValue;
    const float delta = sample - stats.meanRaw;
    stats.meanRaw += delta / (float)(i + 1);
    m2 += delta * (sample - stats.meanRaw);

    if (rawValue < stats.minRaw) {
      stats.minRaw = rawValue;
    }
    if (rawValue > stats.maxRaw) {
      stats.maxRaw = rawValue;
    }

    delay(ADC_SAMPLE_SPACING_MS);
  }

  stats.stdDevRaw = sqrt(m2 / (float)VOLTAGE_SAMPLES);
  return stats;
}

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
  float measuredVoltage = (defaultStats.meanRaw * DEFAULT_REFERENCE_VOLTAGE / 1023.0F) *
                          DIVIDER_RATIO * VOLTAGE_CALIBRATION;

  if (measuredVoltage <= VOLTAGE_REF_SWITCH_THRESHOLD) {
    Serial.print(F("Less or equal to "));
    Serial.print(VOLTAGE_REF_SWITCH_THRESHOLD);
    Serial.println(F(" V, switching to internal reference"));
    selectAnalogReference(INTERNAL);
    Serial.println(F("STATUS: voltage reference = 1.1V"));

    SampleStats internalStats = readSettledSamples(VOLTAGE_ANALOG_PIN);
    if (internalStats.maxRaw == 1023) {
      selectAnalogReference(DEFAULT);
      Serial.println(F("ERROR: voltage out of range on 1.1V reference"));
      return result;
    }

    measuredVoltage = (internalStats.meanRaw * INTERNAL_REFERENCE_VOLTAGE / 1023.0F) *
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

void releaseCapacitanceCircuit() {
  digitalWrite(CHARGE_PIN_10K, LOW);
  digitalWrite(CHARGE_PIN_1K, LOW);
  pinMode(CHARGE_PIN_10K, INPUT);
  pinMode(CHARGE_PIN_1K, INPUT);
  pinMode(DISCHARGE_PIN, INPUT);
}

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

CapacitanceResult measureCapacitance() {
  CapacitanceResult result = measureCapacitanceSample(false);
  if (!result.ok) {
    Serial.println(F("ERROR: capacitance warm-up measurement failed"));
    return result;
  }

  CapacitanceResult samples[3];
  for (int i = 0; i < 3; i++) {
    Serial.print(F("STATUS: capacitance sample "));
    Serial.print(i + 1);
    Serial.println(F(" of 3"));

    samples[i] = measureCapacitanceSample(true);
    if (!samples[i].ok) {
      return samples[i];
    }
  }

  for (int i = 0; i < 2; i++) {
    for (int j = i + 1; j < 3; j++) {
      if (capacitanceNanoFarads(samples[j]) < capacitanceNanoFarads(samples[i])) {
        const CapacitanceResult temporary = samples[i];
        samples[i] = samples[j];
        samples[j] = temporary;
      }
    }
  }

  Serial.println(F("STATUS: reporting median of 3 capacitance samples"));
  return samples[1];
}

float capacitanceNanoFarads(const CapacitanceResult &result) {
  return result.unit[0] == 'u' ? result.value * 1000.0F : result.value;
}

bool calibrateCapacitanceBaselineRange(float resistorOhms, unsigned long timeoutUs, float &baselineNanoFarads) {
  unsigned long elapsedUs = 0;
  CapRangeStatus status = measureCapacitanceRange(resistorOhms, timeoutUs, elapsedUs);
  if (status != CAP_RANGE_SUCCESS) {
    return false;
  }

  float readings[3];
  for (int i = 0; i < 3; i++) {
    status = measureCapacitanceRange(resistorOhms, timeoutUs, elapsedUs);
    if (status != CAP_RANGE_SUCCESS) {
      return false;
    }
    readings[i] = ((float)elapsedUs / resistorOhms) * 1000.0F;
  }

  for (int i = 0; i < 2; i++) {
    for (int j = i + 1; j < 3; j++) {
      if (readings[j] < readings[i]) {
        const float temporary = readings[i];
        readings[i] = readings[j];
        readings[j] = temporary;
      }
    }
  }

  baselineNanoFarads = readings[1];
  return true;
}

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

void printVoltageResult(const VoltageResult &result) {
  if (!result.ok) {
    return;
  }

  Serial.print(F("VOLTAGE: "));
  Serial.print(result.voltage, 2);
  Serial.print(F(" V | REF: "));
  Serial.print(result.usedInternalReference ? F("1.1V") : F("5V"));
  Serial.print(F(" | STDDEV: "));
  Serial.print(result.stdDev, 2);
  Serial.println(F(" V"));
}

void printCapacitanceResult(const CapacitanceResult &result) {
  if (!result.ok) {
    return;
  }

  const float measuredNanoFarads = result.unit[0] == 'u' ? result.value * 1000.0F : result.value;
  const float nominalNanoFarads = findClosestNominalCapacitance(measuredNanoFarads);
  const float differencePercent = calculateCapacitanceDifferencePercent(measuredNanoFarads, nominalNanoFarads);
  const bool differenceExceedsLimit = fabs(differencePercent) > CAP_NOMINAL_DIFFERENCE_LIMIT_PERCENT;

  Serial.print(differenceExceedsLimit ? F("\033[38;5;208m") : F("\033[0;92m"));
  Serial.print(F("CAPACITANCE: "));
  printCapacitanceValue(measuredNanoFarads);
  Serial.print(F(" | Nearest NOMINAL: "));
  printCapacitanceValue(nominalNanoFarads);
  Serial.print(F(" | DIFF: "));
  if (differencePercent >= 0.0F) {
    Serial.print(F("+"));
  }
  Serial.print(differencePercent, 1);
  Serial.print(F("%"));
  Serial.print(F("\033[0m"));

  if (result.unit[0] == 'n' && result.value < CAP_RELIABLE_MIN_NF) {
    Serial.print(F(" | BELOW RELIABLE RANGE (<"));
    Serial.print(CAP_RELIABLE_MIN_NF, 0);
    Serial.println(F(" nF; actual value shown, not reliable)"));
  } else if (differenceExceedsLimit) {
    Serial.print(F(" | WARNING: exceeds "));
    Serial.print(CAP_NOMINAL_DIFFERENCE_LIMIT_PERCENT, 0);
    Serial.println(F("% nominal tolerance"));
  } else {
    Serial.println();
  }
}

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

float calculateCapacitanceDifferencePercent(float measuredNanoFarads, float nominalNanoFarads) {
  return ((measuredNanoFarads - nominalNanoFarads) / nominalNanoFarads) * 100.0F;
}

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
