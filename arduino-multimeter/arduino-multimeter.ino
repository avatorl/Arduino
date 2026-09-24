#include <Arduino.h>
#include <math.h>

// Shared wiring from the source sketches.
const uint8_t ANALOG_PIN = A0;
const uint8_t CHARGE_PIN_10K = 13;
const uint8_t CHARGE_PIN_1K = 8;
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
const float VOLTAGE_STDDEV_LIMIT = 0.05F;

// Capacitance meter constants copied from the dual-resistor sketch.
const unsigned long CAP_10K_TIMEOUT_US = 100000UL;
const unsigned long CAP_1K_TIMEOUT_US = 1000000UL;
const unsigned long CAP_DISCHARGE_TIMEOUT_US = 100000UL;
const int CAP_CHARGE_THRESHOLD = 648;
const int CAP_DISCHARGE_LIMIT = 5;
const float CAP_RESISTOR_10K = 10000.0F;
const float CAP_RESISTOR_1K = 1000.0F;

const int VOLTAGE_SAMPLES = 10;
const unsigned long ADC_SETTLE_DELAY_MS = 5UL;
const unsigned long ADC_SAMPLE_SPACING_MS = 1UL;
const int OSCILLOGRAPH_SAMPLES = 128;
const unsigned int OSCILLOGRAPH_SAMPLE_SPACING_US = 2000U;

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
  bool usedFallbackRange;
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
void settleAfterReferenceChange();
SampleStats readSettledSamples(uint8_t pin);
VoltageResult measureVoltage();
CapacitanceResult measureCapacitance();
OscillographResult runOscillograph();
bool dischargeCapacitor();
CapRangeStatus measureCapacitanceRange(float resistorOhms, unsigned long timeoutUs, unsigned long &elapsedUs);
void printVoltageResult(const VoltageResult &result);
void printCapacitanceResult(const CapacitanceResult &result);
void printOscillographResult(const OscillographResult &result);

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  pinMode(CHARGE_PIN_10K, OUTPUT);
  digitalWrite(CHARGE_PIN_10K, LOW);

  pinMode(CHARGE_PIN_1K, OUTPUT);
  digitalWrite(CHARGE_PIN_1K, LOW);

  pinMode(DISCHARGE_PIN, INPUT);

  analogReference(DEFAULT);
  settleAfterReferenceChange();

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

void settleAfterReferenceChange() {
  delay(ADC_SETTLE_DELAY_MS);
  (void)analogRead(ANALOG_PIN);
  delay(ADC_SAMPLE_SPACING_MS);
}

SampleStats readSettledSamples(uint8_t pin) {
  SampleStats stats;
  stats.meanRaw = 0.0F;
  stats.stdDevRaw = 0.0F;
  stats.minRaw = 1023;
  stats.maxRaw = 0;

  int samples[VOLTAGE_SAMPLES];
  float sum = 0.0F;

  (void)analogRead(pin);
  delay(ADC_SAMPLE_SPACING_MS);

  for (int i = 0; i < VOLTAGE_SAMPLES; i++) {
    const int rawValue = analogRead(pin);
    samples[i] = rawValue;
    sum += (float)rawValue;

    if (rawValue < stats.minRaw) {
      stats.minRaw = rawValue;
    }
    if (rawValue > stats.maxRaw) {
      stats.maxRaw = rawValue;
    }

    delay(ADC_SAMPLE_SPACING_MS);
  }

  stats.meanRaw = sum / (float)VOLTAGE_SAMPLES;

  float sumSquares = 0.0F;
  for (int i = 0; i < VOLTAGE_SAMPLES; i++) {
    const float delta = (float)samples[i] - stats.meanRaw;
    sumSquares += delta * delta;
  }

  stats.stdDevRaw = sqrt(sumSquares / (float)VOLTAGE_SAMPLES);
  return stats;
}

VoltageResult measureVoltage() {
  VoltageResult result;
  result.ok = false;
  result.usedInternalReference = false;
  result.voltage = 0.0F;
  result.stdDev = 0.0F;

  analogReference(DEFAULT);
  settleAfterReferenceChange();

  Serial.println(F("STATUS: voltage reference = 5V"));
  SampleStats defaultStats = readSettledSamples(ANALOG_PIN);
  float measuredVoltage = (defaultStats.meanRaw * DEFAULT_REFERENCE_VOLTAGE / 1023.0F) *
                          DIVIDER_RATIO * VOLTAGE_CALIBRATION;

  if (measuredVoltage <= VOLTAGE_REF_SWITCH_THRESHOLD) {
    analogReference(INTERNAL);
    settleAfterReferenceChange();
    Serial.println(F("STATUS: voltage reference = 1.1V"));

    SampleStats internalStats = readSettledSamples(ANALOG_PIN);
    if (internalStats.minRaw <= 5 || internalStats.maxRaw >= 1018) {
      analogReference(DEFAULT);
      settleAfterReferenceChange();
      Serial.println(F("ERROR: voltage out of range on 1.1V reference"));
      return result;
    }

    measuredVoltage = (internalStats.meanRaw * INTERNAL_REFERENCE_VOLTAGE / 1023.0F) *
                      DIVIDER_RATIO * VOLTAGE_CALIBRATION;
    result.stdDev = (internalStats.stdDevRaw * INTERNAL_REFERENCE_VOLTAGE / 1023.0F) *
                    DIVIDER_RATIO * VOLTAGE_CALIBRATION;
    result.usedInternalReference = true;
  } else {
    if (defaultStats.minRaw <= 5 || defaultStats.maxRaw >= 1018) {
      analogReference(DEFAULT);
      settleAfterReferenceChange();
      Serial.println(F("ERROR: voltage out of range on 5V reference"));
      return result;
    }

    result.stdDev = (defaultStats.stdDevRaw * DEFAULT_REFERENCE_VOLTAGE / 1023.0F) *
                    DIVIDER_RATIO * VOLTAGE_CALIBRATION;
  }

  analogReference(DEFAULT);
  settleAfterReferenceChange();

  if (result.stdDev > VOLTAGE_STDDEV_LIMIT) {
    Serial.println(F("ERROR: voltage unstable"));
    return result;
  }

  result.ok = true;
  result.voltage = measuredVoltage;
  return result;
}

bool dischargeCapacitor() {
  pinMode(DISCHARGE_PIN, OUTPUT);
  digitalWrite(DISCHARGE_PIN, LOW);

  const unsigned long startTime = micros();
  while (analogRead(ANALOG_PIN) > CAP_DISCHARGE_LIMIT) {
    if ((micros() - startTime) > CAP_DISCHARGE_TIMEOUT_US) {
      pinMode(DISCHARGE_PIN, INPUT);
      return false;
    }
  }

  pinMode(DISCHARGE_PIN, INPUT);
  return true;
}

CapRangeStatus measureCapacitanceRange(float resistorOhms, unsigned long timeoutUs, unsigned long &elapsedUs) {
  digitalWrite(CHARGE_PIN_10K, LOW);
  digitalWrite(CHARGE_PIN_1K, LOW);
  analogReference(DEFAULT);
  settleAfterReferenceChange();

  if (!dischargeCapacitor()) {
    return CAP_RANGE_DISCHARGE_FAILED;
  }

  if (resistorOhms >= CAP_RESISTOR_10K) {
    digitalWrite(CHARGE_PIN_10K, HIGH);
  } else {
    digitalWrite(CHARGE_PIN_1K, HIGH);
  }

  const unsigned long startTime = micros();
  while (analogRead(ANALOG_PIN) < CAP_CHARGE_THRESHOLD) {
    if ((micros() - startTime) > timeoutUs) {
      digitalWrite(CHARGE_PIN_10K, LOW);
      digitalWrite(CHARGE_PIN_1K, LOW);
      elapsedUs = 0;
      return CAP_RANGE_TIMEOUT;
    }
  }

  elapsedUs = micros() - startTime;
  digitalWrite(CHARGE_PIN_10K, LOW);
  digitalWrite(CHARGE_PIN_1K, LOW);
  return CAP_RANGE_SUCCESS;
}

CapacitanceResult measureCapacitance() {
  CapacitanceResult result;
  result.ok = false;
  result.usedFallbackRange = false;
  result.value = 0.0F;
  result.unit = "nF";

  Serial.println(F("STATUS: capacitance range = 10k"));

  unsigned long elapsedUs = 0;
  CapRangeStatus status = measureCapacitanceRange(CAP_RESISTOR_10K, CAP_10K_TIMEOUT_US, elapsedUs);
  if (status != CAP_RANGE_SUCCESS) {
    if (status == CAP_RANGE_DISCHARGE_FAILED) {
      Serial.println(F("ERROR: capacitor did not discharge"));
      return result;
    }

    Serial.println(F("STATUS: capacitance range = 1k"));
    result.usedFallbackRange = true;

    status = measureCapacitanceRange(CAP_RESISTOR_1K, CAP_1K_TIMEOUT_US, elapsedUs);
    if (status != CAP_RANGE_SUCCESS) {
      if (status == CAP_RANGE_DISCHARGE_FAILED) {
        Serial.println(F("ERROR: capacitor did not discharge"));
      } else {
        Serial.println(F("ERROR: capacitance timeout on 1k range"));
      }
      return result;
    }

    result.value = (float)elapsedUs / CAP_RESISTOR_1K;
    if (result.value > 1.0F) {
      result.unit = "uF";
    } else {
      result.value *= 1000.0F;
      result.unit = "nF";
    }

    result.ok = true;
    return result;
  }

  result.value = (float)elapsedUs / CAP_RESISTOR_10K;
  if (result.value > 1.0F) {
    result.unit = "uF";
  } else {
    result.value *= 1000.0F;
    result.unit = "nF";
  }

  result.ok = true;
  return result;
}

OscillographResult runOscillograph() {
  OscillographResult result;
  result.ok = false;
  result.usedInternalReference = false;
  result.minRaw = 1023;
  result.maxRaw = 0;

  analogReference(DEFAULT);
  settleAfterReferenceChange();

  Serial.println(F("STATUS: oscillograph preview = 5V"));
  SampleStats previewStats = readSettledSamples(ANALOG_PIN);
  float previewVoltage = (previewStats.meanRaw * DEFAULT_REFERENCE_VOLTAGE / 1023.0F) *
                         DIVIDER_RATIO * VOLTAGE_CALIBRATION;

  if (previewVoltage <= VOLTAGE_REF_SWITCH_THRESHOLD) {
    analogReference(INTERNAL);
    settleAfterReferenceChange();
    Serial.println(F("STATUS: oscillograph preview = 1.1V"));
    result.usedInternalReference = true;
  }

  Serial.println(F("OSCILLOGRAPH: start"));
  for (int i = 0; i < OSCILLOGRAPH_SAMPLES; i++) {
    const int rawValue = analogRead(ANALOG_PIN);
    if (rawValue < result.minRaw) {
      result.minRaw = rawValue;
    }
    if (rawValue > result.maxRaw) {
      result.maxRaw = rawValue;
    }

    float voltage = (rawValue * (result.usedInternalReference ? INTERNAL_REFERENCE_VOLTAGE : DEFAULT_REFERENCE_VOLTAGE) / 1023.0F) *
                    DIVIDER_RATIO * VOLTAGE_CALIBRATION;

    Serial.print(F("OSC:"));
    Serial.print(i);
    Serial.print(F(",RAW:"));
    Serial.print(rawValue);
    Serial.print(F(",V:"));
    Serial.println(voltage, 2);

    delayMicroseconds(OSCILLOGRAPH_SAMPLE_SPACING_US);
  }
  Serial.println(F("OSCILLOGRAPH: end"));

  analogReference(DEFAULT);
  settleAfterReferenceChange();

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

  Serial.print(F("CAPACITANCE: "));
  Serial.print(result.value, result.unit[0] == 'u' ? 2 : 1);
  Serial.print(F(" "));
  Serial.print(result.unit);
  Serial.print(F(" | RANGE: "));
  Serial.println(result.usedFallbackRange ? F("1k") : F("10k"));
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
