#include <Wire.h>
#include <Adafruit_TCS34725.h>

struct PrototypeRgb {
  uint16_t r;
  uint16_t g;
  uint16_t b;
};

struct CalibrationSampleDefinition {
  const char* name;
  PrototypeRgb prototype;
};

const int pinColorSensorLED = 2;
const int pinRgbLedRed = 3;
const int pinRgbLedGreen = 5;
const int pinRgbLedBlue = 6;
const uint8_t colorSensorLEDOnLevel = HIGH;
const uint8_t colorSensorLEDOffLevel = LOW;

Adafruit_TCS34725 colorSensor(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
bool colorSensorDetected = false;
bool calibrationModeActive = false;
bool liveRecognitionActive = false;
bool serialLineReady = false;
char serialLine[96];
uint8_t serialLineLength = 0;
bool timedCaptureModeActive = false;
char timedCaptureLabel[49] = { 0 };
unsigned long timedCaptureDurationSeconds = 0;
unsigned long timedCaptureStartedAtMs = 0;
unsigned long timedCaptureEndsAtMs = 0;
unsigned long timedCaptureLastSampleAtMs = 0;
unsigned long timedCaptureSampleCountSeen = 0;
bool timedCaptureCancelled = false;

const CalibrationSampleDefinition calibrationSamples[] = {
  { "Jade White", { 331, 337, 332 } },
  { "Bambu Green", { 211, 482, 307 } },
  { "Cyan", { 187, 309, 504 } },
  { "Pumpkin Orange", { 529, 263, 208 } },
  { "Maroon Red", { 535, 189, 277 } }
};
const uint8_t calibrationSampleCount = sizeof(calibrationSamples) / sizeof(calibrationSamples[0]);
const uint8_t neutralSampleIndex = 0;
const uint8_t timedCaptureLabelMaxLength = sizeof(timedCaptureLabel) - 1;
const unsigned long timedCaptureSampleIntervalMs = 100UL;
const unsigned long timedCaptureMinDurationSeconds = 1UL;
const unsigned long timedCaptureMaxDurationSeconds = 300UL;
uint16_t calibrationAverageR[calibrationSampleCount] = { 0 };
uint16_t calibrationAverageG[calibrationSampleCount] = { 0 };
uint16_t calibrationAverageB[calibrationSampleCount] = { 0 };
uint16_t calibrationAverageC[calibrationSampleCount] = { 0 };
uint16_t calibrationMinClear[calibrationSampleCount] = { 0 };
uint8_t calibrationSampleIndex = 0;
unsigned long calibrationWaitUntilMs = 0;
unsigned long calibrationScanUntilMs = 0;
unsigned long calibrationSampleCountSeen = 0;
unsigned long calibrationSumR = 0;
unsigned long calibrationSumG = 0;
unsigned long calibrationSumB = 0;
unsigned long calibrationSumC = 0;
uint16_t calibrationMinR = 0;
uint16_t calibrationMinG = 0;
uint16_t calibrationMinB = 0;
uint16_t calibrationMinC = 0;
uint16_t calibrationMaxR = 0;
uint16_t calibrationMaxG = 0;
uint16_t calibrationMaxB = 0;
uint16_t calibrationMaxC = 0;

const uint16_t colorClearMinThreshold = 120;
const uint16_t colorClearWhiteThreshold = 2000;
const uint16_t colorWhiteChannelThreshold = 900;
const float whiteBalanceRedGain = 1.00f;
const float whiteBalanceGreenGain = 1.407f;
const float whiteBalanceBlueGain = 2.493f;
const uint16_t whiteBalancedClearThreshold = 1600;
const uint8_t whiteBalancedMaxSpreadPct = 12;
const uint16_t prototypeDistanceThreshold = 100;
const uint16_t colorMatchClearThreshold = 300;
const uint8_t detectionAttemptBatchSize = 3;
const uint8_t detectionAttemptMinAgreement = 2;
const bool printSkippedDetectionBatches = true;
int8_t detectionAttemptMatches[detectionAttemptBatchSize] = { -1, -1, -1 };
uint8_t detectionAttemptCount = 0;

struct BalancedRgbs {
  uint16_t r;
  uint16_t g;
  uint16_t b;
  uint16_t c;
};

void setRgbLedColor(uint8_t r, uint8_t g, uint8_t b) {
  analogWrite(pinRgbLedRed, 255 - r);
  analogWrite(pinRgbLedGreen, 255 - g);
  analogWrite(pinRgbLedBlue, 255 - b);
}

void turnOffRgbLed() {
  setRgbLedColor(0, 0, 0);
}

void printSampleName(int8_t sampleIndex) {
  if (sampleIndex >= 0 && sampleIndex < calibrationSampleCount) {
    Serial.print(calibrationSamples[sampleIndex].name);
    return;
  }

  Serial.print(F("unknown"));
}

void printSampleList() {
  for (uint8_t i = 0; i < calibrationSampleCount; ++i) {
    if (i > 0) Serial.print(F(", "));
    Serial.print(calibrationSamples[i].name);
  }
}

void printDetectionAttemptBatch() {
  for (uint8_t i = 0; i < detectionAttemptCount; ++i) {
    if (i > 0) Serial.print(F(", "));
    printSampleName(detectionAttemptMatches[i]);
  }
}

void printCalibrationSummary() {
  Serial.println(F("Suggested calibration from current averages:"));
  Serial.println(F("  White balance gains: R=1.000, G=1.407, B=2.493"));
  Serial.print(F("  Match targets: "));
  printSampleList();
  Serial.println();
  Serial.println(F("  Measured prototype centers are stored in the same normalized balanced space used live"));
  Serial.println(F("  White detect: balanced clear >= 1600 and spread <= 12%"));
  Serial.println(F("  Color detect: nearest normalized prototype within threshold"));
  Serial.println(F("  Minimum clear for color matching: 300"));
}

char toLowerAscii(char value) {
  if (value >= 'A' && value <= 'Z') return (char)(value - 'A' + 'a');
  return value;
}

bool isWhitespaceChar(char value) {
  return value == ' ' || value == '\t';
}

void trimWhitespace(char* text) {
  if (text == NULL) return;

  uint8_t start = 0;
  while (text[start] != '\0' && isWhitespaceChar(text[start])) {
    start++;
  }

  uint8_t end = start;
  while (text[end] != '\0') {
    end++;
  }

  while (end > start && isWhitespaceChar(text[end - 1])) {
    end--;
  }

  uint8_t target = 0;
  while (start < end) {
    text[target++] = text[start++];
  }
  text[target] = '\0';
}

bool equalsIgnoreCase(const char* a, const char* b) {
  if (a == NULL || b == NULL) return false;

  while (*a != '\0' && *b != '\0') {
    if (toLowerAscii(*a) != toLowerAscii(*b)) return false;
    a++;
    b++;
  }

  return *a == '\0' && *b == '\0';
}

bool parseUnsignedLongStrict(const char* text, unsigned long* value) {
  if (text == NULL || value == NULL || *text == '\0') return false;

  unsigned long parsed = 0;
  while (*text != '\0') {
    if (*text < '0' || *text > '9') return false;

    unsigned long digit = (unsigned long)(*text - '0');
    unsigned long next = parsed * 10UL + digit;
    if (next < parsed) return false;

    parsed = next;
    text++;
  }

  *value = parsed;
  return true;
}

bool parseTimedCaptureCommand(const char* command, char* labelOut, uint8_t labelOutSize, unsigned long* durationSecondsOut) {
  if (command == NULL || labelOut == NULL || durationSecondsOut == NULL || labelOutSize < 2) return false;

  const char* cursor = command;
  while (*cursor != '\0' && isWhitespaceChar(*cursor)) {
    cursor++;
  }

  if (toLowerAscii(cursor[0]) != 'c' || toLowerAscii(cursor[1]) != 'a' || toLowerAscii(cursor[2]) != 'l') {
    return false;
  }

  cursor += 3;
  if (!isWhitespaceChar(*cursor)) return false;
  while (*cursor != '\0' && isWhitespaceChar(*cursor)) {
    cursor++;
  }

  if (*cursor != '"') return false;
  cursor++;

  uint8_t labelLength = 0;
  while (*cursor != '\0' && *cursor != '"') {
    if (labelLength >= labelOutSize - 1) return false;
    labelOut[labelLength++] = *cursor++;
  }

  if (*cursor != '"' || labelLength == 0) return false;
  labelOut[labelLength] = '\0';
  cursor++;

  while (*cursor != '\0' && isWhitespaceChar(*cursor)) {
    cursor++;
  }

  unsigned long durationSeconds = 0;
  if (!parseUnsignedLongStrict(cursor, &durationSeconds)) return false;
  if (durationSeconds < timedCaptureMinDurationSeconds || durationSeconds > timedCaptureMaxDurationSeconds) return false;

  *durationSecondsOut = durationSeconds;
  return true;
}

void printCsvQuotedLabel(const char* label) {
  Serial.print('"');
  while (*label != '\0') {
    if (*label == '"') Serial.print('"');
    Serial.print(*label);
    label++;
  }
  Serial.print('"');
}

void resetCalibrationStats() {
  calibrationSampleCountSeen = 0;
  calibrationSumR = 0;
  calibrationSumG = 0;
  calibrationSumB = 0;
  calibrationSumC = 0;
  calibrationMinR = 0xFFFF;
  calibrationMinG = 0xFFFF;
  calibrationMinB = 0xFFFF;
  calibrationMinC = 0xFFFF;
  calibrationMaxR = 0;
  calibrationMaxG = 0;
  calibrationMaxB = 0;
  calibrationMaxC = 0;
}

void resetCalibrationAverages() {
  for (uint8_t i = 0; i < calibrationSampleCount; ++i) {
    calibrationAverageR[i] = 0;
    calibrationAverageG[i] = 0;
    calibrationAverageB[i] = 0;
    calibrationAverageC[i] = 0;
    calibrationMinClear[i] = 0;
  }
}

BalancedRgbs applyWhiteBalanceGains(uint16_t r, uint16_t g, uint16_t b, uint16_t c, float redGain, float greenGain, float blueGain) {
  BalancedRgbs balanced;
  balanced.r = (uint16_t)(r * redGain + 0.5f);
  balanced.g = (uint16_t)(g * greenGain + 0.5f);
  balanced.b = (uint16_t)(b * blueGain + 0.5f);
  balanced.c = c;
  return balanced;
}

PrototypeRgb loadPrototype(uint8_t sampleIndex) {
  PrototypeRgb prototype = { 0, 0, 0 };
  if (sampleIndex >= calibrationSampleCount) return prototype;

  return calibrationSamples[sampleIndex].prototype;
}

PrototypeRgb normalizePrototype(uint16_t r, uint16_t g, uint16_t b) {
  PrototypeRgb prototype = { 0, 0, 0 };
  uint32_t sum = (uint32_t)r + (uint32_t)g + (uint32_t)b;
  if (sum == 0) return prototype;

  prototype.r = (uint16_t)(((uint32_t)r * 1000UL + (sum / 2)) / sum);
  prototype.g = (uint16_t)(((uint32_t)g * 1000UL + (sum / 2)) / sum);
  prototype.b = (uint16_t)(((uint32_t)b * 1000UL + (sum / 2)) / sum);
  return prototype;
}

uint16_t prototypeDistance(const PrototypeRgb& a, const PrototypeRgb& b) {
  uint16_t distance = 0;
  distance += (a.r > b.r) ? (a.r - b.r) : (b.r - a.r);
  distance += (a.g > b.g) ? (a.g - b.g) : (b.g - a.g);
  distance += (a.b > b.b) ? (a.b - b.b) : (b.b - a.b);
  return distance;
}

void printSampleDefinitionsBlock(const PrototypeRgb prototypes[], uint8_t sampleCount) {
  Serial.println(F("const CalibrationSampleDefinition calibrationSamples[] = {"));
  for (uint8_t i = 0; i < sampleCount; ++i) {
    Serial.print(F("  { \""));
    Serial.print(calibrationSamples[i].name);
    Serial.print(F("\", { "));
    Serial.print(prototypes[i].r);
    Serial.print(F(", "));
    Serial.print(prototypes[i].g);
    Serial.print(F(", "));
    Serial.print(prototypes[i].b);
    Serial.println(F(" } },"));
  }
  Serial.println(F("};"));
}

void printSuggestedCalibrationCode() {
  if (calibrationAverageR[neutralSampleIndex] == 0 || calibrationAverageG[neutralSampleIndex] == 0 || calibrationAverageB[neutralSampleIndex] == 0) {
    Serial.println(F("Calibration block unavailable: neutral sample was not captured."));
    return;
  }

  float suggestedRedGain = 1.0f;
  float suggestedGreenGain = (float)calibrationAverageR[neutralSampleIndex] / (float)calibrationAverageG[neutralSampleIndex];
  float suggestedBlueGain = (float)calibrationAverageR[neutralSampleIndex] / (float)calibrationAverageB[neutralSampleIndex];

  PrototypeRgb prototypes[calibrationSampleCount];
  for (uint8_t i = 0; i < calibrationSampleCount; ++i) {
    BalancedRgbs balanced = applyWhiteBalanceGains(
      calibrationAverageR[i],
      calibrationAverageG[i],
      calibrationAverageB[i],
      calibrationAverageC[i],
      suggestedRedGain,
      suggestedGreenGain,
      suggestedBlueGain
    );
    prototypes[i] = normalizePrototype(balanced.r, balanced.g, balanced.b);
  }

  uint16_t nearestPrototypeDistance = 0xFFFF;
  for (uint8_t i = 0; i < calibrationSampleCount; ++i) {
    for (uint8_t j = i + 1; j < calibrationSampleCount; ++j) {
      uint16_t distance = prototypeDistance(prototypes[i], prototypes[j]);
      if (distance < nearestPrototypeDistance) {
        nearestPrototypeDistance = distance;
      }
    }
  }

  uint16_t dimmestColorClear = 0xFFFF;
  for (uint8_t i = 0; i < calibrationSampleCount; ++i) {
    if (i == neutralSampleIndex) continue;
    if (calibrationMinClear[i] > 0 && calibrationMinClear[i] < dimmestColorClear) {
      dimmestColorClear = calibrationMinClear[i];
    }
  }

  uint16_t suggestedColorMatchClearThreshold = 300;
  if (dimmestColorClear != 0xFFFF) {
    uint16_t derivedThreshold = (uint16_t)((uint32_t)dimmestColorClear * 85UL / 100UL);
    if (derivedThreshold > colorClearMinThreshold) {
      suggestedColorMatchClearThreshold = derivedThreshold;
    }
  }

  uint16_t suggestedWhiteBalancedClearThreshold = (uint16_t)((uint32_t)calibrationAverageC[neutralSampleIndex] * 25UL / 100UL);
  if (suggestedWhiteBalancedClearThreshold < 1200) {
    suggestedWhiteBalancedClearThreshold = 1200;
  }

  uint16_t suggestedPrototypeDistanceThreshold = prototypeDistanceThreshold;
  if (nearestPrototypeDistance != 0xFFFF) {
    suggestedPrototypeDistanceThreshold = (uint16_t)((uint32_t)nearestPrototypeDistance * 60UL / 100UL);
    if (suggestedPrototypeDistanceThreshold < 60) suggestedPrototypeDistanceThreshold = 60;
    if (suggestedPrototypeDistanceThreshold > 120) suggestedPrototypeDistanceThreshold = 120;
  }

  Serial.println();
  Serial.println(F("Suggested calibration const block:"));
  Serial.println(F("// Paste over the calibration constants near the top of the sketch"));
  Serial.print(F("const uint8_t neutralSampleIndex = "));
  Serial.print(neutralSampleIndex);
  Serial.println(F(";"));
  Serial.print(F("const uint16_t colorClearMinThreshold = "));
  Serial.print(colorClearMinThreshold);
  Serial.println(F(";"));
  Serial.print(F("const uint16_t colorClearWhiteThreshold = "));
  Serial.print(colorClearWhiteThreshold);
  Serial.println(F(";"));
  Serial.print(F("const uint16_t colorWhiteChannelThreshold = "));
  Serial.print(colorWhiteChannelThreshold);
  Serial.println(F(";"));
  Serial.print(F("const float whiteBalanceRedGain = "));
  Serial.print(suggestedRedGain, 3);
  Serial.println(F("f;"));
  Serial.print(F("const float whiteBalanceGreenGain = "));
  Serial.print(suggestedGreenGain, 3);
  Serial.println(F("f;"));
  Serial.print(F("const float whiteBalanceBlueGain = "));
  Serial.print(suggestedBlueGain, 3);
  Serial.println(F("f;"));
  Serial.print(F("const uint16_t whiteBalancedClearThreshold = "));
  Serial.print(suggestedWhiteBalancedClearThreshold);
  Serial.println(F(";"));
  Serial.print(F("const uint8_t whiteBalancedMaxSpreadPct = "));
  Serial.print(whiteBalancedMaxSpreadPct);
  Serial.println(F(";"));
  Serial.print(F("const uint16_t prototypeDistanceThreshold = "));
  Serial.print(suggestedPrototypeDistanceThreshold);
  Serial.println(F(";"));
  Serial.print(F("const uint16_t colorMatchClearThreshold = "));
  Serial.print(suggestedColorMatchClearThreshold);
  Serial.println(F(";"));
  Serial.println();
  printSampleDefinitionsBlock(prototypes, calibrationSampleCount);
}

void printCalibrationPrompt() {
  Serial.print(F("Calibration sample: show "));
  Serial.print(calibrationSamples[calibrationSampleIndex].name);
  Serial.println(F(" sample"));
  Serial.println(F("Place the sample in front of the sensor within 5 seconds."));
  calibrationWaitUntilMs = millis() + 5000UL;
  calibrationScanUntilMs = calibrationWaitUntilMs + 10000UL;
  resetCalibrationStats();
}

void blinkSensorLed(uint8_t blinkCount) {
  for (uint8_t i = 0; i < blinkCount; ++i) {
    digitalWrite(pinColorSensorLED, colorSensorLEDOnLevel);
    delay(200);
    digitalWrite(pinColorSensorLED, colorSensorLEDOffLevel);
    delay(200);
  }
}

void startCalibration() {
  calibrationModeActive = true;
  calibrationSampleIndex = 0;
  resetCalibrationAverages();
  Serial.println(F("Calibration mode started"));
  Serial.println(F("Send 'stop' to exit calibration mode."));
  printCalibrationPrompt();
}

void finishCalibrationSample() {
  if (calibrationSampleCountSeen == 0) {
    Serial.println(F("No readings captured for this sample."));
  } else {
    calibrationAverageR[calibrationSampleIndex] = (uint16_t)(calibrationSumR / calibrationSampleCountSeen);
    calibrationAverageG[calibrationSampleIndex] = (uint16_t)(calibrationSumG / calibrationSampleCountSeen);
    calibrationAverageB[calibrationSampleIndex] = (uint16_t)(calibrationSumB / calibrationSampleCountSeen);
    calibrationAverageC[calibrationSampleIndex] = (uint16_t)(calibrationSumC / calibrationSampleCountSeen);
    calibrationMinClear[calibrationSampleIndex] = calibrationMinC;

    Serial.print(F("Calibration result for "));
    Serial.print(calibrationSamples[calibrationSampleIndex].name);
    Serial.print(F(": count="));
    Serial.print(calibrationSampleCountSeen);
    Serial.print(F(", avg RGBC="));
    Serial.print(calibrationAverageR[calibrationSampleIndex]);
    Serial.print(F(", "));
    Serial.print(calibrationAverageG[calibrationSampleIndex]);
    Serial.print(F(", "));
    Serial.print(calibrationAverageB[calibrationSampleIndex]);
    Serial.print(F(", "));
    Serial.print(calibrationAverageC[calibrationSampleIndex]);
    Serial.print(F(" | min RGBC="));
    Serial.print(calibrationMinR);
    Serial.print(F(", "));
    Serial.print(calibrationMinG);
    Serial.print(F(", "));
    Serial.print(calibrationMinB);
    Serial.print(F(", "));
    Serial.print(calibrationMinC);
    Serial.print(F(" | max RGBC="));
    Serial.print(calibrationMaxR);
    Serial.print(F(", "));
    Serial.print(calibrationMaxG);
    Serial.print(F(", "));
    Serial.print(calibrationMaxB);
    Serial.print(F(", "));
    Serial.println(calibrationMaxC);
  }

  calibrationSampleIndex++;
  if (calibrationSampleIndex >= calibrationSampleCount) {
    calibrationModeActive = false;
    Serial.println(F("Calibration mode complete"));
    printSuggestedCalibrationCode();
    return;
  }

  printCalibrationPrompt();
}

void printTimedCaptureSummary(unsigned long elapsedMs) {
  Serial.print(F("Timed capture "));
  Serial.print(timedCaptureCancelled ? F("cancelled") : F("complete"));
  Serial.print(F(" for "));
  Serial.print(timedCaptureLabel);
  Serial.print(F(": duration_s="));
  Serial.print(timedCaptureDurationSeconds);
  Serial.print(F(", elapsed_ms="));
  Serial.print(elapsedMs);
  Serial.print(F(", sample_count="));
  Serial.println(timedCaptureSampleCountSeen);
}

void stopTimedCapture(bool cancelled) {
  unsigned long elapsedMs = millis() - timedCaptureStartedAtMs;
  timedCaptureCancelled = cancelled;

  Serial.print(F("CAPTURE_END,"));
  printCsvQuotedLabel(timedCaptureLabel);
  Serial.print(F(","));
  Serial.print(timedCaptureSampleCountSeen);
  Serial.print(F(","));
  Serial.println(elapsedMs);

  printTimedCaptureSummary(elapsedMs);

  timedCaptureModeActive = false;
  timedCaptureDurationSeconds = 0;
  timedCaptureStartedAtMs = 0;
  timedCaptureEndsAtMs = 0;
  timedCaptureLastSampleAtMs = 0;
  timedCaptureSampleCountSeen = 0;
  timedCaptureLabel[0] = '\0';
}

void startTimedCapture(const char* label, unsigned long durationSeconds) {
  timedCaptureModeActive = true;
  timedCaptureDurationSeconds = durationSeconds;
  timedCaptureStartedAtMs = millis();
  timedCaptureEndsAtMs = timedCaptureStartedAtMs + durationSeconds * 1000UL;
  timedCaptureLastSampleAtMs = 0;
  timedCaptureSampleCountSeen = 0;
  timedCaptureCancelled = false;
  strncpy(timedCaptureLabel, label, sizeof(timedCaptureLabel) - 1);
  timedCaptureLabel[sizeof(timedCaptureLabel) - 1] = '\0';

  Serial.print(F("CAPTURE_BEGIN,"));
  printCsvQuotedLabel(timedCaptureLabel);
  Serial.print(F(","));
  Serial.println(durationSeconds);

  Serial.print(F("Timed capture started for "));
  Serial.print(timedCaptureLabel);
  Serial.print(F(" ("));
  Serial.print(durationSeconds);
  Serial.println(F(" s)"));
}

void emitTimedCaptureSample(uint16_t r, uint16_t g, uint16_t b, uint16_t c, unsigned long elapsedMs) {
  BalancedRgbs balanced = applyWhiteBalanceGains(r, g, b, c, whiteBalanceRedGain, whiteBalanceGreenGain, whiteBalanceBlueGain);
  PrototypeRgb normalized = normalizePrototype(balanced.r, balanced.g, balanced.b);
  bool normalizedAvailable = balanced.r > 0 || balanced.g > 0 || balanced.b > 0;

  Serial.print(F("CAPTURE_SAMPLE,"));
  Serial.print(elapsedMs);
  Serial.print(F(","));
  printCsvQuotedLabel(timedCaptureLabel);
  Serial.print(F(","));
  Serial.print(r);
  Serial.print(F(","));
  Serial.print(g);
  Serial.print(F(","));
  Serial.print(b);
  Serial.print(F(","));
  Serial.print(c);
  Serial.print(F(","));
  Serial.print(balanced.r);
  Serial.print(F(","));
  Serial.print(balanced.g);
  Serial.print(F(","));
  Serial.print(balanced.b);
  Serial.print(F(","));
  if (normalizedAvailable) {
    Serial.print(normalized.r);
  } else {
    Serial.print(-1);
  }
  Serial.print(F(","));
  if (normalizedAvailable) {
    Serial.print(normalized.g);
  } else {
    Serial.print(-1);
  }
  Serial.print(F(","));
  if (normalizedAvailable) {
    Serial.println(normalized.b);
  } else {
    Serial.println(-1);
  }

  timedCaptureSampleCountSeen++;
}

void updateTimedCapture(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  if (!timedCaptureModeActive) return;

  unsigned long now = millis();
  if (now >= timedCaptureEndsAtMs) {
    stopTimedCapture(false);
    return;
  }

  if (timedCaptureSampleCountSeen == 0 || now - timedCaptureLastSampleAtMs >= timedCaptureSampleIntervalMs) {
    timedCaptureLastSampleAtMs = now;
    emitTimedCaptureSample(r, g, b, c, now - timedCaptureStartedAtMs);
  }
}

void handleSerialCommand(const char* command) {
  if (command == NULL) return;

  char commandCopy[sizeof(serialLine)];
  strncpy(commandCopy, command, sizeof(commandCopy) - 1);
  commandCopy[sizeof(commandCopy) - 1] = '\0';
  trimWhitespace(commandCopy);

  if (commandCopy[0] == '\0') return;

  if (equalsIgnoreCase(commandCopy, "stop") || equalsIgnoreCase(commandCopy, "exit")) {
    if (timedCaptureModeActive) {
      stopTimedCapture(true);
      return;
    }

    if (!calibrationModeActive) {
      Serial.println(F("No capture is active."));
      return;
    }

    calibrationModeActive = false;
    Serial.println(F("Calibration mode stopped"));
    return;
  }

  if (equalsIgnoreCase(commandCopy, "start")) {
    if (timedCaptureModeActive || calibrationModeActive) {
      Serial.println(F("Capture already active. Use 'stop' before starting recognition."));
      return;
    }

    if (liveRecognitionActive) {
      Serial.println(F("Live recognition is already running."));
      return;
    }

    liveRecognitionActive = true;
    Serial.println(F("Live recognition started."));
    return;
  }

  unsigned long durationSeconds = 0;
  char requestedLabel[sizeof(timedCaptureLabel)] = { 0 };
  if (parseTimedCaptureCommand(commandCopy, requestedLabel, sizeof(requestedLabel), &durationSeconds)) {
    if (timedCaptureModeActive || calibrationModeActive) {
      Serial.println(F("Capture already active. Use 'stop' before starting another capture."));
      return;
    }

    startTimedCapture(requestedLabel, durationSeconds);
    return;
  }

  if (equalsIgnoreCase(commandCopy, "cal") || equalsIgnoreCase(commandCopy, "calibrate")) {
    if (timedCaptureModeActive || calibrationModeActive) {
      Serial.println(F("Capture already active. Use 'stop' before starting another capture."));
      return;
    }

    startCalibration();
    return;
  }

  Serial.print(F("Unknown command: "));
  Serial.println(commandCopy);
  Serial.println(F("Use: start | cal | cal \"Color Name\" 15 | stop"));
}

void pollSerialInput() {
  while (Serial.available() > 0) {
    char incoming = (char)Serial.read();
    if (incoming == '\r' || incoming == '\n') {
      if (serialLineLength > 0) {
        serialLine[serialLineLength] = '\0';
        serialLineReady = true;
      }
      continue;
    }

    if (serialLineLength < sizeof(serialLine) - 1) {
      serialLine[serialLineLength++] = incoming;
    }
  }

  if (serialLineReady) {
    handleSerialCommand(serialLine);
    serialLineLength = 0;
    serialLineReady = false;
  }
}

BalancedRgbs applyWhiteBalance(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  BalancedRgbs balanced;
  balanced.r = (uint16_t)(r * whiteBalanceRedGain + 0.5f);
  balanced.g = (uint16_t)(g * whiteBalanceGreenGain + 0.5f);
  balanced.b = (uint16_t)(b * whiteBalanceBlueGain + 0.5f);
  balanced.c = c;
  return balanced;
}

void showLiveRgbColor(const BalancedRgbs& color) {
  if (color.c < colorClearMinThreshold) {
    turnOffRgbLed();
    return;
  }

  uint16_t maxChannel = max(color.r, max(color.g, color.b));
  if (maxChannel == 0) {
    turnOffRgbLed();
    return;
  }

  uint16_t minChannel = min(color.r, min(color.g, color.b));
  uint16_t chroma = maxChannel - minChannel;
  if (chroma == 0 || (uint32_t)chroma * 100UL <= (uint32_t)maxChannel * whiteBalancedMaxSpreadPct) {
    setRgbLedColor(255, 255, 255);
    return;
  }

  uint8_t r = (uint8_t)(((uint32_t)(color.r - minChannel) * 255UL) / chroma);
  uint8_t g = (uint8_t)(((uint32_t)(color.g - minChannel) * 255UL) / chroma);
  uint8_t b = (uint8_t)(((uint32_t)(color.b - minChannel) * 255UL) / chroma);
  setRgbLedColor(r, g, b);
}

int8_t classifySampleIndex(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  if (c < colorClearMinThreshold) return -1;
  if (c < colorMatchClearThreshold) return -1;

  uint16_t maxChannel = max(r, max(g, b));
  uint16_t minChannel = min(r, min(g, b));
  if (c >= whiteBalancedClearThreshold && maxChannel > 0 && (uint32_t)(maxChannel - minChannel) * 100UL <= (uint32_t)maxChannel * whiteBalancedMaxSpreadPct) {
    return neutralSampleIndex;
  }

  if (c > colorClearWhiteThreshold && r > colorWhiteChannelThreshold && g > colorWhiteChannelThreshold && b > colorWhiteChannelThreshold) return neutralSampleIndex;

  uint32_t sum = (uint32_t)r + (uint32_t)g + (uint32_t)b;
  if (sum == 0) return -1;

  PrototypeRgb measured = normalizePrototype(r, g, b);

  uint16_t bestDistance = 0xFFFF;
  int8_t bestSampleIndex = -1;

  for (uint8_t i = 0; i < calibrationSampleCount; ++i) {
    PrototypeRgb prototype = loadPrototype(i);
    uint16_t distance = prototypeDistance(measured, prototype);
    if (distance < bestDistance) {
      bestDistance = distance;
      bestSampleIndex = (int8_t)i;
    }
  }

  if (bestSampleIndex == neutralSampleIndex) {
    return neutralSampleIndex;
  }

  if (bestDistance > prototypeDistanceThreshold) return -1;

  return bestSampleIndex;
}

void resetDetectionAttempts() {
  detectionAttemptCount = 0;
  for (uint8_t i = 0; i < detectionAttemptBatchSize; ++i) {
    detectionAttemptMatches[i] = -1;
  }
}

int8_t findDetectedSampleIndex() {
  for (uint8_t i = 0; i < detectionAttemptCount; ++i) {
    int8_t candidate = detectionAttemptMatches[i];
    if (candidate < 0) continue;

    uint8_t matchCount = 0;
    for (uint8_t j = 0; j < detectionAttemptCount; ++j) {
      if (detectionAttemptMatches[j] == candidate) {
        matchCount++;
      }
    }

    if (matchCount >= detectionAttemptMinAgreement) {
      return candidate;
    }
  }

  return -1;
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }

  pinMode(pinColorSensorLED, OUTPUT);
  digitalWrite(pinColorSensorLED, colorSensorLEDOffLevel);
  pinMode(pinRgbLedRed, OUTPUT);
  pinMode(pinRgbLedGreen, OUTPUT);
  pinMode(pinRgbLedBlue, OUTPUT);
  turnOffRgbLed();

  colorSensorDetected = colorSensor.begin();
  if (colorSensorDetected) {
    Serial.println(F("TCS34725 detected"));
    blinkSensorLed(3);
    digitalWrite(pinColorSensorLED, colorSensorLEDOnLevel);
    Serial.println(F("Sensor LED on D2 enabled"));
    Serial.println(F("Type 'start' to enable live recognition."));
    Serial.println(F("Type 'cal' for guided calibration or 'cal \"Maroon Red\" 15' for timed raw capture."));
    printCalibrationSummary();
  } else {
    Serial.println(F("TCS34725 not detected on I2C"));
  }
}

void updateCalibrationMode(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  unsigned long now = millis();

  if (now < calibrationWaitUntilMs) return;

  if (now <= calibrationScanUntilMs) {
    calibrationSampleCountSeen++;
    calibrationSumR += r;
    calibrationSumG += g;
    calibrationSumB += b;
    calibrationSumC += c;

    if (r < calibrationMinR) calibrationMinR = r;
    if (g < calibrationMinG) calibrationMinG = g;
    if (b < calibrationMinB) calibrationMinB = b;
    if (c < calibrationMinC) calibrationMinC = c;

    if (r > calibrationMaxR) calibrationMaxR = r;
    if (g > calibrationMaxG) calibrationMaxG = g;
    if (b > calibrationMaxB) calibrationMaxB = b;
    if (c > calibrationMaxC) calibrationMaxC = c;

    Serial.print(F("CAL "));
    Serial.print(calibrationSamples[calibrationSampleIndex].name);
    Serial.print(F(" -> RGBC: "));
    Serial.print(r);
    Serial.print(F(", "));
    Serial.print(g);
    Serial.print(F(", "));
    Serial.print(b);
    Serial.print(F(", clear="));
    Serial.println(c);
    return;
  }

  finishCalibrationSample();
}

void loop() {
  pollSerialInput();

  if (!colorSensorDetected) {
    delay(1000);
    return;
  }

  uint16_t r = 0, g = 0, b = 0, c = 0;
  colorSensor.getRawData(&r, &g, &b, &c);

  if (timedCaptureModeActive) {
    updateTimedCapture(r, g, b, c);
    return;
  }

  if (calibrationModeActive) {
    updateCalibrationMode(r, g, b, c);
    delay(100);
    return;
  }

  if (!liveRecognitionActive) {
    turnOffRgbLed();
    delay(250);
    return;
  }

  BalancedRgbs balanced = applyWhiteBalance(r, g, b, c);
  showLiveRgbColor(balanced);
  int8_t matchedSampleIndex = classifySampleIndex(balanced.r, balanced.g, balanced.b, balanced.c);
  detectionAttemptMatches[detectionAttemptCount++] = matchedSampleIndex;

  if (detectionAttemptCount < detectionAttemptBatchSize) {
    delay(250);
    return;
  }

  int8_t detectedSampleIndex = findDetectedSampleIndex();
  if (detectedSampleIndex >= 0) {
    Serial.print(F("Detected: "));
    printSampleName(detectedSampleIndex);
    Serial.println();
  } else if (printSkippedDetectionBatches) {
    Serial.print(F("Skipped batch: "));
    printDetectionAttemptBatch();
    Serial.println();
  }

  resetDetectionAttempts();

  delay(250);
}
