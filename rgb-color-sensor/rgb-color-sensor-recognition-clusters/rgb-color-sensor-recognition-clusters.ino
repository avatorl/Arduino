#include <Wire.h>
#include <Adafruit_TCS34725.h>

struct PrototypeRgb {
  uint16_t r;
  uint16_t g;
  uint16_t b;
};

struct BalancedRgbs {
  uint16_t r;
  uint16_t g;
  uint16_t b;
  uint16_t c;
};

struct ClusterDefinition {
  uint8_t labelIndex;
  PrototypeRgb center;
  uint16_t maxDistance;
};

const int pinColorSensorLED = 2;
const int pinRgbLedRed = 3;
const int pinRgbLedGreen = 5;
const int pinRgbLedBlue = 6;
const uint8_t colorSensorLEDOnLevel = HIGH;
const uint8_t colorSensorLEDOffLevel = LOW;

Adafruit_TCS34725 colorSensor(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
bool colorSensorDetected = false;

// BEGIN GENERATED CLUSTER DATA
const uint16_t colorClearMinThreshold = 160;
const uint16_t colorMatchClearThreshold = 304;
const float whiteBalanceRedGain = 1.000f;
const float whiteBalanceGreenGain = 1.212f;
const float whiteBalanceBlueGain = 2.318f;

const char* colorLabels[] = {
  "White",
  "Brown",
  "Cyan",
  "Green",
  "Grey",
  "Magenta",
  "Orange",
  "Red",
  "Yellow",
};

const ClusterDefinition clusterDefinitions[] = {
  // White: 1 cluster(s), mean_distance=14.8
  { 0, { 334, 333, 333 }, 32 },
  // Brown: 3 cluster(s), mean_distance=12.0
  { 1, { 392, 322, 287 }, 31 },
  { 1, { 415, 305, 280 }, 22 },
  { 1, { 428, 289, 284 }, 25 },
  // Cyan: 1 cluster(s), mean_distance=38.6
  { 2, { 158, 313, 529 }, 60 },
  // Green: 2 cluster(s), mean_distance=24.9
  { 3, { 160, 518, 322 }, 62 },
  { 3, { 203, 482, 315 }, 52 },
  // Grey: 3 cluster(s), mean_distance=12.6
  { 4, { 303, 345, 352 }, 38 },
  { 4, { 327, 331, 342 }, 22 },
  { 4, { 347, 317, 336 }, 27 },
  // Magenta: 3 cluster(s), mean_distance=7.2
  { 5, { 455, 201, 344 }, 14 },
  { 5, { 476, 193, 331 }, 18 },
  { 5, { 488, 186, 326 }, 10 },
  // Orange: 1 cluster(s), mean_distance=12.9
  { 6, { 542, 254, 204 }, 22 },
  // Red: 3 cluster(s), mean_distance=12.6
  { 7, { 514, 180, 305 }, 27 },
  { 7, { 550, 191, 259 }, 36 },
  { 7, { 572, 176, 252 }, 22 },
  // Yellow: 2 cluster(s), mean_distance=8.4
  { 8, { 420, 368, 212 }, 14 },
  { 8, { 432, 353, 215 }, 18 },
};

const uint8_t colorLabelCount = sizeof(colorLabels) / sizeof(colorLabels[0]);
const uint8_t clusterDefinitionCount = sizeof(clusterDefinitions) / sizeof(clusterDefinitions[0]);
// END GENERATED CLUSTER DATA

void setRgbLedColor(uint8_t r, uint8_t g, uint8_t b) {
  analogWrite(pinRgbLedRed, 255 - r);
  analogWrite(pinRgbLedGreen, 255 - g);
  analogWrite(pinRgbLedBlue, 255 - b);
}

void turnOffRgbLed() {
  setRgbLedColor(0, 0, 0);
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
  if (chroma == 0) {
    setRgbLedColor(255, 255, 255);
    return;
  }

  uint8_t r = (uint8_t)(((uint32_t)(color.r - minChannel) * 255UL) / chroma);
  uint8_t g = (uint8_t)(((uint32_t)(color.g - minChannel) * 255UL) / chroma);
  uint8_t b = (uint8_t)(((uint32_t)(color.b - minChannel) * 255UL) / chroma);
  setRgbLedColor(r, g, b);
}

void printColorLabel(int8_t labelIndex) {
  if (labelIndex >= 0 && labelIndex < colorLabelCount) {
    Serial.print(colorLabels[labelIndex]);
    return;
  }

  Serial.print(F("unknown"));
}

int8_t classifyClusterLabelIndex(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  if (c < colorClearMinThreshold) return -1;
  if (c < colorMatchClearThreshold) return -1;

  PrototypeRgb measured = normalizePrototype(r, g, b);
  if (measured.r == 0 && measured.g == 0 && measured.b == 0) return -1;

  int8_t bestLabelIndex = -1;
  uint16_t bestDistance = 0xFFFF;

  for (uint8_t i = 0; i < clusterDefinitionCount; ++i) {
    const ClusterDefinition& cluster = clusterDefinitions[i];
    uint16_t distance = prototypeDistance(measured, cluster.center);
    if (distance > cluster.maxDistance) continue;

    if (bestLabelIndex < 0 || distance < bestDistance) {
      bestDistance = distance;
      bestLabelIndex = (int8_t)cluster.labelIndex;
    }
  }

  return bestLabelIndex;
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
    digitalWrite(pinColorSensorLED, colorSensorLEDOnLevel);
    Serial.println(F("TCS34725 cluster recognition ready"));
  } else {
    Serial.println(F("TCS34725 not detected on I2C"));
  }
}

void loop() {
  if (!colorSensorDetected) {
    delay(1000);
    return;
  }

  uint16_t r = 0, g = 0, b = 0, c = 0;
  colorSensor.getRawData(&r, &g, &b, &c);

  BalancedRgbs balanced = applyWhiteBalance(r, g, b, c);
  showLiveRgbColor(balanced);

  int8_t detectedLabelIndex = classifyClusterLabelIndex(balanced.r, balanced.g, balanced.b, balanced.c);
  if (detectedLabelIndex >= 0) {
    Serial.print(F("Detected: "));
    printColorLabel(detectedLabelIndex);
    Serial.println();
  }

  delay(250);
}
