// Voltage meter to measure external power source voltage up to 12V
// Warning: If output is maxed out at approximately 12.1V, the input voltage can be higher than 12.1V. Do not use with a power source that can exceed 12.1V.

const int ANALOG_PIN = A0;
const int R1_KOHM = 100;
const int R2_KOHM = 10;
const float CALIBRATION = 1.01; // = voltage measured by multimeter / voltage measured by Arduino
const int ANALOG_READ_MAX = 1023;
const float REF_VOLTAGE = 1.1;
const uint8_t NUM_SAMPLES = 8;
const unsigned long SAMPLE_INTERVAL_MS = 1;
const unsigned long REPORT_INTERVAL_MS = 3000;

// Lookup table for 2s 18650 battery pack
// 7.15V is treated as 0% capacity to match the train's low-battery shutdown threshold.
// Steps map down to 7.15V.
const float voltTable[] = {8.40, 8.20, 8.05, 7.90, 7.75, 7.60, 7.45, 7.35, 7.25, 7.20, 7.15};
const uint8_t BATTERY_TABLE_SIZE = sizeof(voltTable) / sizeof(voltTable[0]);

// Track running totals so mean and variance can be computed without storing all samples.
float sum = 0.0;
float sumSquares = 0.0;
uint8_t sampleIndex = 0;
bool isSampling = false;
unsigned long lastSampleMillis = 0;
unsigned long lastReportMillis = 0;

void startMeasurement(unsigned long now);
void captureSample(unsigned long now);
void printMeasurementReport();

void setup() {
  Serial.begin(9600);
  analogReference(INTERNAL); 
  lastReportMillis = millis() - REPORT_INTERVAL_MS;
}

void loop() {
  unsigned long now = millis();

  if (!isSampling) {
    if (now - lastReportMillis < REPORT_INTERVAL_MS) {
      return;
    }

    startMeasurement(now);
  }

  if (now - lastSampleMillis >= SAMPLE_INTERVAL_MS) {
    captureSample(now);
  }
}

void startMeasurement(unsigned long now) {
  sum = 0.0;
  sumSquares = 0.0;
  sampleIndex = 0;
  isSampling = true;

  // Throw away the first read after idling so the ADC settles before sampling.
  analogRead(ANALOG_PIN);
  lastSampleMillis = now - SAMPLE_INTERVAL_MS;
}

void captureSample(unsigned long now) {
  float ratio = (R1_KOHM + R2_KOHM) / (float)R2_KOHM;
  int rawValue = analogRead(ANALOG_PIN);
  float pinVoltage = (rawValue * REF_VOLTAGE) / ANALOG_READ_MAX;
  float sampleVoltage = pinVoltage * ratio * CALIBRATION;

  sum += sampleVoltage;
  sumSquares += sampleVoltage * sampleVoltage;
  sampleIndex++;
  lastSampleMillis = now;

  if (sampleIndex < NUM_SAMPLES) {
    return;
  }

  printMeasurementReport();
  isSampling = false;
  lastReportMillis = now;
}

void printMeasurementReport() {
  // 2. Calculate the Average (Mean)
  float mean = sum / NUM_SAMPLES;

  // 3. Calculate Dispersion (Standard Deviation)
  float variance = (sumSquares / NUM_SAMPLES) - (mean * mean);
  if (variance < 0.0) {
    variance = 0.0;
  }
  float stdDev = sqrt(variance); 

  // 4. Output results rounded to exactly 2 decimal places
  Serial.print("Mean V: ");
  Serial.print(mean, 2); 
  Serial.print("V | Dispersion: ±");
  Serial.print(stdDev, 2); 
  Serial.print(" V | 2s 18650 battery: ");         
  Serial.print(get2SPercent(mean));   
  Serial.println("%");                     
}

int get2SPercent(float voltage) {
  float maxVolt = voltTable[0];
  float minVolt = voltTable[BATTERY_TABLE_SIZE - 1];

  if (voltage >= maxVolt) return 100;
  if (voltage <= minVolt) return 0;

  for (int i = 0; i < BATTERY_TABLE_SIZE - 1; i++) {
    float vUpper = voltTable[i];
    float vLower = voltTable[i + 1];

    if (voltage > vLower) {
      int pUpper = 100 - (i * 10);
      int pLower = pUpper - 10;
      return mapFloat(voltage, vLower, vUpper, pLower, pUpper);
    }
  }
  return 0;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
