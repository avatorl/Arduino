const int ANALOG_PIN = A0;
const float REF_VOLTAGE = 1.1; 
const float R1 = 100000.0; 
const float R2 = 10000.0;  
const float CALIBRATION = 1.01; // = voltage measured by multimeter / voltage measured by Arduino

const int NUM_SAMPLES = 8; 

// Lookup table for 2s 18650 battery pack
// 7.0V is hardlocked to 0% capacity because Arduino is powered from the battery pack
//    and Arduino Nano needs at least 7V to work, so 7V is treated as 0%
// Even 10% steps mapping down to 7.0V.
const float voltTable[] = {8.40, 8.20, 8.05, 7.90, 7.75, 7.60, 7.45, 7.35, 7.25, 7.15, 7.00};
const int pctTable[] = {100,  90,   80,   70,   60,   50,   40,   30,   20,   10,   0};

void setup() {
  Serial.begin(9600);
  analogReference(INTERNAL); 
}

void loop() {
  float samples[NUM_SAMPLES];
  float sum = 0.0;
  float ratio = (R1 + R2) / R2;

  // 1. Prime the ADC with one throwaway read, then collect <NUM_SAMPLES> samples
  analogRead(ANALOG_PIN);

  for (int i = 0; i < NUM_SAMPLES; i++) {
    unsigned long sampleStartMillis = millis();
    int rawValue = analogRead(ANALOG_PIN);
    float pinVoltage = ((float)rawValue * REF_VOLTAGE) / 1023.0;
    samples[i] = pinVoltage * ratio * CALIBRATION;
    sum += samples[i];
    while (millis() - sampleStartMillis < 1) {
    }
  }

  // 2. Calculate the Average (Mean)
  float mean = sum / (float)NUM_SAMPLES;

  // 3. Calculate Dispersion (Standard Deviation)
  float sumDifferenceSquares = 0.0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    float difference = samples[i] - mean;
    sumDifferenceSquares += difference * difference;
  }
  float variance = sumDifferenceSquares / (float)NUM_SAMPLES;
  float stdDev = sqrt(variance); 

  // 4. Output results rounded to exactly 2 decimal places
  Serial.print("Mean V: ");
  Serial.print(mean, 2); 
  Serial.print("V | Dispersion: ±");
  Serial.print(stdDev, 2); 
  Serial.print(" V | 2s 18650 battery: ");         
  Serial.print(get2SPercent(mean));   
  Serial.println("%");                     

  unsigned long cooldownStartMillis = millis();
  while (millis() - cooldownStartMillis < 3000) {
  }
}

int get2SPercent(float voltage) {
  float maxVolt = voltTable[0];
  float minVolt = voltTable[9];

  if (voltage >= maxVolt) return 100;
  if (voltage <= minVolt) return 0;

  for (int i = 0; i < 9; i++) {
    float vUpper = voltTable[i];
    float vLower = voltTable[i + 1];

    if (voltage > vLower) {
      int pUpper = pctTable[i];
      int pLower = pctTable[i + 1];
      return mapFloat(voltage, vLower, vUpper, pLower, pUpper);
    }
  }
  return 0;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
