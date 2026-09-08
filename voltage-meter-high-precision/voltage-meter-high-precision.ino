#include <avr/pgmspace.h> // Library needed to read directly from Flash Memory

const int ANALOG_PIN = A0;
const float REF_VOLTAGE = 1.1; 
const float R1 = 100000.0; 
const float R2 = 10000.0;  
const float CALIBRATION = 1.01; // = voltage measured by multimeter / voltage measured by Arduino

const int NUM_SAMPLES = 10; 

// 1. Store the Look-Up Table directly into Flash Memory (SRAM usage = 0 bytes!)
// To save space, we match 10 voltage boundaries with their matching percentages
// Lookup table: 7.0V is now hardlocked to 0% capacity (Arduino needs 7+V to work we treat 7V as 0%)
// Lookup table: Even 10% steps mapping down to 7.0V
const float voltTable[] PROGMEM = {8.40, 8.20, 8.05, 7.90, 7.75, 7.60, 7.45, 7.35, 7.25, 7.15, 7.00};
const int pctTable[]   PROGMEM = {100,  90,   80,   70,   60,   50,   40,   30,   20,   10,   0};

void setup() {
  Serial.begin(9600);
  analogReference(INTERNAL); 
}

void loop() {
  float samples[NUM_SAMPLES];
  float sum = 0.0;
  float ratio = (R1 + R2) / R2;

  // 1. Collect 100 samples rapidly
  for (int i = 0; i < NUM_SAMPLES; i++) {
    int rawValue = analogRead(ANALOG_PIN);
    float pinVoltage = ((float)rawValue * REF_VOLTAGE) / 1023.0;
    samples[i] = pinVoltage * ratio * CALIBRATION;
    sum += samples[i];
    delay(1); 
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
  Serial.print(get2SPercentFlash(mean));   // Updated to read from Flash Memory     
  Serial.println("%");                     

  delay(3000); 
}

// 2. Modified helper function that extracts data straight from Flash Memory
int get2SPercentFlash(float voltage) {
  // Read first and last index directly from Flash Memory
  float maxVolt = pgm_read_float(&voltTable[0]);
  float minVolt = pgm_read_float(&voltTable[9]);

  if (voltage >= maxVolt) return 100;
  if (voltage <= minVolt) return 0;

  // Loop through the flash arrays to find where the current voltage sits
  for (int i = 0; i < 9; i++) {
    float vUpper = pgm_read_float(&voltTable[i]);
    float vLower = pgm_read_float(&voltTable[i + 1]);

    if (voltage > vLower) {
      int pUpper = pgm_read_word(&pctTable[i]);
      int pLower = pgm_read_word(&pctTable[i + 1]);
      return mapFloat(voltage, vLower, vUpper, pLower, pUpper);
    }
  }
  return 0;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
