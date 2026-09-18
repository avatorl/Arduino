// FILE: GY521_asymmetric_calibration.ino
#include "GY521.h"

GY521 sensor(0x68);

const int SAMPLE_SIZE = 12;
const float POSITIVE_ROLL_ENDPOINT = 19.9;
const float NEGATIVE_ROLL_ENDPOINT = 17.4;
float rollSamples[SAMPLE_SIZE];
int sampleIndex = 0;

void setup()
{
  Serial.begin(115200); 
  Wire.begin();
  delay(100);
  
  while (sensor.wakeup() == false)
  {
    delay(1000);
  }
  
  sensor.setAccelSensitivity(1);  // 4g scale mode 
  sensor.setGyroSensitivity(1);   // 500 dps mode
  sensor.setDLPFMode(6);          // Max hardware noise dampening (5Hz)

  for(int i = 0; i < SAMPLE_SIZE; i++) {
    rollSamples[i] = 0.0;
  }
}

void loop()
{
  sensor.read();
  
  // 1. Fetch raw uncompensated gravity vectors
  float accX = sensor.getAccelX();
  float accZ = sensor.getAccelZ();

  // 2. Base angle calculation from the chip
  float rawRoll = atan2(accX, accZ) * 180.0 / M_PI;

  // 3. Correct the measured sine-shaped response using each endpoint.
  float endpoint = rawRoll >= 0.0 ? POSITIVE_ROLL_ENDPOINT : NEGATIVE_ROLL_ENDPOINT;
  float normalizedRoll = constrain(rawRoll / endpoint, -1.0, 1.0);
  float correctedRoll = asin(normalizedRoll) * 180.0 / M_PI;

  // 4. Apply data buffer averaging for stability
  rollSamples[sampleIndex] = correctedRoll;
  sampleIndex = (sampleIndex + 1) % SAMPLE_SIZE;

  float averagedRoll = 0.0;
  for(int i = 0; i < SAMPLE_SIZE; i++) {
    averagedRoll += rollSamples[i];
  }
  averagedRoll /= SAMPLE_SIZE;

  // Bound limits to fit our display pipeline boundaries cleanly
  averagedRoll = constrain(averagedRoll, -90.0, 90.0);
  
  // Terminal visualization processing
  int centerSpace = 80;
  int offset = map(averagedRoll, -90, 90, -75, 75);
  int targetPosition = centerSpace + offset;
  
  for (int i = 0; i < targetPosition; i++) {
    if (i == centerSpace) {
      Serial.print("|"); 
    } else {
      Serial.print(" ");
    }
  }
  
  Serial.print("✈️"); 
  Serial.print(" ("); Serial.print(averagedRoll, 1); Serial.println("°)");
  
  delay(25); 
}

