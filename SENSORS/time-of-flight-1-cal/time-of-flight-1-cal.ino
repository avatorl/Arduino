#include <Wire.h>
#include "VL53L1X_ULD.h" // Note the capitalization used by ULD

uint16_t dev = 0x52; // Default 8-bit I2C target address

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);

  // Initialize the ULD structure
  uint8_t booted = 0;
  while(booted == 0) {
    VL53L1X_BootState(dev, &booted);
    delay(10);
  }
  
  VL53L1X_SensorInit(dev);
  VL53L1X_SetDistanceMode(dev, 2); // 1 = Short, 2 = Long
  VL53L1X_SetTimingBudgetInMs(dev, 50);

  Serial.println(F("Starting Calibration... Keep target steady at 140mm."));
  delay(2000);

  // 1. Run standard calibration sequences
  uint16_t measuredOffset = 0;
  VL53L1X_CalibrateOffset(dev, 140, &measuredOffset); // Calibrates to target
  
  uint16_t measuredXTalk = 0;
  VL53L1X_CalibrateXtalk(dev, 140, &measuredXTalk);   // Calibrates crosstalk

  // 2. Fetch the newly calculated variables directly from the register map
  uint16_t finalOffset = 0;
  uint16_t finalXTalk = 0;
  
  VL53L1X_GetOffset(dev, &finalOffset);
  VL53L1X_GetXtalk(dev, &finalXTalk);

  // 3. Print out your hardcode declarations
  Serial.println(F("\n--- COPY AND PASTE THIS INTO YOUR PRODUCTION CODE ---"));
  Serial.print(F("const uint16_t HARDCODED_OFFSET = ")); Serial.print(finalOffset); Serial.println(F(";"));
  Serial.print(F("const uint16_t HARDCODED_XTALK  = ")); Serial.print(finalXTalk); Serial.println(F(";"));
  Serial.println(F("----------------------------------------------------------"));
}

void loop() {}
