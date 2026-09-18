#include <Wire.h>
#include "VL53L1X_ULD.h"

uint16_t dev = 0x52;

// PASTE YOUR RAW VALUES FROM STEP 1 HERE:
const uint16_t HARDCODED_OFFSET = 63491;
const uint16_t HARDCODED_XTALK  = 108;

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);

  uint8_t booted = 0;
  while(booted == 0) {
    VL53L1X_BootState(dev, &booted);
    delay(10);
  }
  
  VL53L1X_SensorInit(dev);

  // --- FLASH HARDCODED MATRIX PROPERTIES ---
  VL53L1X_SetOffset(dev, HARDCODED_OFFSET);
  VL53L1X_SetXtalk(dev, HARDCODED_XTALK);
  Serial.println(F("Hardcoded calibration profiles successfully pushed."));

  // Start continuous operations
  VL53L1X_SetDistanceMode(dev, 2); 
  VL53L1X_SetTimingBudgetInMs(dev, 50);
  VL53L1X_StartRanging(dev);
}

void loop() {
  uint8_t isDataReady = 0;
  uint16_t distance = 0;
  uint8_t rangeStatus = 0;

  // Poll for ready state flag
  while (isDataReady == 0) {
    VL53L1X_CheckForDataReady(dev, &isDataReady);
    delay(1);
  }

  // Fetch results
  VL53L1X_GetRangeStatus(dev, &rangeStatus);
  VL53L1X_GetDistance(dev, &distance);
  
  if (rangeStatus == 0) { // 0 represents a solid signal return
    Serial.print(F("Calibrated Distance: "));
    Serial.print(distance);
    Serial.println(F(" mm"));
  }

  // Clear state flags for the next internal sensor cycle
  VL53L1X_ClearInterrupt(dev);
}
