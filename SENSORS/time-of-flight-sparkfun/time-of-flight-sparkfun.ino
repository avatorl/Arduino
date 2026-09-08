// VL53L1X test sketch using the SparkFun VL53L1X library.
//
// Prints distance in mm (and feet) to Serial and demonstrates
// configuring a narrow "cone" (Field of View) via the ROI settings.
//
// Wiring (I2C):
//   VIN -> 3.3V/5V (module dependent), GND -> GND
//   SDA -> SDA, SCL -> SCL
//   XSHUT / INT: optional, unused here.
//
// Library: SparkFun_VL53L1X_4m_Laser_Distance_Sensor
//          (libraries/SparkFun_VL53L1X_4m_Laser_Distance_Sensor)

#include <Wire.h>
#include "SparkFun_VL53L1X.h"

SFEVL53L1X distanceSensor;

// -------- Cone (ROI / Field of View) settings --------------------------
// The full 16x16 SPAD array gives the default ~27 deg FoV.
// Smaller ROI = narrower cone, at the cost of some sensitivity/range.
//   16x16 -> ~27 deg  (default, widest)
//    8x8  -> ~20 deg
//    4x4  -> ~15 deg  (minimum, narrowest)
// Optical center SPAD 199 = center of the array.
static const uint8_t ROI_WIDTH       = 4;
static const uint8_t ROI_HEIGHT      = 4;
static const uint8_t ROI_CENTER_SPAD = 199;

// Distance mode: short (~1.3 m) or long (~4 m).
static const bool USE_SHORT_DISTANCE = true;

// Valid timing budgets (ms): 15, 20, 33, 50, 100, 200, 500.
static const uint16_t TIMING_BUDGET_MS = 50;

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  Serial.println(F("SparkFun VL53L1X test"));

  if (distanceSensor.begin() != 0) {
    Serial.println(F("Sensor failed to begin. Check wiring. Freezing..."));
    while (1) ;
  }
  Serial.println(F("Sensor online!"));

  if (USE_SHORT_DISTANCE) distanceSensor.setDistanceModeShort();
  else                    distanceSensor.setDistanceModeLong();

  distanceSensor.setTimingBudgetInMs(TIMING_BUDGET_MS);

  // Configure the narrow cone (ROI + optical center).
  distanceSensor.setROI(ROI_WIDTH, ROI_HEIGHT, ROI_CENTER_SPAD);

  Serial.print(F("Distance mode: "));
  Serial.println(distanceSensor.getDistanceMode() == 1 ? F("Short") : F("Long"));
  Serial.print(F("Timing budget (ms): "));
  Serial.println(distanceSensor.getTimingBudgetInMs());
  Serial.print(F("ROI size: "));
  Serial.print(distanceSensor.getROIX());
  Serial.print('x');
  Serial.println(distanceSensor.getROIY());
}

void loop()
{
  distanceSensor.startRanging();
  while (!distanceSensor.checkForDataReady()) {
    delay(1);
  }
  int distance = distanceSensor.getDistance();
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();

  float distanceFeet = distance * 0.0393701f / 12.0f;

  Serial.print(F("Distance(mm): "));
  Serial.print(distance);
  Serial.print(F("\tDistance(ft): "));
  Serial.println(distanceFeet, 2);
}
