// VL53L1X test sketch using the Adafruit_VL53L1X library.
//
// Prints distance in mm to Serial and demonstrates configuring a
// narrow "cone" (Field of View) via the ROI settings.
//
// Wiring (I2C):
//   VIN -> 3.3V/5V (module dependent), GND -> GND
//   SDA -> SDA, SCL -> SCL
//   XSHUT / IRQ: optional; set below to -1 if unused.
//
// Library: Adafruit_VL53L1X  (libraries/Adafruit_VL53L1X)

#include <Wire.h>
#include "Adafruit_VL53L1X.h"

#define XSHUT_PIN -1
#define IRQ_PIN   -1

Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

// -------- Cone (ROI / Field of View) settings --------------------------
// The full 16x16 SPAD array gives the default ~27 deg FoV.
// Smaller ROI = narrower cone, at the cost of some sensitivity/range.
//   16x16 -> ~27 deg  (default, widest)
//    8x8  -> ~20 deg
//    4x4  -> ~15 deg  (minimum, narrowest)
// ROI center SPAD 199 is the optical center of the array.
static const uint16_t ROI_WIDTH       = 4;
static const uint16_t ROI_HEIGHT      = 4;
static const uint8_t  ROI_CENTER_SPAD = 199;

// Distance mode: 1 = Short (~1.3 m), 2 = Long (~4 m).
static const uint16_t DISTANCE_MODE = 1;

// Valid timing budgets (ms): 15, 20, 33, 50, 100, 200, 500.
static const uint16_t TIMING_BUDGET_MS = 50;

void setup()
{
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("Adafruit VL53L1X test"));

  Wire.begin();
  if (!vl53.begin(0x29, &Wire)) {
    Serial.print(F("Error on init of VL sensor: "));
    Serial.println(vl53.vl_status);
    while (1) delay(10);
  }
  Serial.print(F("Sensor ID: 0x"));
  Serial.println(vl53.sensorID(), HEX);

  // Distance mode must be set before ROI on this driver.
  vl53.VL53L1X_SetDistanceMode(DISTANCE_MODE);

  // Configure the narrow cone (ROI).
  vl53.VL53L1X_SetROI(ROI_WIDTH, ROI_HEIGHT);
  vl53.VL53L1X_SetROICenter(ROI_CENTER_SPAD);

  uint16_t w = 0, h = 0;
  uint8_t  c = 0;
  vl53.VL53L1X_GetROI_XY(&w, &h);
  vl53.VL53L1X_GetROICenter(&c);
  Serial.print(F("ROI size: "));
  Serial.print(w); Serial.print('x'); Serial.println(h);
  Serial.print(F("ROI center SPAD: "));
  Serial.println(c);

  vl53.setTimingBudget(TIMING_BUDGET_MS);
  Serial.print(F("Timing budget (ms): "));
  Serial.println(vl53.getTimingBudget());

  if (!vl53.startRanging()) {
    Serial.print(F("Couldn't start ranging: "));
    Serial.println(vl53.vl_status);
    while (1) delay(10);
  }
  Serial.println(F("Ranging started"));
}

void loop()
{
  if (vl53.dataReady()) {
    int16_t distance = vl53.distance();
    if (distance == -1) {
      Serial.print(F("Couldn't get distance: "));
      Serial.println(vl53.vl_status);
    } else {
      Serial.print(F("Distance: "));
      Serial.print(distance);
      Serial.println(F(" mm"));
    }
    vl53.clearInterrupt();
  }
}
