// VL53L1X (TOF400C) test sketch using the Pololu VL53L1X library.
//
// Prints distance in mm to Serial and demonstrates configuring a
// narrow "cone" (Field of View) via the ROI settings.
//
// Wiring (I2C):
//   VIN -> 3.3V/5V (module dependent), GND -> GND
//   SDA -> SDA, SCL -> SCL
//   XSHUT -> tied HIGH (or leave floating on most breakouts)
//
// Library: Pololu VL53L1X  (libraries/VL53L1X)

#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;

// -------- Cone (ROI / Field of View) settings --------------------------
// The full 16x16 SPAD array gives the default ~27 deg FoV.
// Smaller ROI = narrower cone, at the cost of some sensitivity/range.
//   16x16 -> ~27 deg  (default, widest)
//    8x8  -> ~20 deg
//    4x4  -> ~15 deg  (minimum, narrowest)
// ROI_CENTER_SPAD = 199 is the optical center of the array.
static const uint8_t ROI_WIDTH       = 4;
static const uint8_t ROI_HEIGHT      = 4;
static const uint8_t ROI_CENTER_SPAD = 199;

// Distance mode: Short / Medium / Long
static const VL53L1X::DistanceMode DISTANCE_MODE = VL53L1X::Short;

// Timing budget (us) and inter-measurement period (ms).
// Short mode minimum timing budget is 20 ms.
static const uint32_t TIMING_BUDGET_US    = 50000;
static const uint32_t INTERMEASUREMENT_MS = 50;

void setup()
{
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin();
  Wire.setClock(400000); // 400 kHz I2C

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println(F("Failed to detect and initialize VL53L1X!"));
    while (1) {}
  }

  sensor.setDistanceMode(DISTANCE_MODE);
  sensor.setMeasurementTimingBudget(TIMING_BUDGET_US);

  // Configure the narrow cone (ROI).
  sensor.setROISize(ROI_WIDTH, ROI_HEIGHT);
  sensor.setROICenter(ROI_CENTER_SPAD);

  // Report the effective ROI (the driver may clamp values).
  uint8_t w = 0, h = 0;
  sensor.getROISize(&w, &h);
  Serial.print(F("ROI size: "));
  Serial.print(w); Serial.print('x'); Serial.println(h);
  Serial.print(F("ROI center SPAD: "));
  Serial.println(sensor.getROICenter());

  sensor.startContinuous(INTERMEASUREMENT_MS);

  Serial.println(F("VL53L1X ready. Streaming distance (mm)..."));
}

void loop()
{
  uint16_t mm = sensor.read();

  Serial.print(mm);
  Serial.print(F(" mm"));
  if (sensor.timeoutOccurred()) { Serial.print(F("  TIMEOUT")); }
  Serial.println();
}
