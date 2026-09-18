// VL53L1X test using the STM32duino/ST driver sequence that was previously
// used with this TOF400C module in arduino-train-v2.
//
// Wiring:
//   VIN -> 5V or 3.3V as required by the breakout
//   GND -> GND
//   SDA -> A4, SCL -> A5 (Arduino Uno)
//   XSHUT -> A3
//
// The driver resets the sensor with XSHUT, waits for boot, then changes its
// I2C address from the default 0x29 to 0x24.

#include <Wire.h>
#include <vl53l1x_class.h>

static const uint8_t XSHUT_PIN = A3;
static const uint8_t SENSOR_ADDRESS_8BIT = 0x48; // 7-bit I2C address 0x24

// Cone (ROI) settings. The minimum 4x4 ROI is approximately a 15 degree FoV.
static const uint16_t ROI_WIDTH = 4;
static const uint16_t ROI_HEIGHT = 4;
static const uint8_t ROI_CENTER_SPAD = 199;

VL53L1X sensor(&Wire, XSHUT_PIN);

bool addressAcknowledges(uint8_t address)
{
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void reportStatus(const __FlashStringHelper *operation, int status)
{
  Serial.print(operation);
  Serial.print(F(": "));
  Serial.println(status);
}

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  Serial.println(F("VL53L1X STM32duino driver test"));
  Serial.println(F("Resetting through XSHUT on A3..."));

  sensor.begin();

  // Match InitSensor's reset/address steps, but impose a finite boot timeout
  // so an absent or nonresponsive sensor cannot leave this test silent.
  sensor.VL53L1X_Off();
  sensor.VL53L1X_On();

  int status = sensor.VL53L1X_SetI2CAddress(SENSOR_ADDRESS_8BIT);
  reportStatus(F("SetI2CAddress"), status);
  if (status != 0) {
    Serial.println(F("Could not move the sensor from 0x29 to 0x24."));
    while (1) delay(1000);
  }

  const uint8_t sensorAddress = SENSOR_ADDRESS_8BIT >> 1;
  Serial.print(F("I2C 0x29 ACK: "));
  Serial.println(addressAcknowledges(0x29) ? F("yes") : F("no"));
  Serial.print(F("I2C 0x"));
  Serial.print(sensorAddress, HEX);
  Serial.print(F(" ACK after address change: "));
  Serial.println(addressAcknowledges(sensorAddress) ? F("yes") : F("no"));
  if (!addressAcknowledges(sensorAddress)) {
    Serial.println(F("The sensor did not accept the VL53L1X address-change command."));
    while (1) delay(1000);
  }

  uint8_t bootState = 0;
  const unsigned long bootStartMs = millis();
  while (millis() - bootStartMs < 1000UL) {
    status = sensor.VL53L1X_BootState(&bootState);
    if (status != 0) {
      reportStatus(F("BootState"), status);
      while (1) delay(1000);
    }
    if (bootState != 0) break;
    delay(2);
  }
  Serial.print(F("Boot state: 0x"));
  Serial.println(bootState, HEX);
  if (bootState == 0) {
    Serial.println(F("Sensor did not report boot complete within 1 second."));
    while (1) delay(1000);
  }

  status = sensor.VL53L1X_SensorInit();
  reportStatus(F("SensorInit"), status);
  if (status != 0) {
    Serial.println(F("Initialization failed."));
    while (1) delay(1000);
  }

  status = sensor.VL53L1X_SetDistanceMode(1); // Short distance mode.
  reportStatus(F("SetDistanceMode"), status);
  if (status != 0) while (1) delay(1000);

  status = sensor.VL53L1X_SetTimingBudgetInMs(50);
  reportStatus(F("SetTimingBudgetInMs"), status);
  if (status != 0) while (1) delay(1000);

  status = sensor.VL53L1X_SetROI(ROI_WIDTH, ROI_HEIGHT);
  reportStatus(F("SetROI"), status);
  if (status != 0) while (1) delay(1000);

  status = sensor.VL53L1X_SetROICenter(ROI_CENTER_SPAD);
  reportStatus(F("SetROICenter"), status);
  if (status != 0) while (1) delay(1000);

  status = sensor.VL53L1X_StartRanging();
  reportStatus(F("StartRanging"), status);
  if (status != 0) while (1) delay(1000);

  Serial.println(F("Ranging started. Distance in mm:"));
}

void loop()
{
  uint8_t dataReady = 0;
  int status = sensor.VL53L1X_CheckForDataReady(&dataReady);
  if (status != 0) {
    reportStatus(F("CheckForDataReady"), status);
    delay(100);
    return;
  }

  if (!dataReady) return;

  uint16_t distanceMm = 0;
  status = sensor.VL53L1X_GetDistance(&distanceMm);
  if (status == 0) {
    Serial.print(distanceMm);
    Serial.println(F(" mm"));
  } else {
    reportStatus(F("GetDistance"), status);
  }

  status = sensor.VL53L1X_ClearInterrupt();
  if (status != 0) reportStatus(F("ClearInterrupt"), status);
}
