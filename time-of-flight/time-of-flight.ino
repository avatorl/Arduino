#include <Wire.h>
#include <VL53L0X.h>

// Set to 1 if XSHUT is wired to D3 instead of tied directly to +5V. D3 control lets the sensor
// be held in reset and assigned a non-default I2C address, needed once more than one VL53L0X
// shares the bus. XSHUT is currently tied to +5V (this stays 0): testing showed this specific
// module never completes a measurement after a D3 reset pulse, even with a 2 second settle
// delay, so treat D3-based reset as unproven/unreliable on this unit until retested with
// different hardware.
#define USE_XSHUT_ON_D3 0

#if USE_XSHUT_ON_D3
const uint8_t PIN_VL53L0X_XSHUT = 3;
const uint8_t SENSOR_I2C_ADDRESS = 0x2A; // reassigned away from the power-on default below
// Longer settle time after the reset pulse than a permanently-high XSHUT ever needed; a fresh
// LOW->HIGH cycle previously reproduced the exact "always TIMEOUT" failure this project started
// with, before XSHUT was tied straight to +5V. Testing whether a longer wait avoids that.
const uint16_t XSHUT_RESET_SETTLE_MS = 2000;
#endif

const uint8_t SENSOR_ADDRESS = 0x29;
const uint16_t SENSOR_TIMEOUT_MS = 500;
const uint16_t MEASUREMENT_PERIOD_MS = 100;
const uint32_t MEASUREMENT_BUDGET_US = 50000;
const uint16_t XSHUT_WAKE_DELAY_MS = 100;
const uint16_t PROBE_RETRY_DELAY_MS = 50;
const uint16_t SENSOR_WAKE_TIMEOUT_MS = 2000;
// VL53L0X datasheet minimum range is ~30 mm; closer targets suffer optical crosstalk.
const uint16_t MIN_RELIABLE_DISTANCE_MM = 30;
// Typical rated max range under good conditions; the sensor's "no target" placeholder
// value (~8190 mm) and anything else implausibly large falls above this.
const uint16_t MAX_PLAUSIBLE_DISTANCE_MM = 2000;

VL53L0X sensor;

bool readDistanceAndStatus(uint16_t* distanceMm, uint8_t* statusCode);
bool canReachSensor(uint8_t address);
bool waitForSensor(uint8_t address, uint16_t* elapsedMs);

void setup()
{
  Serial.begin(9600);
  Serial.println(F("VL53L0X test"));

#if USE_XSHUT_ON_D3
  pinMode(PIN_VL53L0X_XSHUT, OUTPUT);
  digitalWrite(PIN_VL53L0X_XSHUT, LOW);
  delay(10);
  digitalWrite(PIN_VL53L0X_XSHUT, HIGH);
  delay(XSHUT_RESET_SETTLE_MS);
#endif
  delay(XSHUT_WAKE_DELAY_MS);

  Wire.begin();
  // 1000us was too tight for back-to-back register reads and caused false bus timeouts (seen as RAW=0xFF).
  Wire.setWireTimeout(25000, true);

  uint16_t wakeElapsedMs = 0;
  if (!waitForSensor(SENSOR_ADDRESS, &wakeElapsedMs))
  {
    Serial.println(F("Sensor not found at 0x29."));
    Serial.print(F("Waited "));
    Serial.print(wakeElapsedMs);
    Serial.println(F(" ms after power-up."));
    Serial.println(F("Check VIN, GND, A4/SDA, A5/SCL, and XSHUT wiring."));
    while (true)
    {
      delay(1000);
    }
  }

  sensor.setTimeout(SENSOR_TIMEOUT_MS);
  if (!sensor.init())
  {
    Serial.println(F("Sensor init failed."));
    while (true)
    {
      delay(1000);
    }
  }

#if USE_XSHUT_ON_D3
  sensor.setAddress(SENSOR_I2C_ADDRESS);

  // Confirms the reassignment separately from ranging, since D3 reset already causes TIMEOUT
  // on this module regardless of address.
  if (canReachSensor(SENSOR_I2C_ADDRESS))
  {
    Serial.println(F("Address changed to 0x2A."));
  }
  else
  {
    Serial.println(F("Address change to 0x2A failed; sensor not responding there."));
  }
#endif

  sensor.setMeasurementTimingBudget(MEASUREMENT_BUDGET_US);

  Serial.println(F("Reading single-shot distance in mm and cm."));
}

void loop()
{
  uint16_t distanceMm = 0;
  uint8_t statusCode = 0;

  if (!readDistanceAndStatus(&distanceMm, &statusCode))
  {
    Serial.println(F("TIMEOUT"));
    delay(MEASUREMENT_PERIOD_MS);
    return;
  }

  // The sensor's own status nibble (statusCode) has proven unreliable on this module: clearly
  // valid, smoothly-tracking distances have shown up under several different codes. The message
  // below is chosen from the distance value itself; statusCode is still read inside
  // readDistanceAndStatus() because that register read is part of a sequence that must stay
  // intact (see the comment there).
  if (distanceMm < MIN_RELIABLE_DISTANCE_MM)
  {
    Serial.println(F("Too close (< 3 cm, unreliable)"));
  }
  else if (distanceMm >= MAX_PLAUSIBLE_DISTANCE_MM)
  {
    Serial.println(F("Out of range"));
  }
  else
  {
    Serial.print(F("Distance: "));
    Serial.print(distanceMm);
    Serial.print(F(" mm ("));
    Serial.print(distanceMm / 10.0f, 1);
    Serial.println(F(" cm)"));
  }

  delay(MEASUREMENT_PERIOD_MS);
}

// Reimplements VL53L0X::readRangeSingleMillimeters() using only the public register API so
// RESULT_RANGE_STATUS can be captured before the library's internal interrupt-clear step.
// Keep this exact register sequence, including the RESULT_RANGE_STATUS byte read: relying on
// the library's own readRangeSingleMillimeters(), or skipping straight to the 16-bit distance
// read, both reproducibly broke measurements on this module (every reading became "Out of range").
bool readDistanceAndStatus(uint16_t* distanceMm, uint8_t* statusCode)
{
  sensor.writeReg(0x80, 0x01);
  sensor.writeReg(0xFF, 0x01);
  sensor.writeReg(0x00, 0x00);
  const uint8_t stopVariable = sensor.readReg(0x91);
  sensor.writeReg(0x00, 0x01);
  sensor.writeReg(0xFF, 0x00);
  sensor.writeReg(0x80, 0x00);

  sensor.writeReg(0x80, 0x01);
  sensor.writeReg(0xFF, 0x01);
  sensor.writeReg(0x00, 0x00);
  sensor.writeReg(0x91, stopVariable);
  sensor.writeReg(0x00, 0x01);
  sensor.writeReg(0xFF, 0x00);
  sensor.writeReg(0x80, 0x00);

  sensor.writeReg(VL53L0X::SYSRANGE_START, 0x01);

  unsigned long startTime = millis();
  while (sensor.readReg(VL53L0X::SYSRANGE_START) & 0x01)
  {
    if (millis() - startTime > SENSOR_TIMEOUT_MS)
    {
      return false;
    }
  }

  startTime = millis();
  while ((sensor.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
    if (millis() - startTime > SENSOR_TIMEOUT_MS)
    {
      return false;
    }
  }

  *statusCode = (sensor.readReg(VL53L0X::RESULT_RANGE_STATUS) >> 3) & 0x0F;
  *distanceMm = sensor.readReg16Bit((uint8_t)(VL53L0X::RESULT_RANGE_STATUS + 10));

  sensor.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);

  return true;
}

bool canReachSensor(uint8_t address)
{
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

bool waitForSensor(uint8_t address, uint16_t* elapsedMs)
{
  *elapsedMs = 0;

  while (*elapsedMs <= SENSOR_WAKE_TIMEOUT_MS)
  {
    if (canReachSensor(address))
    {
      return true;
    }

    delay(PROBE_RETRY_DELAY_MS);
    *elapsedMs += PROBE_RETRY_DELAY_MS;
  }

  return false;
}
