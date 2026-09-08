#include <VL53L0X.h>

// The VL53L0X is a time-of-flight (ToF) distance sensor: it sends invisible
// infrared light and measures how long the reflection takes to return. The
// Pololu library below handles its I2C register protocol; this tab turns its
// millimetre readings into safe, filtered centimetre values for auto driving.
VL53L0X distanceTof;

// Distance is the most recently accepted, filtered control distance in cm.
// 20-motor.ino reads it through getDistanceReading(); values farther than the
// auto-control range are intentionally capped at AUTO_DISTANCE_MAX_SPEED.
uint8_t Distance = 0;

// A three-reading median removes isolated bad reflections without averaging a
// sudden close obstacle into a dangerously large distance. The buffer contains
// centimetres rather than millimetres because the motor thresholds use cm.
uint16_t distanceBuffer[AUTO_SAMPLES_FOR_MEDIAN];
uint8_t bufferIndex = 0;
bool bufferFilled = false;

// These flags distinguish a sensor that was never detected, one deliberately
// not ranging while auto mode is off, and one that has failed while ranging.
// Keeping those states separate lets Play/Pause retry a real fault safely.
bool distanceTofDetected = false;
bool distanceTofFaultLatched = false;
unsigned long lastTofReadMs = 0;
unsigned long lastGoodTofReadMs = 0;
unsigned long tofRangingStartedMs = 0;
bool tofRangingActive = false;

#if DEBUG_DISTANCE_SENSOR
bool tofDebugSamplingRequested = false;
#endif

// Forget old readings whenever ranging starts or restarts. Otherwise a value
// measured before auto mode was enabled could affect the first motor decision.
void resetDistanceFilter() {
  bufferIndex = 0;
  bufferFilled = false;
  Distance = 0;
}

// Sort the small temporary copy and return its middle value. This is a median,
// not an average: one implausibly high or low measurement cannot dominate the
// result. The caller's source buffer stays in time order for the next update.
uint16_t medianFromUnsortedSamples(uint16_t* values, uint8_t size) {
  if (values == nullptr || size == 0) return 0;

  for (uint8_t i = 0; i < (uint8_t)(size - 1); ++i) {
    for (uint8_t j = (uint8_t)(i + 1); j < size; ++j) {
      if (values[j] < values[i]) {
        const uint16_t swap = values[i];
        values[i] = values[j];
        values[j] = swap;
      }
    }
  }
  return values[size / 2];
}

// Add one raw centimetre sample to the circular buffer. During startup the
// median uses only the samples received so far; afterward it always uses
// AUTO_SAMPLES_FOR_MEDIAN samples.
uint16_t pushDistanceSampleAndGetMedian(uint16_t raw) {
  distanceBuffer[bufferIndex] = raw;
  bufferIndex = (bufferIndex + 1) % AUTO_SAMPLES_FOR_MEDIAN;
  if (bufferIndex == 0) bufferFilled = true;

  uint8_t size = bufferFilled ? AUTO_SAMPLES_FOR_MEDIAN : bufferIndex;
  uint16_t temp[AUTO_SAMPLES_FOR_MEDIAN];
  for (uint8_t i = 0; i < size; ++i) temp[i] = distanceBuffer[i];
  return medianFromUnsortedSamples(temp, size);
}

// A dead distance sensor must not leave auto driving active with no obstacle
// protection. Latch the fault, stop ranging, stop the locomotive, notify the
// user, and optionally record the event. The latch also prevents repeatedly
// playing the warning while the sensor remains unavailable.
void handleDistanceSensorFault() {
  if (distanceTofFaultLatched) return;

  distanceTofFaultLatched = true;
  distanceTof.stopContinuous();
  distanceTofDetected = false;
  tofRangingActive = false;
  Distance = 0;
  pendingMotorStopReason = F("auto: no response from distance sensor");
  exitAutoDistanceMode();
  stopAndResetStepSelection();
  SetRGBColor(RgbColor::Red);
  playPattern(pattern_batteryWarn);
  DBGLN_DISTANCE_SENSOR(F("VL53L0X fault: auto-distance mode disabled until sensor is reinitialized"));
  #if ENABLE_EEPROM_LOGGING
  logBatteryEvent(EEPROM_EVENT_TOF_FAULT, batteryVoltage);
  #endif
}

// Return the current safe control distance in centimetres. A fresh sensor read
// occurs only every tofReadEveryMs; callers in between receive the last
// filtered value so I2C traffic does not block the cooperative main loop.
int getDistanceReading() {
  if (!distanceTofDetected || !tofRangingActive) {
    handleDistanceSensorFault();
    return AUTO_DISTANCE_INVALID;
  }

  const unsigned long now = millis();
  if (now - lastTofReadMs < tofReadEveryMs) {
    if (!bufferFilled && bufferIndex == 0 && lastGoodTofReadMs == 0) return AUTO_DISTANCE_PENDING;
    return Distance;
  }
  lastTofReadMs = now;

  // The sensor runs continuously in hardware. This call obtains the completed
  // measurement; it does not start a new 30 ms measurement each time.
  const uint16_t rawMm = distanceTof.readRangeContinuousMillimeters();
  // rawMm == 0 legitimately happens when an obstacle is right at the sensor face (<1 cm); do not
  // treat it as an invalid sample and let it latch a fault. Report it as 1 cm ("stop, obstacle
  // touching") so the auto-distance controller behaves the same as any other very-close reading.
  if (!distanceTof.timeoutOccurred() && rawMm == 0) {
    distanceTofFaultLatched = false;
    lastGoodTofReadMs = now;
    Distance = 1;
    DBGLN_DISTANCE_SENSOR(F("VL53L0X raw=0 mm (obstacle touching); reporting 1 cm"));
    return Distance;
  }
  if (distanceTof.timeoutOccurred() || rawMm == 65535) {
    DBGLN_DISTANCE_SENSOR(F("VL53L0X invalid sample"));
    if (lastGoodTofReadMs == 0) {
      if (now - tofRangingStartedMs > tofStartupGraceMs) {
        handleDistanceSensorFault();
      }
      return AUTO_DISTANCE_INVALID;
    }
    if (now - lastGoodTofReadMs <= tofFailureGraceMs) return Distance;
    handleDistanceSensorFault();
    return AUTO_DISTANCE_INVALID;
  }

  distanceTofFaultLatched = false;
  lastGoodTofReadMs = now;

  // Convert the hardware's mm unit to the cm unit used by AUTO_DISTANCE_*,
  // then use the median to reject a one-off optical reflection.
  const uint16_t medianCm = pushDistanceSampleAndGetMedian(rawMm / 10U);

  // The motor policy needs no distinction beyond its full-speed distance.
  // Capping here also keeps the shared one-byte Distance value in range.
  const uint8_t controlCm = medianCm < 1
    ? 1
    : (medianCm > AUTO_DISTANCE_MAX_SPEED ? AUTO_DISTANCE_MAX_SPEED : (uint8_t)medianCm);

  DBG_DISTANCE_SENSOR(F("VL53L0X raw="));
  DBG_DISTANCE_SENSOR(rawMm);
  DBG_DISTANCE_SENSOR(F(" mm median="));
  DBG_DISTANCE_SENSOR(medianCm);
  DBGLN_DISTANCE_SENSOR(F(" cm"));

  Distance = controlCm;
  return controlCm;
}

// Both the VL53L0X and colour sensor power up at I2C address 0x29. XSHUT is
// wired to pinDistanceSensorXSHUT (A3) so this sensor can be reset, moved to
// distanceSensorAddress (0x2A), and then coexist with the colour sensor.
void initDistanceSensorHardware() {
  pinMode(pinDistanceSensorXSHUT, OUTPUT);
  digitalWrite(pinDistanceSensorXSHUT, LOW);
  delay(10);  // Let the sensor fully enter hardware shutdown.
  digitalWrite(pinDistanceSensorXSHUT, HIGH);
  delay(10);  // Let its oscillator and I2C interface become ready.
  startDistanceSensorRanging();
}

// Configure the sensor once after XSHUT releases it. Every value comes from
// config.h so the hardware policy is visible in one builder-facing location.
bool startDistanceSensorRanging() {
  // Abort a stalled I2C range-read rather than freezing the main loop.
  distanceTof.setTimeout(distanceTofTimeoutMs);

  // The sensor is awake at 0x29 after XSHUT; this writes its new 0x2A address
  // before distanceTof.init() performs further register configuration.
  distanceTof.setAddress(distanceSensorAddress);
  if (!distanceTof.init()) {
    distanceTofDetected = false;
    DBGLN_DISTANCE_SENSOR(F("VL53L0X not detected on I2C"));
    return false;
  }

  // A return-signal rate is the minimum strength of the reflected light that
  // counts as a valid result. 1.0 Mcps rejects weak, long-range reflections
  // while remaining reliable for this train's sub-50 cm obstacle range.
  if (!distanceTof.setSignalRateLimit(distanceTofSignalRateLimitMcps)) {
    distanceTofDetected = false;
    DBGLN_DISTANCE_SENSOR(F("VL53L0X signal-rate limit rejected"));
    return false;
  }

  // The timing budget is how long the sensor may spend collecting one optical
  // measurement. It must not exceed the 30 ms continuous period: a longer
  // budget improves precision but cannot deliver a new result every period.
  if (!distanceTof.setMeasurementTimingBudget(distanceTofTimingBudgetUs)) {
    distanceTofDetected = false;
    DBGLN_DISTANCE_SENSOR(F("VL53L0X timing budget rejected"));
    return false;
  }

  distanceTofDetected = true;
  distanceTofFaultLatched = false;
  resetDistanceFilter();
  lastGoodTofReadMs = 0;
  lastTofReadMs = 0;
  #if DEBUG_DISTANCE_SENSOR
  const bool rangingRequired = AutoDistanceOnOff || tofDebugSamplingRequested;
  #else
  const bool rangingRequired = AutoDistanceOnOff;
  #endif
  if (rangingRequired) {
    // Timed continuous mode starts a new hardware measurement every configured
    // period. This is more responsive than one-shot reads and avoids a delay in
    // the main loop while the laser measurement is in progress.
    distanceTof.startContinuous(distanceTofContinuousPeriodMs);
    tofRangingStartedMs = millis();
    tofRangingActive = true;
  } else {
    tofRangingActive = false;
  }
  DBGLN_DISTANCE_SENSOR(F("VL53L0X ready"));
  return true;
}

// XSHUT resets the physical sensor to its default I2C address, 0x29, while the
// existing object retains its configured 0x2A address. Reconstructing the local
// Pololu driver changes that in-memory address state without I2C traffic; the
// normal initializer safely returns it to configured 0x2A and restarts range operation.
bool recoverDistanceSensorAfterXshut() {
  distanceTof = VL53L0X();
  return startDistanceSensorRanging();
}

// Start or stop optical ranging as auto mode changes. Stopping it when unused
// avoids needless I2C work; restarting also clears old filter data so a prior
// obstacle cannot affect the newly enabled auto mode.
void setDistanceSensorRangingActive(bool active) {
  // If a previous fault latched the sensor off, try to re-initialize it on a fresh activation
  // request (typically Play/Pause re-arming auto mode). Without this, distanceTofDetected stays
  // false forever after the first fault and auto mode can never restart until power-cycle.
  if (active && !distanceTofDetected && distanceTofFaultLatched) {
    DBGLN_DISTANCE_SENSOR(F("VL53L0X was latched off; retrying init"));
    startDistanceSensorRanging();
    return;
  }
  if (!distanceTofDetected) return;
  #if DEBUG_DISTANCE_SENSOR
  const bool rangingRequired = active || tofDebugSamplingRequested;
  #else
  const bool rangingRequired = active;
  #endif

  if (rangingRequired) {
    resetDistanceFilter();
    lastGoodTofReadMs = 0;
    lastTofReadMs = 0;
    if (!tofRangingActive) {
      distanceTof.startContinuous(distanceTofContinuousPeriodMs);
      tofRangingStartedMs = millis();
      tofRangingActive = true;
      DBGLN_DISTANCE_SENSOR(F("VL53L0X ranging started"));
    }
  } else if (tofRangingActive) {
    distanceTof.stopContinuous();
    tofRangingActive = false;
    DBGLN_DISTANCE_SENSOR(F("VL53L0X ranging stopped"));
  }
}

#if DEBUG_DISTANCE_SENSOR
// Debug sampling permits Serial distance diagnostics without enabling auto
// driving. It shares the normal activation function so only one place controls
// the sensor's continuous-ranging state.
void setDistanceSensorDebugSamplingEnabled(bool enabled) {
  if (tofDebugSamplingRequested == enabled) return;
  tofDebugSamplingRequested = enabled;
  setDistanceSensorRangingActive(AutoDistanceOnOff);
}

// Called from the main loop. The normal interval limiter in getDistanceReading()
// still applies, so debug output cannot read the sensor faster than configured.
void updateDistanceSensorDebugSampling() {
  if (tofDebugSamplingRequested && !AutoDistanceOnOff && !distanceTofFaultLatched) {
    getDistanceReading();
  }
}
#else
void setDistanceSensorDebugSamplingEnabled(bool) {}
void updateDistanceSensorDebugSampling() {}
#endif
