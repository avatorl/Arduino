#include <VL53L1X_ULD.h>

// This backend preserves the centimetre-based interface used by the motor and
// power modules while the ULD driver handles VL53L1X register communication.
VL53L1X_ULD distanceTof;

uint8_t Distance = 0;
uint16_t distanceBuffer[AUTO_SAMPLES_FOR_MEDIAN];
uint8_t bufferIndex = 0;
bool bufferFilled = false;

bool distanceTofDetected = false;
bool distanceTofFaultLatched = false;
bool distanceTofMeasurementUnreliable = false;
unsigned long lastTofReadMs = 0;
unsigned long lastGoodTofReadMs = 0;
bool tofRangingActive = false;

#if DEBUG_DISTANCE_SENSOR
bool tofDebugSamplingRequested = false;
#endif

void resetDistanceFilter() {
  bufferIndex = 0;
  bufferFilled = false;
  Distance = 0;
}

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

uint16_t pushDistanceSampleAndGetMedian(uint16_t raw) {
  distanceBuffer[bufferIndex] = raw;
  bufferIndex = (bufferIndex + 1) % AUTO_SAMPLES_FOR_MEDIAN;
  if (bufferIndex == 0) bufferFilled = true;

  const uint8_t size = bufferFilled ? AUTO_SAMPLES_FOR_MEDIAN : bufferIndex;
  uint16_t temp[AUTO_SAMPLES_FOR_MEDIAN];
  for (uint8_t i = 0; i < size; ++i) temp[i] = distanceBuffer[i];
  return medianFromUnsortedSamples(temp, size);
}

void handleDistanceSensorFault() {
  if (distanceTofFaultLatched) return;

  distanceTofFaultLatched = true;
  distanceTof.StopRanging();
  distanceTofDetected = false;
  tofRangingActive = false;
  Distance = 0;
  pendingMotorStopReason = F("auto: no response from distance sensor");
  exitAutoDistanceMode();
  stopAndResetStepSelection();
  SetRGBColor(RgbColor::Red);
  playPattern(pattern_batteryWarn);
  DBGLN_DISTANCE_SENSOR(F("VL53L1X fault: auto-distance mode disabled until sensor is reinitialized"));
  #if ENABLE_EEPROM_LOGGING
  logBatteryEvent(EEPROM_EVENT_TOF_FAULT, batteryVoltage);
  #endif
}

const __FlashStringHelper* distanceRangeStatusLabel(uint8_t status) {
  switch (status) {
    case RangeValid: return F("RangeValid");
    case SigmaFail: return F("SigmaFail");
    case SignalFail: return F("SignalFail");
    case MinRangeFail: return F("MinRangeFail");
    case PhaseOutOfLimit: return F("PhaseOutOfLimit");
    case HardwareFail: return F("HardwareFail");
    case RangeValidNoWrapCheck: return F("RangeValidNoWrapCheck");
    case WrapTargetFail: return F("WrapTargetFail");
    default: return F("Unknown");
  }
}

// Reads only completed continuous-ranging results, keeping the main loop
// non-blocking. The first result may be pending; later rejected samples retain
// the last safe distance and do not disable auto-distance mode.
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

  uint8_t dataReady = false;
  if (distanceTof.CheckForDataReady(&dataReady) != VL53L1_ERROR_NONE) {
    distanceTofMeasurementUnreliable = true;
    DBGLN_DISTANCE_SENSOR(F("VL53L1X data-ready check failed"));
  } else if (!dataReady) {
    return lastGoodTofReadMs == 0 ? AUTO_DISTANCE_PENDING : Distance;
  } else {
    VL53L1X_Result_t result;
    const VL53L1_Error resultStatus = distanceTof.GetResult(&result);
    const VL53L1_Error clearStatus = distanceTof.ClearInterrupt();
    if (resultStatus == VL53L1_ERROR_NONE &&
        clearStatus == VL53L1_ERROR_NONE &&
        (result.Status == RangeValid || result.Status == RangeValidNoWrapCheck)) {
      distanceTofFaultLatched = false;
      distanceTofMeasurementUnreliable = false;
      lastGoodTofReadMs = now;

      uint16_t rawMm = result.Distance;
      if (rawMm < distanceTofNearRangeCorrectionLimitMm) {
        rawMm += distanceTofNearRangeOffsetMm;
      }
      const uint16_t medianCm = pushDistanceSampleAndGetMedian(rawMm / 10U);
      const uint8_t controlCm = medianCm < 1
        ? 1
        : (medianCm > AUTO_DISTANCE_MAX_SPEED ? AUTO_DISTANCE_MAX_SPEED : (uint8_t)medianCm);
      Distance = controlCm;

      DBG_DISTANCE_SENSOR(F("VL53L1X raw="));
      DBG_DISTANCE_SENSOR(rawMm);
      DBG_DISTANCE_SENSOR(F(" mm median="));
      DBG_DISTANCE_SENSOR(medianCm);
      DBGLN_DISTANCE_SENSOR(F(" cm"));
      return controlCm;
    }
    distanceTofMeasurementUnreliable = true;
    if (resultStatus == VL53L1_ERROR_NONE && clearStatus == VL53L1_ERROR_NONE) {
      DBG_DISTANCE_SENSOR(F("VL53L1X rejected sample, range status="));
      DBG_DISTANCE_SENSOR(result.Status);
      DBG_DISTANCE_SENSOR(F(" ("));
      DBG_DISTANCE_SENSOR(distanceRangeStatusLabel(result.Status));
      DBGLN_DISTANCE_SENSOR(F(")"));
    } else {
      DBGLN_DISTANCE_SENSOR(F("VL53L1X result read failed"));
    }
  }

  // A range-status rejection is an optical-quality result, not proof that the
  // sensor has failed. Continue auto mode using the latest filtered distance;
  // the next valid result clears the unreliable marker and refreshes it.
  return lastGoodTofReadMs == 0 ? AUTO_DISTANCE_PENDING : Distance;
}

// The VL53L1X and TCS34725 both power up at 0x29. XSHUT resets this sensor,
// after which it is moved to 0x2A before the color sensor is initialized.
void initDistanceSensorHardware() {
  pinMode(pinDistanceSensorXSHUT, OUTPUT);
  digitalWrite(pinDistanceSensorXSHUT, LOW);
  delay(10);
  digitalWrite(pinDistanceSensorXSHUT, HIGH);
  delay(10);
  startDistanceSensorRanging();
}

bool configureDistanceSensor() {
  // The ULD API accepts the 8-bit address byte: 0x52 is default 7-bit 0x29.
  if (distanceTof.Begin(distanceSensorDefaultAddress8Bit) != VL53L1_ERROR_NONE ||
      distanceTof.SetI2CAddress(distanceSensorAddress8Bit) != VL53L1_ERROR_NONE ||
      distanceTof.SetOffsetInMm(distanceTofOffsetMm) != VL53L1_ERROR_NONE ||
      distanceTof.SetXTalk(distanceTofXtalkCps) != VL53L1_ERROR_NONE ||
      distanceTof.SetDistanceMode(Long) != VL53L1_ERROR_NONE ||
      distanceTof.SetTimingBudgetInMs(distanceTofTimingBudgetMs) != VL53L1_ERROR_NONE ||
      distanceTof.SetInterMeasurementInMs(distanceTofInterMeasurementMs) != VL53L1_ERROR_NONE ||
      distanceTof.SetROI(distanceTofRoiWidthSpads, distanceTofRoiHeightSpads) != VL53L1_ERROR_NONE ||
      distanceTof.SetROICenter(distanceTofRoiCenterSpad) != VL53L1_ERROR_NONE) {
    distanceTofDetected = false;
    DBGLN_DISTANCE_SENSOR(F("VL53L1X configuration failed"));
    return false;
  }
  #if DEBUG_DISTANCE_SENSOR
  uint8_t reportedFactoryRoiCenter = 0;
  if (distanceTof.GetFactoryROICenter(&reportedFactoryRoiCenter) == VL53L1_ERROR_NONE) {
    DBG_DISTANCE_SENSOR(F("VL53L1X factory ROI center (sensor/configured)="));
    DBG_DISTANCE_SENSOR(reportedFactoryRoiCenter);
    DBG_DISTANCE_SENSOR(F("/"));
    DBG_DISTANCE_SENSOR(distanceTofFactoryRoiCenterSpad);
    DBG_DISTANCE_SENSOR(F(" configured ROI center="));
    DBGLN_DISTANCE_SENSOR(distanceTofRoiCenterSpad);
  } else {
    DBGLN_DISTANCE_SENSOR(F("VL53L1X factory ROI center read failed"));
  }
  #endif
  return true;
}

bool startDistanceSensorRanging() {
  if (!configureDistanceSensor()) return false;

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
    if (distanceTof.StartRanging() != VL53L1_ERROR_NONE) {
      distanceTofDetected = false;
      DBGLN_DISTANCE_SENSOR(F("VL53L1X ranging start failed"));
      return false;
    }
    tofRangingActive = true;
  } else {
    tofRangingActive = false;
  }
  DBGLN_DISTANCE_SENSOR(F("VL53L1X ready"));
  return true;
}

bool recoverDistanceSensorAfterXshut() {
  distanceTof = VL53L1X_ULD();
  return startDistanceSensorRanging();
}

void setDistanceSensorRangingActive(bool active) {
  if (active && !distanceTofDetected && distanceTofFaultLatched) {
    DBGLN_DISTANCE_SENSOR(F("VL53L1X was latched off; retrying init"));
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
      if (distanceTof.StartRanging() != VL53L1_ERROR_NONE) {
        handleDistanceSensorFault();
        return;
      }
      tofRangingActive = true;
      DBGLN_DISTANCE_SENSOR(F("VL53L1X ranging started"));
    }
  } else if (tofRangingActive) {
    distanceTof.StopRanging();
    tofRangingActive = false;
    DBGLN_DISTANCE_SENSOR(F("VL53L1X ranging stopped"));
  }
}

#if DEBUG_DISTANCE_SENSOR
void setDistanceSensorDebugSamplingEnabled(bool enabled) {
  if (tofDebugSamplingRequested == enabled) return;
  tofDebugSamplingRequested = enabled;
  setDistanceSensorRangingActive(AutoDistanceOnOff);
}

void updateDistanceSensorDebugSampling() {
  if (tofDebugSamplingRequested && !AutoDistanceOnOff && !distanceTofFaultLatched) {
    getDistanceReading();
  }
}
#else
void setDistanceSensorDebugSamplingEnabled(bool) {}
void updateDistanceSensorDebugSampling() {}
#endif
