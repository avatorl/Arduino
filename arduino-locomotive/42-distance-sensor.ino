#include <VL53L1X_ULD.h>

// VL53L1X distance-sensor backend
// --------------------------------
// The ST "Ultra Lite Driver" (ULD) library performs the low-level register
// access. This file owns the higher-level policy needed by the locomotive:
//
// - initialize the sensor before the TCS34725, because both start at I2C 0x29;
// - move the VL53L1X to 0x2A, then apply the calibrated short-range profile;
// - poll continuous ranging without blocking loop();
// - median-filter valid physical distances;
// - distinguish fresh, cached, pending, and hardware-invalid readings;
// - confirm repeated no-target SignalFail results before declaring a clear path.
//
// All I2C addresses passed to VL53L1X_ULD are 8-bit address bytes, following
// ST's API convention. Arduino Wire normally displays the equivalent 7-bit
// addresses (0x29 default and 0x2A assigned).
static VL53L1X_ULD distanceTof;

// Distance is the most recent usable control value in centimetres. It is
// capped at AUTO_DISTANCE_MAX_SPEED because the motor controller does not need
// to distinguish larger clear-path distances.
uint8_t Distance = 0;

// The ring buffer stores physical centimetre samples before the control-value
// cap. Keeping chronological insertion order lets each new sample replace the
// oldest one. The temporary copy used during median calculation is only
// AUTO_SAMPLES_FOR_MEDIAN * 2 bytes on the stack.
static uint16_t distanceBuffer[AUTO_SAMPLES_FOR_MEDIAN];
static uint8_t bufferIndex = 0;
static bool bufferFilled = false;

// Lifecycle state shared with the power and motor modules.
bool distanceTofDetected = false;
bool distanceTofFaultLatched = false;
static bool hasUsableDistance = false;
static uint8_t consecutiveSignalFailCount = 0;
static unsigned long lastTofReadMs = 0;
static bool tofRangingActive = false;

// Clear only the physical-sample history. A confirmed no-target result uses
// this when the old obstacle samples no longer describe the clear scene.
static void resetDistanceSampleBuffer() {
  bufferIndex = 0;
  bufferFilled = false;
}

// Start a new ranging session with no usable result or no-target streak.
static void resetDistanceFilter() {
  resetDistanceSampleBuffer();
  Distance = 0;
  hasUsableDistance = false;
  consecutiveSignalFailCount = 0;
}

// Sort a tiny local copy and return its middle value. AUTO_SAMPLES_FOR_MEDIAN
// is compile-time constrained to a non-zero odd number, so there is one exact
// middle element and no averaging or floating-point work is required.
static uint16_t medianFromUnsortedSamples(uint16_t* values, uint8_t size) {
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

// Add one valid physical sample to the ring and calculate the median over all
// samples collected so far. Startup therefore reacts to the first valid range
// immediately; full glitch rejection begins once the ring is full.
static uint16_t pushDistanceSampleAndGetMedian(uint16_t distanceCm) {
  distanceBuffer[bufferIndex] = distanceCm;
  bufferIndex = (bufferIndex + 1) % AUTO_SAMPLES_FOR_MEDIAN;
  if (bufferIndex == 0) bufferFilled = true;

  const uint8_t size = bufferFilled ? AUTO_SAMPLES_FOR_MEDIAN : bufferIndex;
  uint16_t temp[AUTO_SAMPLES_FOR_MEDIAN];
  for (uint8_t i = 0; i < size; ++i) temp[i] = distanceBuffer[i];
  return medianFromUnsortedSamples(temp, size);
}

// Latch a true sensor/lifecycle fault once. Optical range rejections are not
// routed here: they are measurement-quality results and use cached data.
static void handleDistanceSensorFault() {
  if (distanceTofFaultLatched) return;

  distanceTofFaultLatched = true;
  if (tofRangingActive) {
    // StopRanging ends continuous measurements. Failure is not retried here
    // because this path already disables the sensor and makes the train safe.
    distanceTof.StopRanging();
  }
  distanceTofDetected = false;
  tofRangingActive = false;
  resetDistanceFilter();
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

#if DEBUG_DISTANCE_SENSOR
// Translate the ULD range-status byte into readable serial diagnostics.
static const __FlashStringHelper* distanceRangeStatusLabel(uint8_t status) {
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
#endif

static inline DistanceReading pendingDistanceReading() {
  return { 0, DistanceReadingStatus::Pending };
}

static inline DistanceReading cachedDistanceReading() {
  return { Distance, DistanceReadingStatus::Cached };
}

// With no reflecting target in front of the train, this installation reports
// SignalFail rather than a large valid distance. Require several consecutive
// completed results before treating it as clear; a one-off bad result keeps the
// last real distance and cannot release an obstacle stop.
static DistanceReading confirmedClearPathReading() {
  if (!hasUsableDistance || Distance != AUTO_DISTANCE_MAX_SPEED) {
    resetDistanceSampleBuffer();
    Distance = AUTO_DISTANCE_MAX_SPEED;
    hasUsableDistance = true;
    DBGLN_DISTANCE_SENSOR(F("VL53L1X confirmed clear path from SignalFail"));
  }
  return { Distance, DistanceReadingStatus::Valid };
}

// Poll one non-blocking continuous-ranging result.
//
// Status contract:
// - Pending: no usable result exists yet.
// - Valid: a fresh physical range or newly confirmed clear path is available.
// - Cached: no fresh usable result; keep the current motor command unchanged.
// - Invalid: the sensor is unavailable, so the motor controller stops safely.
DistanceReading getDistanceReading() {
  if (!distanceTofDetected || !tofRangingActive) {
    handleDistanceSensorFault();
    return { 0, DistanceReadingStatus::Invalid };
  }

  const unsigned long now = millis();
  if (now - lastTofReadMs < tofReadEveryMs) {
    return hasUsableDistance ? cachedDistanceReading() : pendingDistanceReading();
  }
  lastTofReadMs = now;

  // CheckForDataReady polls the sensor's data-ready register. A normal "not
  // ready yet" result is not an error and does not break a SignalFail streak,
  // because no new completed measurement exists to evaluate.
  uint8_t dataReady = false;
  if (distanceTof.CheckForDataReady(&dataReady) != VL53L1_ERROR_NONE) {
    consecutiveSignalFailCount = 0;
    DBGLN_DISTANCE_SENSOR(F("VL53L1X data-ready check failed"));
  } else if (!dataReady) {
    return hasUsableDistance ? cachedDistanceReading() : pendingDistanceReading();
  } else {
    // GetResult reads distance, status, signal, ambient, and SPAD information
    // in one transaction. ClearInterrupt rearms the next continuous result and
    // must be attempted even when GetResult reports an I2C error.
    VL53L1X_Result_t result;
    const VL53L1_Error resultStatus = distanceTof.GetResult(&result);
    const VL53L1_Error clearStatus = distanceTof.ClearInterrupt();
    if (resultStatus == VL53L1_ERROR_NONE &&
        clearStatus == VL53L1_ERROR_NONE &&
        (result.Status == RangeValid || result.Status == RangeValidNoWrapCheck)) {
      consecutiveSignalFailCount = 0;

      // Apply the installation-specific near-range correction before converting
      // millimetres to centimetres and median filtering.
      uint16_t rawMm = result.Distance;
      if (rawMm < distanceTofNearRangeCorrectionLimitMm) {
        rawMm += distanceTofNearRangeOffsetMm;
      }
      const uint16_t medianCm = pushDistanceSampleAndGetMedian(rawMm / 10U);
      const uint8_t controlCm = medianCm < 1
        ? 1
        : (medianCm > AUTO_DISTANCE_MAX_SPEED ? AUTO_DISTANCE_MAX_SPEED : (uint8_t)medianCm);
      Distance = controlCm;
      hasUsableDistance = true;

      DBG_DISTANCE_SENSOR(F("VL53L1X raw="));
      DBG_DISTANCE_SENSOR(rawMm);
      DBG_DISTANCE_SENSOR(F(" mm median="));
      DBG_DISTANCE_SENSOR(medianCm);
      DBGLN_DISTANCE_SENSOR(F(" cm"));
      return { controlCm, DistanceReadingStatus::Valid };
    }

    if (resultStatus == VL53L1_ERROR_NONE && clearStatus == VL53L1_ERROR_NONE) {
      if (result.Status == SignalFail) {
        if (consecutiveSignalFailCount < distanceTofSignalFailClearCount) {
          ++consecutiveSignalFailCount;
        }
        if (consecutiveSignalFailCount >= distanceTofSignalFailClearCount) {
          // Keep returning Valid while the view remains confirmed clear so the
          // motor controller can continue its timed ramp from minimum to
          // maximum auto speed. The helper logs only the first transition.
          return confirmedClearPathReading();
        }
      } else {
        consecutiveSignalFailCount = 0;
      }
      DBG_DISTANCE_SENSOR(F("VL53L1X rejected sample, range status="));
      DBG_DISTANCE_SENSOR(result.Status);
      DBG_DISTANCE_SENSOR(F(" ("));
      DBG_DISTANCE_SENSOR(distanceRangeStatusLabel(result.Status));
      DBGLN_DISTANCE_SENSOR(F(")"));
    } else {
      consecutiveSignalFailCount = 0;
      DBG_DISTANCE_SENSOR(F("VL53L1X result/clear failed: "));
      DBG_DISTANCE_SENSOR(resultStatus);
      DBG_DISTANCE_SENSOR(F("/"));
      DBGLN_DISTANCE_SENSOR(clearStatus);
    }
  }

  // A transient I2C error or unconfirmed optical rejection cannot prove that
  // the scene changed. Preserve the last usable command until a fresh result.
  return hasUsableDistance ? cachedDistanceReading() : pendingDistanceReading();
}

// Hardware startup must run before initColorSensorHardware(). Both sensors use
// 7-bit address 0x29 after reset, so XSHUT temporarily keeps the VL53L1X off the
// bus, then releases it for configuration and reassignment to 0x2A.
void initDistanceSensorHardware() {
  pinMode(pinDistanceSensorXSHUT, OUTPUT);
  digitalWrite(pinDistanceSensorXSHUT, LOW);  // Hardware shutdown/reset.
  delay(10);                                  // Allow internal rails to discharge.
  digitalWrite(pinDistanceSensorXSHUT, HIGH); // Boot at default address 0x29.
  delay(10);                                  // Datasheet boot settling time.
  startDistanceSensorRanging();
}

// Apply the complete operating profile. Each ULD call is checked separately so
// debug builds identify the exact failed step. A shared numeric diagnostic
// avoids storing a different flash string for every operation.
static bool tofConfigurationStepSucceeded(int status, uint8_t step) {
  if (status == VL53L1_ERROR_NONE) return true;
  #if DEBUG_DISTANCE_SENSOR
  DBG_DISTANCE_SENSOR(F("VL53L1X config step "));
  DBG_DISTANCE_SENSOR(step);
  DBG_DISTANCE_SENSOR(F(" failed, error="));
  DBGLN_DISTANCE_SENSOR(status);
  #else
  (void)step;
  #endif
  return false;
}

bool configureDistanceSensor() {
  // Begin waits briefly for boot, verifies communication, and loads ST's
  // recommended base register configuration. Diagnostic step 1.
  if (!tofConfigurationStepSucceeded(
        distanceTof.Begin(distanceSensorDefaultAddress8Bit), 1)) return false;

  // Move away from 0x29 before the fixed-address TCS34725 is initialized.
  // Diagnostic step 2.
  if (!tofConfigurationStepSucceeded(
        distanceTof.SetI2CAddress(distanceSensorAddress8Bit), 2)) return false;

  // Offset corrects systematic distance error; crosstalk compensates photons
  // reflected by the sensor window/enclosure. Both values are module-specific.
  // Diagnostic steps 3 and 4.
  if (!tofConfigurationStepSucceeded(
        distanceTof.SetOffsetInMm(distanceTofOffsetMm), 3)) return false;
  if (!tofConfigurationStepSucceeded(
        distanceTof.SetXTalk(distanceTofXtalkCps), 4)) return false;

  // Short mode is optimized for nearby targets and stronger ambient-light
  // immunity. The timing budget is measurement time; the inter-measurement
  // period is the interval between starts and must not be shorter. Diagnostic
  // steps 5 through 7.
  if (!tofConfigurationStepSucceeded(
        distanceTof.SetDistanceMode(Short), 5)) return false;
  if (!tofConfigurationStepSucceeded(
        distanceTof.SetTimingBudgetInMs(distanceTofTimingBudgetMs), 6)) return false;
  if (!tofConfigurationStepSucceeded(
        distanceTof.SetInterMeasurementInMs(distanceTofInterMeasurementMs), 7)) return false;

  // ROI limits which SPADs (single-photon detector pixels) participate. A 4x4
  // region narrows the field of view; ROICenter aims that region. Diagnostic
  // steps 8 and 9.
  if (!tofConfigurationStepSucceeded(
        distanceTof.SetROI(distanceTofRoiWidthSpads, distanceTofRoiHeightSpads), 8)) return false;
  if (!tofConfigurationStepSucceeded(
        distanceTof.SetROICenter(distanceTofRoiCenterSpad), 9)) return false;

  #if DEBUG_DISTANCE_SENSOR
  // GetFactoryROICenter is diagnostic only. It reads the optical center stored
  // by ST during factory calibration and does not change sensor behavior.
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

// Initialize the device and start continuous ranging only when auto mode
// currently needs measurements.
bool startDistanceSensorRanging() {
  if (!configureDistanceSensor()) {
    distanceTofDetected = false;
    tofRangingActive = false;
    return false;
  }

  distanceTofDetected = true;
  distanceTofFaultLatched = false;
  resetDistanceFilter();
  lastTofReadMs = 0;
  const bool rangingRequired = AutoDistanceOnOff;
  if (rangingRequired) {
    // StartRanging enters continuous mode. Results are consumed later by
    // getDistanceReading(), so this call does not block for a measurement.
    if (distanceTof.StartRanging() != VL53L1_ERROR_NONE) {
      distanceTofDetected = false;
      tofRangingActive = false;
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

// XSHUT resets both the chip and the ULD object's remembered I2C address. Build
// a fresh wrapper before configuring a sensor that has just rebooted at 0x29.
bool recoverDistanceSensorAfterXshut() {
  distanceTof = VL53L1X_ULD();
  return startDistanceSensorRanging();
}

// Central ranging lifecycle control. Entering auto mode always starts a fresh
// control session, even if this function is called while ranging is already
// active. This guarantees that a stopped train receives a new Valid result
// instead of waiting forever on a cached clear-path value.
void setDistanceSensorRangingActive(bool active) {
  if (active && !distanceTofDetected && distanceTofFaultLatched) {
    DBGLN_DISTANCE_SENSOR(F("VL53L1X was latched off; retrying init"));
    startDistanceSensorRanging();
    return;
  }
  if (!distanceTofDetected) return;
  const bool rangingRequired = active;

  if (rangingRequired) {
    resetDistanceFilter();
    lastTofReadMs = 0;
    if (tofRangingActive) return;

    if (distanceTof.StartRanging() != VL53L1_ERROR_NONE) {
      handleDistanceSensorFault();
      return;
    }
    tofRangingActive = true;
    DBGLN_DISTANCE_SENSOR(F("VL53L1X ranging started"));
  } else if (tofRangingActive) {
    // StopRanging saves sensor power and I2C traffic. The requested off state
    // is recorded even if the bus write fails; no caller should consume ranges
    // while auto mode is disabled.
    const VL53L1_Error stopStatus = distanceTof.StopRanging();
    tofRangingActive = false;
    if (stopStatus == VL53L1_ERROR_NONE) {
      DBGLN_DISTANCE_SENSOR(F("VL53L1X ranging stopped"));
    } else {
      DBGLN_DISTANCE_SENSOR(F("VL53L1X ranging stop failed"));
    }
  }
}
