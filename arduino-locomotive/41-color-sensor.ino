#include <Adafruit_TCS34725.h> // TCS34725 color sensor is driven by the Adafruit_TCS34725 library.

// ================================================================================================
// File description
// ================================================================================================
// This file handles sensor initialization, power control, and color marker
// (action brick) detection logic. Works with original and 3D-printed actions
// bricks. Multiple definitions of the same color are supported,
//    e.g. origial red brick and 3d-printed red brick both will be recognized
//    depsite colors doesn't match exactly.

// Adafruit_TCS34725 library methods used below:
//
//   Adafruit_TCS34725(integration, gain)  -> Constructs the sensor object with
//   the specified integration
//                                            (exposure) time and gain
//                                            (brightness amplification).
//
//   begin()      -> Probes the sensor over I2C (address 0x29), verifies the
//   device ID, and
//                   programs the integration time / gain passed to the
//                   constructor.
//
//   enable()     -> Powers the ADC on so subsequent getRawData() calls return
//   fresh samples.
//
//   disable()    -> Puts the ADC and oscillator back into low-power sleep (~2
//   uA).
//
//   getRawData(&r,&g,&b,&c) -> Performs the library-managed channel reads.
//                              The returned raw ADC counts then should be
//                              white-balanced, normalized, and compared with
//                              the marker clusters.
//
// Note: the shared I2C bus (Wire.begin, clock, timeout) is configured once in
// initI2cBus() in arduino-locomotive.ino before this tab's
// initColorSensorHardware() runs.

// A2 (pinColorSensorLED in config.h) is dedicated to the TCS34725 breakout LED
// control input in this revision. A2 works fine as a digital output; only A6/A7
// on the Nano are analog-input-only.

// The 24 ms integration time is the sensor's exposure time. The 4x gain makes
// readings brighter without changing that exposure; both settings must stay
// aligned with the calibration values. If integration time or gain changed then
// adjust calibration values on the config.h accordingly

// TCS34725_INTEGRATIONTIME_2_4MS  = 0xFF   #  2.4ms - 1 cycle    - Max Count: 1024
// TCS34725_INTEGRATIONTIME_24MS   = 0xF6   # 24ms  - 10 cycles   - Max Count: 10240
// TCS34725_INTEGRATIONTIME_50MS   = 0xEB   #  50ms  - 20 cycles  - Max Count: 20480
// TCS34725_INTEGRATIONTIME_101MS  = 0xD5   #  101ms - 42 cycles  - Max Count: 43008
// TCS34725_INTEGRATIONTIME_154MS  = 0xC0   #  154ms - 64 cycles  - Max Count: 65535
// TCS34725_INTEGRATIONTIME_700MS  = 0x00   #  700ms - 256 cycles - Max Count: 65535

Adafruit_TCS34725 colorSensor(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);

bool colorSensorDetected = false;

// TCS34725 Low-Power Sleep / Power-Down Notes:
// - The sensor IC has an internal sleep/power-down state (~1-2 uA) controlled
// via I2C.
// - Turning off the onboard LED via A2 saves ~15-20 mA (the dominant current
// draw).
// - setColorSensorEnabled(false) powers down both the LED and the sensor core.

// Color-sensor sampling and marker-confirmation state. Consecutive matching
// known samples confirm a marker. Consecutive unknown samples clear it, while
// consecutive samples of a different known color can directly confirm and run
// that new marker without an unknown reading in between.
unsigned long lastColorSensorRead = 0;
uint8_t confirmedTrackMarkerClass = MarkerUnknown;
uint8_t candidateTrackMarkerClass = MarkerUnknown;
uint8_t candidateTrackMarkerSamples = 0;
uint8_t markerLeaveSamples = 0;
unsigned long greenMarkerIgnoreUntil = 0;

void resetMarkerCandidate() {
  candidateTrackMarkerClass = MarkerUnknown;
  candidateTrackMarkerSamples = 0;
}

void resetMarkerLeaveConfirmation() { markerLeaveSamples = 0; }

void resetTrackMarkerDetectionState() {
  confirmedTrackMarkerClass = MarkerUnknown;
  resetMarkerCandidate();
  resetMarkerLeaveConfirmation();
  greenMarkerIgnoreUntil = 0;
}

// Put the TCS34725 core into sleep
void powerDownColorSensorCore() {
  if (colorSensorDetected) {
    colorSensor.disable();
  }
}

// Configure the TCS34725 and its onboard lamp control.
void initColorSensorHardware() {

  pinMode(pinColorSensorLED, OUTPUT);
  digitalWrite(pinColorSensorLED,
               LOW); // Ensure color sensor onboard LED is off initially

  colorSensorDetected =
      colorSensor
          .begin(); // Initialize the color sensor and check if it is detected
  if (colorSensorDetected) {
    colorSensor.disable(); // Keep the sensor IC core in low-power sleep (~2 uA)
                           // until enabled by a user
    DBGLN_COLOR_SENSOR(
        F("Color sensor successfully initialized and put into sleep"));
  } else {
    DBGLN_COLOR_SENSOR(F("Color sensor initialization ERROR"));
  }
}

// Toggle color-sensor mode and related status lighting.
void setColorSensorEnabled(bool enabled) {
  if (enabled && !isColorSensorAllowed()) {
    DBGLN_COLOR_SENSOR(F("Color sensor blocked by power management settings"));
    return;
  }
  if (enabled && !colorSensorDetected) {
    DBGLN_COLOR_SENSOR(F("Color sensor not found"));
    return;
  }

  ColorSensorOnOff = enabled ? 1 : 0;
  digitalWrite(
      pinColorSensorLED,
      enabled ? HIGH
              : LOW); // Toggle the onboard LED according to the sensor state

  if (enabled) {
    resetTrackMarkerDetectionState();
    if (colorSensorDetected)
      colorSensor.enable();
    DBGLN_COLOR_SENSOR(F("Color sensor ON"));
    if (!sirenActive)
      SetRGBLightColor(RgbColor::White);
  } else {
    if (colorSensorDetected)
      colorSensor.disable();
    DBGLN_COLOR_SENSOR(F("Color sensor OFF"));
    markerColorBlinkActive = false;
    resetTrackMarkerDetectionState();
    refreshDriveLights();
  }
}

// Apply per-channel gain correction to raw RGB data.
// Real sensors respond a little differently to red/green/blue light depending
// on their physical construction, so "white" light doesn't naturally read back
// as equal R=G=B without correction. These fixed calibration gains (measured
// once against a known-white surface) scale each channel so a genuinely
// white/grey object reads back roughly balanced, which the color-classification
// code below depends on. "+ 0.5f" before truncating to an integer is a common
// rounding trick (rounds to nearest instead of always rounding down when
// converting float -> uint16_t).
BalancedRgbs applyWhiteBalance(uint16_t r, uint16_t g, uint16_t b) {
  BalancedRgbs balanced;
  balanced.r = (uint16_t)(r * whiteBalanceRedGain + 0.5f);
  balanced.g = (uint16_t)(g * whiteBalanceGreenGain + 0.5f);
  balanced.b = (uint16_t)(b * whiteBalanceBlueGain + 0.5f);
  return balanced;
}

// Convert RGB counts to a 0-1000 normalized triple (R+G+B = 1000)
PrototypeRgb normalizePrototype(uint16_t r, uint16_t g, uint16_t b) {
  PrototypeRgb prototype = {0, 0, 0};
  uint32_t sum = (uint32_t)r + (uint32_t)g + (uint32_t)b;
  if (sum == 0)
    return prototype;

  prototype.r = (uint16_t)(((uint32_t)r * 1000UL + (sum / 2)) / sum);
  prototype.g = (uint16_t)(((uint32_t)g * 1000UL + (sum / 2)) / sum);
  prototype.b = (uint16_t)(((uint32_t)b * 1000UL + (sum / 2)) / sum);
  return prototype;
}

// Manhattan distance between two normalized RGB prototypes.
// "Manhattan distance" (also called taxicab distance) adds up the absolute
// difference of each coordinate separately (|dR| + |dG| + |dB|), unlike
// straight-line ("Euclidean") distance which would involve squaring and a
// square root. It's used here because it's much cheaper to compute on a small
// AVR chip (no multiplication or sqrt() needed) while still being a perfectly
// reasonable way to measure "how different are these two colors".
uint16_t prototypeDistance(const PrototypeRgb &a, const PrototypeRgb &b) {
  uint16_t distance = 0;
  distance += (a.r > b.r) ? (a.r - b.r) : (b.r - a.r);
  distance += (a.g > b.g) ? (a.g - b.g) : (b.g - a.g);
  distance += (a.b > b.b) ? (a.b - b.b) : (b.b - a.b);
  return distance;
}

// Convert raw RGBC values into track marker classes.
uint8_t classifyTrackMarkerColor(uint16_t rawR, uint16_t rawG, uint16_t rawB,
                                 uint16_t rawC) {
  if (rawC > colorSaturationClearThreshold) {
    return MarkerUnknown;
  }

  BalancedRgbs balanced = applyWhiteBalance(rawR, rawG, rawB);
  PrototypeRgb normalized =
      normalizePrototype(balanced.r, balanced.g, balanced.b);
  if (normalized.r == 0 && normalized.g == 0 && normalized.b == 0)
    return MarkerUnknown;

  uint16_t bestDistance = 0xFFFF;
  TrackMarkerClass bestClass = MarkerUnknown;
  bool hadEligibleCluster = false;

  // Walk every known marker color in the PROGMEM table from config.h and
  // keep the closest match that's still within that cluster's own allowed
  // radius.
  for (uint8_t i = 0; i < markerClusterCount; ++i) {
    uint16_t clusterClearThreshold =
        pgm_read_word(&markerClusters[i].minClearThreshold);
    if (rawC < clusterClearThreshold)
      continue;
    hadEligibleCluster = true;

    PrototypeRgb clusterCenter = {pgm_read_word(&markerClusters[i].center.r),
                                  pgm_read_word(&markerClusters[i].center.g),
                                  pgm_read_word(&markerClusters[i].center.b)};
    uint16_t clusterMaxDistance = pgm_read_word(&markerClusters[i].maxDistance);
    TrackMarkerClass clusterClass =
        (TrackMarkerClass)pgm_read_byte(&markerClusters[i].markerClass);
    uint16_t distance = prototypeDistance(normalized, clusterCenter);
    if (distance <= clusterMaxDistance && distance < bestDistance) {
      bestDistance = distance;
      bestClass = clusterClass;
    }
  }

  if (!hadEligibleCluster)
    return MarkerUnknown;
  return (uint8_t)bestClass;
}

#if DEBUG_COLOR_SENSOR
// Debug-only marker name lookup.
const __FlashStringHelper *trackMarkerLabel(uint8_t markerClass) {
  switch (markerClass) {
  case MarkerWhite:
    return F("\033[97;40m██\033[0m white");
  case MarkerBlue:
    return F("\033[94m██\033[0m blue");
  case MarkerGreen:
    return F("\033[92m██\033[0m green");
  case MarkerMagenta:
    return F("\033[95m██\033[0m magenta");
  case MarkerYellow:
    return F("\033[93m██\033[0m yellow");
  case MarkerRed:
    return F("\033[91m██\033[0m red");
  default:
    return F("unknown color");
  }
}
#endif

// Run the action associated with a detected marker color.
void handleTrackMarkerAction(uint8_t markerClass) {
  switch (markerClass) {

  case MarkerWhite:
    DBGLN_COLOR_SENSOR(F("White: RGB toggle"));
    whiteMarkerRgbOverrideActive = !whiteMarkerRgbOverrideActive;
    if (whiteMarkerRgbOverrideActive) {
      SetRGBLightColor(RgbColor::Off);
    } else {
      refreshDriveLights();
    }
    break;

  case MarkerBlue:
    DBGLN_COLOR_SENSOR(F("Blue: siren on"));
    sirenActive = true;
    sirenTimer = millis();
    sirenStartMs = sirenTimer;
    break;

  case MarkerRed:
    DBGLN_COLOR_SENSOR(F("Red: stop"));
    if (AutoDistanceOnOff) {
      exitAutoDistanceMode();
      DBGLN_IR_REMOTE(F("Switched from AUTO to MANUAL mode"));
    }
    DBGLN_MOTOR(F("STOP pressed -> Motors stopped"));
    SetRGBColor(RgbColor::Red);
    stopAndResetStepSelection(true); // default to forward when stopped
    break;

  case MarkerGreen:
    if ((long)(millis() - greenMarkerIgnoreUntil) < 0) {
      DBGLN_COLOR_SENSOR(F("Green: ignored during reverse cooldown"));
      break;
    }
    if (!boostActive && !AutoDistanceOnOff && Speed != 0) {
      DBGLN_COLOR_SENSOR(F("Green: reverse"));
      if (MotorDirection == 1)
        GoBackward();
      else
        GoForward();
      greenMarkerIgnoreUntil = millis() + colorGreenMarkerCooldownMs;
    } else {
      DBGLN_COLOR_SENSOR(F("Green: ignored in this train mode"));
    }
    break;

  case MarkerMagenta:
    DBGLN_COLOR_SENSOR(F("Magenta: no action"));
    break;

  case MarkerYellow:
    DBGLN_COLOR_SENSOR(F("Yellow: random melody"));
    playRandomMelody();
    break;

  default:
    break;
  }
}

// Display one confirmed marker and run its action exactly once.
void showMarkerFeedbackAndRunAction(uint8_t markerClass) {
#if DEBUG_COLOR_SENSOR
  DBG_COLOR_SENSOR(F("Marker confirmed: "));
  DBGLN_COLOR_SENSOR(trackMarkerLabel(markerClass));
#endif

  RgbColor markerColor =
      static_cast<RgbColor>(pgm_read_byte(&markerFeedbackColors[markerClass]));
  if (markerColor != RgbColor::Off) {
    markerColorBlinkActive = true;
    markerColorBlinkEndsAt = millis() + colorMarkerFeedbackDurationMs;
    if (!sirenActive)
      SetRGBLightColor(markerColor);
  }
  handleTrackMarkerAction(markerClass);
}

// Count one known sample toward a marker candidate. A different color restarts
// the consecutive sequence at one; reaching the configured count confirms the
// candidate.
bool advanceMarkerCandidate(uint8_t markerClass) {
  if (candidateTrackMarkerClass != markerClass) {
    candidateTrackMarkerClass = markerClass;
    candidateTrackMarkerSamples = 1;
  } else {
    ++candidateTrackMarkerSamples;
  }
  return candidateTrackMarkerSamples >= colorMarkerConfirmationSamples;
}

// Advance marker entry, direct known-color changes, or confirmed-leave rearming
// for one sample.
void processTrackMarkerSample(uint8_t markerClass) {
  if (confirmedTrackMarkerClass != MarkerUnknown) {
    if (markerClass == confirmedTrackMarkerClass) {
      // Repeated readings of the confirmed color keep it latched and cancel
      // both possible exits.
      resetMarkerCandidate();
      resetMarkerLeaveConfirmation();
      return;
    }

    if (markerClass == MarkerUnknown) {
      // Unknown samples count only when consecutive; they also cancel a pending
      // color change.
      resetMarkerCandidate();
      ++markerLeaveSamples;
      if (markerLeaveSamples >= colorMarkerLeaveSamples) {
        confirmedTrackMarkerClass = MarkerUnknown;
        resetMarkerLeaveConfirmation();
      }
      return;
    }

    // A known color cancels leave confirmation. Consecutive samples of a
    // different known color can directly replace the confirmed marker and run
    // the new action.
    resetMarkerLeaveConfirmation();
    if (advanceMarkerCandidate(markerClass)) {
      confirmedTrackMarkerClass = markerClass;
      resetMarkerCandidate();
      showMarkerFeedbackAndRunAction(markerClass);
    }
    return;
  }

  resetMarkerLeaveConfirmation();
  // Unknown breaks entry confirmation so only consecutive matching known
  // samples run an action.
  if (markerClass == MarkerUnknown) {
    resetMarkerCandidate();
    return;
  }

  if (advanceMarkerCandidate(markerClass)) {
    confirmedTrackMarkerClass = markerClass;
    resetMarkerCandidate();
    showMarkerFeedbackAndRunAction(markerClass);
  }
}

// Poll and process the color sensor periodically. Adafruit getRawData() waits
// for one integration interval, so this updater is cooperative but not
// non-blocking during the library call.
void updateColorSensor() {
  unsigned long now = millis();
  // Feedback expiry must be serviced even if sensing was disabled or a jog
  // temporarily blocks sampling. An active blink never pauses sensing, so
  // leave/re-entry state keeps advancing.
  if (markerColorBlinkActive && (long)(now - markerColorBlinkEndsAt) >= 0) {
    markerColorBlinkActive = false;
    refreshDriveLights();
  }

  if (ColorSensorOnOff == 0 || !colorSensorDetected)
    return;
  if (momentaryActive) {
    // Jogging pauses color reads. Discard half-complete sequences so readings
    // separated by the whole jog cannot be treated as consecutive, but keep the
    // already confirmed marker latched.
    resetMarkerCandidate();
    resetMarkerLeaveConfirmation();
    return;
  }

  if (lightsBlackoutActive && (long)(now - lightsBlackoutEndsAt) >= 0) {
    lightsBlackoutActive = false;
    refreshDriveLights();
  }
  if (now - lastColorSensorRead < colorSensorReadEveryMs)
    return;
  lastColorSensorRead = now;

  uint16_t r = 0, g = 0, b = 0, c = 0;
  colorSensor.getRawData(&r, &g, &b, &c);
  uint8_t markerClass = classifyTrackMarkerColor(r, g, b, c);

#if DEBUG_COLOR_SENSOR
  bool sampleIsClearEnough = false;
  for (uint8_t i = 0; i < markerClusterCount; ++i) {
    uint16_t clusterClearThreshold =
        pgm_read_word(&markerClusters[i].minClearThreshold);
    if (c >= clusterClearThreshold) {
      sampleIsClearEnough = true;
      break;
    }
  }
  const __FlashStringHelper *markerLabel = sampleIsClearEnough
                                               ? trackMarkerLabel(markerClass)
                                               : F("\033[90mtoo dark, rejected\033[0m");
  BalancedRgbs balanced =
      applyWhiteBalance(r, g, b); // Apply white balance to the raw RGB values
  PrototypeRgb normalized = normalizePrototype(
      balanced.r, balanced.g,
      balanced.b); // Normalize the balanced RGB values (R+G+B = 1000)
  DBG_COLOR_SENSOR(F("Balanced RGBC: "));
  DBG_COLOR_SENSOR(F("\033[91m"));
  DBG_COLOR_SENSOR(balanced.r);
  DBG_COLOR_SENSOR(F("\033[0m"));
  DBG_COLOR_SENSOR(F(", "));
  DBG_COLOR_SENSOR(F("\033[92m"));
  DBG_COLOR_SENSOR(balanced.g);
  DBG_COLOR_SENSOR(F("\033[0m"));
  DBG_COLOR_SENSOR(F(", "));
  DBG_COLOR_SENSOR(F("\033[94m"));
  DBG_COLOR_SENSOR(balanced.b);
  DBG_COLOR_SENSOR(F("\033[0m"));
  DBG_COLOR_SENSOR(F(", "));
  DBG_COLOR_SENSOR(F("\033[97;40m"));
  DBG_COLOR_SENSOR(c);
  DBG_COLOR_SENSOR(F("\033[0m"));
  if (!sampleIsClearEnough) {
    DBG_COLOR_SENSOR(F(" | "));
    DBGLN_COLOR_SENSOR(markerLabel);
  } else {
    DBG_COLOR_SENSOR(F(" | Normalized: { "));
    DBG_COLOR_SENSOR(normalized.r);
    DBG_COLOR_SENSOR(F(", "));
    DBG_COLOR_SENSOR(normalized.g);
    DBG_COLOR_SENSOR(F(", "));
    DBG_COLOR_SENSOR(normalized.b);
    DBG_COLOR_SENSOR(F(" } -> "));
    DBGLN_COLOR_SENSOR(markerLabel);
  }
#endif

  processTrackMarkerSample(markerClass);
}
