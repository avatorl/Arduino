  #include <Adafruit_TCS34725.h>

  // ================================================================================================
  // File description
  // ================================================================================================
  // TCS34725 color-sensor handling, track-marker classification, and color-sensor power control.
  // The sensor itself is driven by the Adafruit_TCS34725 library; this tab only wires the library
  // up to the train's sampling schedule, white-balance calibration, and marker action table.
  // Split out of the former 40-sensors.ino; the distance sensor lives in its dedicated VL53L0X tab.
  //
  // Adafruit_TCS34725 library methods used below:
  //   Adafruit_TCS34725(integration, gain)  // construct with fixed ATIME + gain (see constructor
  //                                         // call further down). 50 ms + 4x gain matches the
  //                                  a       // white-balance calibration numbers in config.h.
  //   begin()      -> Probes the sensor over I2C (address 0x29), verifies the device ID, and
  //                   programs the integration time / gain passed to the constructor.
  //   enable()     -> Powers the ADC on so subsequent getRawData() calls return fresh samples.
  //   disable()    -> Puts the ADC and oscillator back into low-power sleep (~2 uA) between reads.
  //   getRawData(&r,&g,&b,&c) -> Reads all four 16-bit channels (red, green, blue, clear) in one
  //                   burst. The values are the raw ADC counts; classifyTrackMarkerColor() below
  //                   handles white balance, normalization, and cluster matching against config.h.
  //
  // Note: the shared I2C bus (Wire.begin, clock, timeout) is configured once in initI2cBus() in
  // arduino-locomotive.ino before this tab's initColorSensorHardware() runs.

  // A2 (pinColorSensorLED in config.h) is dedicated to the TCS34725 breakout LED control input in
  // this revision. A2 works fine as a digital output; only A6/A7 on the Nano are analog-input-only.
  Adafruit_TCS34725 colorSensor(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);
  bool colorSensorDetected = false;

  // TCS34725 Low-Power Sleep / Power-Down Notes:
  // - The sensor IC has an internal sleep/power-down state (~1-2 uA) controlled via I2C.
  // - Turning off the onboard LED via A2 saves ~15-20 mA (the dominant current draw).
  // - setColorSensorEnabled(false) powers down both the LED and the sensor core.

  // Color-sensor sampling and classification state.
  unsigned long lastColorSensorRead = 0;
  uint8_t lastTrackMarkerClass = 255;

  // Put the TCS34725 core into sleep when the system is shutting down permanently.
  void powerDownColorSensorCore() {
    if (colorSensorDetected) {
      colorSensor.disable();
    }
  }

  // Configure the TCS34725 and its onboard lamp control.
  void initColorSensorHardware() {
    pinMode(pinColorSensorLED, OUTPUT);
    digitalWrite(pinColorSensorLED, colorSensorLEDOffLevel);

    colorSensorDetected = colorSensor.begin();
    if (colorSensorDetected) {
      colorSensor.disable();  // Keep the sensor IC core in low-power sleep (~2 uA) until enabled
      DBGLN_COLOR_SENSOR(F("TCS34725 ready (in sleep mode)"));
    } else {
      DBGLN_COLOR_SENSOR(F("TCS34725 not detected on I2C"));
    }
  }

  // Toggle color-sensor mode and related status lighting.
  void setColorSensorEnabled(bool enabled) {
    if (enabled && !isColorSensorAllowed()) {
      DBGLN_COLOR_SENSOR(F("Ignored: color sensor disabled by battery policy"));
      return;
    }
    if (enabled && !colorSensorDetected) {
      DBGLN_COLOR_SENSOR(F("Ignored: TCS34725 not detected"));
      return;
    }

    ColorSensorOnOff = enabled ? 1 : 0;
    digitalWrite(pinColorSensorLED, enabled ? colorSensorLEDOnLevel : colorSensorLEDOffLevel);
    // During VL53L0X troubleshooting, the A2 illumination toggle is also the user's explicit
    // request for live ToF diagnostics. In normal builds this call compiles to a no-op.
    setDistanceSensorDebugSamplingEnabled(enabled);

    if (enabled) {
      if (colorSensorDetected) colorSensor.enable();
      DBGLN_COLOR_SENSOR(F("Color sensor ON"));
      if (!sirenActive) SetRGBLightColor(RgbColor::Cyan);
    } else {
      if (colorSensorDetected) colorSensor.disable();
      DBGLN_COLOR_SENSOR(F("Color sensor OFF"));
      refreshDriveLights();
    }
  }

  // Apply per-channel gain correction to raw RGBC data.
  // Real sensors respond a little differently to red/green/blue light depending on their physical
  // construction, so "white" light doesn't naturally read back as equal R=G=B without correction.
  // These fixed calibration gains (measured once against a known-white surface) scale each channel
  // so a genuinely white/grey object reads back roughly balanced, which the color-classification
  // code below depends on. "+ 0.5f" before truncating to an integer is a common rounding trick
  // (rounds to nearest instead of always rounding down when converting float -> uint16_t).
  BalancedRgbs applyWhiteBalance(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
    BalancedRgbs balanced;
    balanced.r = (uint16_t)(r * whiteBalanceRedGain + 0.5f);
    balanced.g = (uint16_t)(g * whiteBalanceGreenGain + 0.5f);
    balanced.b = (uint16_t)(b * whiteBalanceBlueGain + 0.5f);
    balanced.c = c;
    return balanced;
  }

  // Convert RGB counts to a 0-1000 normalized triple.
  PrototypeRgb normalizePrototype(uint16_t r, uint16_t g, uint16_t b) {
    PrototypeRgb prototype = { 0, 0, 0 };
    uint32_t sum = (uint32_t)r + (uint32_t)g + (uint32_t)b;
    if (sum == 0) return prototype;

    prototype.r = (uint16_t)(((uint32_t)r * 1000UL + (sum / 2)) / sum);
    prototype.g = (uint16_t)(((uint32_t)g * 1000UL + (sum / 2)) / sum);
    prototype.b = (uint16_t)(((uint32_t)b * 1000UL + (sum / 2)) / sum);
    return prototype;
  }

  // Manhattan distance between two normalized RGB prototypes.
  // "Manhattan distance" (also called taxicab distance) adds up the absolute difference of each
  // coordinate separately (|dR| + |dG| + |dB|), unlike straight-line ("Euclidean") distance which
  // would involve squaring and a square root. It's used here because it's much cheaper to compute on
  // a small AVR chip (no multiplication or sqrt() needed) while still being a perfectly reasonable
  // way to measure "how different are these two colors".
  uint16_t prototypeDistance(const PrototypeRgb& a, const PrototypeRgb& b) {
    uint16_t distance = 0;
    distance += (a.r > b.r) ? (a.r - b.r) : (b.r - a.r);
    distance += (a.g > b.g) ? (a.g - b.g) : (b.g - a.g);
    distance += (a.b > b.b) ? (a.b - b.b) : (b.b - a.b);
    return distance;
  }

  // Convert raw RGBC values into track marker classes.
  uint8_t classifyTrackMarkerColor(uint16_t rawR, uint16_t rawG, uint16_t rawB, uint16_t rawC) {
    if (rawC < colorClearMinThreshold || rawC < colorMatchClearThreshold) return MarkerUnknown;

    BalancedRgbs balanced = applyWhiteBalance(rawR, rawG, rawB, rawC);
    PrototypeRgb measured = normalizePrototype(balanced.r, balanced.g, balanced.b);
    if (measured.r == 0 && measured.g == 0 && measured.b == 0) return MarkerUnknown;

    uint16_t bestDistance = 0xFFFF;
    TrackMarkerClass bestClass = MarkerUnknown;

    // Walk every known marker color in the PROGMEM table from config.h and
    // keep the closest match that's still within that cluster's own allowed radius
    // (clusterMaxDistance) - this rejects colors that don't clearly belong to any known marker
    // instead of always forcing a "best guess" match.
    for (uint8_t i = 0; i < markerClusterCount; ++i) {
      PrototypeRgb clusterCenter = {
        pgm_read_word(&markerClusters[i].center.r),
        pgm_read_word(&markerClusters[i].center.g),
        pgm_read_word(&markerClusters[i].center.b)
      };
      uint16_t clusterMaxDistance = pgm_read_word(&markerClusters[i].maxDistance);
      TrackMarkerClass clusterClass = (TrackMarkerClass)pgm_read_byte(&markerClusters[i].markerClass);
      uint16_t distance = prototypeDistance(measured, clusterCenter);
      if (distance <= clusterMaxDistance && distance < bestDistance) {
        bestDistance = distance;
        bestClass = clusterClass;
      }
    }

    return (uint8_t)bestClass;
  }

  #if DEBUG_COLOR_SENSOR
  // Debug-only marker name lookup.
  const __FlashStringHelper* trackMarkerLabel(uint8_t markerClass) {
    switch (markerClass) {
      case MarkerWhite: return F("white");
      case MarkerBrown: return F("brown");
      case MarkerCyan: return F("cyan");
      case MarkerGreen: return F("green");
      case MarkerGrey: return F("grey");
      case MarkerMagenta: return F("magenta");
      case MarkerOrange: return F("orange");
      case MarkerYellow: return F("yellow");
      case MarkerRed: return F("red");
      default: return F("unknown");
    }
  }
  #endif

  // Run the action associated with a detected marker color.
  void handleTrackMarkerAction(uint8_t markerClass) {
    switch (markerClass) {
      case MarkerWhite:
        DBGLN_COLOR_SENSOR(F("Track action: WHITE marker (no action defined yet)"));
        break;

      case MarkerBrown:
        DBGLN_COLOR_SENSOR(F("Track action: BROWN marker (no action defined yet)"));
        break;

      case MarkerCyan:
        DBGLN_COLOR_SENSOR(F("Track action: CYAN marker (no action defined yet)"));
        break;

      case MarkerRed:
        DBGLN_COLOR_SENSOR(F("Track action: RED marker - STOP"));
          if (AutoDistanceOnOff) {
            exitAutoDistanceMode();
            DBGLN_IR_REMOTE(F("Switched from AUTO to MANUAL mode"));
          }
          DBGLN_MOTOR(F("STOP pressed -> Motors stopped"));
          SetRGBColor(RgbColor::Red);
          stopAndResetStepSelection(true);  // default to forward when stopped
          break;
        break;

      case MarkerGreen:
        DBGLN_COLOR_SENSOR(F("Track action: GREEN marker (no action defined yet)"));
        break;

      case MarkerGrey:
        DBGLN_COLOR_SENSOR(F("Track action: GREY marker (no action defined yet)"));
        break;

      case MarkerMagenta:
        DBGLN_COLOR_SENSOR(F("Track action: MAGENTA marker (no action defined yet)"));
        break;

      case MarkerOrange:
        DBGLN_COLOR_SENSOR(F("Track action: ORANGE marker (no action defined yet)"));
        break;

      case MarkerYellow:
        DBGLN_COLOR_SENSOR(F("Track action: YELLOW marker (no action defined yet)"));
        break;

      default:
        break;
    }
  }

  // Poll and process the color sensor without blocking.
  void updateColorSensor() {
    if (ColorSensorOnOff == 0 || !colorSensorDetected) return;

    unsigned long now = millis();
    if (now - lastColorSensorRead < colorSensorReadEveryMs) return;
    lastColorSensorRead = now;

    uint16_t r = 0, g = 0, b = 0, c = 0;
    colorSensor.getRawData(&r, &g, &b, &c);
    uint8_t markerClass = classifyTrackMarkerColor(r, g, b, c);
    #if DEBUG_COLOR_SENSOR
    const __FlashStringHelper* markerLabel = trackMarkerLabel(markerClass);
    DBG_COLOR_SENSOR(F("TCS34725 RGBC: "));
    DBG_COLOR_SENSOR(r);
    DBG_COLOR_SENSOR(F(", "));
    DBG_COLOR_SENSOR(g);
    DBG_COLOR_SENSOR(F(", "));
    DBG_COLOR_SENSOR(b);
    DBG_COLOR_SENSOR(F(", clear="));
    DBG_COLOR_SENSOR(c);
    DBG_COLOR_SENSOR(F(" -> "));
    DBGLN_COLOR_SENSOR(markerLabel);
    #endif

    if (markerClass != lastTrackMarkerClass) {
      lastTrackMarkerClass = markerClass;
      #if DEBUG_COLOR_SENSOR
      DBG_COLOR_SENSOR(F("Track marker changed to: "));
      DBGLN_COLOR_SENSOR(markerLabel);
      #endif
      handleTrackMarkerAction(markerClass);
    }
  }
