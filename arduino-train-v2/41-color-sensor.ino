  // ================================================================================================
  // File description
  // ================================================================================================
  // TCS34725 color-sensor driver, track-marker classification, and color-sensor power control.
  // Split out of the former 40-sensors.ino; the distance sensor lives in 42-distance-sensor.ino and
  // the accelerometer in 43-accelerometer.ino.

  // TCS34725 color-sensor helper.
  // This struct is a minimal, hand-written driver for the TCS34725 RGB color sensor, talking to it
  // directly over I2C register reads/writes instead of using the official "Adafruit_TCS34725"
  // library. This is intentional: the Adafruit library pulls in extra code/RAM for features this
  // sketch does not need (gain/integration-time enums, interrupt thresholds, Adafruit_Sensor
  // framework glue, etc.), and an 8-bit AVR chip like the Nano has very little flash (32 KB) and RAM
  // (2 KB) to spare. By only implementing the handful of registers actually used here, this driver
  // keeps the compiled program much smaller.
  // If you wanted to use the official library instead, the equivalent calls would be:
  //   Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);  // ~= begin_I2C() below
  //   tcs.begin();          // probes the sensor and configures ATIME/gain, same as begin_I2C()
  //   tcs.setInterrupt(false);  // roughly equivalent to enable()/disable() below (turns the ADC on/off)
  //   tcs.getRawData(&r, &g, &b, &c);  // same result as readRawData() below
  struct TrainColorSensorTCS34725 {
    static const uint8_t DefaultAddress = 0x29;
    static const uint8_t CommandBit = 0x80;
    static const uint8_t CommandAutoIncrement = 0x20;
    static const uint8_t RegisterEnable = 0x00;
    static const uint8_t RegisterAtime = 0x01;
    static const uint8_t RegisterId = 0x12;
    static const uint8_t RegisterControl = 0x0F;
    static const uint8_t RegisterClearDataLow = 0x14;
    static const uint8_t EnablePowerOn = 0x01;
    static const uint8_t EnableAdc = 0x02;
    static const uint8_t IntegrationTime50ms = 0xEB;
    static const uint8_t Gain4x = 0x01;

    uint8_t address = DefaultAddress;

    bool begin_I2C(uint8_t i2cAddress = DefaultAddress) {
      Wire.begin();
      #if defined(WIRE_HAS_TIMEOUT)
      Wire.setWireTimeout(3000, true);
      Wire.clearWireTimeoutFlag();
      #endif
      address = i2cAddress;

      uint8_t deviceId = 0;
      if (!probe()) return false;
      if (!readRegister(RegisterId, &deviceId)) return false;
      if (!isSupportedDeviceId(deviceId)) return false;
      if (!writeRegister(RegisterAtime, IntegrationTime50ms)) return false;
      return writeRegister(RegisterControl, Gain4x);
    }

    bool enable() {
      if (!writeRegister(RegisterEnable, EnablePowerOn)) return false;
      delay(3);
      return writeRegister(RegisterEnable, (uint8_t)(EnablePowerOn | EnableAdc));
    }

    bool disable() {
      return writeRegister(RegisterEnable, 0x00);
    }

    bool readRawData(uint16_t* r, uint16_t* g, uint16_t* b, uint16_t* c) {
      uint8_t raw[8] = { 0 };
      if (!readRegisters(RegisterClearDataLow, raw, sizeof(raw))) {
        *r = 0;
        *g = 0;
        *b = 0;
        *c = 0;
        return false;
      }

      // The sensor sends each 16-bit color channel as two separate 8-bit bytes, low byte first
      // ("little-endian"). Combining them back into one 16-bit number uses the same bitmask/shift
      // idiom seen elsewhere in this codebase (e.g., TrainLedMCP23008 above): "raw[1] << 8" moves
      // the high byte into the upper 8 bits of a 16-bit value, and "|" merges it with the low byte.
      *c = (uint16_t)raw[0] | ((uint16_t)raw[1] << 8);
      *r = (uint16_t)raw[2] | ((uint16_t)raw[3] << 8);
      *g = (uint16_t)raw[4] | ((uint16_t)raw[5] << 8);
      *b = (uint16_t)raw[6] | ((uint16_t)raw[7] << 8);
      return true;
    }

   private:
    bool isSupportedDeviceId(uint8_t deviceId) {
      return deviceId == 0x44 || deviceId == 0x4D || deviceId == 0x10;
    }

    bool probe() {
      Wire.beginTransmission(address);
      return Wire.endTransmission() == 0;
    }

    bool writeRegister(uint8_t reg, uint8_t value) {
      Wire.beginTransmission(address);
      Wire.write((uint8_t)(CommandBit | reg));
      Wire.write(value);
      return Wire.endTransmission() == 0;
    }

    bool readRegister(uint8_t reg, uint8_t* value) {
      return readRegisters(reg, value, 1);
    }

    bool readRegisters(uint8_t startReg, uint8_t* buffer, uint8_t length) {
      Wire.beginTransmission(address);
      Wire.write((uint8_t)(CommandBit | CommandAutoIncrement | startReg));
      // Wire.endTransmission(false) sends what was queued but keeps the I2C bus held open with a
      // "repeated start" instead of a full stop (the plain endTransmission() used elsewhere in this
      // file releases the bus). This is required here because the sensor needs to see "here's the
      // register I want, now immediately read back its value" as one unbroken transaction; a
      // separate stop+start could let another device interrupt in between on a shared bus.
      if (Wire.endTransmission(false) != 0) return false;

      // Wire.requestFrom(address, length) asks the sensor to send back "length" bytes; the return
      // value is how many bytes it actually received (compared against the requested length below to
      // detect a short/failed read). Wire.read() then pops one buffered byte at a time out of the
      // I2C receive buffer that requestFrom() just filled.
      uint8_t bytesRead = Wire.requestFrom((int)address, (int)length);
      if (bytesRead != length) return false;

      for (uint8_t i = 0; i < length; ++i) {
        buffer[i] = (uint8_t)Wire.read();
      }
      return true;
    }
  };

  // D4 is dedicated to the TCS34725 breakout LED control input in this revision.
  TrainColorSensorTCS34725 colorSensor;
  bool colorSensorDetected = false;

  // TCS34725 Low-Power Sleep / Power-Down Notes:
  // - The sensor IC has an internal sleep/power-down state (~1-2 uA) controlled via I2C.
  // - Turning off the onboard LED via D4 saves ~15-20 mA (the dominant current draw).
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

    colorSensorDetected = colorSensor.begin_I2C();
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
        DBGLN_COLOR_SENSOR(F("Track action: RED marker (no action defined yet)"));
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
    colorSensor.readRawData(&r, &g, &b, &c);
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
