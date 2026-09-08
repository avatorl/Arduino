  // ================================================================================================
  // File description
  // ================================================================================================
  // Battery monitoring, low-power policy, sleep / wake flow, and EEPROM event logging live here.
  // These settings mainly affect when the train warns, sleeps, and shuts down to protect the battery.

  // (The battery-percent interpolation itself lives in get2SBatteryPercent() further below; it
  // reads the PROGMEM lookup table entry-by-entry with pgm_read_word() instead of copying the
  // whole table onto the stack first, saving 22 bytes of stack per call.)

  // Print millivolts as volts with exactly two fractional digits for readable debug output.
  void printVoltageMv(uint16_t voltageMv) {
    DBG_POWER_MANAGEMENT(voltageMv / 1000U);
    DBG_POWER_MANAGEMENT(F("."));
    const uint16_t hundredths = (voltageMv % 1000U) / 10U;
    if (hundredths < 10U) DBG_POWER_MANAGEMENT(F("0"));
    DBG_POWER_MANAGEMENT(hundredths);
  }

  bool batteryGuardSettling = false;
  bool batteryGuardConfirmingLow = false;
  unsigned long batteryGuardSettleStartedMs = 0;
  uint16_t firstLowBatteryVoltageMv = 0;

  // A manual battery-status read performs its own quiet settle, so it must invalidate any pending
  // periodic settle or retry. The next periodic attempt will then start a fresh quiet interval.
  void cancelPendingBatteryGuardMeasurement() {
    batteryGuardSettling = false;
    batteryGuardConfirmingLow = false;
  }

  // Convert a requested motor voltage into a safe 0-255 PWM value.
  // PWM duty cycle is proportional to average output voltage (see the analogWrite()/PWM explanation
  // in 20-motor.ino), so this rearranges "desired motor voltage = supply voltage * (pwm / 255)" to
  // solve for pwm. It's computed with "255UL * ..." (an unsigned long, i.e. 32-bit) instead of a
  // plain int (16-bit) to avoid the multiplication overflowing before the division happens; adding
  // "supplyVoltageMv / 2" before dividing is the same round-to-nearest trick used elsewhere in this
  // project (e.g. motorVoltageFromDistanceMm() in 20-motor.ino).
  int safePwmFromMilliVolts(uint16_t desiredMotorMv, uint16_t supplyVoltageMv, uint16_t maxSafeMotorMv) {
    if (supplyVoltageMv == 0) return 0;
    const uint16_t clampedMotorMv = (desiredMotorMv < maxSafeMotorMv) ? desiredMotorMv : maxSafeMotorMv;
    const int pwm = (int)((255UL * clampedMotorMv + (supplyVoltageMv / 2U)) / supplyVoltageMv);
    return constrain(pwm, 0, 255);
  }

  #if ENABLE_EEPROM_LOGGING
  // Return true when a battery reading is missing or outside the valid 2S pack range.
  bool isBatteryVoltageStorageInvalid(uint16_t voltageMv) {
    return voltageMv == 0 || voltageMv > BATTERY_MAX_VALID_MV;
  }

  // Encode a battery reading for EEPROM storage.
  // EEPROM (Electrically Erasable Programmable Read-Only Memory) is a small area of persistent
  // storage that survives power loss, but on AVR chips it is both very small (1 KB on the Nano) and
  // slow/limited in write-endurance (~100,000 writes per byte), so data is packed as tightly as
  // possible. Here, a full millivolt value (e.g. 7400 for 7.4V) is compressed into a single byte
  // representing "voltage x 10" (e.g. 74), which is enough precision for a rough battery-history log
  // while using 1 byte instead of 2. 0xFF is reserved as a sentinel meaning "no valid reading".
  uint8_t encodeBatteryVoltageForEeprom(uint16_t voltageMv) {
    if (isBatteryVoltageStorageInvalid(voltageMv)) return 0xFF;
    return (uint8_t)constrain((int)((voltageMv + 50U) / 100U), 0, 254);
  }

  #if DEBUG_EEPROM
  // Return a readable label for one EEPROM event type byte.
  const __FlashStringHelper* eepromEventLabel(uint8_t eventType) {
    switch (eventType) {
      case EEPROM_EVENT_WARNING: return F("low battery warning");
      case EEPROM_EVENT_SHUTDOWN: return F("low battery shutdown");
      case EEPROM_EVENT_TOF_FAULT: return F("distance sensor fault");
      case EEPROM_EVENT_OVERVOLTAGE: return F("critical overvoltage/meter fault");
      default: return F("unknown event");
    }
  }

  // Print one stored battery byte as a human-readable voltage string.
  void printEepromBatteryValue(uint8_t battByte) {
    if (battByte == 0xFF) {
      DBG_EEPROM(F("invalid/not measured"));
      return;
    }
    DBG_EEPROM(battByte / 10);
    DBG_EEPROM(F("."));
    DBG_EEPROM(battByte % 10);
    DBG_EEPROM(F(" V"));
  }

  // Decode one sensor-error bitfield into beginner-friendly text.
  void printEepromBootFlags(uint8_t flags) {
    if (flags == 0) {
      DBG_EEPROM(F("all sensors OK"));
      return;
    }

    bool first = true;
    if (flags & ERR_LED_EXPANDER) {
      DBG_EEPROM(F("LED expander missing"));
      first = false;
    }
    if (flags & ERR_COLOR_SENSOR) {
      if (!first) DBG_EEPROM(F(", "));
      DBG_EEPROM(F("color sensor missing"));
      first = false;
    }
    if (flags & ERR_DISTANCE_TOF) {
      if (!first) DBG_EEPROM(F(", "));
      DBG_EEPROM(F("distance sensor missing"));
      first = false;
    }
    if (flags & ERR_ACCELEROMETER) {
      if (!first) DBG_EEPROM(F(", "));
      DBG_EEPROM(F("accelerometer missing"));
      first = false;
    }
    if (flags & (uint8_t)~(ERR_LED_EXPANDER | ERR_COLOR_SENSOR | ERR_DISTANCE_TOF | ERR_ACCELEROMETER)) {
      if (!first) DBG_EEPROM(F(", "));
      DBG_EEPROM(F("unknown flags present"));
    }
  }

  // Dump the boot and event EEPROM rings in a user-friendly way for setup-time debugging.
  // EEPROM.read(addr) fetches exactly one byte at a given address. EEPROM.get(addr, variable) is a
  // template helper that instead fills in a whole multi-byte variable (here, a uint16_t, 2 bytes) by
  // reading that many bytes starting at addr - simpler than manually reading each byte and combining
  // them, for values that don't fit in a single byte (e.g. bootCounter/bootSeq below).
  void dumpEepromDebugSummary() {
    uint16_t bootCounter = 0;
    EEPROM.get(EEPROM_ADDR_BOOT_COUNT, bootCounter);
    uint8_t bootHead = EEPROM.read(EEPROM_ADDR_LOG_HEAD);
    uint8_t eventHead = EEPROM.read(EEPROM_ADDR_EVENT_HEAD);

    DBGLN_EEPROM(F("EEPROM summary:"));
    DBG_EEPROM(F("  Boot counter: "));
    DBGLN_EEPROM(bootCounter == 0xFFFF ? 0 : bootCounter);
    DBG_EEPROM(F("  Boot log head slot: "));
    DBGLN_EEPROM(bootHead);
    DBG_EEPROM(F("  Event log head slot: "));
    DBGLN_EEPROM(eventHead);

    DBGLN_EEPROM(F("  Boot log entries:"));
    for (uint8_t slot = 0; slot < EEPROM_BOOT_LOG_SIZE; ++slot) {
      uint16_t entryAddr = EEPROM_ADDR_LOG_BASE + (uint16_t)slot * EEPROM_ENTRY_SIZE;
      uint16_t bootSeq = 0;
      EEPROM.get(entryAddr, bootSeq);
      uint8_t flags = EEPROM.read(entryAddr + 2);
      uint8_t battByte = EEPROM.read(entryAddr + 3);
      uint8_t faultCount = EEPROM.read(entryAddr + 4);

      if (bootSeq == 0xFFFF && flags == 0xFF && battByte == 0xFF && faultCount == 0xFF) continue;

      DBG_EEPROM(F("    Slot "));
      DBG_EEPROM(slot);
      DBG_EEPROM(F(": boot #"));
      DBG_EEPROM(bootSeq == 0xFFFF ? 0 : bootSeq);
      DBG_EEPROM(F(", battery="));
      printEepromBatteryValue(battByte);
      DBG_EEPROM(F(", faults="));
      DBG_EEPROM(faultCount == 0xFF ? 0 : faultCount);
      DBG_EEPROM(F(", status="));
      printEepromBootFlags(flags);
      DBGLN_EEPROM(F(""));
    }

    DBGLN_EEPROM(F("  Event log entries:"));
    for (uint8_t slot = 0; slot < EEPROM_EVENT_LOG_SIZE; ++slot) {
      uint16_t entryAddr = EEPROM_ADDR_EVENT_BASE + (uint16_t)slot * EEPROM_EVENT_ENTRY_SIZE;
      uint16_t bootSeq = 0;
      EEPROM.get(entryAddr, bootSeq);
      uint8_t eventType = EEPROM.read(entryAddr + 2);
      uint8_t battByte = EEPROM.read(entryAddr + 3);
      uint32_t uptimeMs = 0;
      EEPROM.get(entryAddr + 4, uptimeMs);

      if (bootSeq == 0xFFFF && eventType == 0xFF && battByte == 0xFF && uptimeMs == 0xFFFFFFFFUL) continue;

      DBG_EEPROM(F("    Slot "));
      DBG_EEPROM(slot);
      DBG_EEPROM(F(": boot #"));
      DBG_EEPROM(bootSeq == 0xFFFF ? 0 : bootSeq);
      DBG_EEPROM(F(", event="));
      DBG_EEPROM(eepromEventLabel(eventType));
      DBG_EEPROM(F(", battery="));
      printEepromBatteryValue(battByte);
      DBG_EEPROM(F(", uptime="));
      DBG_EEPROM(uptimeMs / 1000UL);
      DBGLN_EEPROM(F(" s"));
    }
  }
  #endif

  // Consume one slot from the shared runtime EEPROM write budget for this power cycle.
  bool reserveRuntimeEepromWrite() {
    if (runtimeEepromWrites >= MAX_RUNTIME_EEPROM_WRITES_PER_BOOT) return false;
    runtimeEepromWrites++;
    return true;
  }

  // Append one runtime battery-related event to the EEPROM event ring buffer.
  // This writes into another ring buffer (see the "ring buffer"/wraparound explanation on
  // pushDistanceSampleAndGetMedian() in the distance-sensor module for the general idea), except here the wrap
  // is done with the modulo operator "%" directly on the head index instead of an "if" check.
  // EEPROM.update(addr, value) is used instead of EEPROM.write(addr, value) because it first checks
  // whether the byte already holds that value and skips the write if so - EEPROM cells wear out
  // after roughly 100,000 writes, so avoiding redundant writes meaningfully extends its lifetime.
  // EEPROM.put(addr, value) is the multi-byte counterpart (see EEPROM.get() note above) used here
  // for uptimeMs, which needs more than a single byte.
  void logBatteryEvent(uint8_t eventType, uint16_t voltageMv) {
    if (!reserveRuntimeEepromWrite()) return;

    uint8_t eventHead = EEPROM.read(EEPROM_ADDR_EVENT_HEAD);
    if (eventHead >= EEPROM_EVENT_LOG_SIZE) eventHead = 0;

    uint8_t battByte = encodeBatteryVoltageForEeprom(voltageMv);
    uint16_t entryAddr = EEPROM_ADDR_EVENT_BASE + (uint16_t)eventHead * EEPROM_EVENT_ENTRY_SIZE;
    uint32_t uptimeMs = millis() - bootStartedAt;

    EEPROM.put(entryAddr, currentBootSequence);
    EEPROM.update(entryAddr + 2, eventType);
    EEPROM.update(entryAddr + 3, battByte);
    EEPROM.put(entryAddr + 4, uptimeMs);
    EEPROM.update(EEPROM_ADDR_EVENT_HEAD, (uint8_t)((eventHead + 1) % EEPROM_EVENT_LOG_SIZE));
  }

  // Log boot sequence number (2-byte timestamp), sensor error flags, battery voltage, and initial
  // fault count (0) to the EEPROM ring buffer. Must be called after initBatteryVoltageMeterHardware()
  // and after batteryVoltage has been set.
  // Log boot sequence, sensor status, and battery to EEPROM ring buffer.
  void writeBootErrorCodes() {
    // Increment the 2-byte global boot counter; treat uninitialized EEPROM (0xFFFF) as pre-boot 0.
    uint16_t bootSeq = 0;
    EEPROM.get(EEPROM_ADDR_BOOT_COUNT, bootSeq);
    bootSeq = (bootSeq == 0xFFFF) ? 1 : (uint16_t)(bootSeq + 1);
    EEPROM.put(EEPROM_ADDR_BOOT_COUNT, bootSeq);
    currentBootSequence = bootSeq;

    uint8_t flags = 0;
    if (!trainLedExpanderDetected) flags |= ERR_LED_EXPANDER;
    if (!colorSensorDetected)      flags |= ERR_COLOR_SENSOR;
    if (!distanceTofDetected)      flags |= ERR_DISTANCE_TOF;
    if (!accelerometerDetected)    flags |= ERR_ACCELEROMETER;

    // Store voltage as tenths-of-volt (74 = 7.4 V); 0xFF = not measured or invalid for a 2S pack.
    uint8_t battByte = encodeBatteryVoltageForEeprom(batteryVoltage);

    // Resolve the current write slot; handle uninitialized EEPROM (0xFF) gracefully.
    uint8_t logHead = EEPROM.read(EEPROM_ADDR_LOG_HEAD);
    if (logHead >= EEPROM_BOOT_LOG_SIZE) logHead = 0;
    currentBootSlot = logHead;
    bootFaultCount = 0;  // Reset fault count for this new power-on session

    uint16_t entryAddr = EEPROM_ADDR_LOG_BASE + (uint16_t)logHead * EEPROM_ENTRY_SIZE;
    EEPROM.put(entryAddr,        bootSeq);
    EEPROM.update(entryAddr + 2, flags);
    EEPROM.update(entryAddr + 3, battByte);
    EEPROM.update(entryAddr + 4, 0);  // Initial fault count is 0 on boot
    EEPROM.update(EEPROM_ADDR_LOG_HEAD, (uint8_t)((logHead + 1) % EEPROM_BOOT_LOG_SIZE));

    #if DEBUG_ANY
    DBG_LEDS(F("Boot #")); DBG_LEDS(bootSeq);
    DBG_LEDS(F("  flags=0x")); DBG_LEDS(flags, HEX);
    DBG_LEDS(F("  batt="));
    if (battByte == 0xFF) { DBGLN_LEDS(F("invalid")); }
    else { DBG_LEDS(battByte / 10); DBG_LEDS(F(".")); DBGLN_LEDS(battByte % 10); }
    if (flags & ERR_LED_EXPANDER) { DBGLN_LEDS(F("  ERR 0x01: MCP23008 LED expander not detected")); }
    if (flags & ERR_COLOR_SENSOR) { DBGLN_LEDS(F("  ERR 0x02: TCS34725 color sensor not detected")); }
    if (flags & ERR_DISTANCE_TOF) { DBGLN_LEDS(F("  ERR 0x04: distance sensor not detected")); }
    if (flags & ERR_ACCELEROMETER) { DBGLN_LEDS(F("  ERR 0x08: MPU6050 accelerometer not detected")); }
    if (flags == 0) { DBGLN_LEDS(F("  All sensors OK")); }
    #endif

    if (flags != 0) {
      // Three rapid beeps bypass SoundOnOff — this is a safety notification, not user audio.
      for (uint8_t i = 0; i < 3; ++i) { tone(pinBuzzer, 1500, 80); delay(150); }
      noTone(pinBuzzer);
    }
  }
  #endif

  // Return true when normal user sounds are allowed by the current battery state.
  bool areUserSoundsAllowed() {
    return batteryState == BatteryState::Normal;
  }

  // Return true when the RGB headlights/status lights may be shown.
  bool areRgbLightsAllowed() {
    return batteryState == BatteryState::Normal && !idleSleepActive;
  }

  // Return true when the green status indicator may be shown.
  bool isGreenIndicatorAllowed() {
    return batteryState == BatteryState::Normal && !idleSleepActive;
  }

  // Return true when the color sensor is allowed to run.
  bool isColorSensorAllowed() {
    return batteryState == BatteryState::Normal;
  }

  // Return true when auto-distance mode is allowed to run.
  bool isAutoDistanceAllowed() {
    return batteryState == BatteryState::Normal;
  }

  // Return true when boost mode is allowed to activate.
  bool isBoostAllowed() {
    return batteryState == BatteryState::Normal;
  }

  // Return true when the train may enter inactivity sleep.
  bool canEnterIdleSleep() {
    return batteryState == BatteryState::Normal;
  }

  // Return true when a button is still allowed during low-battery warning mode.
  bool isWarningModeCommandAllowed(uint8_t code) {
    return code == buttonCHminus || code == buttonCH || code == buttonCHplus
      || code == buttonBackward || code == buttonForward || code == buttonMinus
      || code == buttonPlus;
  }

  // Advance the active warning or shutdown light-and-sound signal.
  // The rear light and tone were already switched on once by startBatterySignal(); nothing else
  // touches them while batterySignalActive is true (updateBuzzer(), playPattern(), updateMelody()
  // and the siren all yield to an active battery signal), so this only needs to watch for the end
  // time instead of re-writing the LED expander and re-starting the tone on every single loop pass.
  void updateBatterySignal() {
    if (!batterySignalActive) return;
    if ((long)(millis() - batterySignalEndsAt) < 0) return;  // still running - outputs already set

    batterySignalActive = false;
    batterySignalUsesTone = false;
    SetRearRedLight(false);
    noTone(pinBuzzer);
    digitalWrite(pinBuzzer, LOW);
  }

  // Start a timed battery warning or shutdown signal.
  void startBatterySignal(unsigned long durationMs, bool useTone) {
    batterySignalActive = true;
    batterySignalUsesTone = useTone;
    batterySignalEndsAt = millis() + durationMs;
    sirenActive = false;
    stopMelody();
    clearBuzzerPattern();
    noTone(pinBuzzer);
    digitalWrite(pinBuzzer, LOW);
    SetRearRedLight(true);
    if (useTone) tone(pinBuzzer, BATTERY_ALERT_TONE_HZ);
  }

  // Turn off non-essential features when battery policy enters a restricted state.
  void applyBatteryRestrictions() {
    // Clear physical jog PWM before disarming its watchdog. Jogging intentionally does not
    // change the manual Speed selection, so Speed == 0 alone is not proof the motor is stopped.
    if (boostActive || momentaryActive) Stop();
    boostActive = false;
    cancelJog();
    exitAutoDistanceMode();
    currentStep = min(currentStep, NORMAL_MAX_SPEED_STEP);
    setColorSensorEnabled(false);
    sirenActive = false;
    stopMelody();
    clearBuzzerPattern();
    noTone(pinBuzzer);
    digitalWrite(pinBuzzer, LOW);
    SetGreenLightValue(0);
    SetRGBLightColor(RgbColor::Off);
  }

  // 0 mV is unavailable and ignored; 8500 mV is valid; 8501 mV latches immediately;
  // readings above 8501 mV remain latched without repeating the alert or EEPROM event.
  bool isCriticalOvervoltage(uint16_t voltageMv) {
    return voltageMv > BATTERY_MAX_VALID_MV;
  }

  // Latch a critical overvoltage fault until the next power cycle.
  // The averageRaw parameter distinguishes ADC saturation (likely disconnected sensor or wiring
  // fault) from genuine overvoltage: if averageRaw >= 1023, the ADC is maxed out and the reading
  // is unreliable; we latch for safety but log it as a meter fault rather than confirmed overvoltage.
  void enterCriticalOvervoltage(uint16_t voltageMv, uint16_t averageRaw) {
    if (criticalOvervoltageLatched) return;

    criticalOvervoltageLatched = true;
    applyBatteryRestrictions();
    batterySignalActive = false;
    batterySignalUsesTone = false;
    idleSleepActive = false;
    idleSleepWarningIssued = false;
    idleWakeRequested = false;
    Stop();
    digitalWrite(pinMotorSleep, LOW);
    digitalWrite(pinDistanceSensorXSHUT, LOW);
    distanceTofDetected = false;
    sleepAccelerometer();  // Nothing will read the MPU-6050 again; park it in low-power sleep.
    powerDownColorSensorCore();
    applyCriticalOvervoltageOutputs();
    playPattern(pattern_criticalOvervoltage, true);
    #if ENABLE_EEPROM_LOGGING
    logBatteryEvent(EEPROM_EVENT_OVERVOLTAGE, voltageMv);
    #endif
    
    if (averageRaw >= BATTERY_ADC_MAX) {
      // A saturated ADC (raw 1023 = the meter's ~12.2 V full scale) is ambiguous: it can mean the
      // voltage divider is disconnected/broken OR the input truly exceeds what the meter can
      // measure. Both are reported because the software cannot tell them apart.
      DBG_POWER_MANAGEMENT(F("CRITICAL: Voltage meter saturated (ADC="));
      DBG_POWER_MANAGEMENT(averageRaw);
      DBG_POWER_MANAGEMENT(F(", reads as "));
      DBG_POWER_MANAGEMENT(voltageMv);
      DBG_POWER_MANAGEMENT(F(" mV) - possible sensor/meter error OR voltage above measurable scale; "));
      DBGLN_POWER_MANAGEMENT(F("power cycle required"));
    } else {
      // In-scale but above the 8.5 V limit: a genuine measured overvoltage (a healthy 2S pack
      // never exceeds 8.4 V). The measured value is logged for diagnosis.
      DBG_POWER_MANAGEMENT(F("CRITICAL: Overvoltage detected: "));
      DBG_POWER_MANAGEMENT(voltageMv);
      DBG_POWER_MANAGEMENT(F(" mV (>8.5V limit); "));
      DBGLN_POWER_MANAGEMENT(F("power cycle required"));
    }
  }

  // Enter low-battery warning mode and start its repeating signal.
  void enterBatteryWarning() {
    if (batteryState == BatteryState::Warning) return;
    DBGLN_POWER_MANAGEMENT(F("Battery WARNING level: entering restricted mode"));
    batteryState = BatteryState::Warning;
    applyBatteryRestrictions();
    lastBatteryWarningSignalMs = millis();
    startBatterySignal(BATTERY_WARNING_SIGNAL_MS, true);
    #if ENABLE_EEPROM_LOGGING
    logBatteryEvent(EEPROM_EVENT_WARNING, batteryVoltage);
    #endif
  }

  // Leave low-battery warning mode after the pack has recovered enough.
  void exitBatteryWarning() {
    if (batteryState != BatteryState::Warning) return;
    DBGLN_POWER_MANAGEMENT(F("Battery warning cleared after recharge margin"));
    batteryState = BatteryState::Normal;
    lastBatteryWarningSignalMs = 0;
    if (!batterySignalActive) SetRearRedLight(false);
    refreshDriveLights();
  }

  // Enter the shutdown-preparation state before permanent sleep.
  void enterBatteryShutdown(bool startupLockout, ShutdownCause cause) {
    if (batteryState == BatteryState::Shutdown) {
      if (startupLockout && !shutdownSignalPlayedThisBoot) {
        shutdownSignalPlayedThisBoot = true;
        startBatterySignal(BATTERY_SHUTDOWN_SIGNAL_MS, true);
      }
      return;
    }

    shutdownCause = cause;
    if (cause == ShutdownCause::LowVcc) {
      DBGLN_POWER_MANAGEMENT(F("VCC SHUTDOWN level: entering lockout"));
    } else {
      DBGLN_POWER_MANAGEMENT(F("VIN SHUTDOWN level: entering lockout"));
    }
    batteryState = BatteryState::Shutdown;
    shutdownSignalPlayedThisBoot = startupLockout;
    applyBatteryRestrictions();
    stopAndResetStepSelection(true);
    digitalWrite(pinMotorSleep, LOW);
    digitalWrite(pinDistanceSensorXSHUT, LOW);
    distanceTofDetected = false;
    distanceTofFaultLatched = true;
    startBatterySignal(BATTERY_SHUTDOWN_SIGNAL_MS, true);
    #if ENABLE_EEPROM_LOGGING
    logBatteryEvent(EEPROM_EVENT_SHUTDOWN, batteryVoltage);
    #endif
  }

  // (There is intentionally no exitBatteryShutdown(): a low-battery shutdown is final for the
  // power-on session. Recovery requires recharging the pack and a physical power cycle.)


  // Select the internal 1.1V ADC reference.
  void initBatteryVoltageMeterHardware() {
    analogReference(INTERNAL);
    delay(5);  // Let the internal 1.1V reference settle before discarding the first conversion.
    analogRead(pinBatterySense);
  }

  // Measure the fixed internal bandgap against AVCC, then restore the 1.1V reference required by
  // the battery divider. Reading ADCL before ADCH locks the pair until the high byte is read.
  uint16_t getVccVoltageMv() {
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    delay(2);

    ADCSRA |= _BV(ADSC);
    while (ADCSRA & _BV(ADSC)) {}
    (void)ADCL;
    (void)ADCH;

    ADCSRA |= _BV(ADSC);
    while (ADCSRA & _BV(ADSC)) {}
    uint8_t low = ADCL;
    uint8_t high = ADCH;
    uint16_t bandgapReading = ((uint16_t)high << 8) | low;

    initBatteryVoltageMeterHardware();
    if (bandgapReading == 0) {
      DBGLN_POWER_MANAGEMENT(F("VCC meter read failed"));
      return 0;
    }

    uint32_t scaledMicrovolts = VCC_BANDGAP_CALIBRATION_UV * BATTERY_ADC_MAX;
    return (uint16_t)((scaledMicrovolts + (bandgapReading / 2U)) / bandgapReading / 1000UL);
  }

  bool confirmLowVccAtStartup() {
    consecutiveLowVccSamples = 0;
    for (uint8_t sampleIndex = 0; sampleIndex < VCC_LOW_CONFIRMATION_COUNT; ++sampleIndex) {
      vccVoltage = getVccVoltageMv();
      if (vccVoltage == 0 || vccVoltage >= VCC_LOW_SHUTDOWN_MV) {
        consecutiveLowVccSamples = 0;
        return false;
      }
      ++consecutiveLowVccSamples;
      if (sampleIndex + 1U < VCC_LOW_CONFIRMATION_COUNT) delay(VCC_CHECK_INTERVAL_MS);
    }
    return true;
  }

  void updateVccGuard() {
    if (batteryState == BatteryState::Shutdown) return;

    unsigned long now = millis();
    if (now - lastVccCheckMs < VCC_CHECK_INTERVAL_MS) return;
    lastVccCheckMs = now;

    vccVoltage = getVccVoltageMv();
    if (vccVoltage == 0) {
      consecutiveLowVccSamples = 0;
      return;
    }

    if (vccVoltage < VCC_LOW_SHUTDOWN_MV) {
      if (consecutiveLowVccSamples < VCC_LOW_CONFIRMATION_COUNT) ++consecutiveLowVccSamples;
      DBG_POWER_MANAGEMENT(F("VCC low: "));
      DBG_POWER_MANAGEMENT(vccVoltage);
      DBG_POWER_MANAGEMENT(F(" mV (sample "));
      DBG_POWER_MANAGEMENT(consecutiveLowVccSamples);
      DBG_POWER_MANAGEMENT(F("/"));
      DBG_POWER_MANAGEMENT(VCC_LOW_CONFIRMATION_COUNT);
      DBGLN_POWER_MANAGEMENT(F(")"));
      if (consecutiveLowVccSamples >= VCC_LOW_CONFIRMATION_COUNT) {
        enterBatteryShutdown(false, ShutdownCause::LowVcc);
      }
    } else {
      consecutiveLowVccSamples = 0;
    }
  }

  // Read one battery-percent lookup-table entry from PROGMEM.
  uint16_t batteryPercentVoltageAt(uint8_t index) {
    return pgm_read_word(&batteryPercentMvTable[index]);
  }

  // Enter inactivity sleep and stay there until a valid wake command is captured.
  void goToIdle() {
    DBGLN_IR_REMOTE(F("Idle: turning everything off..."));
    DBGLN_POWER_MANAGEMENT(F("Idle sleep: entering sleep"));
    idleSleepActive = true;

    noTone(pinBuzzer);
    digitalWrite(pinBuzzer, LOW);
    clearBuzzerPattern();

    setColorSensorEnabled(false);
    SetRGBColor(RgbColor::Off);
    SetGreenLightValue(0);
    Stop();

    playPattern(pattern_descend);
    waitForPatternPlayback(2000);  // Blocking only during power-down so the goodbye jingle can finish cleanly.
    digitalWrite(pinBuzzer, LOW);

    // Park the MPU-6050 in its low-power sleep mode (~5 uA instead of ~3.8 mA) for the whole idle
    // sleep; wakeAccelerometer() below restores it once the train wakes up.
    sleepAccelerometer();

    DBGLN_IR_REMOTE(F("Entering sleep mode..."));
    for (;;) {
      digitalWrite(pinMotorSleep, LOW);  // Disable the DRV8833 while the Arduino sleeps.
      digitalWrite(pinDistanceSensorXSHUT, LOW); // Turn off distance sensor for sleep
      idleWakeRequested = false;
      // attachInterrupt(pin, function, mode) tells the chip to run "function" automatically the
      // instant the given pin changes state matching "mode" (CHANGE = either LOW->HIGH or
      // HIGH->LOW), even while the CPU is in deep sleep below - this is how the sketch "wakes up"
      // as soon as the IR receiver pin toggles (i.e., a remote signal arrives), without having to
      // constantly poll it. wakeUp() (the interrupt service routine / ISR) just sets
      // idleWakeRequested = true (see the "volatile" explanation on that variable in
      // arduino-locomotive.ino) so this loop below can react to it once the CPU resumes.
      attachInterrupt(digitalPinToInterrupt(pinIRReceiver), wakeUp, CHANGE);
      uint8_t sleepCyclesSinceHeartbeat = 0;
      while (!idleWakeRequested) {
        #if defined(__AVR__)
        wdt_disable();
        #endif
        // LowPower.powerDown(SLEEP_8S, ...) sleeps for up to 8 seconds at a time (the longest single
        // step the AVR watchdog-based sleep timer supports) rather than forever, so this loop can
        // periodically wake on its own (even with no remote signal) to run the "heartbeat" red-LED
        // blink below, confirming the train is still alive and just sleeping. The watchdog timer is
        // disabled beforehand and re-armed after, since LowPower's timed sleep repurposes the same
        // watchdog hardware that would otherwise reset the chip if left enabled during a long sleep.
        LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
        #if defined(__AVR__)
        wdt_enable(WDTO_2S);
        wdt_reset();
        #endif
        if (idleWakeRequested) break;
        if (++sleepCyclesSinceHeartbeat >= (IDLE_SLEEP_HEARTBEAT_MS / 8000UL)) {
          SetRearRedLight(true);
          delay(IDLE_SLEEP_HEARTBEAT_ON_MS);
          SetRearRedLight(false);
          sleepCyclesSinceHeartbeat = 0;
        }
      }
      detachInterrupt(digitalPinToInterrupt(pinIRReceiver));

      digitalWrite(pinDistanceSensorXSHUT, HIGH); // Wake up distance sensor
      delay(10); // boot time
      recoverDistanceSensorAfterXshut();

      digitalWrite(pinMotorSleep, HIGH);
      delay(1);  // DRV8833 wake-up time before checking its diagnostic output.
      wakeAccelerometer();  // Resume MPU-6050 measurements; its configuration survived the sleep.
      initBatteryVoltageMeterHardware();  // ADC was powered down; re-establish the 1.1V reference.
      if (captureWakeIrCommand(250UL)) {
        break;
      }
    }
    idleSleepActive = false;
    lastActive = millis();
    DBGLN_POWER_MANAGEMENT(F("Idle sleep: wake successful"));
    playToneSequence_P(melodyWakeReady, false);
    refreshDriveLights();
  }

  // Interrupt callback used only to wake the MCU.
  // This is the ISR (Interrupt Service Routine) that attachInterrupt() above registers - the AVR
  // hardware calls it automatically the instant the watched pin changes, interrupting whatever the
  // main loop was doing. ISRs must stay extremely short and simple: no delay(), no Serial printing,
  // and no lengthy calculations, because the rest of the program is paused the whole time it runs.
  // That's why this one does nothing but set a flag (idleWakeRequested) and return immediately,
  // leaving all the real work to be handled afterward in normal code (goToIdle() above), once
  // execution resumes.
  void wakeUp() {
    idleWakeRequested = true;
  }

  // Poll briefly after wake so the waking IR press can be reused as a command.
  bool captureWakeIrCommand(unsigned long timeoutMs) {
    unsigned long startedAt = millis();
    while ((millis() - startedAt) < timeoutMs) {
      #if defined(__AVR__)
      wdt_reset();
      #endif
      uint8_t code = irReceive();
      if (code != 0) {
        pendingIRCommand = code;
        pendingIRWasRepeat = lastWasRepeat;
        return true;
      }
      delay(2);
    }
    return false;
  }

  // Measure battery voltage directly under the current load.
  // Read pack voltage through the resistor divider.
  // The battery pack's real voltage (up to ~8.4V on a 2S pack) is higher than the Nano's ADC can
  // safely measure directly, so the hardware uses a resistor divider (two resistors forming a
  // voltage-proportional "tap point") to scale it down to a safe range before it reaches
  // pinBatterySense; BATTERY_MILLIVOLT_SCALE_NUMERATOR (from config.h) undoes that scaling in
  // software to recover the real pack voltage. BATTERY_ADC_SAMPLES readings are taken and the
  // median is used instead of a plain mean: the median rejects transient half-scale outliers
  // caused by ADC sample-and-hold cross-talk when other analog pins are read nearby (a single bad
  // sample would drag a mean down noticeably, but cannot move the middle value of the sorted set).
  // For an even sample count the median is the mean of the two middle samples, which keeps 0.5-LSB
  // resolution. As with other integer math in this project, the "+ half-of-divisor" before the
  // final division rounds to the nearest value instead of always truncating downward.
  uint16_t getBatteryVoltageDirect(bool underLoad = false) {
    #if DISABLE_VOLTAGE_METERING
    return 5000;  // 5.0V constant
    #else
    uint16_t rawSamples[BATTERY_ADC_SAMPLES];

    // VCC checks temporarily select AVCC to measure the internal bandgap. Re-select the 1.1V
    // reference for every divider measurement so a missed/rapid reference transition cannot make
    // the 5V-referenced raw value look like an implausibly low battery voltage.
    initBatteryVoltageMeterHardware();

    // Throw away one additional conversion so the ADC sampling capacitor settles on the battery divider.
    analogRead(pinBatterySense);

    for (uint8_t sampleIndex = 0; sampleIndex < BATTERY_ADC_SAMPLES; ++sampleIndex) {
      rawSamples[sampleIndex] = (uint16_t)analogRead(pinBatterySense);
    }

    // Insertion sort ascending. BATTERY_ADC_SAMPLES is tiny (8), so O(N^2) is cheaper in flash
    // and stack than any smarter sort, and it runs in well under a millisecond on the ATmega328P.
    for (uint8_t i = 1; i < BATTERY_ADC_SAMPLES; ++i) {
      uint16_t key = rawSamples[i];
      uint8_t j = i;
      while (j > 0 && rawSamples[j - 1] > key) {
        rawSamples[j] = rawSamples[j - 1];
        --j;
      }
      rawSamples[j] = key;
    }

    // Median: for even N, mean of the two middle samples (keeps 0.5-LSB resolution as raw*2).
    // For odd N, both middle indices land on the same sample, so the same formula still works.
    uint16_t midLow = rawSamples[(BATTERY_ADC_SAMPLES - 1U) / 2U];
    uint16_t midHigh = rawSamples[BATTERY_ADC_SAMPLES / 2U];
    uint32_t medianRawTimes2 = (uint32_t)midLow + (uint32_t)midHigh;  // == 2 * median
    uint16_t medianRaw = (uint16_t)((medianRawTimes2 + 1U) / 2U);     // rounded to nearest raw count

    // mV = median_raw * SCALE / ADC_MAX, computed from medianRawTimes2 to preserve the half-count.
    uint32_t scaledDoubled = medianRawTimes2 * BATTERY_MILLIVOLT_SCALE_NUMERATOR;
    uint16_t medianVoltageMv = (uint16_t)((scaledDoubled + (uint32_t)BATTERY_ADC_MAX)
                             / ((uint32_t)BATTERY_ADC_MAX * 2UL));

    #if DEBUG_POWER_MANAGEMENT
    // Print every completed ADC burst so button-9, periodic, and low-reading confirmation paths
    // can be compared directly in the serial log.
    uint16_t medianRawTimes10 = (uint16_t)(medianRawTimes2 * 5U);
    DBG_POWER_MANAGEMENT(underLoad ? F("Battery ADC (loaded) median raw=") : F("Battery ADC median raw="));
    DBG_POWER_MANAGEMENT(medianRawTimes10 / 10);
    if (medianRaw >= BATTERY_ADC_MAX) {
      DBGLN_POWER_MANAGEMENT(F(" -> meter saturated/disconnected (critical latch)"));
    } else if (medianVoltageMv > BATTERY_MAX_VALID_MV) {
      DBGLN_POWER_MANAGEMENT(F(" -> >8.5V overvoltage (critical latch)"));
    } else if (medianRawTimes10 >= (BATTERY_ADC_MAX * 10U)) {
      DBGLN_POWER_MANAGEMENT(F(" -> meter unavailable/ADC error"));
    } else {
      DBG_POWER_MANAGEMENT(F("."));
      DBG_POWER_MANAGEMENT(medianRawTimes10 % 10);
      DBG_POWER_MANAGEMENT(F(" -> V="));
      printVoltageMv(medianVoltageMv);
      DBGLN_POWER_MANAGEMENT(F(" V"));
    }
    #endif

    if (isCriticalOvervoltage(medianVoltageMv)) {
      enterCriticalOvervoltage(medianVoltageMv, medianRaw);
    }
    return medianVoltageMv;
    #endif
  }

  // Stop loads, let the pack settle, then measure.
  uint16_t getBatteryVoltageSettledForStatus() {
    cancelPendingBatteryGuardMeasurement();
    exitAutoDistanceMode();
    cancelJog();
    stopAndResetStepSelection(true);
    Speed = 0;
    setColorSensorEnabled(false);
    sirenActive = false;
    stopMelody();
    noTone(pinBuzzer);
    clearBuzzerPattern();

    SetRGBColor(RgbColor::Off);
    SetGreenLightValue(0);
    Stop();

    delay(200);  // Blocking: let the pack settle unloaded before sampling for an accurate reading.
    return getBatteryVoltageDirect();
  }

  // Judge pack health only while stopped (no motor sag) and no more than once per interval.
  // Periodically check voltage and enforce low-battery limits.
  void updateBatteryGuard() {
    unsigned long now = millis();
    if (batteryState == BatteryState::Warning
        && !batterySignalActive
        && now - lastBatteryWarningSignalMs >= BATTERY_WARNING_REPEAT_MS) {
      lastBatteryWarningSignalMs = now;
      DBGLN_POWER_MANAGEMENT(F("Battery WARNING level: issuing periodic signal"));
      startBatterySignal(BATTERY_WARNING_SIGNAL_MS, true);
    }

    if (batteryState == BatteryState::Shutdown) return;
    if (Speed != 0 || momentaryActive || motorDrivePending || motorFaultLatched) {
      batteryGuardSettling = false;
      batteryGuardConfirmingLow = false;
      return;
    }
    // Skip battery checks entirely while auto-distance mode is on. The train may be momentarily
    // stopped at Speed==0 waiting for an obstacle to clear; a check here would fire mid-session
    // and could latch a spurious shutdown that then locks out the IR remote.
    if (AutoDistanceOnOff) {
      batteryGuardSettling = false;
      batteryGuardConfirmingLow = false;
      return;
    }
    // The button-9 percentage pattern rapidly switches the buzzer pin. Unlike the button-9 status
    // read, this guard does not stop user feedback before sampling, so defer until every audio
    // output is quiet instead of treating a switching-noise burst as the pack voltage.
    if (batterySignalActive || sirenActive || melodyPlaying || isBuzzerPatternPlaying()) {
      batteryGuardSettling = false;
      batteryGuardConfirmingLow = false;
      return;
    }
    if (now - lastBatteryCheckMs < BATTERY_CHECK_INTERVAL_MS) return;

    // Match the manual button-9 measurement's quiet interval without blocking the main loop.
    if (!batteryGuardSettling) {
      // Print before the quiet interval. At low serial baud rates the UART remains electrically
      // active for milliseconds after this call, so logging immediately before analogRead() would
      // make the periodic path less quiet than the button-9 path.
      DBGLN_POWER_MANAGEMENT(F("Battery measurement: periodic safety guard (settling)"));
      batteryGuardSettling = true;
      batteryGuardSettleStartedMs = millis();
      return;
    }
    if (now - batteryGuardSettleStartedMs < 200UL) return;

    uint16_t v = getBatteryVoltageDirect();
    if (criticalOvervoltageLatched) return;
    if (v <= 0) return;

    // A low first burst is not trusted until a second independently settled burst agrees. This
    // prevents a short A0/reference disturbance from becoming one of the three shutdown samples.
    if (!batteryGuardConfirmingLow && v <= BATTERY_LOW_WARNING_MV) {
      firstLowBatteryVoltageMv = v;
      batteryGuardConfirmingLow = true;
      // Give the UART time to finish this diagnostic line before the independent retry burst.
      DBGLN_POWER_MANAGEMENT(F("Battery measurement: periodic low-reading confirmation (settling)"));
      batteryGuardSettleStartedMs = millis();
      return;
    }
    if (batteryGuardConfirmingLow) {
      batteryGuardConfirmingLow = false;
      if (v > BATTERY_LOW_WARNING_MV) {
        DBG_POWER_MANAGEMENT(F("Periodic low reading rejected: "));
        printVoltageMv(firstLowBatteryVoltageMv);
        DBG_POWER_MANAGEMENT(F(" V -> retry "));
        printVoltageMv(v);
        DBGLN_POWER_MANAGEMENT(F(" V"));
        batteryGuardSettling = false;
        lastBatteryCheckMs = now;
        return;
      }
    }

    batteryGuardSettling = false;
    lastBatteryCheckMs = now;
    uint16_t previousBatteryVoltage = batteryVoltage;
    batteryVoltage = v;
    if (previousBatteryVoltage == 0 || abs((int)v - (int)previousBatteryVoltage) >= 50) {
      configureSpeedSteps();
    }

    #if !DISABLE_VOLTAGE_METERING
    // PRODUCTION-only low-battery enforcement. In a DISABLE_VOLTAGE_METERING=1 testing build these
    // checks are compiled out entirely: the assumed constant 5000 mV "reading" from a bench 5 V
    // supply sits below every 2S threshold and would otherwise trigger an instant false shutdown.
    //
    // A single low measurement is not trusted: transient ADC disturbances (S/H cross-talk, AREF
    // dips, motor kickback coinciding with the sample window) have been observed to make an entire
    // burst read low even after the median filter in getBatteryVoltageDirect(). We require
    // BATTERY_LOW_CONFIRMATION_COUNT consecutive low readings before latching, and treat
    // implausibly low values (< BATTERY_IMPLAUSIBLE_MV on a 2S pack that was just healthy) as pure
    // glitches that do not advance the counter at all.
    if (v < BATTERY_IMPLAUSIBLE_MV) {
      DBG_POWER_MANAGEMENT(F("Battery reading implausibly low ("));
      DBG_POWER_MANAGEMENT(v);
      DBGLN_POWER_MANAGEMENT(F(" mV) - treating as ADC glitch, ignored"));
      // Do not advance the low-battery counters; also do not clear them yet - a real deep
      // discharge would just produce another sub-threshold reading next cycle.
      batteryVoltage = previousBatteryVoltage;  // keep the last trusted value visible to the rest of the sketch
    } else if (v <= BATTERY_LOW_SHUTDOWN_MV) {
      if (consecutiveLowBatterySamples < BATTERY_LOW_CONFIRMATION_COUNT) ++consecutiveLowBatterySamples;
      DBG_POWER_MANAGEMENT(F("Battery below shutdown threshold: "));
      printVoltageMv(v);
      DBG_POWER_MANAGEMENT(F(" V ("));
      DBG_POWER_MANAGEMENT(consecutiveLowBatterySamples);
      DBG_POWER_MANAGEMENT(F("/"));
      DBG_POWER_MANAGEMENT(BATTERY_LOW_CONFIRMATION_COUNT);
      DBGLN_POWER_MANAGEMENT(F(")"));
      if (consecutiveLowBatterySamples >= BATTERY_LOW_CONFIRMATION_COUNT) {
        enterBatteryShutdown();
      }
    } else if (v <= BATTERY_LOW_WARNING_MV) {
      consecutiveLowBatterySamples = 0;
      if (consecutiveWarnBatterySamples < BATTERY_LOW_CONFIRMATION_COUNT) ++consecutiveWarnBatterySamples;
      if (consecutiveWarnBatterySamples >= BATTERY_LOW_CONFIRMATION_COUNT) {
        enterBatteryWarning();
      }
    } else {
      consecutiveLowBatterySamples = 0;
      consecutiveWarnBatterySamples = 0;
      if (batteryState == BatteryState::Warning && v >= BATTERY_WARNING_RECOVERY_MV) {
        exitBatteryWarning();
      }
    }
    #endif
  }

  // Map pack voltage to a 0-100% charge estimate for the remote's battery-test readout.
  // batteryPercentMvTable (config.h) is a small PROGMEM lookup table of voltage breakpoints in
  // descending order (100%, 90%, 80%, ... down to 0%), since battery voltage doesn't drop in a
  // straight line as it discharges. This finds which two breakpoints the reading falls between and
  // linearly interpolates between their percentages (same spirit as motorVoltageFromDistanceMm() in
  // 20-motor.ino) for a smoother estimate than snapping to the nearest 10% step.
  // Entries are read one at a time straight from flash via batteryPercentVoltageAt() /
  // pgm_read_word() instead of copying the whole 22-byte table onto the stack first.
  int get2SBatteryPercent(uint16_t voltageMv) {
    const uint16_t maxVolt = batteryPercentVoltageAt(0);
    const uint16_t minVolt = batteryPercentVoltageAt(batteryPercentTableSize - 1);

    if (voltageMv >= maxVolt) return 100;
    if (voltageMv <= minVolt) return 0;

    for (uint8_t i = 0; i < (uint8_t)(batteryPercentTableSize - 1); ++i) {
      const uint16_t vUpper = batteryPercentVoltageAt(i);
      const uint16_t vLower = batteryPercentVoltageAt(i + 1);
      if (voltageMv > vLower) {
        const int pUpper = 100 - (i * 10);
        const int pLower = pUpper - 10;
        const long numerator = (long)(voltageMv - vLower) * (long)(pUpper - pLower);
        const long denominator = (long)(vUpper - vLower);
        return pLower + (int)(numerator / denominator);
      }
    }

    return 0;
  }

  // Return the pack voltage used by motor-control calculations.
  // No live measurement is performed here: motor PWM math uses the last cached reading taken by
  // updateBatteryGuard() every BATTERY_CHECK_INTERVAL_MS (see 50-power-management.ino), which is a
  // no-load reading captured while parked/between commands. Measuring under load produced noisy
  // sag values (motor inrush pulling the pack to ~5 V for tens of milliseconds) that were only
  // ever remapped back to 8400 mV by the safety floor below, so the extra ADC work bought nothing.
  //
  // Guard against ADC/divider miscalibration and startup-before-first-measurement: any reading
  // under 6.0 V is well below the healthy range of a 2S 18650 pack (a real pack that low would
  // have already tripped shutdown), so it is replaced with 8400 mV (nominal fully-charged 2S) for
  // motor-control math only. This prevents safePwmFromMilliVolts() from clamping to 255 and
  // over-driving the motor. Battery shutdown/warning logic in updateBatteryGuard() is
  // intentionally NOT covered by this fallback.
  uint16_t getLoadedBatteryVoltageForMotorControl() {
    if (criticalOvervoltageLatched) return 0;
    uint16_t supplyMv = batteryVoltage;
    if (supplyMv == 0 || supplyMv < 6000) supplyMv = 8400;
    return supplyMv;
  }

  // Convert a requested motor voltage into PWM based on the current pack voltage.
  // Convert a target motor voltage to a safe PWM value.
  int safePWMFromVoltage(uint16_t desiredMotorMv, uint16_t supplyVoltageMv) {
    return safePwmFromMilliVolts(desiredMotorMv, supplyVoltageMv, MAX_SAFE_MOTOR_MV);
  }