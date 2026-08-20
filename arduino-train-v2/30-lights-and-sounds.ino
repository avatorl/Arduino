  // ================================================================================================
  // File description
  // ================================================================================================
  // LED output, buzzer patterns, siren effects, and melody playback live here.
  // These settings mainly affect the train's visual feedback and sound behavior.

  // Initialize the MCP23008 outputs used for all train lights.
  void initTrainLedHardware() {
    trainLedExpanderDetected = trainLedExpander.begin_I2C(mcpAddressTrainLeds);
    if (!trainLedExpanderDetected) {
      DBGLN_LEDS(F("MCP23008 not found; train LEDs expect the expander wiring"));
      return;
    }

    const LedRoute routes[] = { led1R, led1G, led1B, led2R, led2G, led2B, ledGreen, ledRearRed };
    for (uint8_t i = 0; i < sizeof(routes) / sizeof(routes[0]); ++i) {
      trainLedExpander.pinMode(routes[i].expanderPin, OUTPUT);
      trainLedExpander.digitalWrite(routes[i].expanderPin, LOW);
    }

    DBGLN_LEDS(F("MCP23008 ready for train LEDs"));
  }

  // ================================================================================================
  // Buzzer (non-blocking pattern player)
  // ================================================================================================
  // Reset the queue used by updateBuzzer(). Safe to call before loading a new pattern.
  // Reset queued buzzer timing data.
  inline void clearBuzzerPattern() {
    for (int j = 0; j < BUZZER_PATTERN_MAX; ++j) buzzerPattern[j] = 0;
    buzzerIndex = 0;
    buzzerTimer = 0;
  }

  // Return true while a non-blocking buzzer pattern still has steps left to play.
  bool isBuzzerPatternPlaying() {
    return buzzerPattern[buzzerIndex] != 0;
  }

  // Block briefly while pumping updateBuzzer() so a short pattern can finish cleanly.
  void waitForPatternPlayback(unsigned long timeoutMs) {
    unsigned long startedAt = millis();
    while (isBuzzerPatternPlaying() && (millis() - startedAt) < timeoutMs) {
      #if defined(__AVR__)
      wdt_reset();
      #endif
      updateBuzzer();
      delay(1);
    }
    noTone(pinBuzzer);
    digitalWrite(pinBuzzer, LOW);
    clearBuzzerPattern();
  }

  // Advance the current buzzer pattern one timing step at a time.
  // Advance non-blocking buzzer patterns.
  // This is a "non-blocking state machine": instead of using delay() to time each beep (which would
  // freeze the whole sketch and make it deaf to remote/tilt/battery events), it remembers where it
  // is (buzzerIndex) and when the current step started (buzzerTimer), and only acts once per loop()
  // call when enough time (millis() - buzzerTimer) has passed. tone(pin, freq) starts a continuous
  // square wave on that pin (using a hardware timer) until noTone(pin) stops it - this is how
  // Arduino generates a specific audio pitch without the code having to toggle the pin manually.
  void updateBuzzer() {
    // Siren owns the buzzer while active; queued patterns resume once the siren stops.
    if (batterySignalActive) return;
    if (sirenActive) return;
    if (melodyPlaying) return;
    if (buzzerPattern[buzzerIndex] == 0) return;
    unsigned long now = millis();
    if (now - buzzerTimer >= (unsigned long)buzzerPattern[buzzerIndex]) {
      ++buzzerIndex;
      buzzerTimer = now;
      if (buzzerPattern[buzzerIndex] == 0) {
        noTone(pinBuzzer);
        digitalWrite(pinBuzzer, LOW);
        clearBuzzerPattern();
        return;
      }
      // Even indices are ON durations, odd indices are OFF gaps.
      // "(buzzerIndex & 1)" checks only the lowest bit of buzzerIndex, which is a fast way to test
      // even/odd (equivalent to "buzzerIndex % 2" but cheaper on an 8-bit AVR chip): even index means
      // an ON step (start the tone), odd index means an OFF gap (silence).
      if ((buzzerIndex & 1) == 0) tone(pinBuzzer, buzzerPatternToneHz);
      else {
        noTone(pinBuzzer);
        digitalWrite(pinBuzzer, LOW);
      }
    }
  }

  // ------------------------------------------------------------------------------------------------
  // Initiate one of the predefined sound patterns. Then updateBuzzer plays the pattern.
  // ------------------------------------------------------------------------------------------------
  // Start a predefined buzzer pattern.
  // bypassBatteryGate = true skips the areUserSoundsAllowed() battery-restriction check, for short
  // denial cues that must still be audible while battery Warning/Shutdown restrictions are active.
  // It still respects the SoundOnOff mute toggle and defers to an active siren or battery safety
  // signal so those are never interrupted.
  void playPattern(const uint16_t* pattern, bool bypassBatteryGate) {
    if (!bypassBatteryGate && !areUserSoundsAllowed()) return;
    if (SoundOnOff != 1) return;
    if (batterySignalActive || sirenActive) return;
    stopMelody();
    noTone(pinBuzzer);
    digitalWrite(pinBuzzer, LOW);
    clearBuzzerPattern();

    int i = 0;
    for (; i < BUZZER_PATTERN_MAX - 1; ++i) {
      // pattern[] arrays (like pattern_tiltBeep, pattern_horn, etc.) are stored in PROGMEM (flash),
      // not regular RAM, to save SRAM since these tables never change at runtime. Because the AVR
      // CPU cannot read flash the same way it reads RAM, a normal "pattern[i]" access would read
      // garbage; pgm_read_word(&pattern[i]) is the special macro that correctly fetches a 16-bit
      // value from a flash address (using "&pattern[i]" to get that address) instead of RAM.
      uint16_t v = pgm_read_word(&pattern[i]);
      buzzerPattern[i] = v;
      if (v == 0) break;
    }
    buzzerPattern[i] = 0;

    buzzerTimer = millis();
    if (buzzerPattern[0] > 0) tone(pinBuzzer, buzzerPatternToneHz);
  }

  // ------------------------------------------------------------------------------------------------
  // Initiate battery percentage sound pattern. Then updateBuzzer plays the pattern.
  //    - 10%..100% -> 1..10 short beeps
  //    - 0% -> one extra-short beep
  // ------------------------------------------------------------------------------------------------
  // Speak battery voltage with long/short beeps.
  void playVoltagePattern(int batteryPercent) {
    if (!areUserSoundsAllowed() || SoundOnOff != 1) return;
    if (batterySignalActive || sirenActive) return;
    stopMelody();
    noTone(pinBuzzer);
    digitalWrite(pinBuzzer, LOW);
    clearBuzzerPattern();

    int beepCount = constrain(batteryPercent / 10, 0, 10);
    int cap = BUZZER_PATTERN_MAX - 1;
    int idx = 0;

    if (beepCount == 0) {
      buzzerPattern[idx++] = 80;
      buzzerPattern[idx] = 0;
      buzzerTimer = millis();
      tone(pinBuzzer, buzzerPatternToneHz);
      return;
    }

    for (int i = 0; i < beepCount && idx < cap; ++i) {
      if (idx < cap) buzzerPattern[idx++] = 150;
      if (i < beepCount - 1 && idx < cap) buzzerPattern[idx++] = 150;
    }

    buzzerPattern[idx] = 0;
    buzzerTimer = millis();
    if (buzzerPattern[0] > 0) tone(pinBuzzer, buzzerPatternToneHz);
  }

  // ================================================================================================
  // LEDs
  // ================================================================================================
  // Return a human-readable label for one expander output channel.
  const __FlashStringHelper* ledRouteLabel(const LedRoute& route) {
    switch (route.expanderPin) {
      case 0: return F("LED1 red");
      case 1: return F("LED1 green");
      case 2: return F("LED1 blue");
      case 3: return F("LED2 red");
      case 4: return F("LED2 green");
      case 5: return F("LED2 blue");
      case 6: return F("green status LED");
      case 7: return F("rear red LEDs");
      default: return F("unknown output");
    }
  }

  // Lowest-level LED output helper; all train light changes funnel through here.
  // Write one expander-backed LED channel.
  inline void writeTrainOutput(const LedRoute& route, bool Value) {
    if (trainLedExpanderDetected) {
      trainLedExpander.digitalWrite(route.expanderPin, Value ? HIGH : LOW);
    } else {
      DBG_LEDS(F("Expander pin GP"));
      DBG_LEDS(route.expanderPin);
      DBG_LEDS(F(" to "));
      if (Value) {
        DBG_LEDS(F("ON ("));
        DBG_LEDS(ledRouteLabel(route));
        DBGLN_LEDS(F(")"));
      } else {
        DBGLN_LEDS(F("OFF"));
      }
    }
  }

  // Drive one RGB LED as discrete color channels.
  inline void writeRGBPins(const LedRoute& rPin, const LedRoute& gPin, const LedRoute& bPin, bool R, bool G, bool B) {
    // This helper forwards the requested channel state to the expander-backed LED output.
    writeTrainOutput(rPin, R);
    writeTrainOutput(gPin, G);
    writeTrainOutput(bPin, B);
  }

  // Rear red OUT7 channel is reserved for battery warning, shutdown, and inactivity-sleep indication.
  void SetRearRedLight(bool enabled) {
    writeTrainOutput(ledRearRed, enabled);
  }

  // Apply raw RGB channel states to one headlight or both.
  // Apply raw RGB values to one or both headlights.
  void SetRGBLight(bool R, bool G, bool B, int led) {
    if (FrontLightOnOff == 0) return;
    if (!areRgbLightsAllowed()) {
      R = false;
      G = false;
      B = false;
    }
    if (led == 0 || led == 1) writeRGBPins(led1R, led1G, led1B, R, G, B);
    if (led == 0 || led == 2) writeRGBPins(led2R, led2G, led2B, R, G, B);
  }

  // Apply a named RGB color.
  void SetRGBLightColor(RgbColor color, int led) {
    bool R = false, G = false, B = false;
    switch (color) {
      case RgbColor::Red: R = true; break;
      case RgbColor::Green: G = true; break;
      case RgbColor::Blue: B = true; break;
      case RgbColor::Yellow: R = true; G = true; break;
      case RgbColor::Cyan: G = true; B = true; break;
      case RgbColor::Magenta: R = true; B = true; break;
      case RgbColor::White: R = true; G = true; B = true; break;
      default: break;
    }
    SetRGBLight(R, G, B, led);
  }

  // Apply normal status colors while respecting higher-priority modes like siren and sensor mode.
  // Apply status color unless siren/sensor mode overrides it.
  inline void SetRGBColor(RgbColor color, int led) {
    if (sirenActive) return;  // ignore during siren
    if (ColorSensorOnOff == 1 && color != RgbColor::Off) {
      if (led == 0 || led == 1) {
        SetRGBLightColor(RgbColor::Cyan, 1);
      }
      if (led == 0 || led == 2) {
        SetRGBLightColor(color, 2);
      }
      return;
    }
    SetRGBLightColor(color, led);
  }

  // Control the green status LED.
  void SetGreenLightValue(int Value) {
    // analogWrite(pinLEDGreen, Value);
    // The green status LED is driven through the expander-backed output stage.
    if (!isGreenIndicatorAllowed()) Value = 0;
    writeTrainOutput(ledGreen, (Value > 0));  // Any non-zero means ON
  }

  // Restore status lights from the current drive state.
  void refreshDriveLights() {
    if (!areRgbLightsAllowed()) {
      SetRGBLightColor(RgbColor::Off);
      SetGreenLightValue(0);
      return;
    }
    if (sirenActive) return;

    SetGreenLightValue(AutoDistanceOnOff ? 255 : 0);

    if (ColorSensorOnOff == 1) {
      RgbColor driveColor = (Speed == 0) ? RgbColor::Red : (MotorDirection == 2 ? RgbColor::Blue : RgbColor::White);
      SetRGBLightColor(RgbColor::Cyan, 1);
      SetRGBLightColor(driveColor, 2);
      return;
    }

    if (Speed == 0) SetRGBColor(RgbColor::Red);
    else if (MotorDirection == 2) SetRGBColor(RgbColor::Blue);
    else SetRGBColor(RgbColor::White);
  }

  // Start a non-blocking 2-pulse acknowledgement blink; final state restored by updateGreenBlink().
  // Non-blocking status-LED acknowledgement blink.
  void GreenLEDBlink() {
    greenBlinkRemaining = 2;
    greenBlinkOn = true;
    greenBlinkStepMs = millis();
    SetGreenLightValue(255);
  }

  // Advance the acknowledgement blink and restore the auto-mode indicator when finished.
  // Non-blocking status-LED acknowledgement blink.
  void updateGreenBlink() {
    if (greenBlinkRemaining <= 0) return;
    unsigned long now = millis();
    unsigned long dur = greenBlinkOn ? greenBlinkOnMs : greenBlinkOffMs;
    if (now - greenBlinkStepMs < dur) return;
    greenBlinkStepMs = now;
    if (greenBlinkOn) {
      SetGreenLightValue(0);
      greenBlinkOn = false;
    } else {
      greenBlinkRemaining--;
      if (greenBlinkRemaining > 0) {
        SetGreenLightValue(255);
        greenBlinkOn = true;
      } else {
        SetGreenLightValue(AutoDistanceOnOff ? 255 : 0);  // restore auto-speed indicator
      }
    }
  }

  // Force the train into its final no-recovery sleep state after shutdown.
  void performPermanentShutdown() {
    #if defined(__AVR__)
    wdt_disable();
    #endif
    batterySignalActive = false;
    batterySignalUsesTone = false;
    idleWakeRequested = false;
    AutoDistanceOnOff = false;
    ColorSensorOnOff = false;
    momentaryActive = false;
    motorDrivePending = false;
    boostActive = false;
    sirenActive = false;
    stopMelody();
    clearBuzzerPattern();
    noTone(pinBuzzer);
    digitalWrite(pinBuzzer, LOW);
    Stop();
    digitalWrite(pinMotorSleep, LOW);
    digitalWrite(pinVL53L0X_XSHUT, LOW);
    distanceTofDetected = false;
    powerDownColorSensorCore();
    digitalWrite(pinColorSensorLED, colorSensorLEDOffLevel);
    SetRearRedLight(false);
    SetGreenLightValue(0);
    SetRGBLight(false, false, false, 0);
    if (irReceiverStarted) {
      IrReceiver.stop();
      irReceiverStarted = false;
    }
    // detachInterrupt() disconnects the wake-up interrupt so the pin can no longer trigger the
    // wakeUp() ISR (see 50-power-management.ino) - appropriate here because this is a *permanent*
    // shutdown with no wake path, unlike the temporary idle-sleep which keeps the interrupt armed.
    detachInterrupt(digitalPinToInterrupt(pinIRReceiver));

    // LowPower.powerDown(...) puts the whole chip into its deepest sleep mode, drawing very little
    // current so the battery lasts as long as possible while parked. SLEEP_FOREVER disables the
    // periodic watchdog wake-up (normal powerDown() calls elsewhere use a timed interval instead),
    // ADC_OFF turns off the analog-to-digital converter, and BOD_OFF disables the brown-out
    // detector - both are extra power-hungry chip features not needed while permanently asleep.
    // The "for (;;)" infinite loop re-enters sleep immediately in the rare case the chip is ever
    // woken by some other means, since this state is meant to be final until power is removed.
    for (;;) {
      LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    }
  }

  // Siren animation is time-based so pitch and light sweep remain stable if loop timing varies.
  // Animate siren lights and pitch sweep.
  void updateSiren() {
    if (!sirenActive) return;
    if (!areUserSoundsAllowed()) {
      sirenActive = false;
      noTone(pinBuzzer);
      SetRGBLightColor(RgbColor::Off);
      return;
    }

    unsigned long now = millis();

    // LED swap every 300 ms (always runs, even when muted)
    if (now - sirenTimer > 300) {
      sirenPhase = !sirenPhase;
      if (sirenPhase) {
        SetRGBLight(true, false, false, 1);  // LED1 red
        SetRGBLight(false, false, true, 2);  // LED2 blue
      } else {
        SetRGBLight(false, false, true, 1);  // LED1 blue
        SetRGBLight(true, false, false, 2);  // LED2 red
      }
      sirenTimer = now;
    }

    // ---- Deterministic triangle sweep for pitch (immune to loop jitter) ----
    const unsigned long cycleMs = 2UL * sirenSweepMs;  // Total cycle = up + down
    unsigned long t = (now - sirenStartMs) % cycleMs;

    int f;
    if (t < sirenSweepMs) {
      // ramp up: Fmin → Fmax
      f = sirenFmin + (int)((unsigned long)(sirenFmax - sirenFmin) * t / sirenSweepMs);
    } else {
      // ramp down: Fmax → Fmin
      unsigned long td = t - sirenSweepMs;
      f = sirenFmax - (int)((unsigned long)(sirenFmax - sirenFmin) * td / sirenSweepMs);
    }

    // Sound part only if not muted
    if (SoundOnOff == 1) tone(pinBuzzer, f);
    else noTone(pinBuzzer);
  }

  // ================================================================================================
  // Tone melody player (non-blocking)
  //   - Sequence format: flat int array [freq, duration_ms, freq, duration_ms, ...]
  //   - freq = 0 -> rest (silence) for 'duration_ms'
  //   - Call updateMelody() in the main loop to handle playback timing.
  //   - Respects SoundOnOff flag (must be 1 to play).
  //   - Automatically stops if sirenActive is true.
  //   - Supports looping playback.
  //   - Use playToneSequenceRaw(...) to start a new melody.
  //   - Use stopMelody() to stop playback manually.
  // ================================================================================================

  // Stops melody playback and resets related state.
  // Stop melody playback and clear melody state.
  void stopMelody() {
    if (!batterySignalActive && !sirenActive) {
      noTone(pinBuzzer);
      digitalWrite(pinBuzzer, LOW);
    }
    melodyPlaying = false;
    melodyLenPairs = 0;
    melodyIdxPair = 0;
    melodyStepStarted = 0;
  }

  // Reads the note value (frequency or duration) at flat index `idx` from the active
  // sequence, transparently handling PROGMEM- vs RAM-backed sources.
  // Read one note or duration value from the active melody source.
  // melodySrcIsProgmem records whether the currently-playing melody array lives in flash (PROGMEM,
  // read with pgm_read_word - see the pgm_read_word explanation in playPattern above) or in normal
  // RAM (read with a plain array index). Branching here lets one melody player support both kinds
  // of source arrays without needing two separate copies of all the playback code.
  inline int melodyReadAt(int idx) {
    return melodySrcIsProgmem ? (int)(int16_t)pgm_read_word(&melodySrc[idx]) : melodySrc[idx];
  }

  // Advance the current melody using millis()-based timing instead of delay().
  // Advance non-blocking melody playback.
  void updateMelody() {
    if (!melodyPlaying) return;  // Exit if no melody is playing
    if (!areUserSoundsAllowed() || SoundOnOff != 1 || sirenActive) {
      stopMelody();
      return;
    }  // Stop if sound is muted or siren is active

    unsigned long now = millis();

    // Start the first note
    if (melodyStepStarted == 0) {
      int f = melodyReadAt(melodyIdxPair * 2);      // Frequency
      if (f > 0) tone(pinBuzzer, f);
      else noTone(pinBuzzer);
      melodyStepStarted = now;
      return;
    }

    // Continue checking if the current note's duration has elapsed
    int dCur = melodyReadAt(melodyIdxPair * 2 + 1);
    if (now - melodyStepStarted >= (unsigned long)dCur) {
      melodyIdxPair++;  // Move to the next note
      if (melodyIdxPair >= melodyLenPairs) {
        if (melodyLoop) melodyIdxPair = 0;  // Loop back if enabled
        else {
          stopMelody();
          return;
        }  // Stop if done
      }
      int f = melodyReadAt(melodyIdxPair * 2);
      if (f > 0) tone(pinBuzzer, f);
      else noTone(pinBuzzer);
      melodyStepStarted = now;
    }
  }

  // Point the playback state at a flat [frequency, duration] sequence and arm playback.
  // Supports either RAM-backed arrays or PROGMEM-backed arrays depending on isProgmem;
  // notes are read on-the-fly by updateMelody(), no RAM copy is made.
  // Load and start a melody from RAM or PROGMEM.
  void playToneSequenceRaw(const int* seqFD, int pairCount, bool loopPlayback, bool isProgmem) {
    if (pairCount <= 0) return;
    if (!areUserSoundsAllowed() || SoundOnOff != 1) return;
    if (batterySignalActive || sirenActive || isBuzzerPatternPlaying()) return;

    // Limit the number of pairs to prevent overflow
    melodyLenPairs = (pairCount > MELODY_MAX_PAIRS) ? MELODY_MAX_PAIRS : pairCount;

    // Reference the sequence directly; no RAM copy needed.
    melodySrc = (const int16_t*)seqFD;
    melodySrcIsProgmem = isProgmem;

    // Initialize playback state
    melodyIdxPair = 0;
    melodyLoop = loopPlayback;
    melodyPlaying = true;
    melodyStepStarted = 0;  // Triggers first note on next update
  }
