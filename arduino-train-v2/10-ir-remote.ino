  // ================================================================================================
  // File description
  // ================================================================================================
  // IR remote button definitions, decode helpers, and command routing live here.
  // This file owns the mapping from raw NEC button codes to train actions.

  // ================================================================================================
  // Remote Button Codes (NEC protocol, "Car MP3" remote control)
  // ================================================================================================
  // Format: Decimal code | Label
  //  69 | CH-     70 | CH      71 | CH+
  //  68 | <<      64 | >>      67 | Play/Pause
  //   7 | -       21 | +        9 | EQ
  //  22 | 0       25 | 100+    13 | 200+
  //  12 | 1       24 | 2       94 | 3
  //   8 | 4       28 | 5       90 | 6
  //  66 | 7       82 | 8       74 | 9

  // IR remote button assignments
  const int buttonCHminus = 69;     // Speed -
  const int buttonCH = 70;          // Stop
  const int buttonCHplus = 71;      // Speed +
  const int buttonBackward = 68;    // Momentary backward
  const int buttonForward = 64;     // Momentary forward
  const int buttonPlayPause = 67;   // Auto-speed toggle, start/stop
  const int buttonEQ = 9;           // Mute / Unmute
  const int button0 = 22;           // Color sensor ON/OFF
  const int button100plus = 25;     // Horn
  const int button200plus = 13;     // Siren
  const int button1 = 12;           // Play music 1
  const int button2 = 24;           // Play music 2
  const int button3 = 94;           // Play music 3
  const int button4 = 8;            // Play music 4
  const int button5 = 28;           // Play music 5
  const int button6 = 90;           // Play music 6
  const int button7 = 66;           // Play music 7
  const int button8 = 82;           // Play music 8
  const int button9 = 74;           // Battery Test

  #if DEBUG_IR_REMOTE
  // "const __FlashStringHelper*" is the type that F("...") strings actually have (see the F() macro
  // explanation in arduino-train-v2.ino). Returning this type instead of a normal "const char*"
  // means every string below (F("CH-"), F("Stop"), etc.) stays stored in flash instead of being
  // copied into SRAM, and each "switch" acts as a simple lookup table converting a raw button code
  // into its matching text label.
  // Return a short printed label for a raw IR button code.
  const __FlashStringHelper* irButtonLabel(uint8_t code) {
    switch (code) {
      case buttonCHminus: return F("CH-");
      case buttonCH: return F("CH");
      case buttonCHplus: return F("CH+");
      case buttonBackward: return F("<<");
      case buttonForward: return F(">>");
      case buttonPlayPause: return F("Play/Pause");
      case buttonEQ: return F("EQ");
      case button0: return F("0");
      case button100plus: return F("100+");
      case button200plus: return F("200+");
      case button1: return F("1");
      case button2: return F("2");
      case button3: return F("3");
      case button4: return F("4");
      case button5: return F("5");
      case button6: return F("6");
      case button7: return F("7");
      case button8: return F("8");
      case button9: return F("9");
      default: return F("Unknown");
    }
  }

  // Return a human-readable action summary for a raw IR button code.
  const __FlashStringHelper* irButtonActionDescription(uint8_t code) {
    switch (code) {
      case buttonCHminus: return F("Speed -");
      case buttonCH: return F("Stop");
      case buttonCHplus: return F("Speed +");
      case buttonBackward: return F("Momentary backward");
      case buttonForward: return F("Momentary forward");
      case buttonPlayPause: return F("Auto-speed toggle, start/stop");
      case buttonEQ: return F("Mute / Unmute");
      case button0: return F("Color sensor ON/OFF");
      case button100plus: return F("Horn");
      case button200plus: return F("Siren");
      case button1: return F("Play music 1");
      case button2: return F("Play music 2");
      case button3: return F("Play music 3");
      case button4: return F("Play music 4");
      case button5: return F("Play music 5");
      case button6: return F("Play music 6");
      case button7: return F("Play music 7");
      case button8: return F("Play music 8");
      case button9: return F("Battery Test");
      default: return F("Unassigned button");
    }
  }
  #endif

  // Map a remote button code to a melody and start playback.
  bool tryPlayMelodyForButton(uint8_t code) {
    switch (code) {
      case button1: playToneSequence_P(melodyDemo, false); return true;
      case button2: playToneSequence_P(melodyTwinkle, false); return true;
      case button3: playToneSequence_P(melodyOdeToJoy, false); return true;
      case button4: playToneSequence_P(melodyMary, false); return true;
      case button5: playToneSequence_P(melodyWheels, false); return true;
      case button6: playToneSequence_P(melodyHappy, false); return true;
      case button7: playToneSequence_P(melodyBabyShark, false); return true;
      case button8: playToneSequence_P(melodyJingle, false); return true;
      default: return false;
    }
  }

  // Wrap IRremote so the rest of the sketch sees a simple 8-bit command stream.
  // Decode NEC commands, including repeat frames.
  // The NEC infrared protocol sends one full code when a button is first pressed, then - while the
  // button stays held down - short "repeat" frames that don't carry the button code again, only a
  // "still held" marker. IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT (a bitwise AND
  // against a single-bit flag) detects this case, and the code reuses lastIRCommand (the most
  // recent real button code) so callers still know which button is being held.
  // pendingIRCommand is a tiny one-slot queue: some code paths need to "hold onto" a decoded command
  // for the next call to irReceive() instead of consuming it immediately (see cancelJog()/other
  // call sites), and this static-like queue variable is how that hand-off happens.
  uint8_t irReceive() {
    if (pendingIRCommand != 0) {
      uint8_t queuedCommand = pendingIRCommand;
      lastWasRepeat = pendingIRWasRepeat;
      pendingIRCommand = 0;
      pendingIRWasRepeat = false;
      return queuedCommand;
    }

    lastWasRepeat = false;
    uint8_t received = 0;

    if (IrReceiver.decode()) {
      if (IrReceiver.decodedIRData.protocol == NEC) {
        if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) {
          lastWasRepeat = true;
          received = lastIRCommand;
        } else {
          received = IrReceiver.decodedIRData.command;  // 8-bit
          lastIRCommand = received;
          #if DEBUG_IR_REMOTE
          DBG_IR_REMOTE(F("IR pressed: "));
          DBG_IR_REMOTE(received);
          DBG_IR_REMOTE(F(" | "));
          DBGLN_IR_REMOTE(irButtonLabel(received));
          #endif
        }
      }
      IrReceiver.resume();
    }
    return received;  // 0 = no key received
  }

  // Identify the subset of buttons that are allowed to bring the motor driver back online.
  // Identify commands that may re-arm the motor driver.
  // Decide whether a button press is allowed to clear a motor-driver fault latch.
  bool isMotorControlCommand(uint8_t code) {
    return code == buttonCHminus || code == buttonCH || code == buttonCHplus
      || code == buttonBackward || code == buttonForward || code == buttonPlayPause;
  }

  // Central behavior router for the handheld remote.
  // This function runs once per loop() and drives every remote-controlled feature. It first decodes
  // one button code (or 0 if nothing new arrived), then works through a series of early-return guard
  // checks (jog cancellation, repeat filtering, battery lockouts, tilt lockout) before finally
  // reaching the big "switch (code)" below, which is the actual command dispatcher: each "case"
  // matches one button and its own { ... } block runs only for that button, then "break;" exits the
  // switch so no other case's code runs afterward.
  void translateIR() {
    uint8_t code = irReceive();
    #if DEBUG_IR_REMOTE
    if (code != 0 && !lastWasRepeat) {
      DBG_IR_REMOTE(F("Assigned action: "));
      DBGLN_IR_REMOTE(irButtonActionDescription(code));
    }
    #endif

    // Cancel jog if a different non-repeat key appears
    if (momentaryActive && code != 0 && !lastWasRepeat && code != momentaryButton) {
      DBGLN_IR_REMOTE(F("Cancelling jog due to new key"));
      Stop();
      SetRGBColor(RgbColor::Red);
      cancelJog();
    }

    // Ignore repeats for non-jog use cases (prevents CH± spam)
    if (lastWasRepeat && !momentaryActive) {
      return;
    }

    // No new code; if jogging and repeats stopped -> timeout
    if (code == 0) {
      if (momentaryActive && (millis() - momentaryLastSeen > momentaryTimeout)) {
        DBGLN_MOTOR(F("Jog timeout -> STOP"));
        Stop();
        SetRGBColor(RgbColor::Red);
        cancelJog();
      }
      return;
    }

    if (batteryState == BatteryState::Shutdown) {
      if (!lastWasRepeat) {
        DBGLN_IR_REMOTE(F("Ignored: battery shutdown lockout"));
      }
      return;
    }

    if (batteryState == BatteryState::Warning && !isWarningModeCommandAllowed(code)) {
      if (!lastWasRepeat) {
        DBGLN_IR_REMOTE(F("Ignored: battery warning restrictions"));
        // Denial cue: bypass the battery-restriction sound gate so the user still hears it.
        playPattern(pattern_tiltBeep, true);
      }
      return;
    }

    if (tiltStopLatched && isMotorControlCommand(code)) {
      if (!lastWasRepeat) {
        DBGLN_TILT_SENSOR(F("Ignored: tilt lockout active"));
        // Denial cue: bypass the battery-restriction sound gate so the user still hears it.
        playPattern(pattern_tiltBeep, true);
      }
      return;
    }

    lastActive = millis();
    idleSleepWarningIssued = false;

    // Blink green LED once for any valid button press
    GreenLEDBlink();

    // A motor-control button may re-arm only after the diagnostic output is clear.
    if (motorFaultLatched && isMotorControlCommand(code) && !rearmMotorDriver()) {
      return;
    }

    switch (code) {
      case buttonCHminus:
        {  // Speed -
          if (momentaryActive) {
            DBGLN_IR_REMOTE(F("Ignored: CH- during jog"));
            break;
          }
          if (AutoDistanceOnOff == 0) {
            decreaseStep();
            DBG_MOTOR(F("Manual Speed Down: "));
            DBGLN_MOTOR(Speed);
            if (Speed == 0) {
              Stop();
              SetRGBColor(RgbColor::Red);
            } else {
              if (MotorDirection == 1) GoForward();
              if (MotorDirection == 2) GoBackward();
            }
          } else {
            DBGLN_IR_REMOTE(F("Ignored: Auto-speed active"));
          }
          break;
        }

      case buttonCH:
        {  // Stop
          if (AutoDistanceOnOff) {
            exitAutoDistanceMode();
            DBGLN_IR_REMOTE(F("Switched from AUTO to MANUAL mode"));
          }
          DBGLN_MOTOR(F("STOP pressed -> Motors stopped"));
          SetRGBColor(RgbColor::Red);
          stopAndResetStepSelection(true);  // default to forward when stopped
          DBGLN_IR_REMOTE(F("Manual mode reset: next CH+ will start at step 1 (~3.5V)"));
          break;
        }

      case buttonCHplus:
        {  // Speed +
          if (momentaryActive) {
            DBGLN_IR_REMOTE(F("Ignored: CH+ during jog"));
            break;
          }
          if (AutoDistanceOnOff == 0) {
            increaseStep();
            DBG_MOTOR(F("Manual Speed Up: "));
            DBGLN_MOTOR(Speed);
            if (MotorDirection == 1) {
              GoForward();
              SetRGBColor(RgbColor::White);
            }
            if (MotorDirection == 2) {
              GoBackward();
              SetRGBColor(RgbColor::Blue);
            }
          } else {
            DBGLN_IR_REMOTE(F("Ignored: Auto-speed active"));
          }
          break;
        }

      case buttonBackward:
        {  // << momentary backward (jog)
          // Jog only starts from a fully stationary train: Speed == 0 rules out an active manual
          // forward/backward drive, and AutoDistanceOnOff == 0 rules out auto-distance mode (auto
          // can also leave Speed == 0 momentarily while waiting for an obstacle to clear, so both
          // checks are required). Direction is fixed by the button pressed; JogDrive() always runs
          // at the fixed step-1 speed (pwmSteps[1]) regardless of the last selected manual step.
          if (AutoDistanceOnOff == 0 && Speed == 0) {
            JogDrive(Dir::Backward);
            SetRGBColor(RgbColor::Blue);
            momentaryActive = true;
            momentaryButton = buttonBackward;
            momentaryLastSeen = millis();
            DBGLN_MOTOR(F("Momentary BACKWARD running (hold to move)"));
          } else {
            DBGLN_IR_REMOTE(F("Ignored: << only when stationary & not in AUTO"));
          }
          break;
        }

      case buttonForward:
        {  // >> momentary forward (jog)
          // Same stationary-only guard as buttonBackward above: Speed == 0 blocks jog while a
          // manual drive is running, and AutoDistanceOnOff == 0 blocks jog while auto-distance mode
          // is active (even if it is momentarily stopped at Speed == 0). Direction is fixed by the
          // button pressed; JogDrive() always runs at the fixed step-1 speed (pwmSteps[1]).
          if (AutoDistanceOnOff == 0 && Speed == 0) {
            JogDrive(Dir::Forward);
            SetRGBColor(RgbColor::White);
            momentaryActive = true;
            momentaryButton = buttonForward;
            momentaryLastSeen = millis();
            DBGLN_MOTOR(F("Momentary FORWARD running (hold to move)"));
          } else {
            DBGLN_IR_REMOTE(F("Ignored: >> only when stationary & not in AUTO"));
          }
          break;
        }

      case button0:
        {  // Color sensor toggle
          setColorSensorEnabled(ColorSensorOnOff == 0);
          break;
        }

      case buttonPlayPause:
        {  // Auto-speed toggle
          if (boostActive) Stop();
          AutoDistanceOnOff = !AutoDistanceOnOff;
          SetGreenLightValue(AutoDistanceOnOff ? 255 : 0);
          if (AutoDistanceOnOff) {
            DBGLN_MOTOR(F("Driving started (auto-speed)"));
            SetRGBColor(RgbColor::White);
            GoForward();
            playPattern(pattern_double);
          } else {
            DBGLN_MOTOR(F("Driving stopped"));
            SetRGBColor(RgbColor::Red);
            stopAndResetStepSelection();
            Speed = 0;
            playPattern(pattern_descend);
            DBGLN_IR_REMOTE(F("Manual mode rearmed: next step = 1 (~3.5V)"));
          }
          break;
        }

      case buttonEQ:
        {  // Mute / Unmute
          SoundOnOff = !SoundOnOff;
          if (SoundOnOff) {
            DBGLN_SOUND(F("Sound ON"));
          } else {
            DBGLN_SOUND(F("Sound OFF"));
          }

          if (!SoundOnOff) {
            // Hard stop any audio that's playing
            noTone(pinBuzzer);
            digitalWrite(pinBuzzer, LOW);
            clearBuzzerPattern();
            // DO NOT touch sirenActive or LEDs -> lights continue flashing if sirenActive==true
          } else {
            playPattern(pattern_double);  // short confirmation chirp
          }
          break;
        }

      case button100plus:
        {  // Horn
          DBGLN_SOUND(F("Horn activated"));
          playPattern(pattern_horn);
          break;
        }

      case button200plus:
        sirenActive = !sirenActive;
        if (!sirenActive) {
          noTone(pinBuzzer);
          SetRGBColor(RgbColor::Off);
        } else {
          sirenTimer = millis();      // for LED swap
          sirenStartMs = sirenTimer;  // for deterministic audio sweep
          DBGLN_SOUND(SoundOnOff ? F("Siren ON (with sound)") : F("Siren ON (lights only, muted)"));
        }
        break;

      case button9:
        {  // Beep battery level in 10% steps
          playPattern(pattern_double);
          waitForPatternPlayback(500);
          uint16_t vIn = getBatteryVoltageSettledForStatus();
          int batteryPercent = get2SBatteryPercent(vIn);
          DBG_VOLTAGE_METER(F("Battery Voltage: "));
          DBG_VOLTAGE_METER(vIn / 1000);
          DBG_VOLTAGE_METER(F("."));
          DBGLN_VOLTAGE_METER((vIn % 1000) / 10);
          DBG_VOLTAGE_METER(F("Battery level: "));
          DBG_VOLTAGE_METER(batteryPercent);
          DBGLN_VOLTAGE_METER(F("%"));
          playVoltagePattern(batteryPercent);
          break;
        }

      default:
        if (tryPlayMelodyForButton(code)) {
          break;
        }
        break;
    }

    // Refresh jog heartbeat if the same jog key is still active
    if (momentaryActive && (code == momentaryButton)) {
      momentaryLastSeen = millis();
    }
  }
