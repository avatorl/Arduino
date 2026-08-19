# Arduino DUPLO Train v2

A 3D-printed DUPLO-compatible train powered by an Arduino Nano and controlled by an IR remote.

Original inspiration: <https://cults3d.com/en/3d-model/game/locomotora-sofia-controlada-por-infrarojos-con-control-de-velocidad-por-ultrasonidos-y-motor-superior-multiusos-lego-duplo>

Arduino code fully rewritten from scratch and significantly extended by Andrzej Leszkiewicz.

3D-print profile: <https://makerworld.com/en/models/1854728-arduino-train-locomotive-remote-controlled#profileId-1983131>

## What this train can do

- Remote-controlled forward and backward driving
- 3 manual speed levels for normal driving + boost mode
- Headlights and backlights / colored status lights
- Horn and siren sounds + 8 melodies
- Battery status announced by beeps
- Automatic sleep after inactivity, with wake-up from the IR remote
- Tilt detection: stops the motor when the train is on its side
- Obstacle detection with automatic stop / restart when using the distance sensor
- Color-marker detection when using the color sensor - the train can react to colored markers on the track with different actions, such as stopping, changing speed, or playing a melody (DUPLO or custom "action blocks" can be used as markers)

## What you need to buy

### Core electronics

- Arduino Nano 3.0 compatible board, ATmega328P
- TSOP4838 IR receiver diode
- DRV8833 motor driver module
- DC geared motor, around 3 V to 6 V, 1:48 ratio
- 21-button IR remote, commonly sold as a "Car MP3" remote
- Passive buzzer
- 2 common-anode RGB LEDs
- 1 green LED
- 2 red LEDs
- 2 x 18650 Li-ion cells
- 2S battery protection module (BMS)
- 2S USB charger module
- Buck converter module to step down to 5 V
- Main power switch
- Fuse

### Sensors

- VL53L0X distance sensor module, for obstacle detection
- SW-520D tilt sensor, for tilt protection
- TCS34725 color sensor module, for color-marker actions

### LED control parts

- MCP23008 I2C GPIO expander (module with MOSFETs)
- alternative: do custom wiring with bare MCP23008 and NPN transistors or logic-level MOSFETs (e.g. 8 x MOSFETs for LED channels)
- resistors for LEDs

### Build materials

- 3D-printed train body and mechanical parts from the linked model/profile
- Wires, headers, small perfboard or PCB, connectors, and mounting hardware

## Which parts are optional

You can build a simpler version if you do not need every feature.

- Without the `VL53L0X`, the train still works, but there is no obstacle-aware auto driving.
- Without the `TCS34725`, the train still works, but there are no color-marker actions.
- Without the `SW-520D`, the train still works, but there is no tilt stop.
- Without the `MCP23008`, you need a different LED-driving approach.

## High-level wiring overview

- The motor is powered from the 2S battery pack through the `DRV8833`.
- The Arduino Nano is the main controller.
- The IR receiver handles the remote control.
- The buzzer provides horn, siren, and melodies.
- The LEDs provide headlights and status colors.
- The `VL53L0X`, `TCS34725`, and `MCP23008` share the I2C bus.

## SAFETY WARNING: Before you start

- Make sure you are comfortable working with Li-ion cells and a DIY battery pack. This is a potentially dangerous part of the project. The power source of the train uses two 18650 Li-ion cells in series (up to 8.4V), BMS, USB charger, and buck converter that must be properly wired together. Wrong wiring can cause fire or explosion or damage Arduino and other electronics. Use DIY battery pack only if you are confident in your skills and have tested the pack for safety. If you are not sure, get help from someone experienced or do not attempt it.

- The train contains small parts and electronics. Such toys usually aren't recommended for kids under 3 years. But check your local safety regulations and age recommendations and use your own parent's mind when deciding if it is appropriate for your child.

- Author just provides the information for educational purposes for adults willing to learn Arduino and electronics. Author doesn't take responsibility for any damage or injury caused to you or your kids even if you strictly followed the instructions.

## Project status

Current hardware and software support the features listed above.

Planned / future improvement:

- Motor overcurrent protection

## How to explain sleep, warning, and shutdown

This section explains the train's battery-related behavior in plain language.

- **Sleep** means the train is resting because it has been inactive for a while.
- **Warning** means the battery is getting low, so the train starts saving energy.
- **Shutdown** means the battery is too low, so the train stops until it is charged again.

### Sleep

If the train sits still for a few minutes with no remote use, it goes to sleep to save the battery.

- The train turns its normal lights and extras off.
- The small rear red light gives a brief blink from time to time to show that the train is sleeping, not switched off completely.
- Pressing the remote wakes it up again.
- When it wakes, it plays a short ready melody.

### Warning

When the battery starts getting low, the train gives a warning before it fully stops.

- About once per minute, the rear red warning light comes on and the train makes a long warning beep for about 3 seconds.
- The train can still be driven with the basic manual controls.
- To save power, extra features are turned off until the battery is recharged.

### Shutdown

When the battery gets too low, the train protects itself and shuts down.

- The rear red warning light comes on and the train makes a long beep for about 10 seconds.
- After that, it stays off and does not respond to the remote.
- This is normal behavior: it is protecting the battery from being drained too far.
- The fix is simple: recharge the battery pack.

## Color marker calibration

The train's generated color-marker block comes from the shared sensor capture
data. After deliberately updating those captures, regenerate the block with:

```powershell
python rgb-color-sensor\calibration_tool.py analyze-clusters --input rgb-color-sensor\data --target train
```

Replace only the generated-data region in
`arduino-train-v2\arduino-train-v2.ino`. The export maps the supported marker
labels to the train's action enums. It fails for a new unsupported label, so
add its enum/action mapping intentionally before inserting its calibration
data.

## More details

- Public project note: [doc/README.md](doc/README.md)
- Internal notes: `docs-internal/`
