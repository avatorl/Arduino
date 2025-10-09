# Lego Duplo compatible 3D-printed train with Arduino

## Features
- 3 speed levels: effective voltage on the motor: 3.5V, 4.5V, 6.0V; measured voltage (unloaded train) - around 4.4V, 5.0V, 6.0V volts
- Stop button: stops, red lights
- Speed up button: speed level +1, moves forward; white lights
- Speed down button: speed level -1, moves forward until speed level = 0; white lights
- Move backward button: moves backward at speed level 1 while the button is pressed; blue lights
- Move forward button: moves forward at speed level 1 while the button is pressed; white lights
- Auto button: moves forward with obstacle detection enabled. Stops if there is an obstacle.
-    Moves forward if the obstacle is removed. Speed depends on the distance to the nearest obstacle. White lights and green light.
- Horn button: horn sound effect
- Siren button: red and blue lights, siren sounds
- Mute button: sound off/on
- Battery status button: indicates battery level by sound beeps, e.g. 7 long beeps and 3 short beeps = 7.3V
- Battery status detection: warning level with red lights and sound; shutdown level
- Sleep mode: powers down automatically after 5 minutes without IR remote input (can be woken up again with the remote)
- Tilt sensor: stop when the train is on its side
- TBD: motor overcurrent protection (shunt resistor)
