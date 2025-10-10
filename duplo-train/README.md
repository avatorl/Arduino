// ================================================================================================
// Train compatible with Lego DUPLO (3D-printed parts + Arduino)
// Based on: https://cults3d.com/en/3d-model/game/locomotora-sofia-controlada-por-infrarojos-con-control-de-velocidad-por-ultrasonidos-y-motor-superior-multiusos-lego-duplo
// Arduino code fully rewritten & extended by Andrzej Leszkiewicz
// Get the most recent version of the code at https://github.com/avatorl/Arduino/blob/main/duplo-train/duplo-train.ino
// 3D-printing profile: https://makerworld.com/en/models/1854728-arduino-train-locomotive-remote-controlled#profileId-1983131
// ================================================================================================

// 💡 Features ====================================================================================
// 3 speed levels: effective voltage on the motor: 3.5V, 4.5V, 6.0V; measured voltage (unloaded train) - around 4.4V, 5.0V, 6.0V
// Stop button: stops; red lights
// Speed up button: speed level +1, moves forward; white lights
// Speed down button: speed level -1, moves forward until speed level = 0; white lights while moving, red lights is stopped
// Move backward button: moves backward at speed level 1 while the button is pressed; blue lights
// Move forward button: moves forward at speed level 1 while the button is pressed; white lights
// Auto button: moves forward with obstacle detection enabled. Stops if there is an obstacle.
//    Moves forward if the obstacle is removed. Speed depends on the distance to the nearest obstacle. White lights and green light
// Horn button: horn sound effect
// Siren button: red and blue lights; siren sounds
// Music buttons: 8 different melodies
// Mute button: sound on/off
// Battery status button: indicates battery level by sound beeps, e.g., 7 long beeps and 3 short beeps = 7.3V
// Battery status detection: warning level with red lights and sound; shutdown level
// Sleep mode: powers down automatically after 5 minutes without IR remote input (can be woken up again with the remote)
// Tilt sensor: stops when the train is on its side; red lights; if the train is back in a vertical position - yellow light
// TBD: motor overcurrent protection (shunt resistor)

// ================================================================================================
// Electronic components
//   Arduino Nano 3.0 ATMEGA328 CH340
//   IR sensor HX1838 with wiring adapter + IR remote controller: "car MP3 remote controller" (21 buttons)
//   Buzzer
//   2 x RGB LEDs, 1 x green LED
//   DC motor with 1:48 gear, motor driver HG7881 (L9110S)
//   Ultrasonic distance sensor HC-SR04
//   Tilt sensor SW-520D
//
// Resistors:
//   LED RGB: R - 100 Ω + 47 Ω, G - 100 Ω, B - 100 Ω
//   LED Green: 100 Ω + 47 Ω
//   Voltage divider: 10K Ω and 4.7K Ω
// ===============================================================================================