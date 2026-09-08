#include <LiquidCrystal.h>

// Uses ultrasonic sensor to measure distance (cm) and displays on LCD screen
// https://arduinogetstarted.com/tutorials/arduino-ultrasonic-sensor-lcd

// LCD ============================================================================================

// VSS   GND
// VDD   +5V
// V0    middle pin of a 10Kom potentiometer
// RS    2
// RW    GND
// E     3
// D0    none
// D1    none
// D2    none
// D3    none
// D4    4
// D5    5
// D6    6
// D7    7

const int rs = 2, en = 3, d4 = 4, d5 = 5, d6 = 6, d7 = 7;  // LCD pins

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// ultrasonic sensor ==============================================================================

// GND     GND
// ECHO    19
// TRIG    18
// VCC     5V

int trigPin = 19, echoPin = 18;  // ultrasonic sensor pins

float duration_us;
int distance_cm;

// ================================================================================================

void setup() {
  lcd.begin(16, 2);  // initialize the lcd

  pinMode(trigPin, OUTPUT);  // config trigger pin to output mode
  pinMode(echoPin, INPUT);   // config echo pin to input mode
}

void loop() {

  // generate 10-microsecond pulse to TRIG pin
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // measure duration of pulse from ECHO pin
  duration_us = pulseIn(echoPin, HIGH);

  // calculate the distance
  distance_cm = round(0.017 * duration_us);

  lcd.clear();
  lcd.setCursor(0, 0);  // start to print at the first row
  lcd.print("Distance: ");
  lcd.print(distance_cm);
  lcd.print(" cm");

  delay(500);
}
