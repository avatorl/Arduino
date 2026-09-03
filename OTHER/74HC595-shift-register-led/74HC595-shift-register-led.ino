#include <LiquidCrystal_PCF8574.h> // LCD connected via PCF8574

// Works with: Rejestr przesuwający 74HC595
// Elementy korzystają z 8-bitowego sygnału i współpracują z napięciem od 2 V do
// 6 V. Pobór prądu pojedynczego komponentu wynosi do 1 uF.
// https://botland.com.pl/uklady-logiczne/1660-rejestr-przesuwajacy-74hc595-8-bitowy-5szt-5904422359003.html

// LEDs connected to a shift register
// https://electropeak.com/learn/74hc595-with-arduino-how-it-works-how-to-use-full-guide/
// https://www.arduino.cc/reference/tr/language/functions/advanced-io/shiftout/
// https://lastminuteengineers.com/74hc595-shift-register-arduino-tutorial/

int rs = 5, e = 4, d4 = 0, d5 = 1, d6 = 2, d7 = 3;

LiquidCrystal_PCF8574
    lcd(0x20, rs, e, d4, d5, d6,
        d7); // set the LCD address to 0x20 for a 16 chars and 2 line display

int dataPin = 1;
int latchPin = 2;
int clockPin = 3;

// list of commans to enable/disable pins (8 pins)
byte controlPins[] = {

    0b00000000, 0b01110000, 0b01000000, 0b00100000,

    0b00010000, 0b01010000, 0b01000000, 0b00100000, 0b00010000, 0b01110000};

void setup() {
  ShiftRegisterInit(); // initialize the shift register
  lcd.begin(16, 2);    // initialize the lcd
  // lcd.setBacklight(255); // Doesn't work
}

void loop() {
  int items = sizeof controlPins;
  for (int i = 0; i < items; i++) {
    lcd.clear();
    lcd.home();
    lcd.print("i=" + String(i));
    lcd.setCursor(0, 1);
    lcd.print(binaryToString(controlPins[i]));
    ShiftRegisterWrite(controlPins[i]); // Write data to shift register;
  }
}

void ShiftRegisterInit() {
  pinMode(dataPin, OUTPUT);  // Output Pin Mode
  pinMode(latchPin, OUTPUT); // Output Pin Mode
  pinMode(clockPin, OUTPUT); // Output Pin Mode
}

// Convert binary data (byte) to printable string (with leading zeroes)
String binaryToString(byte data) {
  String dataString;
  dataString = "100000000" + String(data, BIN);
  dataString = dataString.substring(dataString.length() - 8);
  return dataString;
}

void ShiftRegisterWrite(byte data) {
  digitalWrite(latchPin, LOW);
  // shiftOut(dataPin, clockPin, MSBFIRST, data);
  shiftOut(dataPin, clockPin, LSBFIRST, data);
  digitalWrite(latchPin, HIGH);
  delay(3000);
}
