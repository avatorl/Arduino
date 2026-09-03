// https://arduinogetstarted.com/tutorials/arduino-rgb-led

#include <LiquidCrystal_PCF8574.h>

// Configure LCD connection
int rs = 0, e = 1, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal_PCF8574
  lcd(0x20, rs, e, d4, d5, d6,
      d7);  // set the LCD address to 0x20 for a 16 chars and 2 line display


const int PIN_RED = 3;
const int PIN_GREEN = 5;
const int PIN_BLUE = 6;

//LED with common anode: 255 - value
void setColor(int R, int G, int B, int t = 1000) {

  lcd.clear();
  lcd.home();
  lcd.print(R);
  lcd.print(", ");
  lcd.print(G);
  lcd.print(", ");
  lcd.print(B);

  analogWrite(PIN_RED, 255 - R);
  analogWrite(PIN_GREEN, 255 - G);
  analogWrite(PIN_BLUE, 255 - B);

  delay(t);
}

void setup() {
  pinMode(PIN_RED, OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_BLUE, OUTPUT);

  lcd.begin(16, 2);

  setColor(0, 0, 0);

  setColor(255, 255, 255);

  setColor(255, 0, 0);

  setColor(0, 255, 0);

  setColor(0, 0, 255);

  setColor(255, 0, 255);

  setColor(0, 255, 255);

  setColor(255, 255, 0);

  setColor(64, 0, 0);

  for (int i = 0; i < 255; i++) {
    setColor(i, 0, 0, 50);
  }
  for (int i = 0; i < 255; i++) {
    setColor(0, i, 0, 50);
  }
  for (int i = 0; i < 255; i++) {
    setColor(0, 0, i, 50);
  }
  for (int i = 0; i < 255; i++) {
    setColor(i, i, i, 50);
  }
}

void loop() {
}
