#include <LiquidCrystal.h>
#include <Keypad.h>

// https://lastminuteengineers.com/arduino-keypad-tutorial/

// LCD
const int rs = 12, e = 13, d4 = 2, d5 = 3, d6 = 4, d7 = 5;

int button = 0;

LiquidCrystal lcd(rs, e, d4, d5, d6, d7);

const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};

// black pins = rows
// white pins = columns

byte rowPins[ROWS] = { 11, 10, 9, 8 };    //row pins
byte colPins[COLS] = { A3, A2, A1, A0 };  //column pins

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

void setup() {

  // set up the LCD's number of columns and rows:

  lcd.begin(16, 2);

  // Print a message to the LCD.

  lcd.print("hello, world!");
  //delay(1000);

}

void loop() {

  char key = keypad.getKey();
  if (key) {
    lcd.clear();
    lcd.print(key);
  }
}