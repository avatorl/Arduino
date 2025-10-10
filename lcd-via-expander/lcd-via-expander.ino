#include <LiquidCrystal_PCF8574.h>

// LCD connected via PCF8574 expander

// Expander PCF8574N pins ================================================
// Left Side:
// A0    GND
// A1    GND
// A2    GND
// P0    LCD RS
// P1    LCD E
// P2    nothing
// P3    nothing
// VSS   GND

// Right Side:
// VDD   5V
// SDA   Arduino SDA
// SCL   Arduino SCL
// INT   nothing
// P7    LCD D7
// P6    LCD D6
// P5    LCD D5
// P4    LCD D4

// LCD pins ==============================================================
// VSS   GND
// VDD   5V
// V0    10Kohm potentiometr (middle pin)
// RS    Expander 0
// RW    GND
// E     Expander 1
// D0    nothing
// D1    nothing
// D2    nothing
// D3    nothing
// D4    Expander 4
// D5    Expander 5
// D6    Expander 6
// D7    Expander 7

// Configure LCD connection
int rs = 0, e = 1, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal_PCF8574
    lcd(0x20, rs, e, d4, d5, d6,
        d7); // set the LCD address to 0x20 for a 16 chars and 2 line display

int show = 0;

// Custom character
byte customChar[8] = {0b00000, 0b01010, 0b11111, 0b11111,
                      0b01110, 0b00100, 0b00000, 0b00000};

// see https://arduinogetstarted.com/tutorials/arduino-lcd-i2c

void setup() {

  Serial.begin(9600);

  lcd.begin(16, 2); // initialize the lcd

} // setup()

void loop() {
  switch (show) {
  case 0:
    // lcd.setBacklight(255); // Doens't work
    lcd.home();
    lcd.clear();
    lcd.print("Hello LCD");
    delay(1000);
    // lcd.setBacklight(0); // Doesn't work
    delay(4000);
    // lcd.setBacklight(255);
    break;

  case 1:
    lcd.clear();
    lcd.print("Cursor On");
    lcd.cursor();
    break;

  case 2:
    lcd.clear();
    lcd.print("Cursor Blink");
    lcd.blink();
    break;

  case 3:
    lcd.clear();
    lcd.print("Cursor OFF");
    lcd.noBlink();
    lcd.noCursor();
    break;

  case 4:
    lcd.clear();
    lcd.print("Display Off");
    lcd.noDisplay();
    break;

  case 5:
    lcd.clear();
    lcd.print("Display On");
    lcd.display();
    break;

  case 7:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("*** first line.");
    lcd.setCursor(0, 1);
    lcd.print("*** second line.");
    break;

  case 8:
    lcd.scrollDisplayLeft();
    break;

  case 9:
    lcd.scrollDisplayLeft();
    break;

  case 10:
    lcd.scrollDisplayLeft();
    break;

  case 11:
    lcd.scrollDisplayRight();
    break;

  case 12:
    lcd.clear();
    lcd.print("write-");
    break;

  case 13:
    lcd.clear();
    lcd.print("custom 1:<\01>");
    lcd.setCursor(0, 1);
    lcd.print("custom 2:<\02>");
    break;

  case 14:
    lcd.clear();
    lcd.createChar(0,
                   customChar); // create a new custom character (),
                                // lcd.createChar(index, customChar), where
                                // index = 0 to 7 for up to 8 custom characters
    lcd.setCursor(2, 0);        // move cursor to (2, 0)
    lcd.write((byte)0);         // print the custom char at (2, 0)
    break;
  }

  delay(1000);
  show = (show + 1) % 16;
} // loop()
