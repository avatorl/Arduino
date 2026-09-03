#include <Wire.h> // Wire is Arduino’s I²C/TWI driver. It handles the 2-wire bus (SDA/SCL)
#include <LiquidCrystal_PCF8574.h>

// LCD connected via PCF8574 expander

// Expander PCF8574N pins ================================================
// Left Side:
// A0    GND (encodes address)
// A1    GND (encodes address)
// A2    GND (encodes address)
// P0    LCD RS
// P1    LCD E
// P2    nothing
// P3    BC547 base via 4.7K (for controlable on/off backlight) or nothing (if backlight on/off not required); PWN pin on Arduino for backlight brightness control
// VSS   GND

// Right Side:
// VDD   5V
// SDA   Arduino SDA; 5V via 4.7K pull-up
// SCL   Arduino SCL; 5V via 4.7K pull-up
// INT   nothing
// P7    LCD D7
// P6    LCD D6
// P5    LCD D5
// P4    LCD D4

// Wiring for backlight
// Transistor BC547 pinout: with flat side facing you, left to right: collector, base, emitter
//  collector - K (on LCD)
//  base - 4.7k - P3 on PCF8574; base - 100k - GND
//  emitter - GND

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
// A     5V (for backlight)
// K     BC547 collector (for controlable backlight) or GND (for backlight always on)

// ───────────────────────────────────────────────────────────────────────
const uint8_t LCD_ADDR = 0x20;
const uint8_t LCD_COLS = 16;
const uint8_t LCD_ROWS = 2;

//const uint8_t BL_PIN = 5;  // PWM-capable pin (for backlight brightness control)

const int rs = 0, e = 1, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
const uint8_t backlight_pin = 3; // P3 on PCF8574, use instead of PWM-capable pin if backlight dimming is not required: only lcd.setBacklight(255) and lcd.setBacklight(0)
LiquidCrystal_PCF8574 lcd(LCD_ADDR, (uint8_t)rs, (uint8_t)255, (uint8_t)e,
                          (uint8_t)d4, (uint8_t)d5, (uint8_t)d6, (uint8_t)d7,
                          backlight_pin);

uint8_t show = 0;

byte customChar[8] = {
  0b00000, 0b01010, 0b11111, 0b11111,
  0b01110, 0b00100, 0b00000, 0b00000
};

void printCentered(uint8_t row, const char* msg) {
  int len = 0; while (msg[len] != '\0') len++;
  int col = (len >= LCD_COLS) ? 0 : (LCD_COLS - len) / 2;
  lcd.setCursor(col, row);
  lcd.print(msg);
}

void clearAndHome() {
  lcd.clear();
  delay(20);
  lcd.home();
}

// ---- PCF8574 BL control on P3 (bit 3) -----------------
const uint8_t PCF_ADDR = 0x20;

// backlight brightness control
// void setBacklightPct(uint8_t pct) {           // 0..100 (%)
//   analogWrite(BL_PIN, map(pct, 0, 100, 0, 255));
// }

void setup() {

  // Serial.begin(9600);

  Wire.begin(); // initializes the hardware I²C controller and configures the SDA/SCL pins
  Wire.setClock(100000); // sets the bus speed to 100 kHz

  lcd.begin(LCD_COLS, LCD_ROWS);
  lcd.createChar(0, customChar);

  // pinMode(BL_PIN, OUTPUT);

  lcd.setBacklight(255);
  clearAndHome();
  printCentered(0, "LCD via PCF8574");
  printCentered(1, "Init complete");
  delay(2000);
  lcd.setBacklight(0);
  delay(2000);
  lcd.setBacklight(255);
  delay(2000);

}

void loop() {
  switch (show) {
    case 0: { clearAndHome(); lcd.print("Hello LCD"); break; }
    case 1: { clearAndHome(); lcd.print("Cursor On"); lcd.cursor(); break; }
    case 2: { clearAndHome(); lcd.print("Cursor Blink"); lcd.blink(); break; }
    case 3: { clearAndHome(); lcd.print("Cursor OFF"); lcd.noBlink(); lcd.noCursor(); break; }
    case 4: { clearAndHome(); lcd.setCursor(0,0); lcd.print("1) first line"); lcd.setCursor(0,1); lcd.print("2) second line"); break; }
    case 5: { clearAndHome(); lcd.print("custom char: "); lcd.write((byte)0); break; }
    default: break;
  }
  delay(2000);
  show = (show + 1) % 6;
}
