// To use with Arduino Uno as a MIDI device (won't be dected as MIDI device by Windows) aditional steps are required:
// install https://www.tobias-erichsen.de/software/loopmidi.html, create 2 virtual ports
// use https://github.com/raspy135/serialmidi/
// in Windows command prompt:
// python.exe .\serialmidi.py --serial_name=COM6 to get Midi port names then run
// python.exe .\serialmidi.py --serial_name=COM6 --baud=38400 --midi_in_name="loopMIDI0 0" --midi_out_name="loopMIDI1 2"
// python.exe .\serialmidi.py --serial_name=COM6 --baud=38400 --midi_in_name="loopMIDI0 0" --midi_out_name="loopMIDI1 2" --debug (version with debug output enabled)

// https://www.instructables.com/Virtual-Drum-Kit-Using-Arduino-Uno/

// Definitions for Code Alpha

#include <LiquidCrystal_PCF8574.h>
#include <Keypad.h>

// https://lastminuteengineers.com/arduino-keypad-tutorial/

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
      d7);  // set the LCD address to 0x20 for a 16 chars and 2 line display

int button = 0;

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

byte rowPins[ROWS] = { 9, 8, 7, 6 };  //row pins
byte colPins[COLS] = { 5, 4, 3, 2 };  //column pins

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

const int piezoPinsAlpha[] = { A0, A0, A0, A0 };  // Piezo sensor pins for Code Alpha

const int midiNotesAlpha[] = { 36, 42, 53, 57 };  // MIDI note numbers for each sensor (see Cakewalk for available =)

unsigned long lastHitTimesAlpha[] = { 0, 0, 0, 0 };  // Variables to store the time of the last hit for each sensor

#define HiHat_closed 69

#define HiHat_open 70

#define MIDI_COMMAND_NOTE_ON 0x90  // 144

#define MIDI_COMMAND_NOTE_OFF 0x80  // 128

const int thresholdHiHat = 10;  // Increased threshold for less sensitivity HIGH HAT

const unsigned long triggerDelay = 65;  // Delay to prevent double triggering (milliseconds)

unsigned char vel = 127;  // Default velocity

unsigned long lastTriggerTime = 0;  // Variable to store the time of the last trigger



// Definitions for Code Beta SNARE

const int piezoPinBeta = A0;  // Piezo sensor connected to analog pin A0

const int thresholdBeta = 280;  // Threshold for hit detection

const int sensitivity = 500;  // Sensitivity for velocity calculation

const int scanTime = 5;  // Scan time for peak detection (in milliseconds)

const int maskTime = 20;  // Mask time to prevent retrigger (in milliseconds)

const int midiNoteBeta = 60;  // MIDI note for the drum hit

unsigned long lastHitTimeBeta = 0;  // Time of the last hit

int peakValue = 0;  // Peak value during scan time


void setup() {

  Serial.begin(38400);  // MIDI baud rate

  pinMode(2, INPUT);  // HiHat pedal

  lcd.begin(16, 2);

  // Print a message to the LCD.

  lcd.print("hello, world!");
}

void sendMIDI(byte note, byte velocity) {

  // MIDI note-on message
  Serial.write(MIDI_COMMAND_NOTE_ON);  // Note on command channel 1
  Serial.write(note);                  // Note number
  Serial.write(velocity);              // Velocity

  // MIDI note-off message
  Serial.write(MIDI_COMMAND_NOTE_OFF);  // Note off command channel 1
  Serial.write(note);                   // Note number
  Serial.write(0x00);                   // Velocity
}

void loop() {

  int i = 0;
  char key = keypad.getKey();

  if (key) {

    lcd.clear();
    lcd.print(key);

    switch (key) {
      case "1":
        i = 1;
        break;
      case "2":
        i = 2;
        break;
      case "3":
        i = 3;
        break;
      case "A":
        i = 0;
        break;
    }
  }

  int sensorValueAlpha = map ( analogRead(A5), 0, 1023, 0, 127);

  sendMIDI(midiNotesAlpha[i], sensorValueAlpha);  // Send MIDI note on

  delay(200);

}
