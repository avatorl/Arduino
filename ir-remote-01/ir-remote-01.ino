// === IR Remote ===================================================================================
#define DECODE_NEC       // DECODE_NEC
#include <IRremote.hpp>  // Do not change header order.

// Works with Odbiornik podczerwieni TSOP4838 - 38 kHz
// Odbiornik podczerwieni TSOP4838. Działający na częstotliwości 38 kHz. Działa z napięciem od 4,5 V do 6 V. Średni pobór prądu wynosi 5 mA.
// https://botland.com.pl/odbiorniki-podczerwieni/1047-odbiornik-podczerwieni-tsop4838-38khz-5904422355913.html

// https://github.com/Arduino-IRremote/Arduino-IRremote?tab=readme-ov-file#converting-your-2x-program-to-the-4x-version
// https://wokwi.com/projects/338611596994544210

// IR Remote Button Codes ("Car MP3" Remote)

// Format: Code | Label

// Top Rows
//  69 | CH-       70 | CH       71 | CH+
//  68 | <<        64 | >>        67 | Play/Pause
//   7 | -         21 | +          9 | EQ
//  22 | 0         25 | 100+      13 | 200+

// Number Pad
//  12 | 1         24 | 2         94 | 3
//   8 | 4         28 | 5         90 | 6
//  66 | 7         82 | 8         74 | 9

// Przypisania przycisków IR (z pilotem typu „Car MP3”)
const int buttonCHminus = 69;    // CH- : (wolne)
const int buttonCH = 70;         // CH  : (wolne)
const int buttonCHplus = 71;     // CH+ : (wolne)
const int buttonBackward = 68;   // <<  : (wolne)
const int buttonForward = 64;    // >>  : (wolne)
const int buttonPlayPause = 67;  // Play/Pause : (wolne)
const int buttonMinus = 7;       // - : (wolne)
const int buttonPlus = 21;       // + : (wolne)
const int buttonEQ = 9;          // EQ : (wolne)
const int button0 = 22;          // 0 : (wolne)
const int button100plus = 25;    // 100+ : (wolne)
const int button200plus = 13;    // 200+ : (wolne)
const int button1 = 12;          // 1 : (wolne)
const int button2 = 24;          // 2 : (wolne)
const int button3 = 94;          // 3 : (wolne)
const int button4 = 8;           // 4 : (wolne)
const int button5 = 28;          // 5 : (wolne)
const int button6 = 90;          // 6 : (wolne)
const int button7 = 66;          // 7 : (wolne)
const int button8 = 82;          // 8 : (wolne)
const int button9 = 74;          // 9 : (wolne)

// IR sensor pin
const byte pinIRReceiver = 0;  // (Pin "D0" używany do odbiornika podczerwieni)

// get button code from IR Receiver
uint16_t irReceive() {
  uint16_t received{ 0 };

  if (IrReceiver.decode()) {
    if (IrReceiver.decodedIRData.protocol == NEC) {
      received = IrReceiver.decodedIRData.command;
      Serial.println(received, DEC);
    }
    IrReceiver.resume();
  }
  return received;
}

// Usage

// setup

// IrReceiver.begin(pinIRReceiver);  // Inicjalizuje odbiornik IR

// loop

// switch (irReceive()) { ...

// === end of IR Remote

// LED pins
const byte red = 18;
const byte green = 19;
const byte blue = 1;

void setup() {
  Serial.begin(9600);

  // initialize LED pins
  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(blue, OUTPUT);

  // turn off all leds
  digitalWrite(red, LOW);
  digitalWrite(green, LOW);
  digitalWrite(blue, LOW);

  // initialize IR Receiver
  IrReceiver.begin(pinIRReceiver);
}

void loop() {

  switch (irReceive()) {

    case buttonCHminus:
      digitalWrite(red, HIGH);
      digitalWrite(green, LOW);
      digitalWrite(blue, LOW);
      break;

    case buttonCH:
      digitalWrite(red, LOW);
      digitalWrite(green, HIGH);
      digitalWrite(blue, LOW);
      break;

    case buttonCHplus:
      digitalWrite(red, LOW);
      digitalWrite(green, LOW);
      digitalWrite(blue, HIGH);
      break;
      
  }
}