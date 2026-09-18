#include <ADCTouch.h>

// Connect a piece of wire to PIN_TOUCH for touch sensing.
// Touch the wire (works through wire insulation as well) to light up a LED on PIN_LED.
// A metal tray connected to PIN_TOUCH with a piece of wire detects a hand from
// around 10 cm away without touching it.

// Capacitive Touch Sensing with AVR and a Single ADC Pin
// https://web.archive.org/web/20170912232909/http://tuomasnylund.fi/drupal6/content/capacitive-touch-sensing-avr-and-single-adc-pin

int baseline;

const int PIN_LED = 12; // LED connected to this pin
const int PIN_TOUCH =
    A0; // Touch sensor (e.g a piece of wire) connected to this pin

const int touchValueThreshold =
    10; // change this threshold value as needed; decrease to
        // increase sensitivity or increase to increase
        // reliability in noisy environment

void setup() {

  Serial.begin(115200);

  pinMode(PIN_LED, OUTPUT);

  // Measures the baseline capacitance
  baseline = ADCTouch.read(PIN_TOUCH, 100);
  Serial.print("Baseline: ");
  Serial.println(baseline);
}

void loop() {

  // Read current value and subtract the baseline
  touchValue = ADCTouch.read(PIN_TOUCH) - baseline;

  Serial.print("Value: ");
  Serial.println(touchValue);

  if (touchValue > touchValueThreshold) {
    Serial.println("\e[0;32m--> TOUCHED THE WIRE!\e[0m");
    digitalWrite(PIN_LED, HIGH);
  } else {
    digitalWrite(PIN_LED, LOW);
  }
  delay(100);
}
