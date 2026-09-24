/* Capacitance Meter Example

 *  Demonstrates use of RC time constants to measure the value of a capacitor

 * Theory   A capacitor will charge, through a resistor, in one time constant,
 defined as T seconds where

 *    TC = R * C
 *    TC = time constant period in seconds
 *    R = resistance in ohms
 *    C = capacitance in farads (1 microfarad (ufd) = .000001 farad = 10^-6
 farads )

 *    The capacitor's voltage at one time constant is defined as 63.2% of the
 charging voltage.

 *  Hardware setup:

 *  Test Capacitor between common point and ground (positive side of an
 electrolytic capacitor  to common)

 *  Test Resistor between chargePin and common point (10k ohm for small
 capacitors, 1k ohm for large capacitors)
 *  220 ohm resistor between dischargePin and common point (limits discharge
 current)
 *  Wire between common point and analogPin (A/D input)

 */

// https://docs.arduino.cc/tutorials/generic/capacitance-meter/
// https://www.instructables.com/Measure-Capacitance-with-Arduino/

// The following code has been updated for Dual-Resistor Auto-Ranging (10k ohm
// and 1k ohm)

#define analogPin 0     // Analog pin for measuring capacitor voltage
#define chargePin10k 13 // Pin for small capacitors (10k resistor)
#define chargePin1k 8   // NEW: Pin for large capacitors (1k resistor)
#define dischargePin 11 // Pin to discharge the capacitor

unsigned long startTime;
unsigned long elapsedTime;
float microFarads;
float nanoFarads;
float activeResistorValue; // Dynamic tracker for which resistor is active

void setup() {
  pinMode(chargePin10k, OUTPUT);
  digitalWrite(chargePin10k, LOW);

  pinMode(chargePin1k, OUTPUT); // Initialize the new 1k charging pin
  digitalWrite(chargePin1k, LOW);

  Serial.begin(115200);
}

void loop() {
  // --- STEP 1: Try charging with the 10k resistor first (High Precision Range)
  // ---
  activeResistorValue = 10000.0F;
  digitalWrite(chargePin10k, HIGH);
  startTime = micros();

  bool timeout = false;
  while (analogRead(analogPin) < 648) {
    // If it takes more than 100,000 uS (100 ms), the capacitor is too big for
    // the 10k resistor
    if ((micros() - startTime) > 100000) {
      timeout = true;
      break;
    }
  }

  // --- STEP 2: If it timed out, switch to the 1k resistor (High Capacity
  // Range) ---
  if (timeout) {
    // Abort the 10k test and discharge briefly to reset baseline
    digitalWrite(chargePin10k, LOW);
    pinMode(dischargePin, OUTPUT);
    digitalWrite(dischargePin, LOW);
    while (analogRead(analogPin) > 0)
      ;
    pinMode(dischargePin, INPUT); // stop discharging

    // Switch range to 1k resistor
    activeResistorValue = 1000.0F;
    digitalWrite(chargePin1k, HIGH);
    startTime = micros();

    while (analogRead(analogPin) < 648) {
      // Loop until charged (no timeout needed here as 1k handles up to several
      // thousand uF easily)
    }
  }

  elapsedTime = micros() - startTime; // Save final timing string

  // --- STEP 3: Math and Output ---
  microFarads = ((float)elapsedTime / activeResistorValue);

  Serial.print("Resistor: ");
  Serial.print((int)activeResistorValue);
  Serial.print(" ohms | ");
  Serial.print(elapsedTime);
  Serial.print(" uS -> ");

  if (microFarads > 1.0) {
    Serial.print(microFarads, 2);
    Serial.println(" microFarads");
  } else {
    nanoFarads = microFarads * 1000.0;
    Serial.print(nanoFarads, 1);
    Serial.println(" nanoFarads");
  }

  // --- STEP 4: Thorough Discharge Routine ---
  digitalWrite(chargePin10k, LOW);
  digitalWrite(chargePin1k, LOW);
  pinMode(dischargePin, OUTPUT);
  digitalWrite(dischargePin, LOW);

  while (analogRead(analogPin) > 0) {
    // Wait until completely empty
  }

  pinMode(dischargePin, INPUT);

  delay(1000); // 1-second pause to easily track range shifts in the Serial
               // Monitor
}
