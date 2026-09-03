# Arduino Train v2 VCC Rail Shutdown Design

## Goal

Protect the Arduino Nano and its 5 V-powered peripherals from an unstable
supply rail without prematurely discarding usable 2S battery capacity. Keep
the existing battery-divider measurement for charge reporting and low-battery
warning, add an AVCC rail monitor for stability shutdown, and retain a 6.00 V
VIN emergency cutoff.

## Existing Wiring and Measurement Contract

The existing battery divider remains on `pinBatterySense`, currently configured
as A0 in `arduino-train-v2/config.h`; this work does not alter board wiring or
the pin assignment. Battery voltage is measured through that divider with the
AVR internal 1.1 V ADC reference and is represented in millivolts.

The 5 V rail is measured in software by using AVCC as the ADC reference and
converting an ADC measurement of the ATmega328P's internal 1.1 V bandgap
channel. The conversion's nominal calibration factor must be a named
configuration constant so it can be adjusted to match a multimeter measurement
for the specific Nano.

## Voltage Policy

The policies are independent:

| Measurement | Purpose | Threshold and behavior |
| --- | --- | --- |
| VIN divider | Battery status | Preserve the 7.25 V warning threshold and 7.35 V recovery threshold. |
| VIN divider | Emergency battery protection | Permanently shut down at or below 6.00 V. |
| AVCC / 5 V rail | MCU and peripheral stability | Permanently shut down after three consecutive samples below 4.85 V. |

VIN warning is not itself a shutdown trigger. Either the 6.00 V VIN cutoff or
the confirmed 4.85 V AVCC cutoff invokes the existing permanent shutdown
sequence. The threshold is deliberately above the 16 MHz ATmega328P's 4.5 V
minimum operating voltage to provide transient and measurement margin.

## Sampling and ADC Coordination

The AVCC monitor samples once every 100 ms during normal operation, including
while the motor is running. A sample at or above 4.85 V resets the consecutive
low-rail counter. Three low samples therefore confirm the condition over about
300 ms before shutdown.

The VCC helper temporarily selects AVCC plus the internal bandgap ADC input,
performs a settled conversion, and computes millivolts. It must then restore
the `INTERNAL` ADC reference used by the battery divider and discard or settle
the next battery-divider conversion before treating it as valid. This prevents
the temporary reference/channel selection from corrupting subsequent VIN
measurements.

During setup, the sketch initializes the battery ADC as it does today and
checks AVCC before normal operation begins. If AVCC is below the rail threshold,
the same three-sample confirmation rule applies before permanent shutdown.

## State, Diagnostics, and Failure Handling

Add configuration constants for the AVCC threshold, sample interval,
confirmation count, and calibrated bandgap conversion factor. Maintain a small
counter for consecutive low AVCC samples. The counter resets after a valid
sample and after a permanent-shutdown transition.

Add a shutdown-cause indicator so debug messages distinguish:

- an AVCC rail-stability shutdown,
- the 6.00 V VIN emergency shutdown, and
- existing critical overvoltage/meter-fault shutdown.

The normal battery state machine remains responsible for warning signals,
warning recovery, and the existing final-shutdown outputs. No new wake or
recovery behavior is introduced: a confirmed permanent shutdown still requires
a physical power cycle.

## Validation

Compile for the Arduino Nano ATmega328P target. Review the resulting behavior
to confirm that:

1. A VIN value below 7.25 V can warn without shutting down unless VIN reaches
   6.00 V or AVCC meets the low-rail confirmation rule.
2. A rail dip below 4.85 V shuts down only after three samples, while a
   recovered sample resets the confirmation counter.
3. VCC sampling during driving does not change the 1.1 V reference used by the
   divider measurement.
4. Startup, periodic checks, logging, and permanent-shutdown outputs stay
   consistent with the existing battery state machine.
