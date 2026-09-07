# Battery Voltage Display Design

## Goal

Correct diagnostic battery-voltage text so a millivolt value such as 8064 is
shown as `8.06 V`, rather than the ambiguous and incorrect-looking `8.6 V`.

## Scope

Add one shared debug-output helper that accepts a `uint16_t` millivolt value
and emits whole volts plus a zero-padded two-digit fractional component.
Replace the duplicated voltage formatting in:

- the battery ADC diagnostic trace;
- the boot-time battery message;
- the manual battery-status message.

## Behavior

The helper preserves the existing measurement value and prints:

```text
8064 mV -> 8.06
8052 mV -> 8.05
8000 mV -> 8.00
```

It does not round, recalibrate, sample the ADC, alter battery thresholds, or
change PWM and motor-control behavior.

## Validation

Compile the selected Arduino sketch with its documented Nano target and local
libraries. Verify the helper's integer decomposition produces padded
hundredths for representative millivolt values.
