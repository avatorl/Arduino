# I2C Bus Diagnostic Design

## Goal

Provide an independent Arduino Nano sketch that verifies the electrical
reliability of the train's MCP23008, VL53L0X, and TCS34725 I2C bus at 100 kHz
and 400 kHz without testing sensor measurement quality or changing actuator
outputs.

## Wiring Contract

The test uses Nano A4/SDA and A5/SCL, MCP23008 at `0x20`, TCS34725 at `0x29`,
and VL53L0X XSHUT on A3. The accepted schematic wiring is authoritative,
including the VL53L0X connection. All stale MCP23008 `0x24` references in the
main sketch must be corrected to `0x20`.

## Test Flow

The test starts serial output, performs a standard nine-clock I2C bus recovery
if SDA is low, then verifies SDA and SCL idle high. For each test speed (100
kHz and 400 kHz), it:

1. Holds VL53L0X XSHUT low and repeatedly reads MCP23008 and TCS34725
   identification/status registers at their normal addresses.
2. Releases XSHUT, waits for boot, write-reassigns VL53L0X from `0x29` to
   `0x2A`, and verifies its model ID there.
3. Repeats non-destructive register reads for each device 1,000 times,
   recording transaction failures, short reads, and Wire timeout flags.
4. Verifies bus idle levels after each run and prints per-device and aggregate
   pass/fail results.

The sketch does not write MCP23008 output registers and does not evaluate
distance, color, or accelerometer data.

## Limits and Failure Handling

The sketch reports stuck-low lines, missing devices, invalid identities,
transaction errors, and Wire timeouts explicitly. It attempts bus recovery
before starting each speed run and continues to report all failures where
possible.

Software can establish reliable I2C communication symptoms but cannot prove
pull-up resistor values, signal voltage margins, rise times, ringing, or
electrical noise. Those require a multimeter and, for waveform quality, an
oscilloscope or logic analyzer.
