# I2C Bus Diagnostic Design

## Goal

Provide an independent Arduino Nano sketch that verifies the electrical
reliability of the train's MCP23008, VL53L0X, and TCS34725 I2C bus at 100 kHz
and 400 kHz without testing sensor measurement quality or changing actuator
outputs.

The artifact is `arduino-train-v2/test-i2c/test-i2c.ino`, compiled for
`arduino:avr:nano:cpu=atmega328old`.

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
   registers at their normal addresses.
2. Releases XSHUT, waits for boot, write-reassigns VL53L0X from `0x29` to
   `0x2A`, and verifies its model ID there.
3. Repeats non-destructive register reads for each device 1,000 times,
   recording transaction failures, short reads, and Wire timeout flags.
4. Verifies bus idle levels after each run and prints per-device and aggregate
   pass/fail results.

The sketch does not write MCP23008 output registers and does not evaluate
distance, color, or accelerometer data.

## Transactions and Result Rules

Each test run sets `Wire.setClock()` to the active speed, configures
`Wire.setWireTimeout(3000, true)` where supported, and clears the timeout flag
before every transaction.

- MCP23008: read the one-byte IODIR register (`0x00`) at `0x20`. It has no
  chip ID, so the first successful value is retained and every later value
  must match it.
- TCS34725: read ID using command byte `0x92` (`0x80 | 0x12`) at `0x29`; the
  value must be `0x44`.
- VL53L0X: after readdressing, read one byte from model-ID register `0xC0` at
  `0x2A`; the value must be `0xEE`.

Every register read uses write-register-pointer, repeated start, and exact
one-byte receive. The test records total attempts, successful reads, I2C
transmission failures, short reads, mismatched values, and Wire timeouts for
each device and speed. A run passes only when every expected device is
identified, all 1,000 reads for every device succeed with no mismatches or
timeouts, and both lines are idle high after the run. The final summary passes
only when both speed runs pass.

## Limits and Failure Handling

The sketch reports stuck-low lines, missing devices, invalid identities,
transaction errors, short reads, mismatches, and Wire timeouts explicitly. It
attempts the standard nine-clock recovery and STOP condition before every
speed run, verifies both SDA and SCL afterward, and repeats recovery after a
timeout or non-idle result. Every recovery result is printed with its before
and after SDA/SCL state.

Every speed run starts by holding XSHUT low, testing MCP23008/TCS34725, then
releasing XSHUT and moving VL53L0X to `0x2A`. It ends by holding XSHUT low, so
the next run begins from the VL53L0X default address and the finished test
cannot leave a device colliding with TCS34725 at `0x29`.

After releasing XSHUT, the sketch waits 10 ms, then sends exactly one
write-only address transaction to `0x29`: register pointer `0x8A`, followed by
the seven-bit address value `0x2A`. `Wire.endTransmission()` must return zero.
The sketch then makes up to 10 model-ID read attempts at `0x2A`, spaced 2 ms
apart; the first read of register `0xC0` returning `0xEE` confirms the move.
Failure of the address write or all 10 confirmation reads marks VL53L0X absent
for that speed run and skips its 1,000-read stress loop.

`config.h` and `arduino-train-v2.ino` will replace their obsolete MCP23008
`0x24` wiring/checklist comments with the confirmed `0x20` default address.

Software can establish reliable I2C communication symptoms but cannot prove
pull-up resistor values, signal voltage margins, rise times, ringing, or
electrical noise. Those require a multimeter and, for waveform quality, an
oscilloscope or logic analyzer.
