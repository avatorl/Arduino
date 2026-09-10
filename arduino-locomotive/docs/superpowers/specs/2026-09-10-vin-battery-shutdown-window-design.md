# VIN Battery Shutdown Window

## Purpose

Enable permanent low-battery shutdown only when the existing A0 VIN
measurement shows that a connected 2S pack is discharged, while rejecting
the lower voltage leaked through the MCP23008 expander when the pack is
disconnected and the Nano is powered over USB.

## Configuration

`config.h` will define two named millivolt thresholds:

- `VIN_BATTERY_SHUTDOWN_MIN_MV` set to 5500 mV.
- `VIN_BATTERY_SHUTDOWN_MAX_MV` set to 6500 mV.

The eligible range is exclusive: shutdown is permitted only for VIN readings
strictly greater than the minimum and strictly less than the maximum.

VIN low-battery shutdown will have its own enabled-by-default configuration
switch. The existing global shutdown switch will be replaced or split so it
does not suppress the requested VIN protection. The low-VCC (+5 V rail)
shutdown switch will remain separate and disabled by default. Existing
critical-overvoltage shutdown policy remains disabled.

## Behavior

A shared shutdown-eligibility helper will be used by both existing
low-battery shutdown routes:

1. Boot-time low-battery lockout.
2. The periodic, stopped-only battery guard after its existing confirmation
   rules have been satisfied.

The helper will only decide whether an already measured voltage can cause a
permanent shutdown. It will not take an ADC reading or change any existing
measurement scheduling, quiet-settle interval, median filtering, low-reading
confirmation, or implausibly-low handling.

Low-battery warning behavior remains unchanged. Critical-overvoltage behavior
remains unchanged. The +5 V rail shutdown policy remains disabled.

## Safety and Diagnostics

Readings at or below 5500 mV do not trigger permanent shutdown, preventing
USB-powered debug sessions with expander leakage from latching the train off.
Readings at or above 6500 mV likewise do not trigger the low-battery shutdown
path. Existing diagnostic logging and the normal last-trusted battery-voltage
handling remain in place.

## Validation

The native logic tests will cover the inclusive boundary rejections at 5500
and 6500 mV and acceptance of values inside the window. Tests will also
verify that boot-time and periodic shutdown checks apply the helper only after
their present measurement and confirmation flows. The complete production
Arduino Nano sketch will compile using its established local library
configuration.
