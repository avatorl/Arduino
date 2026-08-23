# Arduino Train v2 Serial Startup Cleanup Design

## Goal

Ensure a serial-monitor session starts at an unambiguous firmware boundary,
without displaying bytes emitted by the pre-reset instance of the sketch.

## Scope

Add a PowerShell monitor launcher for the train sketch and a firmware startup
delimiter. This affects diagnostics only; it must not change train control,
power policy, or EEPROM contents.

## Design

### Host monitor launcher

Add a PowerShell script under `arduino-train-v2` that opens `COM8` at 115200
baud through .NET's `System.IO.Ports.SerialPort`.

The script will:

1. open the port with DTR and RTS enabled, which preserves the existing
   Nano auto-reset behavior;
2. wait longer than the sketch's 1000 ms debug attachment delay;
3. discard every input byte accumulated during the reset window;
4. display only bytes received after that discard until Ctrl+C closes the
   port.

The port, baud rate, and reset-settle duration will be script parameters with
the current hardware defaults. Failure to open the port will report the
underlying error and return a non-zero exit code.

### Firmware delimiter

Add one debug-gated startup line immediately after the existing serial
attachment delay and before any peripheral initialization:

`=== TRAIN STARTUP ===`

The delimiter is emitted only when `DEBUG_ANY` enables serial support. It
does not wait for a host connection, flush the UART, change the existing
1000 ms delay, or alter output when every debug category is disabled.

The launcher removes stale pre-reset bytes; the delimiter lets a user
recognize a real later reset while reading live output.

## Error Handling

The launcher must close the serial port in a `finally` block. It must not
silently fall back to another port or baud rate. Firmware makes no claim that
it can remove bytes transmitted before a DTR reset; that responsibility
remains with the launcher.

## Validation

- Run the launcher against COM8 and confirm its first displayed line is
  `=== TRAIN STARTUP ===`.
- Open the launcher repeatedly and confirm no prior EEPROM-dump fragments
  appear before the delimiter.
- Build for the Arduino Nano with debug enabled and disabled; confirm the
  delimiter is absent when debug is fully disabled.
- Confirm a deliberate reset during monitoring produces one new delimiter.
