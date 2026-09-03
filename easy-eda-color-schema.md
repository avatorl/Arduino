# EasyEDA Circuit Diagram Color Schema

Use this guide when creating or editing EasyEDA schematic JSON files in this repository. Preserve manually assigned exceptions unless the user explicitly asks to normalize them.

## Color Palette

| Purpose | Color | Hex |
| --- | --- | --- |
| Component and symbol outlines | Dark grey | `#666666` |
| Positive power | Red | `#FF0000` |
| Ground and negative supply | Black | `#000000` |
| SDA | Blue | `#0000FF` |
| SCL | Purple | `#9900FF` |
| General signal wires | Green | `#008800` |
| Other module pins and pin labels | Brown | `#880000` |
| Component names and references | Dark blue | `#000080` |
| Canvas | White | `#FFFFFF` |

## Symbols

- Draw component and module outlines in dark grey (`#666666`). This includes rectangles, circles, arcs, polygons, and other graphical primitives.
- Keep symbol bodies unfilled unless a meaningful fill is specified below.
- Do not recolor external wires while changing symbol graphics. In EasyEDA JSON, component symbols are normally stored in `LIB~` entries; schematic wires are separate `W~` entries.
- Keep the drawing frame and frame outlines dark grey. Keep frame coordinate labels brown.
- Keep component names, values, and reference designators dark blue unless a deliberate existing exception is required.

## LED Fills

- Fill each LED diode body with the color of the light it represents while retaining a dark-grey outline.
- Use red (`#FF0000`), green (`#00FF00`), or blue (`#0000FF`) for the corresponding LED body.
- For RGB LEDs, arrange and label channels `R`, `G`, `B` from left to right.
- Do not apply a body fill to ordinary diodes such as `D1`.
- Do not apply a fill to the USB output graphic on the charger module.
- Do not recolor decorative LED arrows unless specifically requested.

## Wires And Junctions

| Net type | Wire color | Width | Junction size |
| --- | --- | ---: | ---: |
| Positive power at 5 V or below (`+5V`, `3V3`, and similar) | Red (`#FF0000`) | 1 | 2 |
| Positive power above 5 V (`V+`, `VIN`, `B+`, `P+`, and similar, only when the actual rail exceeds 5 V) | Red (`#FF0000`) | 3 | 4 |
| Ground or negative return (`GND`, `G`, `B-`, `P-`, and similar), at any voltage | Black (`#000000`) | 1 | 2 |
| SDA | Blue (`#0000FF`) | 1 | 2 |
| SCL | Purple (`#9900FF`) | 1 | 2 |
| Other signals | Green (`#008800`) | 1 | 2 |

- A junction dot must use the same color as the wire or net it joins.
- Ground and negative-return wires always use width 1 with junction size 2, regardless of voltage.
- Use width 1 and junction size 2 for non-ground signals and positive power rails at 5 V or below.
- Reserve width 3 with junction size 4 for positive power rails above 5 V.
- A name such as `VIN`, `VCC`, or `V+` does not by itself justify a wide line. Confirm that the actual voltage is above 5 V.
- Preserve wires that were manually set to brown (`#880000`). Do not normalize them to green automatically.
- Preserve any other intentional, manually assigned wire color unless the requested edit explicitly changes that net.
- At a mixed-width junction, use the size associated with the widest connected wire.
- Do not change the global wire or junction palette as a shortcut. Edit the intended `W~` and `J~` records so unrelated nets retain their colors.

## Module Pins And Pin Labels

Apply these semantic pin colors only to chip-based, multi-pin modules such as Arduino boards, sensor modules, motor drivers, expanders, chargers, and similar `U` components.

| Pin type | Pin line, pin name, and pin number color |
| --- | --- |
| Ground or negative supply (`GND`, `B-`, `P-`, and similar) | Black (`#000000`) |
| Positive power (`V+`, `VCC`, `VDD`, `VIN`, `+5V`, `3V3`, `VS`, `B+`, `P+`, and similar) | Red (`#FF0000`) |
| SDA | Blue (`#0000FF`) |
| SCL | Purple (`#9900FF`) |
| Other wired module pins | Same color as the connected wire |
| Other unwired module pins | Dark grey (`#666666`) |

- Apply pin colors in this order: semantic exceptions for positive, negative, SDA, and SCL; connected-wire color; dark-grey fallback for unwired pins.
- The pin line, pin name, and pin number must all use the selected color.
- `G` is an ordinary input pin, not an alias for `GND`. Color it from its connected wire, or dark grey when unwired.
- Do not apply these module-pin rules to LEDs, ordinary diodes, resistors, capacitors, switches, batteries, motors, fuses, buzzers, or other discrete components.
- Preserve discrete-component pin styling unless the user explicitly requests a change.

## Text And Labels

- Keep component names, values, and reference designators dark blue (`#000080`).
- Use the module-pin semantic colors above for pin names and pin numbers.
- Keep positive voltage labels red (`#FF0000`), including single values such as `+5.0V` and ranges such as `+7.0V..+8.4V`.
- Keep labels and annotations added by the user dark grey (`#666666`) by default.
- Explicit semantic exceptions take precedence over the grey default. For example, positive-voltage labels remain red, and labels intentionally identifying SDA or SCL may match their blue or purple net colors.
- Do not recolor labels when changing only component graphics.

## Safe Editing Procedure

1. Parse the current JSON before editing; never rely on an older copy or previous chat state.
2. Identify the exact record type: `LIB~` for symbols, `P~` subrecords for pins, `W~` for wires, `J~` for junctions, and `T~` for text.
3. Make the smallest record-scoped change. Avoid global color replacement because identical colors can occur in symbols, pins, labels, wires, and fills.
4. Preserve unknown and manually assigned colors unless the requested rule clearly covers them.
5. Parse the JSON again after editing.
6. Validate wire colors and widths, junction colors and sizes, symbol outlines and fills, and module pin/name/number colors independently.
