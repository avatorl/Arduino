# Arduino Train Beginner Learning Guide Design

## Goal

Create `arduino-train-v2\LEARN.md`, a short beginner-friendly reference for the Arduino and C++ concepts that are actually used by the Arduino Train v2 sketch.

## Audience and scope

The reader is comfortable following the train README but is still learning Arduino and C++. The guide will be project-linked rather than generic: explanations will point to real files and use small examples based on the sketch. It will explain concepts without reproducing large implementation sections or attempting to document every sensor register.

## Content structure

1. How the sketch is organized and how Arduino `.ino` tabs are compiled.
2. Memory on an Arduino Nano:
   - flash/program memory versus SRAM;
   - `PROGMEM`, `pgm_read_*()`, and `F()`;
   - EEPROM, persistence, wear, and `EEPROM.update()`.
3. C++ building blocks used in the project:
   - declarations, definitions, scope, functions, references, pointers, `const`;
   - integer types and choosing widths;
   - `constexpr`, `enum class`, plain enums, structs, arrays, templates, and `static_assert`;
   - `nullptr`, casts, `constrain()`, and overflow-aware arithmetic.
4. Arduino hardware basics:
   - pin modes and digital/analog reads and writes;
   - PWM and what `analogWrite()` really does;
   - Nano timer ownership, including the IRremote Timer1 / `tone()` Timer2 arrangement and the D9/D10 limitation;
   - I2C addressing and shared SDA/SCL devices;
   - active-low inputs and pull-ups.
5. Timing and control flow:
   - `setup()`/`loop()`;
   - `millis()` timing and rollover-safe comparisons;
   - non-blocking state machines versus `delay()`;
   - the project’s motor reverse delay, buzzer, sleep, and sensor polling patterns.
6. Preprocessor directives and compile-time configuration:
   - `#include`, `#define`, `#if/#ifdef`, `#ifndef`, conditional debug macros, and `#pragma once`;
   - when to prefer `constexpr` over a macro.
7. Embedded programming patterns visible in the project:
   - bit masks and shifts;
   - lookup tables and integer interpolation;
   - ring buffers;
   - lightweight register-level drivers;
   - safety latches and explicit error returns.
8. A short project map, common beginner mistakes, and a recommended reading order.

## Editorial rules

- Keep explanations short and practical.
- Explain each term before using it.
- Prefer ASCII diagrams/tables and small code blocks.
- Clearly distinguish AVR/Nano-specific behavior from portable C++.
- Link examples to project files, but avoid brittle line-number references.
- Do not prescribe unrelated refactors or change source behavior.

## Validation

Review the guide against the current `arduino-train-v2` source and README for accuracy, especially pin/timer assignments, memory sizes, data types, and library behavior. Since this is documentation-only, no firmware behavior changes or new automated tests are required.
