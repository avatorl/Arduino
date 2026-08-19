# Manual Speed Boost Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a timed 7 V fourth manual speed level with a 50-second boost-only lockout.

**Architecture:** Extend the existing manual step state from 0–3 to 0–4 while
keeping auto-distance explicitly capped at normal level 3. A small non-blocking
boost state machine owns active, expiry, and cooldown timestamps.

**Tech Stack:** Arduino C++, Arduino Nano, `arduino-cli`.

---

### Task 1: Add manual boost state

**Files:**
- Modify: `arduino-train-v2/arduino-train-v2.ino:1045-1200, 1450-1625, 1780-1905`

- [ ] Add named normal/boost voltage, step-index, duration, cooldown, active,
  expiry, and cooldown-end constants/state.
- [ ] Expand manual steps to `{0, 3.5, 4.5, 6.0, 7.0}` and keep auto-distance
  explicitly pinned to normal level 3.
- [ ] Add a wrap-safe non-blocking update that returns active boost to level 3
  after 10 seconds and starts cooldown.
- [ ] Make stop, decrease, auto-mode entry, and fault cancellation end an
  active boost and start cooldown; normal operations must not start cooldown.
- [ ] Make `CH+` enter level 4 only after level 3 and only when cooldown has
  expired; during lockout retain level 3.

### Task 2: Validate behavior

**Files:**
- Modify: `arduino-train-v2/README.md`

- [ ] Document boost level, duration, cooldown, and manual-only scope.
- [ ] Compile:

```powershell
arduino-cli compile --fqbn arduino:avr:nano arduino-train-v2
```

- [ ] Confirm compiler output is successful and normal plus auto modes remain
  limited to 6 V.
