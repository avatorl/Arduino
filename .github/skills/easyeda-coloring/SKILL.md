---
name: easyeda-coloring
description: Apply the repository's EasyEDA schematic JSON color schema when creating or editing EasyEDA schematics.
---

# EasyEDA Schematic JSON Coloring

Use this skill only when creating or editing EasyEDA schematic JSON.

Before making edits, read the repository-root `easy-eda-color-schema.md`.

1. Parse the current JSON and identify the relevant `LIB~`, `P~`, `W~`, `J~`, and `T~` records.
2. Make only the requested record-scoped changes.
3. Preserve unknown or intentionally manual colors and exceptions unless they are explicitly targeted.
4. Never use global color replacement or palette-wide edits.
5. Parse and validate the edited JSON. Independently check affected wire/junction, symbol, and module-pin styling.

Do not duplicate palette contents from the canonical guide. Do not modify the canonical guide, schematics, firmware, or vendored libraries.
