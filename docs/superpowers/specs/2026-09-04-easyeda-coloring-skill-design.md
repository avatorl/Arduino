# EasyEDA Coloring Skill Design

## Purpose

Provide a repository-local Copilot skill that consistently applies this
repository's EasyEDA schematic JSON color conventions during schematic
creation and editing.

## Scope

Add `.github/skills/easyeda-coloring/SKILL.md`. It is a thin wrapper around
the existing root-level `easy-eda-color-schema.md`, which remains the
authoritative human-readable schema.

The skill applies when an agent creates or edits an EasyEDA schematic JSON
file. It directs the agent to read the canonical schema before making a
change, determine the relevant EasyEDA record types, and follow the
schema's color, width, fill, pin, and text rules.

## Behavior

The skill will require this workflow:

1. Read `easy-eda-color-schema.md` from the repository root before changing
   an EasyEDA schematic JSON file.
2. Parse the current JSON and identify the target record types, including
   `LIB~`, `P~`, `W~`, `J~`, and `T~`.
3. Make only the requested, record-scoped updates.
4. Preserve intentionally manual colors and exceptions unless the request
   explicitly changes them.
5. Parse and validate the edited JSON, including independently checking
   wire/junction styling, symbol graphics, and module-pin styling where
   affected.

The skill will prohibit global color replacement and palette-wide edits as a
shortcut. Installing this skill will not alter Arduino firmware, vendored
libraries, existing schematic content, or the canonical schema itself.
When subsequently invoked for a schematic task, the skill is specifically
intended to guide the requested schematic JSON changes.

## Documentation and Compatibility

`easy-eda-color-schema.md` remains in its existing root location, so the
existing repository instruction continues to work. The skill references this
single source of truth instead of duplicating its palette, preventing future
rule drift.

`SKILL.md` will contain YAML front matter with `name` set to
`easyeda-coloring` and a concise `description` that states when to invoke the
skill. Its Markdown body will provide the workflow and safety constraints.

## Validation

Validate that `SKILL.md` has the expected skill metadata and references the
canonical guide at the correct relative path. No project compilation or
hardware validation is needed because this change only adds agent guidance.
