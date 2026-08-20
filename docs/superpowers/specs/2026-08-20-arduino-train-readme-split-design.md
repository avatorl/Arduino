# Arduino Train README Split Design

## Goal

Rewrite `arduino-train-v2\README.md` so it is short, friendly, and focused on what the train can do, what features it has, and what hardware it uses, while moving technical details into `docs-internal`.

## Scope

- Keep `README.md` user-facing.
- Keep `docs-internal\LEARN.md` as the beginner Arduino/C++ concepts guide.
- Add `docs-internal\TECHNICAL.md` for project-specific technical details removed from the README.

## Content boundaries

### README.md

Include only:

- plain-language project overview;
- feature list;
- main hardware parts and optional parts;
- safety warning for the battery pack;
- links to `docs-internal\LEARN.md` and `docs-internal\TECHNICAL.md`.

Remove from the README:

- sleep/warning/shutdown behavior details;
- electrical and power notes;
- build and verification commands;
- color-marker calibration instructions;
- any other implementation-specific or troubleshooting detail.

### docs-internal\TECHNICAL.md

Move the removed project-specific information here, keeping it readable but technical. Cover:

- sleep, warning, shutdown, and wake behavior;
- electrical and power notes;
- build and verification commands;
- color-marker calibration workflow;
- any other detailed notes that no longer belong in the public README.

## Editorial rules

- Keep README concise and welcoming for beginners.
- Avoid duplicating the detailed concept explanations already covered by `LEARN.md`.
- Use Markdown links and short sections.
- Preserve existing factual behavior and hardware descriptions.

## Validation

- Review the rewritten README for clarity and brevity.
- Review `docs-internal\TECHNICAL.md` for completeness.
- Check that the new doc links are correct.
- No firmware tests are required because this is documentation-only.
