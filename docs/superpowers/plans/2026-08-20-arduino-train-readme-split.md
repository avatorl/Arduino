# Arduino Train README Split Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rewrite `arduino-train-v2\README.md` to be short, friendly, and user-facing, while moving detailed technical content into `docs-internal`.

**Architecture:** Split the documentation into two clear layers. The public README will answer “what is this train, what can it do, and what do I need?”, while `docs-internal\TECHNICAL.md` will hold the detailed operational notes that were previously mixed into the README. `docs-internal\LEARN.md` remains the beginner concept guide and is only linked from the README.

**Tech Stack:** Markdown, existing Arduino Train v2 source tree, PowerShell for validation, git.

---

## File map

- Modify: `arduino-train-v2\README.md` — public overview, feature list, hardware list, safety warning, and links to internal docs.
- Create: `arduino-train-v2\docs-internal\TECHNICAL.md` — technical details removed from the README.
- Modify: `arduino-train-v2\docs-internal\LEARN.md` — only if a link or title needs to be adjusted for the new README wording.

### Task 1: Draft the internal technical reference

**Files:**
- Create: `arduino-train-v2\docs-internal\TECHNICAL.md`
- Reference: `arduino-train-v2\README.md`
- Reference: `arduino-train-v2\docs-internal\LEARN.md`

- [ ] **Step 1: Collect the technical sections that no longer belong in the README**

  Read the current README and identify the sections that should move: sleep/warning/shutdown behavior, electrical and power notes, build and verification commands, color marker calibration, and any other implementation-specific notes.

- [ ] **Step 2: Write the new internal technical file**

  Create `docs-internal\TECHNICAL.md` with short technical sections that preserve the removed details in a structured way. Keep it practical and factual, not beginner-oriented, and link out to `LEARN.md` where concepts need explanation.

- [ ] **Step 3: Validate the new technical doc**

  Review the Markdown for clear headings, correct links, and no lost content from the old README technical sections.

  Run:

  ```powershell
  Select-String -Path D:\GITHUB\Arduino\arduino-train-v2\README.md -Pattern '## How to explain sleep, warning, and shutdown|## Electrical and power notes|## Build and verification|## Color marker calibration'
  ```

  Expected: matches in the old README that have been fully represented in `docs-internal\TECHNICAL.md`.

- [ ] **Step 4: Commit the internal doc**

  ```powershell
  git add -- arduino-train-v2/docs-internal/TECHNICAL.md
  git commit -m "Add train technical documentation

  Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
  ```

### Task 2: Rewrite the public README

**Files:**
- Modify: `arduino-train-v2\README.md`
- Reference: `arduino-train-v2\docs-internal\TECHNICAL.md`
- Reference: `arduino-train-v2\docs-internal\LEARN.md`

- [ ] **Step 1: Decide the final public README outline**

  Keep only:
  - project name and one-sentence summary;
  - what the train can do;
  - what hardware it uses;
  - optional parts;
  - a short safety warning;
  - links to `docs-internal\LEARN.md` and `docs-internal\TECHNICAL.md`.

- [ ] **Step 2: Remove technical detail from the public README**

  Delete the sleep/warning/shutdown deep dive, electrical notes, build commands, and calibration instructions from the public README. Replace them with short links to the internal docs.

- [ ] **Step 3: Tighten the tone for beginners**

  Rewrite bullets and headings so the README feels approachable and non-technical. Prefer plain language and short sentences. Avoid implementation jargon unless it is absolutely necessary for identifying hardware.

- [ ] **Step 4: Add internal doc links**

  Add a compact “Learn more” or “Internal docs” section linking to:
  - `docs-internal\LEARN.md`
  - `docs-internal\TECHNICAL.md`

- [ ] **Step 5: Commit the README rewrite**

  ```powershell
  git add -- arduino-train-v2/README.md arduino-train-v2/docs-internal/LEARN.md
  git commit -m "Simplify train README and link internal docs

  Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
  ```

### Task 3: Validate the split

**Files:**
- Review: `arduino-train-v2\README.md`
- Review: `arduino-train-v2\docs-internal\TECHNICAL.md`
- Review: `arduino-train-v2\docs-internal\LEARN.md`

- [ ] **Step 1: Check that the README is user-facing only**

  Confirm the public README no longer contains build commands, calibration steps, or detailed battery/sleep implementation notes.

- [ ] **Step 2: Check that technical details still exist internally**

  Confirm every removed technical topic is present in `docs-internal\TECHNICAL.md` and that the beginner concept reference still points readers to the right place.

- [ ] **Step 3: Check links and formatting**

  Verify all Markdown links render correctly and the README still reads cleanly from top to bottom.

- [ ] **Step 4: Measure readability**

  Use a rough word-count check on the README to make sure it stayed concise.

  Run:

  ```powershell
  (Get-Content D:\GITHUB\Arduino\arduino-train-v2\README.md -Raw | Measure-Object -Word).Words
  ```

  Expected: significantly shorter than the original README and easy to scan in under a minute.

- [ ] **Step 5: Commit any final fixes**

  ```powershell
  git add -- arduino-train-v2/README.md arduino-train-v2/docs-internal/TECHNICAL.md arduino-train-v2/docs-internal/LEARN.md
  git commit -m "Finalize train documentation split

  Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
  ```
