# Repository Instructions

## Workspace model

This repository is a collection of independent Arduino projects, not a single
application or build. Select the target project before editing: a project is
normally centered on its sketch directory and its `.ino` entry point.

- `arduino-locomotive` is the actively developed multi-tab train firmware.
- `duplo-train` is the earlier single-sketch train implementation. DO NOT EDIT IT. IT's a legacy project.
- `SENSORS`, `TOOLS`, `OTHER` folders contain standalone sketches and
  experiments.
- `libraries` holds local Arduino library sources required by some sketches.
  Treat them as vendored dependencies: change them only when the requested
  work explicitly targets that library.

Before working in a project, read the closest `copilot-instructions.md` when
present, followed by that project's `README.md` and internal technical docs.
Project-local instructions take precedence over this file.

When a request names a specific project, work only within that project's
subdirectory. Do not read from or write to other Arduino root directories
unless the request explicitly requires them.

## Quick updates

When the user calls a request a "quick update", make only the required edits
as quickly as possible. Unless directly requested, do not brainstorm, create
specification or plan files, or create a commit.

## Build, test, and lint

There is no repository-wide build, test, or lint command. Compile the selected
sketch rather than the repository. Use its documented board FQBN and pass the
repository's local libraries directory to Arduino CLI when required:

```powershell
arduino-cli compile --fqbn <board-fqbn> --libraries D:\GITHUB\Arduino\libraries <sketch-directory> --warnings all
```

Run project-local test scripts directly. For example, the native train-logic
test runner accepts a compiler selection and falls back to Python when `g++`
is unavailable:

```powershell
& D:\GITHUB\Arduino\arduino-locomotive\test\native\run-tests.ps1
& D:\GITHUB\Arduino\arduino-locomotive\test\native\run-tests.ps1 -Compiler g++
```

No cross-project lint command is configured.

## Cross-project conventions

- Arduino sketches are often split across same-directory `.ino` tabs. Arduino
  combines those tabs into one translation unit, so shared declarations,
  globals, and include ordering can affect every tab in that sketch.
- Preserve each project's board target, pin assignments, I2C addresses, and
  hardware safety behavior unless the task explicitly changes the hardware
  design. Do not assume that a pin map or sensor setup transfers between
  projects.
- Prefer the repository-local copy of a library over an automatically
  downloaded library when compiling a project that depends on it.
- Hardware-dependent behavior should be validated by compiling the selected
  sketch; host-side tests cover only logic deliberately separated from hardware
  access.

## EasyEDA schematic JSON

When editing EasyEDA schematic JSON files, follow
`easy-eda-color-schema.md`. Make record-scoped changes after parsing the
current JSON; preserve intentional manual color exceptions; then parse and
validate the edited JSON again. In particular, do not use global color
replacement across `LIB~`, `W~`, `J~`, `P~`, and `T~` records.
