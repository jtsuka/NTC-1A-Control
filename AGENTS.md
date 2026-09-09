# TC106 Project AI Instructions

Before analyzing or modifying this project, read these files in order:

1. `docs/AI_PROJECT_CONTEXT.md`
2. `docs/TC106_BASELINE.md`
3. `docs/TC106_DECISIONS.md`
4. `docs/TC106_UNRESOLVED.md`

## Rules

- Treat primary source code, original circuit diagrams, and measured waveforms as authoritative.
- Do not replace confirmed facts with assumptions or conclusions based only on prior AI conversation.
- Distinguish **CONFIRMED**, **DERIVED**, **HYPOTHESIS**, and **UNRESOLVED**.
- When an interpretation is corrected, preserve the correction in `docs/TC106_DECISIONS.md` so the old interpretation is not reintroduced.
- When a new hardware/software fact is confirmed, update the relevant context/baseline/decision document.
- For current executable/sketch/GUI filenames, inspect the current Git tree rather than relying on historical chat memory.

## Critical Direction-B warning

Do **not** interpret "Direction-B is one shared line" as meaning CH1 and CH2 share one Main Controller receive input.

Current confirmed model:

- Main Controller CH1 and CH2 have independent physical Direction-B receive paths.
- Within each CH group, multiple TC106 units share the group's return bus through the interface-board topology.
- `windlen1_*` / `windlen2_*` therefore must not be interpreted as individual TC106 wire numbers.

See the four documents above before continuing analysis.
