# Pi GUI v2.5 BETA — TC106 four-digit actual tension review

**Status:** BETA / NOT PRODUCTION / independent Claude review pending  
**Date:** 2026-10-24  
**Repository:** `jtsuka/NTC-1A-Control`  
**Baseline before GUI beta:** `503b7155b42df8d2017cc351c4e77c83638f7a46`

## Objective and basis

2026-10-23 real TC106 Direction-B data contains valid `tens_actual=1023` encoded as
`00 00 00 03 66 7F` when LEN=0. Pi serial v1.3.4 rejects the valid
`0x66` upper-tens field. The new Pi communication beta
`Python/NTC_1A_serial_comm_v1_3_5_beta.py` accepts 0..1023 by
field-specific checks. GUI v2.4 still imports the old communication module.

## Newly added files — existing files are unchanged

- `Python/ntc_pygame_ui_v2_5_beta.py`: copy of v2.4 with beta serial import and fitted actual-tension text.
- `Python/tests/test_ntc_gui_v2_5_beta_20261024.py`: hardware-free AST helper and source-contract regression.
- This document: specification, test and independent review instructions.

Original `Python/ntc_pygame_ui_v2_4.py` and
`Python/NTC_1A_serial_comm_v1_3_4.py` remain unchanged.

## v2.5 BETA behavior

1. Import `NTC_1A_serial_comm_v1_3_5_beta` rather than v1.3.4.
2. Keep the v2.4 2-column, 3-row layout and `RIGHT_MONITOR_VALUE_X_SHIFT=0.04`.
3. Only the `ACTUAL_TENSION` read-only value uses a fitted text renderer.
4. Fit text to the existing `rect.width-10` available width; reduce font size if necessary,
   with a last-resort proportional scale for exceptionally narrow displays.
5. Keep right-aligned value drawing; `WIND_LENGTH`, `RX_STATUS`,
   editable fields, commands, test panel and serial port startup unchanged.
6. Use the GUI title `NTC-1A Main Controller UI v2.5 BETA` to distinguish it.
7. Do not change the active launcher/startup service or GUI v2.4 import.
8. This GUI is intended for explicit beta execution, not automatic switchover.

## Offline test (not a hardware E2E claim)

```sh
python3 -m unittest discover -s Python/tests -p 'test_ntc_gui_v2_5_beta_20261024.py' -v
python3 -m unittest discover -s Python/tests -p 'test_direction_b_tension_beta_20261024.py' -v
```

The GUI test checks import isolation, full text width at 0/9/99/100/999/1000/1023,
very narrow display fallback, and retention of v2.4 monitor layout.
It deliberately extracts the new helper through Python AST without importing
the GUI module: importing it directly would start a serial thread and pygame.

**Execution status at creation:** not yet run on Raspberry Pi or CI. Test source is
registered for independent execution and Claude review. Do not report PASS until executed.

## Explicit Raspberry Pi beta test (manual, not yet performed)

1. Copy the two beta modules into the same `Python` directory, preserving originals.
2. Start `python3 ntc_pygame_ui_v2_5_beta.py --port /dev/serial0` only after
   confirming the UART is available and other readers are stopped.
3. Feed known-good 6-byte telemetry through the actual ESP/Pi serial path.
4. Confirm `ACTUAL TENSION` displays `1023` in full, and LEN / RX STATUS
   remain correct; visually inspect at the real screen resolution.
5. Repeat with 0, 999, 1000, and malformed packets, and record screenshots/logs.
6. If it fails, stop the beta and use the untouched v2.4 as rollback.

Do not inject arbitrary voltage directly into the Raspberry Pi UART or LA.
Previous measured TC106 relay HIGH was approximately 5.3..5.4 V; use
appropriate interface circuitry and verify analyzer input ratings.

## Claude independent cross-review checklist

- Diff v2.4 against v2.5 beta: confirm ONLY the import, title, helper and
  `ACTUAL_TENSION` rendering changed; original is byte-identical.
- Review `render_fitted_actual_tension` on 800x480, 1024x600 and real Pi
  display sizes; confirm right alignment, no overlap, legibility.
- Review side effects of importing v1.3.5 beta and all 6-byte validation boundaries.
- Run both offline test files and report exact command, Python version, PASS/FAIL.
- Check existing GUI test panel, SEND/RESET/SENS.ADJ, Boot Magic and port behavior.
- Check beta startup is explicit and rollback requires no Git history reset.
- Identify any issue blocking Stage C and do not claim real-device PASS without
  Pi+ESP hardware evidence.

## Review result

Pending. No automatic production change authorized.
