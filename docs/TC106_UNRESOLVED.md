# TC106 Unresolved Items

**Updated:** 2026-11-28 (Priority D added; Priorities A-C as of 2026-09-09, not re-reviewed)  
**Rule:** Do not promote an item in this file to fact without source or measurement evidence.

## Priority A — physical machine confirmation

### A1. Interface-board count and CH assignment

**Question:** Are there physically separate interface-board instances for Main CH1 and CH2, as implied by the two-level topology?

**Known:** Main CH1/CH2 receive paths are independent; one interface-board design has a shared `TC_TX_REP` return within its group.

**Needed evidence:** count boards in the real machine and trace their outputs to `ch1糸長入力` / `ch2糸長入力`, or locate authoritative assembly/BOM/wiring documentation.

### A2. Exact TC106 / winding-position identity inside each CH group

`windlen1_*` and `windlen2_*` identify Main receive domains, not individual TC106 wires. Determine how multiple TC106 units on a shared group are distinguished by timing/order/state and map them to physical winding positions.

## Priority A — Main ASM

### A3. Why live `Disp_set` path selects `windlen2_0`

Trace `ten_set`, `set_ch`, `length1_change_do`, `length2_change_do`, and all callers/state transitions. Determine the UI/design intent without assuming `ten_set` means only “CH1 tension setting.”

### A4. Full receive-channel/data-slot mapping

Trace `Data_110bps_1` and `Data_110bps_2` from PIC input bit through byte/frame counters into `windlen1..4` / `realtens1..4`. Establish how logical channels beyond the two physical Main receive inputs are formed.

## Priority B — real TC106 communication

### B1. Continuous Direction-B frame interval

Derived estimate is ~336.6 ms for a back-to-back 6-byte frame using ~3.3 ms x 17 slots/byte. Confirm with real TC106 logic-analyzer capture.

### B2. Direction-B coexistence with Direction-A

Confirm real TC106 continues/behaves correctly during SEND, RESET, and SENS.ADJ activity.

### B3. Actual-tension engineering units

`tens_actual` / transmitted actual-tension value is ADC-derived. Establish calibration and relationship to physical tension (e.g. gf) from real hardware/source/calibration evidence before assigning engineering units.

### B4. Wound-length behavior

Observe value immediately after RESET, during winding, and after stop. Compare old Main and replacement readings.

## Priority C — display/legacy behavior

### C1. Old Main 7-seg live behavior

Source proves an executable received wound-length display path. Verify what the real legacy Main visibly shows and correlate the displayed value to CH/group/physical winding position.

### C2. Historical purpose of dead display modes

`disp_ten` and `disp_len` dedicated modes appear dead. Their historical intended use is not established. Do not infer history from completed-looking dead code alone.

## Priority D — Pi GUI and Pi unit (added 2026-11-28)

State and evidence: `TC106-Doc/CURRENT_SYSTEM_STATUS.md` §7.

### D1. Two different v3.6 GUI files

`Python/ntc_pygame_ui_v3_6_beta.py` (`cf23e596145e70bd`, BASE v2.6, USB auto-selection) and
`Python/ntc_pi_gui_v3_6_beta_all_20261123/ntc_pygame_ui_v3_6_beta.py` (`a472268dd57db880`,
BASE v2.5 beta, the hash recorded in the v3.6 design document) differ. v3.7-v3.9 are built on
the first. **Needed:** which one the 2026-11-23 field tests ran; whether the design document
should be corrected or the archive copy marked superseded.

### D2. Pi software revision numbering

The v3.6 plan (§3.1) says all Pi modules bump to the same number together. v3.7-v3.9 changed
the GUI only; the serial and log modules are still v3.6-beta and unchanged. **Decide:** keep
GUI-only bumps (and say so), or re-align the numbers.

### D3. Real-Pi tests of REBOOT / WIFI SCAN / WIFI SETUP

REBOOT dialog seen on the real display; the actual reboot and the `NOTE,REBOOT` → `STOP`
CSV lines are not confirmed. v3.8 / v3.9 were tested headless with faked `iw` / `nmcli` only.
**Needed:** whether the user `ntc` can run `systemctl reboot` and `nmcli` (directly or with
`sudo -n`); touch usability of the button layout (EXIT-REBOOT gap 0.07 of screen height).

### D4. Missing or Pi-local artifacts

`Python/tests/test_rxlog_v3_6_beta_20261123.py` (named in the v3.6 design document) is not in
the tree; no `pi-gui-v3.6-beta` tag exists; `/home/ntc/start_ntc_gui.sh` is not in Git;
Stage B / C results of the v3.6 test plan are not recorded.

### D5. Pi restarts when the ESP32 is powered

Reported during the 2026-11-23 … 25 field tests; cause not found (power-supply sharing and
ground sharing are candidates only — HYPOTHESIS). Wi-Fi drops were separately traced mainly to
weak signal (DERIVED, `CURRENT_SYSTEM_STATUS.md` §7.3); persistent journald logging was
enabled on 2026-11-27 so the next occurrence leaves a record (`journalctl -b -1`).

### D6. Promotion of a v3.x beta to production

Not decided. Until it is, v2.4 / v1.3.4 remain the formal reference in `CURRENT_SYSTEM_STATUS.md` §1.

## Closure format

When closing an item, record the evidence and conclusion in `TC106_DECISIONS.md`, then update `TC106_BASELINE.md` and this file in the same change set when practical.
