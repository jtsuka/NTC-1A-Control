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

Updated 2026-11-28. Confirmed on the real Pi: v3.9.x starts; button layout; WIFI SCAN readout;
WIFI SETUP list and SAVE of a new network (v3.9.1); REBOOT cancel / reboot / GUI auto-restart;
`ntc` may run the reboot command. SIGTERM alone writes `STOP`. The pre-reboot CSV
lacked `STOP` with v3.9.1 — v3.9.2 closes the log first and **passed on the real Pi
(2026-11-28)**: `NOTE,REBOOT` then `STOP`. **Still open:** OS-refused reboot (headless only); SAVE of an existing profile; CONNECT NOW;
the WIFI SETUP button is partly hidden while WIFI SCAN's panel is open (cosmetic, v3.9).

### D8. Wi-Fi stability after the router move (observation pending)

2026-11-28 survey: -77 → about -70 dBm at the original bad spot after raising the router 1 m
(details: `CURRENT_SYSTEM_STATUS.md` §7.3). Only about 40 minutes without a drop were seen.
**Evening observation 2026-11-28 (CONFIRMED from ssh output, 2F desk):** -68 dBm, rx 57.7 /
tx 72.2 Mb/s; connected 840 s, tx 1034 packets, `tx failed` 51 (about 4.9 %; the morning "4" has no
packet count, so the two are not comparable). The journals of the current and previous boot contain
**no** wpa/NetworkManager drop events (`CTRL-EVENT-DISCONNECTED`, deauth) — the only "disconnect"
lines are boot-time state changes. `last -x` shows two abrupt power losses (`crash`, no `shutdown`;
around 19:27 and 19:39–19:40), most likely battery/relocation handling (HYPOTHESIS), not Wi-Fi: no
under-voltage message in the journal (the last minutes before a power cut may be missing). Correction:
the tailscaled "rebind" at 19:39:00 was caused by a **clock jump** (`slept 10m28s`: the Pi has no RTC,
time starts old and is stepped by NTP), not by a Wi-Fi change — `wlan0` kept 192.168.11.36.
Use `mono_s`, not `wall_time`, for intervals inside a CSV. **Not yet done:** `vcgencmd get_throttled`,
a repeat of `tx failed` / `tx packets` after 10 min to get the failure rate trend.
**Needed:** a day or more of undisturbed running with the Pi fixed in place — count disconnect
lines with `journalctl --since yesterday | grep -i -E "disconnect|deauth|CTRL-EVENT-DISCONNECTED"`
and re-read `tx failed`. If drops return: move the router toward the door/hall (next step,
not tried), then consider an EasyMesh-compatible repeater (model compatibility **not checked**).

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

### D7. File naming and running-version identification (Pi side implemented 2026-11-28; ESP32 side open)

**Status 2026-11-28:** Pi side (item 1 below) implemented in v3.9.3 (PR pending): single file `Python/ntc_pygame_ui.py`, build id on screen / title / log `build=`, old GUI files in `Python/OLD/gui/`, tag scheme `gui-vX.Y.Z-beta`. **Real-Pi PASS 2026-11-28:** build id seen on screen, in the log START line and in `git describe` (all `gui-v3.9.2-beta-2-g87265d4`). Tag `gui-v3.9.2-beta` was created and pushed from the Pi (this session cannot push tags); launcher `APP` now `ntc_pygame_ui.py`. **Still open:** ESP32 version reporting (spec first), consolidation of other version-named files (`NTC_1A_serial_comm_v*`, `ESP32/` sketches, `pi_logger_*`), the launcher script is still not in Git.


Observed 2026-11-28: the tree holds many version-named files side by side (e.g. `ntc_pygame_ui_v2_4.py` ... `v3_9_beta.py`, many `ESP32/` sketch folders), and the running version is identified only by file name and the `GUI_REVISION` string.

Direction proposed by the operator (RCS-style `$Revision$` idea): keep **one** file name per component; take the version from Git instead of the file name; make the running version visible at run time, **including the ESP32 firmware**.

Candidate approach (DERIVED from how Git works; nothing implemented, nothing tested):

1. Pi / GUI: one file name; at start-up the GUI reads `git describe --tags --dirty` from its own clone, shows it small in a screen corner, and writes it into the evidence-log header. (Git does not expand `$Revision$`; `ident` gives only a blob hash.)
2. ESP32: no Git on the device, so the version string must be embedded at build time. Report it to the Pi once at boot, next to the existing Boot Magic (`DE AD BE EF 01 7F`), in a form that cannot be confused with a Direction-B 6-byte frame; or add a query command. **Either option changes the Pi/ESP32 serial protocol and needs a written spec and a regression run first.**
3. Order: Pi side first (no protocol change) → ESP32 spec → ESP32 file consolidation. Start only after the real-Pi checks in D3 are finished.

Open points: the tag naming scheme; what to do with the existing version-named files (Pi `start_ntc_gui.sh`, hashes recorded in `CURRENT_SYSTEM_STATUS.md` §7 and the v3.6 design document all depend on the current names); how the ESP32 build gets the string (Arduino IDE has no pre-build hook by default).

## Closure format

When closing an item, record the evidence and conclusion in `TC106_DECISIONS.md`, then update `TC106_BASELINE.md` and this file in the same change set when practical.
