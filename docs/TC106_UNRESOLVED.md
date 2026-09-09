# TC106 Unresolved Items

**Updated:** 2026-09-09  
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

## Closure format

When closing an item, record the evidence and conclusion in `TC106_DECISIONS.md`, then update `TC106_BASELINE.md` and this file in the same change set when practical.
