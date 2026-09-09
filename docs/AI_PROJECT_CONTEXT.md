# TC106 AI Project Context

**Updated:** 2026-09-09  
**Purpose:** Compact shared memory for ChatGPT, Claude, and human developers. Read before TC106 analysis.

## Project objective

Develop a replacement Main Controller for the TC106 tension-controller system using the latest verified TC106 C source and original Main Controller assembler as the protocol basis. ESP32-S3 performs the timing-critical bridge/communication work and Raspberry Pi provides the upper GUI. Arduino Nano Every is used as an emulator/test source.

## Source priority

1. Original Main Controller ASM and original TC106 C source
2. Original Main Controller / TC106 circuit diagrams
3. Measured real-unit waveforms and verified logic-analyzer captures
4. Reverse-engineered interface-board KiCad/netlist
5. Test/emulator source and project reports
6. AI interpretation

Historical TC106 primary source: `TC-FW/History/FW_TC106-1_v0.0_250201.c`  
Normalized analysis reference: `TC-FW/FW_TC106-1_v0.0_250201-UTF8.c`

## CRITICAL FIXED PREMISE — Direction-B physical hierarchy

**Main Controller CH1 and CH2 are independent physical receive paths.** The Main Controller circuit diagram has separate `ch1糸長入力` and `ch2糸長入力` signal-conditioning/PIC input paths, corresponding to `rx_ch1` / `Data_110bps_1` and `rx_ch2` / `Data_110bps_2` in ASM.

At the interface-board level, multiple TC106 units within a CH group share the Direction-B return bus (`TC_TX_REP`). The interface-board material also contains the note `RX CN4のみ結線`.

Therefore the correct two-level model is:

```text
CH1 group: multiple TC106 -> shared return bus -> Main ch1糸長入力 -> rx_ch1 -> Data_110bps_1 -> windlen1_*
CH2 group: multiple TC106 -> shared return bus -> Main ch2糸長入力 -> rx_ch2 -> Data_110bps_2 -> windlen2_*
```

**Never reduce this to “all Direction-B signals are one Main receive line.”**  
**Never interpret `windlen1_*` / `windlen2_*` as individual TC106 physical wire numbers.**

The physical existence/count of separate CH1/CH2 interface-board copies is still to be verified on the real machine; do not present “two boards exist” as confirmed until physically/documentarily verified.

## Confirmed Direction-B payload

TC106 sends a 6-byte frame:

```text
byte0..2 = lastlen[0..2]  (wound length)
byte3..4 = txtens[0..1]   (actual tension data)
byte5    = 0x7F            (terminator)
```

Main ASM stores corresponding received data into `windlen*` / `realtens*` regions and recognizes `0x7F` as the frame end.

Original TC106 behavior is continuous Direction-B transmission. The later `tx_enable` gated implementation is treated as an experimental/test modification, not the formal original communication profile.

## Direction-A confirmed frame structure

- RESET: 12 bytes / 13 slots
- SEND: 4 bytes / 13 slots
- SENS.ADJ: 1 byte / 11 slots
- 7-bit LSB first
- approximately 3.3 ms per slot

## Display finding — do not regress

Dedicated display mode bits `disp_ten` / `disp_len` have unreachable setters and are treated as dead dedicated modes.

However, a separate **live** path exists: received wound-length change can set `ten_set`, call `Disp_set`, and display received wound-length data. Therefore the statement “the old Main never displayed received values” is incorrect.

In the discovered `Disp_set` path, `windlen2_0` is selected under the relevant state. `windlen2_*` is now understood to originate from the Main physical CH2 receive path. Why this value is selected by that UI/display path remains an ASM/UI-design question.

## Major completed milestones

- Step3C: CLOSED / FORMAL PASS
- Step4A: PASS
- Step4B: CLOSED / RUNTIME PASS
- Nano Every -> ESP Direction-B physical transmission: PASS
- Direction-B 6-byte decode: PASS
- Pi GUI actual tension / wound length / RX status: PASS
- Boot Magic: PASS
- RX timeout LOST -> resume OK: PASS
- SEND / RESET / ADJ coexistence in emulator environment: PASS

## Current investigation discipline

When answering a new TC106 question:

1. Read this file and the baseline/decision/unresolved files.
2. Inspect current Git source when exact filenames or current implementation matter.
3. Mark conclusions as confirmed, derived, hypothesis, or unresolved.
4. Record important corrections in `TC106_DECISIONS.md`.
