# TC106 Decisions and Corrections

**Purpose:** Preserve important conclusions and, especially, corrections so an AI or developer does not reintroduce superseded interpretations.

## 2026-09-09 — Direction-B topology correction

### Previous incorrect/insufficient interpretation

“Direction-B is one shared return line” was interpreted too broadly, leading to the possibility that Main CH1 and CH2 might not have independent physical receive paths.

### Evidence

- Main Controller PC-1348 circuit diagram shows separate `ch1糸長入力` and `ch2糸長入力` receive circuits/PIC paths.
- Main ASM has independent `rx_ch1` / `Data_110bps_1` and `rx_ch2` / `Data_110bps_2` receive processing.
- Interface-board material has one `TC_TX_REP` return net for the board/group and note `RX CN4のみ結線`.

### Corrected decision

The two statements describe different hierarchy levels:

- CH1 and CH2 are physically independent at the Main Controller.
- Multiple TC106 units **within each CH group** share that group's Direction-B return bus.

Therefore:

```text
CH1 group shared return -> Main CH1 -> Data_110bps_1 -> windlen1_*
CH2 group shared return -> Main CH2 -> Data_110bps_2 -> windlen2_*
```

Do not say “Direction-B is globally one line across CH1 and CH2.”

### Still unresolved

Whether the actual machine physically contains two identical interface-board instances (CH1 and CH2) has not yet been directly verified. Treat this as a strong topology hypothesis pending physical/documentary confirmation.

---

## 2026-09-09 — Received-value display correction

### Superseded interpretation

“The old Main receives and stores actual values but does not use them in normal display.”

### New source finding

The dedicated `disp_ten` / `disp_len` mode setters are unreachable, so those dedicated modes remain dead. However, received wound-length change handling can set `ten_set` and call `Disp_set`; a normal executable display path exists and selects received wound-length data including `windlen2_0` under the relevant state.

### Corrected decision

Use this wording:

> Dedicated actual-tension/wound-length display modes are dead code, but a separate live path exists that can reflect received wound-length data on the 7-segment display.

Do not return to the blanket statement that the old Main never displayed received values.

---

## 2026-09-03 — `tx_enable` classification

Historical `tx_enable` gating was added during communication/noise investigation. It is not currently supported as original TC106 protocol behavior by the primary source hierarchy.

Decision: formal target is **Original Continuous Direction-B**. Keep `tx_enable`-gated behavior classified as experimental/test history unless primary evidence changes this conclusion.

---

## Decision-recording rule

For future corrections include:

1. old interpretation,
2. new evidence,
3. corrected decision,
4. remaining uncertainty,
5. affected implementation/test documents.
