# TC106 Current System Status

**Updated:** 2026-11-28 — **only the Pi rows of §1 and the new §7**. Sections 1 (other rows) to 6 are still as of 2026-09-29 and were not re-verified.
**Repo HEAD reviewed:** `f230fbb5a6f5640c7a55361812862e6c439b4811` (2026-09-29) for §§1 (non-Pi rows)-6; `9085370` (`main`, 2026-11-26) plus branch `claude/gui-v3-9-wifi-setup` (2026-11-28) for the Pi rows and §7.
**Purpose:** The single current-state summary for ChatGPT, Claude, and human developers.
Read this file first, in every new thread, before reading anything else in `docs/`.
Long history/rationale lives in `docs/AI_PROJECT_CONTEXT.md`, `docs/TC106_BASELINE.md`,
`docs/TC106_DECISIONS.md`, `docs/TC106_UNRESOLVED.md`. This file states only what is
current; it does not explain *why*.

**Rule for all three parties (Amano-san, ChatGPT, Claude):** confirm in this order —
GitHub current HEAD → KiCad/netlist → firmware source → physical hardware. This order
mixes two different kinds of truth; keep them separate (full statement in §6):

- **Design source of truth = GitHub.** Which net a TP sits on, what a byte sequence
  should look like, which file is current — check GitHub HEAD first. Chat memory is
  never the final answer for these; fix this file (or GitHub) instead of trusting the
  chat.
- **Physical fact = real-hardware measurement** (DMM / continuity / LA capture / board
  photo). Whether the board in hand actually matches that design — GitHub cannot answer
  this by itself.
- **When GitHub's design and a real measurement disagree, do not silently prefer
  either.** Record both, and treat the gap as a finding to investigate (board revision
  mismatch, assembly difference, wiring error) — see the TP8 entry in §2 for a live
  example.
- Chat memory is not the final authority for either kind of truth.

---

## 1. Current firmware / software (by SHA256, verified against GitHub HEAD above)

| Role | File | SHA256 (first 16) |
|---|---|---|
| ESP32-S3 production build | `ESP32/ESP32_TC_Bridge_RTOS_Final_Silent_BootMagic_20260907/ESP32_TC_Bridge_RTOS_Final_Silent_BootMagic_20260907.ino` | `449db4634114…` |
| ESP32-S3 Evidence V3 (diagnostic-only; not production) | `ESP32/TCBridge_EvidenceV3/TCBridge_EvidenceV3.ino` | `17a5bcb94aba…` |
| Nano Every — Direction-B source (zero data) | `ArduinoNanoEvery/NanoEvery_TCEmulator_V2_8_Original250201/NanoEvery_TCEmulator_V2_8_Original250201.ino` | `44e94e2c7da4…` |
| Nano Every — Direction-B source (known non-zero: len=10523, tension=37) | `ArduinoNanoEvery/NanoEvery_TCEmulator_V2_8_Step4D_NonZeroE2E_20260901/NanoEvery_TCEmulator_V2_8_Step4D_NonZeroE2E_20260901.ino` | `73cea68abeaa…` |
| Pi communication module — formal/production reference | `Python/NTC_1A_serial_comm_v1_3_4.py` | `c87335a60a40…` |
| Pi GUI — formal/production reference | `Python/ntc_pygame_ui_v2_4.py` | `3d08b3c8443e…` |
| Pi communication module — v3.6 beta (current beta line) | `Python/NTC_1A_serial_comm_v3_6_beta.py` | `2da777bced35…` |
| Pi RX/TX evidence logger — v3.6 beta | `Python/NTC_1A_rxlog_v3_6_beta.py` | `6a56f879c78b…` |
| Pi GUI — v3.7 beta (what the bench Pi starts; see §7) | `Python/ntc_pygame_ui_v3_7_beta.py` | `66a05427623d…` |
| Pi GUI — v3.8 beta / v3.9 beta (on branch `claude/gui-v3-9-wifi-setup` until merged) | `Python/ntc_pygame_ui_v3_8_beta.py` / `Python/ntc_pygame_ui_v3_9_beta.py` | `6fe255d2c88f…` / `abe1e9dd17a4…` |

*The v3.x Pi files are BETA / not production; the v2.4 / v1.3.4 rows stay the formal reference until a promotion is decided (§7.5). Full Pi GUI history: §7.*
| Pi Evidence V3 logger | `Python/pi_logger_evidence_v3.py` | `387f8472f40c…` |
| LA analyzer (independent, Formal Gate) | `Python/TC106_LA_Analyzer_v1.1/tc106_la_analyzer.py` v1.1.0 | `fec62c08f841…` (full hash: `…6a294b1b1534aea68510267c557b3`) |

Main Controller / TC106 origin references (byte-for-byte protocol basis):
- `TC-FW/zhukongban.asm` — sha256 `f3a2e91391…`
- `TC-FW/FW_TC106-1_v0.0_250201-UTF8.c` — sha256 `408f040ec0…` (legacy analysis reference; GBK despite filename — see note below)
- `TC-FW/History/FW_TC106-1_v0.0_250201.c` — historical primary source (unmodified)

**⚠ Corrected 2026-09-28 (byte-level check, not just blob SHA/size):** a second file,
`TC-FW/FW_TC106-1_v0.0_250201_UTF8_FROM_ORIGINAL.c`, exists in the repo.
`FW_TC106-1_v0.0_250201-UTF8.c` and `History/FW_TC106-1_v0.0_250201.c` are byte-identical
once CRLF is normalized to LF (both are the **original GBK — Simplified Chinese —
byte stream**, despite the `-UTF8` filename). `FROM_ORIGINAL.c` is a **proper UTF-8
re-encoding of that same GBK original** (confirmed: decoding the other two as GBK, not
Shift-JIS, yields readable Chinese comments matching `FROM_ORIGINAL.c`, e.g. "设定的长度").
So: **the C logic is the same file lineage, not two diverged originals** — but the file
currently used as the "legacy analysis reference" (`-UTF8.c`) has **mojibake'd
comments** (GBK bytes misread as anything else), while `FROM_ORIGINAL.c` is actually
readable. A line-level, comments-stripped code diff after correct decoding still shows a
small residual difference (cause not yet identified — possibly whitespace from the
conversion tool); this does not affect the mojibake finding above but means the two are
not proven byte-for-byte identical in the code body either. **Recommendation: use
`FROM_ORIGINAL.c` when reading comments for meaning; treat `-UTF8.c` as legacy until the
residual code diff is explained.**

**⚠ Repo hygiene note (updated 2026-09-29):** the former top-level
`ESP32_TC_Bridge_RTOS_Final_Silent_BootMagic_20260907_PollEviden/` folder has been
removed, so the earlier "ESP32/ outside" issue is resolved. Two similarly named diagnostic
folders now exist under `ESP32/`: `TCBridge_PollEvidence120s/` and
`ESP32_TC_Bridge_RTOS_Final_Silent_BootMagic_PollEvidence120s/`. Their INO files are
different; do **not** assume they are duplicates.
`TCBridge_PollEvidence120s/TCBridge_PollEvidence120s.ino` (sha256 `b597b1c1…`) has been
identified as the **V1 Evidence diagnostic build analyzed on 2026-09-27**. That run showed
an approximately 76.1 s Pi-telemetry gap; source review found its blocking-capable USB CDC
summary print in the same Core1 task loop as the SerialPi drain, so the measurement was
classified INVALID due to observer-effect risk. It is superseded by Evidence V2/V3, which
avoid USB result retrieval on the timing-critical DUT path. The sibling folder's provenance
is still unconfirmed.

**⚠ Formal Gate spec docs are not yet in the repo.** `A_TC106_FormalGate_TestSpec_Template_v1.3`
and `B_TC106_EvidenceV3_Rerun_LA_TestSpec_v1.3` currently exist only as zip attachments
shared in chat with Claude/ChatGPT, not as committed files. Per the GitHub-is-source-of-
-truth agreement (§6), these should be committed (suggested path: `TC106-Doc/FormalGate/`)
so a new thread can find them without relying on chat history.

---

## 2. Hardware: JP settings, TP list, D0/D1 connection

Board: XIAO ESP32-S3 sniffer/bridge board, silk `TC106 Snidar ESP32S3 Ver 1.0 2026.03.15`
("基板#2", the physical unit currently in use for bench testing).

### Test points (source: `TC106-Doc/ESP32S3_Snifar.net`, KiCad netlist — re-verified 2026-09-28)

| TP | Net | Notes |
|---|---|---|
| TP1 | near J1 (RX_SIG) | |
| TP2 | `/SIG_DIV` | 15V-side divider (R1/R3/R5/C3). **No signal observed under current JP4/Nano wiring** (bench fact, 2026-09-27) |
| TP3 | `/TC_MOS_DRAIN` | Q2 drain. **Main→TC TX bus, bus-side polarity** (idle HIGH / START LOW). Used as the TX channel in the real-TX correlation preflight |
| TP4 | `GND` | **Corrected 2026-09-28.** Previously misrecorded as "MOS_GATE" in this project's memory — that was a mix-up with the unrelated `/MOS_GATE` net (Q2 gate, a different net, not connected to TP4). TP4 is plain ground |
| TP5 | `/TC_MCU_RX` | Final Direction-B receive line into ESP32 GPIO4, after JP4 mode select. Confirmed active in every LA capture so far |
| TP6 | (design) `/Pi_Rx_MCU`, on U1 pin D0 = `PIN_PI_TX` (`ESP32 TX -> Pi_Rx_MCU`) | **Corrected 2026-09-28 (was backwards).** Design says ESP→Pi direction: telemetry / Evidence RESULT. **Physical fact not yet checked** — no continuity/voltage measurement on real hardware yet |
| TP7 | (design) `/Pi_Tx_MCU`, on U1 pin D1 = `PIN_PI_RX` (`Pi_Tx_MCU -> ESP32 RX`) | **Corrected 2026-09-28 (was backwards).** Design says Pi→ESP direction: EVIDENCE_START / SEND / RESET / SENS.ADJ. **Physical fact not yet checked** — no continuity/voltage measurement on real hardware yet |
| TP8 | (design) `/Nano_MOS_DRAIN`, in `KiCad/ESP32S3_Snifar/ESP32S3_Snifar_3.0.net` | Design fact only, not yet a physical fact. Present in the newer (Ver2.0/3.0-line) KiCad netlist; **absent from `TC106-Doc/ESP32S3_Snifar.net`**, the netlist matching 基板#2 (Ver1.0, silk 2026.03.15) currently on the bench. Consistent with "TP8 is a Ver2.0+ addition, not on 基板#2" — but this is a *design* comparison between two netlist files, not a look at the real board. **Physical confirmation still needed: full board top-down photo + TP8 close-up have not been provided yet.** Do not treat either netlist as settling whether TP8 exists on the physical board in hand |

### JLCPCB order-image evidence for board #1 (2026-09-29)

- Reference image: `TC106-Doc/Hardware/Board_No1_JLCPCB_Order_Ver1_0_20260315.jpg`.
- The operator identifies this as the **JLCPCB order image for board #1**.
- In the order render, the silkscreen shows **TP1 through TP7; TP8 is not present**.
- The same render also shows `TC106 Snidar ESP32S3 Ver 1.0 2026.03.15`, matching the
  recorded revision/date string for the current bench board (#2). This is strong evidence
  that board #1 and board #2 are from the **same design revision**; it does **not** by itself
  prove that they were fabricated in the same manufacturing batch.
- This is consistent with the older `TC106-Doc/ESP32S3_Snifar.net`, which has no TP8.
- This image is evidence of the **fabrication/order design for board #1**, not a physical photograph or measurement of the current bench board (#2). Do not silently equate board #1 and board #2.
- On a board without TP8, the equivalent `/Nano_MOS_DRAIN` signal has no dedicated TP8 access point. **The exact physical clamp point on the real board is currently unconfirmed (TBD).**

### JP3 (bench-confirmed, takes priority over schematic pin-number reading)

- 1-2 shorted = Nano Every mode
- 1-3 shorted = real-TC106-bus mode
- Physical pad correspondence confirmed on the real board; footprint-rotation mismatch
  between silk numbering and schematic symbol was the reason bench confirmation was
  needed. (This detail and other bench-confirmed hardware facts are tracked in Claude's
  private memory file `hardware-reference.md`, which is **not part of this GitHub repo**
  — it will not be found by searching the tree. If this file's content needs to survive
  outside a Claude conversation, it should be copied into this repo, e.g. under
  `TC106-Doc/`.)

### Current bench wiring (as of the 2026-09-28 real-TX correlation preflight)

- **D0 → TP3** (Main→TC TX / Direction-A)
- **D1 → TP5** (TC→ESP Direction-B RX)
- TP6/TP7 not yet added to the capture. Planned next: 4-channel capture, see §5.

---

## 3. CLOSED gates

| Gate | Status | Scope / caveat |
|---|---|---|
| TC106 communication protocol origin cross-check (RESET/SEND/SENS.ADJ/Direction-B byte layout) | **CLOSED / FORMAL PASS** (2026-09-07) | Confirmed against `zhukongban.asm` and `FW_TC106-1_v0.0_250201-UTF8.c` directly |
| ESP32 final integration (Stage5 + Step4D merge) | **CLOSED** (v0.3 GO, 2026-09-07) | Code-reviewed, diffed line-by-line against both source branches; TcMainTx untouched |
| Second Poll (RX/TX coexistence, TcMainTx.poll() timing under Direction-B load) | **CLOSED**, with recorded scope limit | Nano emulator only, each of SEND/RESET/SENS.ADJ sent once, all data values zero. `tc_main_tx_overrun_count` delta = 0. NOT re-opened by the 9/28 Evidence timeout (see §4) |

---

## 4. Currently in progress

**Real-TX correlation preflight (Formal Gate readiness for the LA Analyzer)**

- Analyzer v1.1.0 independently reviewed (Claude + ChatGPT, both APPROVE) and frozen at
  the hash in §1. Status: **REVIEW CANDIDATE, not yet RUN_READY.**
- 2026-09-28 real `.sr` capture (D0=TP3, D1=TP5) analyzed by both parties: **TX waveform
  matches the `TcMainTx` model exactly** (SEND/RESET/SENS.ADJ byte-for-byte, marker
  polarity, slot timing; LA-vs-ESP clock offset ≈+46–52 ppm, residual after removal
  ≤2.3 µs). Direction-B continuity confirmed (593 frames, all zero payload, no sync
  misses). **No reason found to revise the Analyzer.**
- **Evidence RESULT timeout on the same run: UNRESOLVED.** Confirmed this was NOT simply
  "ESP not reset between one-shot runs" — the operator re-flashed `TCBridge_EvidenceV3.ino`
  immediately before this attempt, so a fresh reset is on record. The real cause of the
  timeout is still open. Does not reopen the Second Poll CLOSED gate (§3); does block
  Formal Gate promotion until understood or reliably reproduced/ruled out.

**Formal Gate promotion checklist (from `B_TC106_EvidenceV3_Rerun_LA_TestSpec_v1.3`, not yet committed — see §1 open item):**
- [x] Analyzer independently reviewed and hash-frozen
- [x] Real TX waveform matches the model (this preflight)
- [ ] Evidence RESULT timeout cause understood, or reliably ruled out as a hardware/wiring issue
- [x] TX observation point confirmed as TP3 (bus-side), 2026-09-28 real capture: netlist
      (`/TC_MOS_DRAIN`), bench voltage (4.74V idle), and waveform-vs-model match all agree.
      Note: TP4 is GND, not the gate; the gate-side alternative, if ever needed, is the
      `/MOS_GATE` net, which has no TP assigned today. No `--tx-invert` needed at TP3
- [ ] TP6/TP7 wiring and voltage (3.3V expected) confirmed on real hardware
- [ ] Offset model (`RAW` vs `WIRE_CORRECTED`) declared with evidence, ideally from a
      capture that includes TP7 (Pi→ESP) so the Pi-sent command byte is observed directly
      on the same LA time axis as the resulting TX frame on TP3
- [ ] New spec SHA recorded in §6.1 of the (not-yet-committed) spec B

---

## 5. Next single action

**Confirm TP6 and TP7 physically and electrically before adding them to any LA capture.**

1. Photo: full board top-down (silk TP1–TP8 legible)
2. Photo: close-up of TP6/TP7 area
3. Power off: continuity check **TP6 ↔ XIAO D0 pad**, **TP7 ↔ XIAO D1 pad**
   (per netlist: TP6 is on `/Pi_Rx_MCU`, U1 pin D0; TP7 is on `/Pi_Tx_MCU`, U1 pin D1 —
   double-checked against the firmware's own `PIN_PI_RX=D1`/`PIN_PI_TX=D0` comments)
4. Power on: DMM idle voltage at TP6 and TP7 — expect ≈3.3V each. **If either reads near
   5V, stop and do not connect the LA before resolving why.**
5. Only after 1–4: move to a **4-channel capture** for the next timeout investigation —
   `D0=TP3` (Main→TC TX), `D1=TP5` (Direction-B), `D2=TP7` (Pi→ESP: START/SEND/RESET/
   SENS.ADJ), `D3=TP6` (ESP→Pi: telemetry/Evidence RESULT). One 200 s capture on this
   layout can distinguish, in a single run, whether EVIDENCE_START reached the ESP,
   whether a RESULT was ever transmitted, and whether it reached the Pi — narrowing the
   §4 timeout considerably. The analyzer needs no code change for this (`info` already
   reads any named channel); a dedicated 4-channel correlation script is a small addition
   to write when this capture is in hand, not yet written.

Do not skip to wiring changes from memory. If in doubt about a pin, re-open
`TC106-Doc/ESP32S3_Snifar.net` from the current GitHub HEAD first.

---

## 6. Roles (agreed 2026-09-28)

- **GitHub (`jtsuka/NTC-1A-Control`)** = source of truth **for design intent**: what the
  circuit, firmware, and test tooling are *supposed to be*. C/ASM originals, ESP/Nano/Pi
  firmware, KiCad, the LA Analyzer, test specs.
- **Physical hardware (DMM / continuity / LA capture / board photo)** = source of truth
  **for present fact**: what the board in hand *actually is and does right now*. GitHub
  cannot confirm this by itself — a netlist says what was designed, not that this exact
  board was fabricated, assembled, or wired to match it.
- **These two are not interchangeable, and neither alone is "GitHub wins":**
  - Design questions (which net a TP sits on, what a byte sequence should look like, which
    file is current) → check GitHub first; chat memory only if GitHub doesn't answer it.
  - Physical questions (is TP8 present on 基板#2, is TP6 really ≈3.3V, does the real TX
    waveform match the model) → GitHub cannot answer these; measure the real board.
  - When physical measurement disagrees with what GitHub's design says, **record both**:
    the design fact (from GitHub) and the measured fact (from the bench), and treat the
    gap itself as a finding — don't silently prefer one.
- **ChatGPT** = checks GitHub each time; drives analysis and hardware test execution.
- **Claude** = reads the same GitHub; independent cross-review.
- **Chat history** = background and rationale ("why we did it this way"). Never the
  final authority on current wiring or which file is current — GitHub is, for design;
  the bench is, for physical fact. This file records both, dated, so a new thread does
  not have to re-derive either.

---

## 7. Pi GUI and Pi unit — current state (added 2026-11-28)

**Scope and limits of this section.** Written from the 2026-11-23 … 2026-11-28 work
sessions. It was added because §1 still listed GUI v2.4 as the only Pi GUI while the
tree already holds v2.6 and v3.6–v3.9 beta. Only this section and the Pi rows of §1 were
updated on 2026-11-28; every other section is still as of 2026-09-29 and was **not**
re-verified. Items are tagged CONFIRMED / DERIVED / HYPOTHESIS / UNRESOLVED. Pi-unit
facts (7.3) are bench facts about one device, not design facts. Nothing here claims a
real-machine (TC106 line) result.

### 7.1 Pi GUI version line (CONFIRMED from the Git tree)

| Version | File (under `Python/`) | SHA256 (first 16) | Notes |
|---|---|---|---|
| v2.4 | `ntc_pygame_ui_v2_4.py` | `3d08b3c8443e3430` | Formal/production reference (§1). Not replaced |
| v2.5 beta | `ntc_pygame_ui_v2_5_beta.py` | `2ec31aa112887b52` (value recorded in the v3.6 design doc) | 0..1023 tension display |
| v2.6 | `ntc_pygame_ui_v2_6.py` | `e4b199fe8064d3ed` | 2026-11-04. Adds startup serial-port auto-selection: first USB tty (`/dev/ttyACM*`, then `/dev/ttyUSB*`), `--port` wins, else `/dev/serial0` |
| v3.6 beta (top level) | `ntc_pygame_ui_v3_6_beta.py` | `cf23e596145e70bd` | **BASE = v2.6** (keeps the USB auto-selection). The v3.7+ files below are based on this one |
| v3.6 beta (archive copy) | `ntc_pi_gui_v3_6_beta_all_20261123/ntc_pygame_ui_v3_6_beta.py` | `a472268dd57db880` | **BASE = v2.5 beta.** This is the hash the v3.6 design document records. **Different file from the top-level v3.6 (115 differing lines, header/base only seen so far)** — see 7.5 |
| v3.7 beta | `ntc_pygame_ui_v3_7_beta.py` | `66a05427623de68d` | + REBOOT button with confirmation dialog; SIGTERM/SIGHUP end the GUI like EXIT (log stopped, flushed, `os.sync()`) |
| v3.8 beta | `ntc_pygame_ui_v3_8_beta.py` | `6fe255d2c88f9453` | + WIFI SCAN: live Wi-Fi link readout (read-only `iw`) |
| v3.9 beta | `ntc_pygame_ui_v3_9_beta.py` | `abe1e9dd17a4bc2c` | + WIFI SETUP: SSID list + on-screen ANSI keyboard, saves/joins a network with `nmcli`; EXIT / REBOOT / WIFI SCAN / WIFI SETUP evenly spaced (pitch 0.13 of screen height) |

Supporting modules (unchanged since v3.6): `NTC_1A_serial_comm_v3_6_beta.py`
`2da777bced353889`, `NTC_1A_rxlog_v3_6_beta.py` `6a56f879c78b5043`,
`NTC_1A_utils.py` `3ee33c5bb5a5ac43`. The two v3.6 modules are byte-identical to the copies
in `ntc_pi_gui_v3_6_beta_all_20261123/`. The ESP32 production firmware is unchanged by any of this.

v3.8 / v3.9 were first committed on branch `claude/gui-v3-9-wifi-setup`
(commit `6e0f508`, 2026-11-28) and are on `main` only after that branch is merged.

**None of v3.6 … v3.9 is production.** All are BETA. The v3.6 design document
(`Python/ntc_pi_gui_v3_6_beta_all_20261123/TC106_Pi_GUI_v3_6_Beta_RxLog_20261123.md`) keeps
its 2026-11-23 status line ("not tested on a real unit"); that line is historical.

### 7.2 What the GUI shows and does (CONFIRMED: code + bench photo 2026-11-26)

- Fields: SET TENSION, SET LENGTH, SET COUNT (editable); **TENS ADC**, WIND LENGTH,
  RX STATUS (read-only). Keypad: 0-9, CLR, ENT, SEND, ADJ, RESET; SAFE ON/OFF; port
  selector; RUN / STOP / MARK note buttons; `LOG n` indicator.
- **TENS ADC** is the TC106's 16-sample ADC average, 10-bit count 0..1023, not zero-
  corrected, **not a force in gf**. The old label "ACTUAL TENSION" was retired on 2026-11-23.
  Do not add a unit before calibration evidence exists (see `docs/TC106_UNRESOLVED.md` B3).
- RX STATUS: WAIT / OK / LOST; LOST after 2.0 s without a valid Direction-B frame
  (`TELEMETRY_LOST_TIMEOUT_SEC`, serial module).
- Evidence log (v3.6+): `~/ntc_logs/rxlog_*.csv` (raw bytes, Direction-B frames, status
  changes, TX, operator notes). Details: the v3.6 design document.
- Operator-support features added after v3.6, none of which touches the Direction-A/B
  control or telemetry path:
  - **REBOOT** (v3.7): opens a 10 s confirmation dialog; on accept runs
    `systemctl reboot --no-ask-password`, else `sudo -n systemctl reboot ...`; refusal is
    shown on screen. Affects only the Pi; the ESP32 and TC106 keep running.
  - **WIFI SCAN** (v3.8): once per second runs `iw dev wlan0 link` and `station dump`
    in a background thread, only while switched on. Shows signal (dBm) with a rough
    GOOD / FAIR / WEAK / BAD label (>= -60 / -70 / -80 dBm; a project guide, not a
    standard), SSID, frequency, rx/tx bitrate, min..max, and tx-failed since start.
  - **WIFI SETUP** (v3.9): full-screen page. SSID list from `nmcli` (2.4 GHz only,
    strongest first) or typed by hand; password on an on-screen ANSI keyboard (masked,
    SHOW toggle). SAVE stores a NetworkManager profile only; CONNECT NOW stores and
    switches (the current connection and any ssh session drop). `nmcli` is retried once
    with `sudo -n` if refused for privilege. The password is passed to `nmcli` as an
    argument, never logged, cleared on close; the page closes itself after 180 s idle.
    Limits: ASCII only, 2.4 GHz only, WPA/WPA2/WPA3-personal and open networks only.

### 7.3 Bench Pi unit (CONFIRMED from Pi shell output supplied by the operator)

- Raspberry Pi Zero 2 W with a 7-inch touch display (see
  `TC106-Doc/RaspberryPiZero_7inch_WAVESHARE-24693_Confirmed_20261024.md`). ESP32 on
  USB serial (`/dev/ttyUSB0` in the 2026-11-26 photo).
- Repo clone: `/home/ntc/NTC-1A-Control` (updated with `git pull`). GUI launcher:
  `/home/ntc/start_ntc_gui.sh` (**Pi-local, not in Git**): `cd` to the repo's `Python/`
  folder and `exec` the venv Python `/home/ntc/kivy-venv/bin/python3` on `APP=...`; log
  `/home/ntc/ntc_gui.log`. On 2026-11-25 `APP` was changed from v3.6 to
  `ntc_pygame_ui_v3_7_beta.py` (backups `~/start_ntc_gui.sh.bak-20261123` / `.bak-20261125`).
  The tty1 auto-login → `~/.bash_profile` → launcher chain was set up by the operator and
  is **not re-verified here**. To move to v3.9: pull, then change `APP` and reboot.
- Network manager is NetworkManager. `iw dev wlan0 link` / `station dump` work as the
  normal user; `iw ... scan` needed `sudo`.
- **Wi-Fi measurements (2.4 GHz, ch 9):** at the first location -77 dBm, rx 2.0 Mbit/s,
  `tx failed` 5099 (2026-11-26 20:05). After moving the Pi next to the router: -41 dBm,
  72.2 Mbit/s, `tx failed` 0 (2026-11-26 21:32). From the 2026-11-27 04:40 boot to 21:00 the
  journal shows no disconnect/deauth/under-voltage; `get_throttled=0x0`. Reading: weak signal
  was the main cause of the earlier drops — **DERIVED**; needs a longer observation, and
  the period before 2026-11-27 04:40 left no log (the journal was volatile).
- **journald:** `/etc/systemd/journald.conf` had `Storage=volatile`. Changed to
  `Storage=persistent` on 2026-11-27, with `SystemMaxUse=50M` in
  `/etc/systemd/journald.conf.d/size.conf`; backup `journald.conf.bak-20261127`. Logs
  now survive a reboot, from that change onward.
- The Wi-Fi network name, addresses and passwords are deliberately **not** recorded in
  this repo.

### 7.4 Test status (be exact about what was and was not run)

- v3.8 / v3.9 were tested **headless only** (SDL dummy driver, faked `iw` / `nmcli`,
  screen sizes 1024x600 and 800x480): layout without overlap, list parsing (2.4 GHz only,
  duplicates merged, `:` in SSID, 802.1X/WEP refused), keyboard typing (no double typing
  on a touch+mouse event pair), short-password refusal, SAVE / CONNECT NOW command
  sequences, `sudo -n` fallback, idle close, password absent from stdout and from error
  text. **No real-Pi run of v3.8 or v3.9 has been done.**
- v3.7 REBOOT: the button and the confirmation dialog were seen on the real display
  (photos 2026-11-26). Whether REBOOT → REBOOT completes a reboot and leaves
  `NOTE,REBOOT` then `STOP` in the CSV has **not** been confirmed.
- The v3.6 plan's Stage B / C results are not recorded in the repo.

### 7.5 Open items for the Pi side (also in `docs/TC106_UNRESOLVED.md`, Priority D)

1. Two different v3.6 GUI files (7.1). Which one the 2026-11-23 field tests used is not recorded.
2. The v3.6 plan said all Pi modules bump together; v3.7-v3.9 bumped the GUI only.
3. `ntc` may or may not be allowed to run `systemctl reboot` / `nmcli` (directly or via
   `sudo -n`). Real-Pi test of REBOOT, WIFI SCAN, WIFI SETUP is pending.
4. `Python/tests/test_rxlog_v3_6_beta_20261123.py` (named in the v3.6 design doc) is not
   in the tree; no tag `pi-gui-v3.6-beta` exists. The launcher script is not in Git.
5. Pi reboots when the ESP32 is powered (reported during the 2026-11-23 … 25 field
   tests; cause not found). Field-test record is in Notion, outside Git.
6. Promotion of any v3.x beta to production is not decided; §1 keeps v2.4 / v1.3.4 as the
   formal reference.
