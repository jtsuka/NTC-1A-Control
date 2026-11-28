# ESP32 firmware version report — specification DRAFT

Status: **DRAFT, nothing implemented, nothing tested** (2026-11-28). Part of
`docs/TC106_UNRESOLVED.md` D7. A protocol change: do **not** implement before this draft is
reviewed and the regression plan (section 6) is agreed.

Labels: CONFIRMED / DERIVED / HYPOTHESIS / UNRESOLVED as in `AGENTS.md`.

## 1. Goal

The operator wants to see, on the Pi GUI and in the evidence log, **which ESP32 firmware is
running**, in the same style as the Pi side (`git describe` build id, D7 Pi side done in v3.9.3).
The ESP32 has no Git, so the identity must be embedded at build time and reported to the Pi.

## 2. Constraints from the existing code (CONFIRMED by reading the files)

- ESP32 → Pi already sends a 6-byte **Boot Magic** `DE AD BE EF 01 7F` once at start-up
  (`ESP_BOOT_MAGIC` in the production sketch; recognised by `NTC_1A_serial_comm_v3_6_beta.py`,
  event `rx_boot`).
- Direction-B telemetry is 6 bytes, footer `0x7F`; the Pi accepts it only if bytes 0–2 are within
  the length-group maximum and the tension bytes are within range (`_is_direction_b_telemetry`).
  A first byte of `0xDE` can therefore **never** be mistaken for telemetry (DERIVED from the code).
- The Pi RX loop also keeps legacy 12- and 8-byte `checksum7` frames (`rx_legacy`). Bytes that
  match none of the rules are dropped one at a time once 12 bytes are buffered (`rx_drop`).
- The Pi → ESP32 parser reads `buf[0]` as the command; command codes in use: 0x01 RESET, 0x02 SEND,
  0x03 SENS.ADJ (+ SAFE and evidence commands). Adding a command touches that parser.

## 3. Options

**A (recommended for review): unsolicited VERSION frame right after the Boot Magic.**
12 bytes, shaped like a legacy `checksum7` frame:

| byte | meaning |
|---|---|
| 0–3 | `DE AD BE EF` (same prefix as Boot Magic) |
| 4 | `02` = VERSION frame (Boot Magic uses `01`) |
| 5, 6, 7 | firmware major, minor, patch (0–127 each) |
| 8 | flags: bit0 = built from a dirty tree, other bits 0 |
| 9, 10 | commits since the firmware tag, 14 bit (two 7-bit bytes, high first) |
| 11 | `checksum7` of bytes 0–10 (sum & 0x7F) |

Why this shape (DERIVED): an **old Pi** (v3.6–v3.9.x) reading it finds a first6 that is neither Boot
Magic nor telemetry, then matches the 12-byte `checksum7` rule and logs it as `rx_legacy` — no
`rx_drop`, no telemetry effect. A **new Pi** recognises the prefix `DE AD BE EF 02` first.
Sent once per boot; no periodic traffic, so Direction-B timing (about 336 ms per frame) is untouched.
Open: the Git hash is **not** on the wire (only tag + commit count). The full hash would be printed
on the ESP32's USB debug port at boot (UNRESOLVED whether that is enough).

**B: Pi asks for it** with a new command (for example `0x05`, reply = the frame above). Cleaner
(an old Pi never asks, nothing unsolicited) but changes the Pi → ESP32 parser and needs a reply
path; larger regression. Keep as the fallback if A fails the regression.

**C: Boot Magic with a different last byte** — rejected: carries only one byte and would break the
exact-match Boot Magic check in every existing Pi version.

## 4. Build-time identity on the ESP32

- Source of truth: Git. Firmware tag naming: `esp32-vX.Y.Z` (the Pi side uses `gui-vX.Y.Z-beta`).
- Generate `git describe --tags --match "esp32-v*" --dirty` into compile-time defines
  (`FW_MAJOR`, `FW_MINOR`, `FW_PATCH`, `FW_COUNT`, `FW_DIRTY`) with a small script
  (`tools/gen_esp32_version.*`, not yet written).
- Arduino IDE has no pre-build hook. HYPOTHESIS (not checked on the operator's core version): the
  ESP32 Arduino core reads a `build_opt.h` file in the sketch folder for extra compiler flags, so the
  script can write `-DFW_...` there. Alternative: `arduino-cli --build-property`.
- If the script was not run, the defines default to `0.0.0`, count 0, dirty = 1, so an un-stamped
  build is visibly wrong rather than silently "clean".

## 5. Pi side (after the protocol is approved)

- Parser: new hook kind `rx_version`; log event `ESP_VER` (CSV) with the decoded text; GUI shows
  `ESP32 vX.Y.Z+N` (and `dirty`) next to the Pi build id; "ESP32 version unknown" when no frame was
  seen (old firmware or the frame was missed).
- The Pi must not block or change state when the frame is absent.

## 6. Regression plan (must pass before merging any implementation)

1. Old Pi GUI (v3.9.2) + new ESP32: frame logged as `rx_legacy`, **no** `rx_drop`, telemetry and
   `RX STATUS` unchanged, Boot Magic still detected.
2. New Pi + old ESP32 (no VERSION frame): GUI shows "unknown", everything else unchanged.
3. New Pi + new ESP32: version shown; unchanged Direction-B decode (`17 05 01 07 03 7F` → length
   10523, tension 37 on the Nano Every setup); `0x7F` footer handling; RX timeout → LOST → resume → OK.
4. Nano Every emulator: SEND / RESET / ADJ coexistence still passes.
5. Real TC106 on the bench: Direction-B about 336 ms cadence not disturbed (logic analyzer, as in
   the Formal Gate method).
6. ESP32 reset repeated several times: exactly one VERSION frame per boot, always after Boot Magic.

## 7. Out of scope / not decided

- Which firmware tag to use as the first stamp (UNRESOLVED).
- Consolidating the many version-named ESP32 sketch folders (separate item of D7).
- Reporting the **TC106** PIC firmware version (not possible from the Pi; the writer is on the
  bench side only).
