# TC106 Engineering Baseline

**Baseline date:** 2026-09-09  
**Status:** Working formal baseline; update only from source/measurement-backed findings.

## 1. System role

Replacement of the legacy Main Controller while preserving TC106 communication behavior. The replacement also exposes received wound length and actual-tension data for monitoring/logging.

## 2. Architecture

```text
Raspberry Pi GUI
      <-> ESP32-S3 bridge/controller
              <-> interface/level conversion
                      <-> TC106 system

Arduino Nano Every = emulator/test instrument, not production TC106.
```

## 3. Direction definitions

- Direction-A: Main -> TC106, control/settings commands
- Direction-B: TC106 -> Main, measured wound length / actual-tension data

Direction-B should not be described merely as a command response; Original TC106 continuously transmits it.

## 4. Confirmed communication baseline

### Direction-A

| Command | Bytes | Slots/byte |
|---|---:|---:|
| RESET | 12 | 13 |
| SEND | 4 | 13 |
| SENS.ADJ | 1 | 11 |

- 7-bit LSB first
- slot approximately 3.3 ms

### Direction-B

- 6 bytes/frame
- 17 slots/byte
- 7-bit LSB first
- payload = 3 wound-length bytes + 2 actual-tension bytes + `0x7F`
- Original TC106 transmission profile = continuous

Derived timing: 17 slots x ~3.3 ms x 6 bytes ~= 336.6 ms/frame if frames are back-to-back. This is **DERIVED** and should be checked on the real TC106 waveform.

## 5. Direction-B physical topology

**CONFIRMED at Main side:** CH1 and CH2 have independent physical receive inputs and PIC receive paths.

**CONFIRMED at one interface-board design:** TC106 return signals inside the relevant CH group use shared `TC_TX_REP`; interface documentation states `RX CN4のみ結線`.

Thus use the two-level model:

```text
CH1 TC106 group -> shared CH1 return -> Main CH1 receive path -> windlen1_*/realtens1_*
CH2 TC106 group -> shared CH2 return -> Main CH2 receive path -> windlen2_*/realtens2_*
```

**UNRESOLVED:** physical verification that the machine contains separate copies/instances of the interface board for CH1 and CH2.

## 6. Received-value display baseline

- Dedicated `disp_ten` and `disp_len` display modes: dead/unreachable in the inspected Main ASM.
- Separate live display behavior exists through `ten_set` and `Disp_set` after received wound-length change detection.
- `windlen2_0` appears in the live display selection path.
- `windlen2_*` belongs to the Main CH2 receive domain; exact machine winding-position identity still requires physical/topology confirmation.

## 7. Original vs test-gated TC106 TX

A historical `tx_enable` modification guarded `_T1Interrupt()` TX, enabled transmission before `rxsend()`-type paths, and disabled TX after the 6-byte frame. Treat this as a test/noise-isolation modification unless new primary evidence proves otherwise.

Formal reproduction target: **Original Continuous** TC106 behavior.

## 8. Passed test baseline

- Step3C CLOSED / FORMAL PASS
- Step4A PASS
- Step4B CLOSED / RUNTIME PASS
- Direction-B physical transfer and decode PASS
- 6-byte terminator handling PASS
- Pi GUI received-value display PASS
- Boot Magic PASS
- RX LOST/recovery PASS
- Direction-A SEND/RESET/ADJ coexistence in Nano emulator environment PASS

## 9. Real-TC106 confirmation priorities

1. Confirm interface-board count/assignment and CH1/CH2 wiring on the real machine.
2. Capture continuous Direction-B frames and actual frame interval.
3. Confirm wound length immediately after RESET and during winding.
4. Confirm actual-tension value tracks physical tension changes; do not equate ADC-derived value directly to gf without calibration evidence.
5. Confirm Direction-B behavior while SEND/RESET/SENS.ADJ occur.
6. Observe old Main 7-seg received wound-length behavior and correlate it with CH1/CH2 physical paths.

## 10. Change rule

If a new finding contradicts this baseline, do not silently overwrite history. Add the correction and evidence to `TC106_DECISIONS.md`, then update this baseline.
