# TC106 Direction-B tension range — BETA revisions (2026-10-24)

**Status:** BETA / NOT PRODUCTION / hardware Stage C not executed.  
**Baseline GitHub main:** `33c10ec76a2687c8ba5cbbe9b4b512f1a60356ab` before adding these files.  
**Source of truth:** TC106 original C, real TC106 2026-10-23 logic-analyzer captures, Claude independent review (APPROVE WITH CHANGES).

## Purpose

Real TC106 Direction-B encodes a 10-bit averaged ADC value `tens_actual` (0..1023) in two fields:
`tens[0] = tens_actual % 10` and `tens[1] = tens_actual / 10`.
For `tens_actual=1023`, the 6-byte example at LEN=0 is
`00 00 00 03 66 7F`. The original Pi v1.3.4 and four evidence-loggers
reject this valid frame because they apply the limit 99 to every payload byte.

## Newly added files (originals remain untouched)

| Existing source (preserved) | New beta file |
|---|---|
| `Python/NTC_1A_serial_comm_v1_3_4.py` | `Python/NTC_1A_serial_comm_v1_3_5_beta.py` |
| `Python/pi_logger_evidence_v2.py` | `Python/pi_logger_evidence_v2_1_beta.py` |
| `Python/pi_logger_evidence_v3.py` | `Python/pi_logger_evidence_v3_1_beta.py` |
| `Python/pi_logger_synced_v2.py` | `Python/pi_logger_synced_v2_1_beta.py` |
| `ESP32/TCBridge_EvidenceV2/pi_logger_evidence_v2.py` | `ESP32/TCBridge_EvidenceV2/pi_logger_evidence_v2_1_beta.py` |

The two Evidence V2 copies have identical original contents; both are preserved and each has its own beta copy.

## BETA acceptance criteria

- Exactly 6 bytes, footer `0x7F`.
- LEN groups `b0,b1,b2`: each 0..99.
- `b3=tens[0]`: 0..9; `b4=tens[1]`: 0..102.
- Reconstructed tension `b4*10+b3`: 0..1023.
- Boot Magic `DE AD BE EF 01 7F` remains handled by the existing dedicated check.
- No changes to TX, RESET, SEND, SENS.ADJ, Evidence RESULT, timing, or resynchronization.
- No change to the active GUI import: `ntc_pygame_ui_v2_4.py` still imports `NTC_1A_serial_comm_v1_3_4`.
- No automatic production switchover; beta must be explicitly selected for testing.

## Independent evidence and scope

2026-10-23 real TC106 capture 4: the earlier independent waveform decoder counted
421 complete frames, including a leading frame; Claude's 2529-byte stream began
one byte later and yielded 420 complete frames. On Claude's stream, the original
Pi accepted 128/420; the reviewed v1.3.5 candidate accepted 420/420 and the
original parser produced 7 legacy 8/12-byte checksum false matches. On the
421-frame waveform reconstruction, 293 frames had `tens_actual=1023`.
These results describe offline parsing only, **not** an ESP/Pi hardware PASS.

Claude's review verdict was APPROVE WITH CHANGES; its conditions were real-capture
regression and a policy for the four evidence-loggers. This commit provides
new beta copies for the loggers and an offline regression test. A future
independent cross-review should verify their exact diffs before Stage C.

## Offline test

```sh
python3 -m unittest discover -s Python/tests -p 'test_direction_b_tension_beta_20261024.py' -v
```

Test scope: all 1024 legal ADC values for each beta, field boundaries, invalid
encoding, the 1023 real-capture example, Boot Magic rejection by field validators,
and Pi value reconstruction. It does not replace full raw .sr re-decoding or
on-device end-to-end tests.

## Open issues and guardrails

1. Original legacy 8/12-byte parser may still false-match on mid-frame startup;
   this change intentionally does not modify that pre-existing behavior.
2. Real TC106 `lastlen` is declared signed char; rare underflow encodings have
   not been validated on real hardware.
3. LA measured about 5.3..5.4 V HIGH. Check analyzer absolute maximum input
   voltage before another direct hookup.
4. For Stage C, test the beta module explicitly (GUI still imports the original)
   and confirm 1023 displays correctly; use beta evidence-loggers, not old ones.
5. Real capture files are retained in the external 2026-10-24 review package,
   not duplicated here. This repository test uses a deterministic captured frame
   and synthetic full-range vectors.
