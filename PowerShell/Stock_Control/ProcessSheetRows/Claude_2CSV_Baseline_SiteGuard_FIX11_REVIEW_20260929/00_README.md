# Claude 2CSV Baseline Site Guard FIX11 review package — 2026-09-29

## Purpose
FIX10 received **GO WITH MINOR** from Claude. The remaining Major item was that the implemented Baseline marker did not match the already-fixed REV5/REV5.1 design:

- current implementation: J1=`BASELINE_INITIALIZED` only (+ K1 timestamp)
- design: K1:L3 metadata including **BaselineInitializedSite**

This package freezes FIX10 and adds only the Baseline identity/safety correction as **FIX11**.

## Frozen source
`ProcessSheetRows_20260929_r15_10_REV5_1_2CSV_FIX10_BASELINE_GATE_CANDIDATE.ps1`

SHA-256:
`B0A24465532879C9700318D4873D9E3986899B5D74CEE755B7EC219F49943380`

## FIX11 candidate
`ProcessSheetRows_20260929_r15_10_REV5_1_2CSV_FIX11_BASELINE_SITE_GUARD_CANDIDATE.ps1`

SHA-256:
`4B9DDC63A3142617000698E2B4AC161F27994C2FA803FF3C958520D28C601B42`

## What FIX11 changes
1. Restores formal K1:L3 Baseline metadata from the design.
2. Requires metadata + current SiteName match before `BaselineInitialized=True`.
3. Rejects legacy J1-only marker for 2CSV (fail-closed).
4. Rejects SiteName mismatch (fail-closed).
5. Rejects CandidateKey history rows without formal Baseline metadata (fail-closed).
6. Verifies metadata immediately after Baseline Initialize, before Save.
7. Handles the design-allowed `Baseline candidates = 0` case by creating A:I history headers if the metadata setter must create the sheet.

## Not changed
- FIX10 Baseline Gate logic
- CandidateKey logic
- Main/Sub dataset boundary logic
- Phase3 Move logic
- staging deletion contract
- 1CSV behavior
- automatic Baseline initialization (still prohibited)

## Current production facts carried forward
- 外注鉄 pending manual destination instructions: 43
- 成果連絡表（色物）: 24
- total: 67
- both production copies used as evidence have no `新規商品コード登録履歴` sheet
- 2026-09-29 production log: both 2CSV sites remained `BaselineInitialized=False`, Phase3 skipped, Phase3Moved=0

## Important
**Do not deploy FIX11 to production and do not run production Baseline Initialize from this package alone.**

Review order:
1. `01_Claudeレビュー依頼_FIX11_20260929.md`
2. `02_FIX10_GO_WITH_MINOR_受領結果_20260929.md`
3. `03_FIX11最小修正仕様書_20260929.md`
4. `04_FIX11テスト仕様書_20260929.md`
5. `diffs/FIX10_to_FIX11_BASELINE_SITE_GUARD.diff.txt`
