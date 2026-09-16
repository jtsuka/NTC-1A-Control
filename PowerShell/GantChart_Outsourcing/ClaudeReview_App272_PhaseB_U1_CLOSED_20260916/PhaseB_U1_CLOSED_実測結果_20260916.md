# App272 Phase B U1 実測結果 — PASS / CLOSED

## 1. 概要

2026-09-16、App292（TEST）で Phase B U1 を実施した。

目的は「既存 `Batch` レコードについて、CSV側の納期だけが変更された場合に、基幹納期更新とK日程再計算を安全に実行できること」の実証。

## 2. 前提

- TEST: App292
- PROD: App272
- U12複数PUT原子性: `ATOMIC_ALL_OR_NOTHING`
- 初期 `PutBatchSize=10`
- Policy: `2026-09-16-U12-1`

## 3. 対象レコード

```text
PO       : 2609249755
RecordId : 3
Revision : 4（U1 Execute前）
```

基準状態:

```text
VENDOR_TEXT   = ㈱吉川鉄工所
VENDOR_DD     = ㈱吉川鉄工所
STAFF_TEXT    = 木村　和也
STAFF_DD      = 木村　和也
手配日        = 2026-09-15
納期          = 2026-11-02
K加工着手日   = 2026-10-26
K完成検査日   = 2026-10-30
K日程種別     = Batch
K暫定設定     = ON
K計算根拠     = Company
```

## 4. テスト入力

CSVは対象POを1行だけ抽出し、納期だけ変更。

```text
旧納期 : 2026-11-02
新納期 : 2026-11-06
変更   : 納期のみ
```

## 5. DryRun結果

```text
=== App292 Phase B Update v0.8.0.3 TEST ===
Mode             : DRYRUN
App              : 292
CSV rows         : 1
Updates          : 1
Policy           : 2026-09-16-U12-1
U12              : ATOMIC_ALL_OR_NOTHING
PutBatchSize     : 10

PO=2609249755 ID=3 Rev=4 Class=UPDATE_RECALC
Changed : 日付_1,K加工着手日,K完成検査日
Due     : 2026-11-02 -> 2026-11-06
KStart  : 2026-10-26 -> 2026-11-02
KInspect: 2026-10-30 -> 2026-11-05
KType   : Batch -> Batch
KOnOff  : ON -> ON
KBasis  : 'Company' -> 'Company'
Hash    : 2a6a0416d8c969858a2bfd188e2395bb16492b5a5c68e4729571266638de9354

WRITE            : NONE
RESULT           : DRYRUN PASS
```

## 6. Execute結果

```text
=== App292 Phase B Update v0.8.0.3 TEST ===
Mode             : EXECUTE
App              : 292
CSV rows         : 1
Updates          : 1
Policy           : 2026-09-16-U12-1
U12              : ATOMIC_ALL_OR_NOTHING
PutBatchSize     : 10

PO=2609249755 ID=3 Rev=4 Class=UPDATE_RECALC
Changed : 日付_1,K加工着手日,K完成検査日
Due     : 2026-11-02 -> 2026-11-06
KStart  : 2026-10-26 -> 2026-11-02
KInspect: 2026-10-30 -> 2026-11-05
KType   : Batch -> Batch
KOnOff  : ON -> ON
KBasis  : 'Company' -> 'Company'
Hash    : 2a6a0416d8c969858a2bfd188e2395bb16492b5a5c68e4729571266638de9354

WRITE            : MULTI PUT EXECUTED
RESULT           : EXECUTE PASS / post reGET verified
```

## 7. 安全設計で実際に通過した項目

- App292固定
- Execute確認文字列
- `ExpectedUpdatePo` 完全一致
- `MaxUpdates=1`
- Execute直前再GET
- RecordId一致
- revision一致
- before値一致
- changeHash一致
- PUT後再GET
- ChangedFields実値照合

## 8. 判定

**U1 = PASS / CLOSED**

