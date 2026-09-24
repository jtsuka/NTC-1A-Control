# App292 TEST確認手順 — vendor_heatmap v0.6.1.8.1 ODERNO fix

## 前提
- PROD App272は触らない。
- App292(TEST)のみでClear→NoJOIN再投入→画面確認する。
- 修正版JS: `vendor_heatmap_v0618_1_ODERNO_fix.js`
- CSSはv0.6.1.8の現行CSSをそのまま使用。

## 1. App292 Clear DryRun
既存のApp292用ClearスクリプトでDryRunを行い、対象件数を確認する。

## 2. App292 Clear Execute
DryRun結果確認後のみ全削除を実行し、0件を確認する。

## 3. NoJOIN取込 DryRun
937件CSVをApp292へDryRunし、以下を確認する。
- CSVRows=937
- UniquePO=937
- HandOrderPresent=327
- HandOrderBlank=610
- Missing VENDOR_DD=0
- Missing STAFF_DD=0

## 4. NoJOIN取込 Execute
PostVerify=937 / mismatch=0を確認する。

## 5. 修正版JS適用
App292のJavaScriptカスタマイズを `vendor_heatmap_v0618_1_ODERNO_fix.js` に差し替える。
CSSは変更しない。

## 6. 画面確認
### T01 ODERNOあり
`ODERNO` に値がある代表レコードを開く。
例: `T259011080`。
期待: ヒートマップの日程リスク/案件表示に `T259011080` が表示される。

### T02 旧Lookup空
同じレコードで旧Lookup `ルックアップ` が空でも、T01の表示が維持される。

### T03 ODERNOなし
手配書番号なしレコードでは `手配書番号なし` が表示される。

### T04 注文書番号
注文伝票番号 `数値_44` は従来どおり「注文書番号」として表示される。

### T05 回帰
外注先・担当・加工着手日・完成検査日・納期・計算根拠・日程種別が従来どおり表示される。

## PASS条件
T01〜T05すべてPASS。
