# Ver0.6.1.6 外注先クイックフィルタ 実装ノート

作成日: 2026-09-14
母体: Ver0.6.1.5 `vendor_heatmap_v0615_today_marker.js / .css`
設計: `外注課負荷ヒートマップ_Ver0.6.1.6_外注先クイックフィルタ_詳細仕様設計書_20260914_Rev2.md`

## 実装内容

- 外注先クイックフィルタを追加。
- チェックボックス複数選択。選択社間はOR条件。
- 0社選択は全社表示。
- 候補検索、選択数表示、選択済みチップ、個別解除、全解除を追加。
- 候補はkintone標準絞り込み後の `state.records` に存在する `vendor` から生成。
- `state.records` は原母集団として保持し、`visibleRecords` を描画用に派生。
- `renderHeader()` と `renderSummary()` の隠れた `state.records` 依存を引数化。
- Heatmap / Risk / Detail は同じ `visibleRecords` を使用。
- `selectedVendor` が表示対象外になった場合は表示対象先頭へ安全に再選択。
- 標準絞り込みで母集団が変わった場合はクイック選択を全解除。
- クイックフィルタ操作はAPI再取得・PUT/POST/DELETEを行わない。
- 入荷操作ハンドラの `state.records.find(...)` は変更していない。
- 入荷PUT中はクイックフィルタによる再描画を抑止し、更新処理を優先。

## Rev.2条件の反映確認

`state.records` の実コード参照は、コメントを除き以下だけに限定した。

1. API取得結果の代入（原母集団）
2. 初回 `chooseInitialVendor(state.records)`
3. `getQuickFilteredRecords(state.records)` の入力
4. クイックフィルタ候補生成用 `renderVendorQuickFilter(state.records)`
5. 入荷操作の `state.records.find(...)`（変更禁止箇所）

`renderHeader()` / `renderSummary()` / `renderRiskSummary()` / `renderVendorDetail()` の描画系は `visibleRecords` 基準。

## 静的確認

- `node --check`: PASS
- CSS `{}` 数一致: PASS (475 / 475)
- Ver0.6.1.5→0.6.1.6 JS/CSS diff作成済み

## 未実施

- kintone実機（Windows Chrome）
- Mac回帰
- Claude diffクロスレビュー

したがって現時点は **STATIC BUILD PASS / DEVICE TEST PENDING / CROSS REVIEW PENDING**。
