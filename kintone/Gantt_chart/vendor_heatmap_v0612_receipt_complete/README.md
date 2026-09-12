# vendor_heatmap_v0612_receipt_complete

## Status

- Version: Ver0.6.1.2
- Specification review: CROSS REVIEW GO / FROZEN
- Implementation diff review: GO
- CSS integration review: GO
- Static syntax check: PASS (`node --check`)
- Real-device test: IN PROGRESS (receipt editor display confirmed; fit fix pending retest)
- FORMAL PASS: NOT YET

## Purpose

Ver0.6.1.1 を母体に、App272 の `外注入荷実績日` を案件単位で記録し、入荷済み案件を下段ガントで完了（グレー）表示する候補版。

## Rollback policy

**Ver0.6.1.1 のフォルダ／JS／CSSは変更しない。**

実機試験で問題が発生した場合は、kintone のカスタマイズ設定を以下へ戻す。

- JS: `kintone/Gantt_chart/vendor_heatmap_v0611_scrollpreserve/vendor_heatmap_v0611_scrollpreserve.js`
- CSS: `kintone/Gantt_chart/vendor_heatmap_v0611_scrollpreserve/vendor_heatmap_v0603_standard_gantt_layout.css`

これにより Ver0.6.1.1 へ即時ロールバック可能とする。

## Ver0.6.1.2 reviewed artifacts

- `vendor_heatmap_v0612_receipt_complete.js`
  - SHA-256: `991e4823fd63ad8e2d20b6aafa8c471940298a7f810cd0a67cae9025a404be13`
- `vendor_heatmap_v0612_receipt_complete.css`
  - Updated: `7f9551f`（UI fit fix。再レビュー時にSHA-256を再確定）

CSS は Ver0.6.0.3 の既存CSSを母体に、Ver0.6.1.2追加分を末尾統合した1本構成。新規追加分の `.eh-gantt-info` 誤参照は `.eh-gantt-cell` へ修正済み。

## Important implementation rules

- 保存成功後に `render()` 全体を呼ばない。
- 該当行のみピンポイントDOM更新する。
- PUT対象は `外注入荷実績日` のみ。
- `$revision` を必須使用して競合更新を防ぐ。
- 保存中disableはクリックした行だけ。
- 入荷ボタン操作は `event.stopPropagation()` でレコードリンク誤発火を防ぐ。
- Manual / Batch / Auto および K日程項目を変更しない。
- 上段ヒートマップの負荷計算は Ver0.6.1.2 では変更しない。

## Deployment

従来どおり kintone には **JS 1本 + CSS 1本** を登録する。

実機1件試験および回帰試験完了までは FORMAL PASS としない。


### Receipt editor fit fix (2026-09-12)

- 標準表示の手配書番号列（120px）内で日付入力欄と保存・キャンセルが欠ける問題をCSSで修正。
- ラベルと日付入力を縦積みにし、入力欄を列幅100%、操作ボタンを2等分配置。
- 固定列幅、ガント領域、JS保存処理は変更していない。
