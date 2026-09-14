# App272 外注先負荷ヒートマップ Ver0.6.1.5
## Claude クロスレビュー依頼 / 2026-09-14

**母体:** Ver0.6.1.4 Windows横スクロール改善版
**変更目的:** 上段ヒートマップの「今日」視認性改善のみ
**実装状態:** STATIC BUILD PASS / CROSS REVIEW PENDING / REAL DEVICE TEST PENDING

## レビューしてほしい点
1. Ver0.6.1.4からの差分が最小で、既存機能を変更していないか。
2. `todayKey = formatDate(startOfDay(new Date()))` と各日付の `formatDate()` 比較が既存日付処理と整合するか。
3. ヘッダと本体セルへのclass付与方式で、再描画時に重複DOM/線が残らないか。
4. CSS疑似要素 `::before` による2px縦線が、赤/黄/緑の負荷背景色や土日着色を壊さないか。
5. `position:relative` 追加がsticky外注先列や表レイアウトへ副作用を与えないか。
6. 既存の初期今日スクロール、スクロール保持、Windows Shift+wheel、Mac横スクロールへ影響しないか。
7. 表示期間に今日が含まれない場合、安全にclassが付かないだけで終わるか。
8. 下段案件詳細、入荷記録/取消、レコードリンク等に影響がないか。

## 実装方針
- 今日列全体を塗りつぶさない。
- 負荷色を変更しない。
- 列幅・行高を変更しない。
- JSで専用classを既存セルへ付け、CSSで線を重ねるだけ。
- オーバーレイDOMを別途生成しないため、再描画時の残骸を作らない。

## 期待する判定
`GO / GO WITH CONDITIONS / NO-GO` と、Major / Minor / Observationを分けてください。
特に既存Ver0.6.1.4の回帰リスクがあれば具体的に指摘してください。

## 添付
- `vendor_heatmap_v0614_windows_horizontal_scroll.js`
- `vendor_heatmap_v0614_windows_horizontal_scroll.css`
- `vendor_heatmap_v0615_today_marker.js`
- `vendor_heatmap_v0615_today_marker.css`
- `vendor_heatmap_v0614_to_v0615_today_marker.diff`
- `vendor_heatmap_v0615_today_marker_IMPLEMENTATION_NOTE.md`
- `案件ヒートマップ.jpg`
- `明細チャート.jpg`

## 静的確認
- JavaScript `node --check`: PASS
- JS SHA-256: `3f3a5394167745b80342736d95fc9cc43a526fa3c510b677f6a6673c0714754e`
- CSS SHA-256: `7372255c608ff08cfc5911e2f40319761d4fc5d0ee9ec9a89326b53ef9c92554`
