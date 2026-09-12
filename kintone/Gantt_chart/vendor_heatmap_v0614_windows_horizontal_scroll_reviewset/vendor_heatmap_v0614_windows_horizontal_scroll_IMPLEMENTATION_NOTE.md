# App272 外注負荷ヒートマップ Ver0.6.1.4 実装メモ

## 母体
- `vendor_heatmap_v0613_receipt_undo.js`
- SHA-256: `a8f923e0a7114813df60f4d4469f298d50e2283b15732855d83084e50bbbac27`
- CSS: Ver0.6.1.3と同一内容。

## FROZEN仕様
- `App272_外注負荷ヒートマップ_Ver0.6.1.4_Windows横スクロール操作改善_仕様書_FROZEN_Rev3_20260912.md`
- SPEC FROZEN / IMPLEMENTATION AUTHORIZED。

## 実装内容
- `getHeatmapScrollContainer(root)` を追加し、`.eh-heatmap.closest('.eh-table-scroll')` で上段本体を特定。
- `jumpToTodayAfterRender()` / `captureScrollPositions()` / `restoreScrollPositions()` を共通ヘルパーへ統一。
- `normalizeHorizontalWheelDelta()` を追加し、deltaMode 0/1/2を考慮。
- `bindHorizontalWheelSupport()` を追加。
  - 通常ホイールは非介入。
  - Shift＋deltaY主成分のみ横移動へ変換。
  - deltaX主成分はブラウザ標準へ任せる。
  - 自前変換イベントは左右端でも `preventDefault()`。
  - `passive:false`。
  - 上段・下段は独立。
- `bindEvents(root)` から現行DOMへwheel listenerを登録。
- CSS変更なし。
- REST API、入荷登録/修正/取消、日程計算、K日程フィールドへの変更なし。

## 静的確認
- node --check: PASS
- 上段共通ヘルパー: PASS
- jumpToToday共通ヘルパー使用: PASS
- capture共通ヘルパー使用: PASS
- restore共通ヘルパー使用: PASS
- 旧決め打ち除去: PASS
- passive:false: PASS
- 通常ホイール非介入: PASS
- deltaX優勢ネイティブ: PASS
- 左右端でもpreventDefault: PASS
- CSS同一: PASS
- 追加差分にREST/K日程書込変更なし: PASS

## 未実施
- T00: Windows Chrome/Edge上の現行Ver0.6.1.3でネイティブShift＋ホイール挙動を確認
- Claudeによる逆側実コードクロスレビュー
- Windows普通マウス実機テスト
- Macトラックパッド回帰テスト
- 入荷登録・修正・取消の実機回帰

## 判定
**STATIC BUILD PASS / CROSS REVIEW PENDING / REAL DEVICE TEST PENDING**

現時点では FORMAL PASS ではない。
