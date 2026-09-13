# Phase 2C-4 v0.2.9

v0.2.8の `ui` スコープ不具合修正版です。

## 症状
保存自体は成功した後、保存後再取得処理で

`Can't find variable: ui`

となり、利用者には「保存に失敗しました」と表示されることがありました。

## 原因
`openReservationDialog(selection)` 内の保存処理で、

- `ui.status.textContent`
- `clearDragPreview(ui)`
- `renderTimeline(ui)`

を使用していましたが、`ui` は `openReservationDialog` のスコープに存在していませんでした。

## 修正
`openReservationDialog(ui, selection)` に変更し、
ドラッグ完了時に `ui` を明示的に渡します。

## 重要
v0.2.8で「保存失敗」と表示された場合でも、
POST自体は成功している可能性があります。

同じ時間帯を再保存せず、App 291標準一覧または重複チェックで
既存レコードを確認してください。

## 実機確認
1. 既に保存済みの09:15〜10:45は再保存しない
2. 別の空き時間で1件だけ新規保存
3. 「保存しました。レコード番号: XX」が見える
4. 約0.5秒後にDialogが閉じる
5. Timelineに新規予約バーが即時表示
6. App 291標準一覧にも1件だけ存在
7. lookupコピー先が反映

## kintone設定
JavaScript
1. https://unpkg.com/vis-timeline@7.7.3/standalone/umd/vis-timeline-graph2d.min.js
2. reservation_phase2c4_v0.2.9.js

CSS
1. https://unpkg.com/vis-timeline@7.7.3/styles/vis-timeline-graph2d.min.css
2. reservation_phase2c4_v0.2.9.css
