# Phase 2C-2 v0.2.3

Phase 2C-1 v0.2.2 をベースに、ドラッグ完了後の予約入力ダイアログを追加。

## 実装済み
- 15分ドラッグ
- 施設固定
- 選択帯表示
- 日付/施設/開始/終了を読み取り専用表示
- 予約者入力
- 件名/用件入力
- メモ入力
- Esc / キャンセル / 背景クリックで閉じる
- 「入力確認」で入力値取得確認

## 未実装
- 必須バリデーション
- 重複チェック
- REST登録
- 保存後再描画

## kintone設定

JavaScript
1. https://unpkg.com/vis-timeline@7.7.3/standalone/umd/vis-timeline-graph2d.min.js
2. reservation_phase2c2_v0.2.3.js

CSS
1. https://unpkg.com/vis-timeline@7.7.3/styles/vis-timeline-graph2d.min.css
2. reservation_phase2c2_v0.2.3.css
