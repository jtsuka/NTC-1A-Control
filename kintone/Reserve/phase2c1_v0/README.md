# Phase 2C-1 v0.2.0

Phase 2A v0.1.6をベースに、15分単位のドラッグ選択を統合した実機試験版です。

## kintoneへの設定

JavaScript:
1. vis-timeline 7.7.3
2. `reservation_phase2c1_v0.2.0.js`

CSS:
1. vis-timeline 7.7.3 CSS
2. `reservation_phase2c1_v0.2.0.css`

カスタマイズ一覧HTML:
```html
<div id="reservation-timeline-root"></div>
```

## この版で実装する範囲

- Phase 2A表示維持
- 予約バークリック維持
- 15分ドラッグ
- 施設固定
- 07:00〜19:00
- 選択帯表示
- 選択内容確認
- Esc/選択解除

予約登録はまだ行いません。
