# 外注負荷ヒートマップ Ver0.6.1.2 外注入荷実績記録＋ガント完了表示 実装記録（CSS統合版）

作成日: 2026-09-12

## 状態

- 仕様: CROSS REVIEW GO / FROZEN
- 実装: CANDIDATE BUILT
- Claude実コードレビュー: GO WITH CONDITIONS → CSS 1箇所修正反映済み
- 静的構文確認: `node --check` PASS
- 実機試験: 未実施
- FORMAL PASS: 未判定

## 母体

### JS
`vendor_heatmap_v0611_scrollpreserve(1).js`

SHA-256: `b7572fd60f58e2075e648c9dfa8d417063b171610a1ce4af31924266ec0b58d9`

### CSS
`vendor_heatmap_v0603_standard_gantt_layout.css`

GitHub blob SHA: `5f8ebf5361f0298710dfa5042dc9db62bf3c31bd`

SHA-256: `12a3252f6da98d321f093f9c8cb08b6de7b08993238ca1cc48099fc8d41322db`

上記GitHub blobと完全一致するCSSを母体として使用。

## Ver0.6.1.2 納品ファイル

### JS
`vendor_heatmap_v0612_receipt_complete.js`

SHA-256: `991e4823fd63ad8e2d20b6aafa8c471940298a7f810cd0a67cae9025a404be13`

### CSS（統合版 / kintone登録用）
`vendor_heatmap_v0612_receipt_complete.css`

SHA-256: `b7890606501e7dc4f028f5d6c01454370a5884705768688db88aa33ef4d9c66c`

Ver0.6.0.3 CSS + Ver0.6.1.2入荷UI/完了表示CSSを1本へ統合。
kintoneでは従来どおり **JS 1本 + CSS 1本** で登録する。

## Claudeレビュー指摘反映

追加CSS内の以下を修正。

```css
.eh-schedule-scroll .eh-completed:hover .eh-gantt-info,
.eh-schedule-scroll .eh-completed:focus-within .eh-gantt-info
```

↓

```css
.eh-schedule-scroll .eh-completed:hover .eh-gantt-cell,
.eh-schedule-scroll .eh-completed:focus-within .eh-gantt-cell
```

`.eh-gantt-info` はVer0.6.0.2以降の現行下段DOMでは生成されず、`.eh-gantt-cell` が現行クラスのため。

## 実装安全性確認

- 保存成功後に `render()` 全体を呼ばない: PASS
- PUT payloadは `外注入荷実績日` のみ: PASS
- `$revision` 必須使用: PASS
- Manual / Batch / Auto更新なし: PASS
- scroll preserve / renderGeneration変更なし: PASS
- 行単位disable: PASS
- `event.stopPropagation()`: PASS
- 保存成功後は該当行だけDOM更新: PASS
- 上段ヒートマップ負荷計算は変更しない: PASS

## デプロイ方法

旧Ver0.6.1.1から切り替える際は、kintoneカスタマイズ設定を以下の2本にする。

1. `vendor_heatmap_v0612_receipt_complete.js`
2. `vendor_heatmap_v0612_receipt_complete.css`

**旧 `vendor_heatmap_v0603_standard_gantt_layout.css` と追加CSSを同時登録しない。**
統合版CSS 1本だけを登録する。

## 実機前提

App272にDATEフィールド `外注入荷実績日` が存在し、フィールドコードも同一であること。

## 次工程

1. ClaudeへCSS修正後の最終diff確認
2. App272 TESTで1レコードのみ実機試験
3. `外注入荷実績日` 以外のフィールドが変化していないことを確認
4. T01〜T15回帰試験
5. FORMAL PASS判定
