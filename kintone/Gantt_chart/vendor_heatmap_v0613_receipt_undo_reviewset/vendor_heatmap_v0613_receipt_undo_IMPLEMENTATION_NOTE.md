# App272 外注入荷実績取消 Ver0.6.1.3 実装メモ

## 母体
- GitHub現行 `vendor_heatmap_v0612_receipt_complete_fixed_position` のfixed-position設計を踏襲。
- 既存のスクロール保持・render generation guard・上段ヒートマップ・K日程系ロジックは変更対象外。

## 追加/修正
1. body直下の入荷ポップアップも `activeReceipt` 経由で保存/取消中にdisable。
2. 入荷済み案件の「入荷日修正」ポップアップだけに `取り消し` ボタンを表示。
3. 取消は `updateReceiptDate(record, '')` で `外注入荷実績日` のみ空文字PUT。
4. 取消成功時は `record.receiptDate = null` とし、応答revisionをローカルへ反映。
5. `applyUncompletedStateToRow()` で該当行だけ未入荷表示へ戻し、`render()` は呼ばない。
6. `activeReceipt.saving` を追加し、保存/取消の二重PUTを防止。
7. PUT中は外側クリックでポップアップを閉じない。

## 実機確認必須
- T17〜T22。
- 特に `外注入荷実績日: { value: "" }` がApp272テスト環境で日付を空欄化すること。
- 取消→再入荷でrevision競合が起きないこと。
- 取消/保存連打でPUTが1回だけであること。

## 状態
- STATIC BUILD / node --check待ち（生成時点）
- 実機未確認。FORMAL PASSではない。
