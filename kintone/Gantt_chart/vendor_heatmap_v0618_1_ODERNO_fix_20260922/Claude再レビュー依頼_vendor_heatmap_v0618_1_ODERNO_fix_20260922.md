# Claude再レビュー依頼 — vendor_heatmap v0.6.1.8.1 ODERNO field-code fix

## 目的
NoJOIN方式で旧Lookupを使用しない場合でも、ヒートマップ上に実際の手配書番号を表示できるようにする。

## 不具合
`CONFIG.fields.key` が画面上の表示名 `K手配書番号` を指定しており、実際のkintoneフィールドコードと不一致だった。
そのため `fieldValue(r, f.key)` は常に空となり、旧Lookup `ルックアップ` が空のNoJOINデータでは `手配書番号なし` 表示になっていた。

## 実フィールドコード
- 手配書番号の実フィールドコード: `ODERNO`
- 旧Lookup: `ルックアップ`
- 注文伝票番号: `数値_44`

## 修正差分
修正は1箇所のみ。

```diff
-      key: 'K手配書番号',
+      key: 'ODERNO',
```

`normalizeRecord()` 以下の既存ロジックは変更していない。

```javascript
displayKey:
  fieldValue(r, f.key) ||
  fieldValue(r, f.keyFallback) ||
  '手配書番号なし'
```

よって、修正後は以下の優先順位となる。

1. `ODERNO`
2. 旧Lookup `ルックアップ`
3. `手配書番号なし`

## 静的確認
- `node --check`: PASS
- 元JSとの差分: 1行のみ
- CSS変更: なし
- REST取得条件変更: なし
- K日程ロジック変更: なし
- App270依存追加: なし

## SHA256
- 元 `vendor_heatmap_v0618_no_arrangement_label.js`
  - `edc4d81d36c82bdc3d33104e056862925c728dd68d36dd2e3cb109ea6df4967a`
- 修正版 `vendor_heatmap_v0618_1_ODERNO_fix.js`
  - `2740572d099d4f4f3fbae172642040df86951092f4acfe93dd7011cf14321de8`

## 再レビュー依頼事項
1. `CONFIG.fields.key = 'ODERNO'` が今回の目的に対して正しい最小修正か。
2. `displayKey` / `arrangementNo` の既存フォールバック順序に副作用がないか。
3. NoJOIN方式で `ルックアップ` が空でも `ODERNO` があれば正常表示されるか。
4. `ODERNO` も空の場合のみ `手配書番号なし` になるか。
5. 他のヒートマップ表示・日程リスク判定・入荷関連処理への副作用がないか。

## 実機テスト方針
PROD App272は937件の初期投入が完了済みなので、全削除テストは行わない。
App292(TEST)でNoJOINデータをClear→再投入し、修正版JSを適用して確認する。

代表確認:
- ODERNOあり: `T259011080` 等が `手配書番号なし` ではなく表示されること。
- ODERNOなし: `手配書番号なし` と表示されること。
- 注文書番号（画面表示）から対象レコードへ追跡できること。
