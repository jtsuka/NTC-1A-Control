# schedule_preview_vendor_v3.0 クロスレビュー用メモ

対象コード：`schedule_preview_vendor_v3.0_rev3_3.js`
基準仕様：`外注先別_暫定日程設定_コードレビュー仕様書_Rev3_3_20260908.md`

## 実装した主要変更

1. 外注先プルダウンを追加し、App272の `VENDOR_DD` の実在値を重複除去して表示。
2. 外注先ごとの `検 / 着` を `schedulePreview.vendorRules` 1キーのJSONでlocalStorage保存。
3. 暫定ONは以下をすべて満たすレコードだけPUT。
   - `VENDOR_DD = 選択外注先`
   - `K暫定設定 = ON`
   - `K日程種別 = Auto`
4. 暫定OFFは以下だけを対象に `K加工着手日 / K完成検査日` を空欄化。
   - `VENDOR_DD = 選択外注先`
   - `K日程種別 = Auto`
5. `Manual` はON/OFF双方から保護。
6. `app.record.edit.show` で編集開始時のK日付を保持し、`app.record.edit.submit` で保存時の値と比較。最終値が変わった場合のみ `K日程種別 = Manual` を自動設定。
7. 旧v2.1のグローバルトグル、全件復元、`isAutoProvisionalRecord()` 推測判定を削除。
8. ON/OFFそれぞれで対象分類を行い、確認ダイアログと結果件数を表示。
9. REST PUTは100件単位、`$revision` を指定。

## クロスレビューで重点確認してほしい点

- kintone標準編集画面の `app.record.edit.show` → `app.record.edit.submit` の比較で、K日付変更時に `K日程種別` が同一保存処理で確実に `Manual` 保存されるか。
- REST APIによる一括PUTでは上記編集画面イベントが発火せず、`Auto` が維持されるという前提が妥当か。
- 日付フィールドのクリア値として `{ value: null }` が対象環境で問題ないか。
- `K日程種別` フィールドの編集権限がオペレーターにない場合でも、クライアントJSからsubmitイベント内で値を書き換えられるか。正式運用前にフィールド権限確認が必要。
- 既存レコードで `K日程種別` / `K暫定設定` が空欄の場合、厳密一致により対象外になる。Rev.3.3仕様の事前初期化が必要。
- OFFは仕様どおり `K暫定設定` を条件にしていない。選択外注先のAuto日程を一括クリアする設計。
- 外注先一覧取得は一覧表示後にAPIで全レコードの `$id,VENDOR_DD` のみ取得。766件規模では軽量だが、将来件数増加時の負荷は要確認。
- 祝日は2026年固定、会社独自カレンダー未対応（仕様どおり）。
- localStorageはブラウザ／端末ごとで共有されない（TEST段階の仕様どおり）。

## 静的確認済み

- `node --check`：PASS
- 旧 `stateStorageKey / loadToggleState / saveToggleState / buildRestoreUpdates / isAutoProvisionalRecord`：残存なし
- 必須フィールドコード：`VENDOR_DD / K暫定設定 / K日程種別 / K加工着手日 / K完成検査日`：実装済み

## 実機テスト推奨順

1. 既存TESTレコードの初期化状態を確認（`K暫定設定=OFF`, `K日程種別=Auto`）。
2. 1社・1～2件だけ `K暫定設定=ON` にする。
3. 暫定ON → 対象社・Auto・ONだけ更新されることを確認。
4. 標準編集画面で1件のK日付を変更して保存 → `K日程種別=Manual` 自動化を確認。
5. 同じ外注先で再度暫定ON → Manual件が保持されることを確認。
6. 暫定OFF → Autoだけ空欄、Manual件が保持されることを確認。
7. 別外注先のデータが一切変わっていないことを確認。
