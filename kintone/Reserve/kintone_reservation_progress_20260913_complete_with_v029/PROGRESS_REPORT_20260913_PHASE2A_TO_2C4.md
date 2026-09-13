# 会議室・設備予約 kintone
## 2026-09-13 進捗報告 / Claudeクロスレビュー用
### Phase 2A〜Phase 2C-4 到達点まとめ

作成日：2026-09-13
対象：
- App 290：施設予約リソースマスタ
- App 291：会議室・設備予約
- 現行コード：Phase 2C-4 v0.2.9

---

# 1. 本日の結論

本日、日次タイムライン表示から、
ドラッグ選択・予約入力ダイアログ・Validator・本保存・保存後即時反映まで到達した。

現時点の判定：

- Phase 2A：CLOSED / PASS
- Phase 2B：CLOSED / PASS
- Phase 2C-1：PASS
- Phase 2C-2：PASS
- Phase 2C-3：CLOSED / PASS
- Phase 2C-4：管理者権限でPASS候補

未完了：

**一般ユーザー権限での実機確認**

これが通ればPhase 2C-4を正式CLOSE候補とする。

---

# 2. Phase 2A 日次タイムライン

## 実装内容

- App 290の使用中施設を縦軸表示
- 07:00〜19:00
- 30分表示軸
- 予約自体は15分単位
- 予約バー表示：`予約者｜件名`
- 予約者名ハッシュによる固定パレット色分け
- 前日 / 今日 / 翌日 / 任意日付
- 施設表示順：DISPLAY_ORDER
- 予約データページング
- XSS対策

## 実機結果

予約0件：
- 7施設均等表示：PASS
- 第一会議室だけ縦長になる問題：解消

予約5件：
- 複数施設：PASS
- 同一施設の隣接予約：PASS
- 時間位置：PASS
- 同一予約者同色：PASS
- 行高崩れなし：PASS

最終的に高さ制御は：

- height指定なし
- minHeight指定なし
- groupHeightMode:'fixed'
- vis-timeline自身に高さ計算を任せる

で安定。

---

# 3. Phase 2B 詳細遷移

確認済み：

- 予約バークリック → App 291該当詳細へ遷移
- 空白クリック → 遷移しない
- 日次タイムライン以外の標準一覧 → 通常利用可能

判定：

**CLOSED / PASS**

---

# 4. Phase 2C-1 15分ドラッグ選択

初期実装ではDOMのmousedownを利用したが、
vis-timeline内部イベントと相性が悪く動作しなかった。

最終的に：

- `timeline.on('mouseDown')`
- `timeline.on('mouseMove')`
- `timeline.on('mouseUp')`

へ変更。

実機で：

- 15分単位
- 最低15分
- 逆方向ドラッグ
- mousedown時施設固定
- 07:00〜19:00
- 選択帯表示
- 選択内容表示

を確認。

---

# 5. Phase 2C-2 予約入力ダイアログ

ドラッグ完了後に「新しい予約」ダイアログを表示。

読み取り専用：

- 日付
- 施設
- 開始時刻
- 終了時刻

入力：

- 予約者
- 件名／用件
- メモ

実機で入力値取得まで確認。

---

# 6. Phase 2C-3 Validator

実装：

- 予約者必須
- 件名必須
- 07:00〜19:00
- 15分単位
- start < end
- 既存予約重複チェック
- 隣接許可

重複式：

`NewStart < ExistingEnd && NewEnd > ExistingStart`

Claudeレビューを受けて以下修正：

- Dialog keydown listenerを一元管理
- キャンセル / 背景クリック / Esc / 日付変更でも確実にcleanup
- `parseTimeToMinutes()` を堅牢化
- 不正時刻文字列を明示エラー
- 既存予約側の壊れた時刻にもNumber.isFiniteガード

判定：

**CLOSED / PASS**

---

# 7. Phase 2C-4 本保存

## 保存フロー

1. 入力Validator
2. `state.reservations`で一次重複判定
3. isSaving=true
4. 保存ボタンdisabled
5. サーバーから最新予約GET
6. 最新状態で重複再確認
7. `kintone.api()` POST
8. 保存成功表示
9. 当日予約再取得
10. 件数更新
11. 約500ms後にDialog close
12. 選択解除
13. Timeline再描画

## POSTするフィールド

- RESERVE_DATE
- RESOURCE_CODE
- START_TIME
- END_TIME
- RESERVED_BY
- PURPOSE
- NOTE

POSTしない：

- RESOURCE_NAME
- RESOURCE_TYPE
- RESOURCE_ORDER

理由：

RESOURCE_CODE lookupで自動反映させる。

---

# 8. Phase 2C-4で見つかった不具合と修正履歴

## v0.2.6

保存後処理で未定義関数を使用：

- `ReservationService.fetchByDate()` → 誤
- `updateSummary(ui)` → 未定義

POST自体は成功するが、
後処理例外で「保存失敗」表示となる設計ミス。

## v0.2.7

修正：

- `ReservationService.getByDate()`
- `ui.status.textContent = ...`

さらに `destroyTimeline()` に `closeReservationDialog()` を復元。

## v0.2.8

副作用：

`renderTimeline()` → `destroyTimeline()` → `closeReservationDialog()`
により成功表示が見える前にDialogが閉じる。

修正：

`renderTimeline(ui)`を500ms後へ移動。

## v0.2.9

実機で：

`Can't find variable: ui`

が発生。

原因：

`openReservationDialog()` のスコープに `ui` がないのに、
保存後処理で

- ui.status
- clearDragPreview(ui)
- renderTimeline(ui)

を使用していた。

修正：

`openReservationDialog(ui, selection)`

としてuiを明示的に渡す。

---

# 9. v0.2.9 実機結果

v0.2.8時点の最初の保存は、
エラー表示が出たもののPOST自体は成功していた。

その証拠として、
同じ時間帯を再保存しようとした際に、

`既存予約と重複しています: 09:15〜10:45 / つか｜うちあわせ`

が表示された。

v0.2.9へ修正後、
別の空き時間へ新規予約を保存。

実機スクリーンショットで：

- 新規予約バーがTimelineへ表示
- 件数が `7施設 / 7件` へ更新
- 既存予約も維持
- 行高崩れなし
- 複数予約者色分け維持

を確認。

追加された例：

`たにやま｜外注管理`

第一応接室へ正常表示。

したがって管理者権限では、

**POST → 再取得 → Timeline即時反映までPASS候補**

---

# 10. 既知の軽微事項

保存成功後500ms以内に日付変更した場合、
遅延予約されていたrenderTimeline()がもう一度走る可能性がある。

ただし最終的には新しいselectedDateで再描画されるため、
表示内容が壊れるわけではない。

現時点では修正不要と判断。

---

# 11. 次回最優先

## 一般ユーザー権限試験

本番Phase 2C-4はAPIトークンではなく、
ブラウザ上の `kintone.api()` セッション認証。

確認すべき権限：

### App 291
- レコード追加：可
- 編集：不可（現暫定仕様）
- 削除：不可（現暫定仕様）

### App 290
- レコード閲覧：可

確認項目：

1. 一般ユーザーで日次タイムライン表示
2. ドラッグ
3. Dialog入力
4. Validator
5. 保存
6. RESOURCE_CODE lookupが自動反映
7. RESOURCE_NAME / TYPE / ORDERコピー
8. 保存後Timeline即時反映
9. 標準一覧へレコード存在
10. 一般ユーザーが編集・削除できない
11. 権限エラー時のメッセージが理解可能

この試験がPASSすればPhase 2C-4を正式CLOSE候補とする。

---

# 12. Claudeへのレビュー依頼

以下を確認してほしい。

1. v0.2.9の保存フローにコード上の問題が残っていないか
2. `openReservationDialog(ui, selection)` によるui受け渡しは適切か
3. 保存直前の再取得+重複再判定設計は十分か
4. 保存後 `getByDate()` → Timeline再描画の流れは妥当か
5. 500ms遅延再描画の軽微事項を現状放置してよいか
6. 一般ユーザー権限試験項目に不足がないか
7. Phase 2C-4を一般ユーザーPASS後にCLOSEしてよいか
8. 次工程として編集・キャンセル・月間表示等のどれを優先すべきか

特に、今後大きな機能追加へ進む前に、
現行v0.2.9を「新規予約のMVP完成版」とみなしてよいかを評価してほしい。

---

# 13. 添付ファイル

- reservation_phase2c4_v0.2.9.js
- reservation_phase2c4_v0.2.9.css
- README.md
- Phase 2A〜2C-4 実機スクリーンショット一式

---

# 14. 現在の自己判定

Phase 2A：CLOSED / PASS  
Phase 2B：CLOSED / PASS  
Phase 2C-1：PASS  
Phase 2C-2：PASS  
Phase 2C-3：CLOSED / PASS  
Phase 2C-4：管理者権限 PASS候補 / 一般ユーザー試験待ち

次回は一般ユーザー権限試験から再開。
