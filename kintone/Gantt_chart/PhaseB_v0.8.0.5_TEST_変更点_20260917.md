# Phase B v0.8.0.5 TEST 変更点

## 背景
U2 Executeで `GAIA_LO04`。
App270には `T268059011` が実在し、App270 READ用APIトークンで取得できることを確認済み。

kintone公式仕様では、ルックアップフィールドを含むレコード登録/更新でAPIトークン認証を使う場合、
ルックアップ先アプリとルックアップ元アプリの複数APIトークンを指定できる。

## v0.8.0.5
- App292 token: 更新対象App用
- App270 token: ルックアップ元 READ ONLY用
- ルックアップ変更があるExecute時のみApp270 tokenを入力
- PUT直前にApp270へREAD ONLY照合
- 値がApp270に1件存在しなければSafetyStop
- PUTの `X-Cybozu-API-Token` は `App292Token,App270Token`
- App292 GET / post verifyは従来どおりApp292 tokenのみ
- App270へのWRITEは一切なし

U2期待:
- MANUAL_BASE_ONLY
- changedFields = ODERNO,ルックアップ
- K日程系不変
- App270 source precheck PASS
- PUT PASS
- post reGET PASS
