# PS CheckLog拡張版 差分説明

基準正式版 SHA256:
`5608015c6898892a9f8f1a8e2c59b53b3e5f3c634993c286f75cc83ac8668de6`

検証専用:
`Print-ValueTag_Ship_v0.10.9_VBAFit_20260924_CheckLogExt.ps1`

追加したもの:
1. bodyOverridesへ診断メタ `__DetailNo`, `__LineNo`
2. Draw-Blockへ診断引数 `Area`
3. CheckLog CSVへ `DetailNo,LineNo,Area,Position,ActualCell`
4. BF1をArea=OTHER、H1をArea=GENKAUMUとして記録

変更していないもの:
- Build-Override
- Apply-DerivedFields / Apply-BodyDerivedFieldsの帳票値決定
- AV5
- BJ12優先順位
- 原価割れ
- Gray8
- ページ分割
- 描画座標計算
- フォント/罫線
- Preview/Print処理

正式版は置換しない。FINAL検証時だけ拡張版を使用する。
