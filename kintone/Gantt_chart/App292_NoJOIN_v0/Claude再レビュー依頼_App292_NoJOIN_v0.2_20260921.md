# Claude再レビュー依頼 — App292 NoJOIN Windows版 v0.2

v0.1レビューでいただいた BLOCKER / MUST / SHOULD を反映しました。

## 今回の追加条件

実際の試験環境は **Windows PC / Windows PowerShell 5.1** です。
関連ファイルは `C:\HPDB` に配置し、App292 APIトークンは既存の
`C:\HPDB\Secrets\App292_Test.token` を使用します。

テスト用コードはすべて `.ps1` に統一しました。

## v0.1指摘への対応

1. 重複ヘッダ対策
   - 標準 `Import-Csv` を廃止
   - `Import-CsvAllowDuplicateHeaders` を採用
   - CP932固定
2. 途中POST失敗時
   - 部分再開禁止
   - App292全削除→0件確認→最初から再実行
3. PostVerify
   - K加工着手日
   - K完成検査日
   - K日程種別
   - K暫定設定
   - K計算根拠
   を追加
4. Windows向け
   - C:\HPDB既定
   - PowerShell 5.1
   - DPAPIトークンファイル使用

## 再レビュー対象

- `Run-App292-NoJoin-Test_v0.2.ps1`
- `Clear-App292-TestData_v0.2.ps1`
- `Prepare-App292-NoJoinMasterOptions_v0.2.ps1`
- `Import-App292-NoJoin_v0.2.ps1`
- `App292_NoJOIN_全件総合試験_仕様書_v0.2_20260921.md`
- `外注課納期管理_NoJOIN想定_全937件_App292_TEST_20260921.csv`

特に、**このv0.2をWindows PowerShell 5.1上でDryRunへ進めてよいか**を判定してください。

判定は BLOCKER / MUST / SHOULD / INFO でお願いします。
