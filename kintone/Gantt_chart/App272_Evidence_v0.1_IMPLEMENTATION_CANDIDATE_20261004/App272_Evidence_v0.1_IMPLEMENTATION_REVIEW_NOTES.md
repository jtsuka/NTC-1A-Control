# App272 Evidence v0.1 実装レビュー要点

## 実装したもの
`Export-App272AcceptanceEvidence_v0.1_CANDIDATE.ps1`

- App293/App272へはGETのみ（WRITE=0）。
- 現行Heatmap JSのAcceptanceSync 7分類をPowerShell側で再現。
- 全件Analysis CSV、未受入期限超過CSV、Summary TXTを出力。
- RunId JSONLを読み、`UPDATED_THIS_RUN`を
  PRE_EXECUTE + PUT_SUCCEEDED + POST_VERIFY_PASS + Before空欄 + After=Candidate
  でのみ認定。
- SOURCE_ONLYのApp272固有列は空欄。
- App293ReceiptNumbersはソート済み`;`区切り。
- 空欄日付は期限超過にしない。非空欄不正日付はFail Closed。
- CSV再読込ExportVerify、SHA-256、同名非上書き。
- JSONL末尾破損を検知し正式EvidenceはFail Closed。
- `-ReadOnlyTest`ではRunLogなしでGET/分析/CSV生成を試験できるが、
  Summaryへ `READ_ONLY_TEST_NON_FORMAL` と明示する。

## 重要な実装上の発見（Phase 1のArchitecture Gate）
現行AcceptanceSyncはkintoneブラウザJavaScriptです。
ブラウザJavaScriptから `C:\HPDB\...jsonl` のようなWindowsローカルファイルへ、
無人・追記専用で直接書き込むことは通常のWebセキュリティモデル上できません。

したがって、v0.3.1の「AcceptanceSync Execute周辺からJSONLへ追記」を
そのまま実装したと称するのは不正確です。

Phase 1は次のいずれかを先に決める必要があります。
1. kintone内に監査ログAppを用意してイベントを書き、PowerShellがGETしてJSONL化する。
2. 社内HTTPエンドポイント/ローカル補助サービスへイベントを送る。
3. ブラウザ側ではRun結果をダウンロードさせる（ただしクラッシュ途中の耐障害性はv0.3.1要件を満たしにくい）。

現時点では安全のため、Heatmap JSのPUT経路には変更を加えていません。

## 最初の実機試験
正式証跡ではなくREAD ONLY Gateとして実行:
```powershell
$token = Read-Host "App272/App293 READ ONLY API Token" -AsSecureString

.\Export-App272AcceptanceEvidence_v0.1_CANDIDATE.ps1 `
  -Subdomain "<your-subdomain>" `
  -ApiToken $token `
  -ReadOnlyTest `
  -OutputRoot "C:\HPDB\App272_Export\AcceptanceEvidence_TEST"
```

注意:
- 1つのAPI TokenでApp293とApp272の両方をGETできる権限が必要。
  別トークン運用なら次版で `-SourceApiToken` / `-TargetApiToken` に分離する。
- このGateではPOST/PUT/DELETEを実行しない。
