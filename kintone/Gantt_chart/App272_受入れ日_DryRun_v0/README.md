# App272 受入れ日 DryRun v0.1

## 内容

- `Compare-App272AcceptanceDryRun_v0.1.ps1`
  - PROD App272 READ ONLY
  - 受入実績Excelと発注番号で突合
  - GETのみ
  - PUT/POST/DELETE/PATCHなし
- `App272_受入れ日_DryRun_v0.1_詳細仕様書_20260923.md`
- `DryRun_v0.1_分類一覧.md`
- `STATIC_CHECK.txt`
- `SHA256SUMS.txt`

## 重要

この処理は基幹システムに**入力済み**の受入実績だけを利用します。

現場で受入済みでも基幹入力が省略されている場合、
この処理では検知できません。

したがって、

`NO_SOURCE_DATA_UNKNOWN` は「未受入」を意味しません。

## 実行

```powershell
.\Compare-App272AcceptanceDryRun_v0.1.ps1 `
  -InputXlsx "C:\HPDB\受け入れ情報3か月.xlsx" `
  -TokenPath "C:\HPDB\Secrets\App272_Prod.token"
```

出力先既定:

`C:\HPDB\Logs`
