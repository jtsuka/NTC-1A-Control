# 入力CSV作成スクリプト（Make-ShipUnbilledCsv）提出物 2026-10-22

RK-10シナリオの「未計上明細_218001_YYYYMMDD.csv」を作る部分（手順41〜66のExcel操作）を置き換えるPowerShell。
仕様は `仕様書_Make-ShipUnbilledCsv_20261022.md`。Windows PowerShell 5.1。

## 人間側のローカル試験の手順（navicsv に書き込まない）
1. このフォルダ一式を任意の場所へ置く（例 `D:\test\MakeCsv`）。
2. 元データ `未計上明細（全件）_PRONES.csv` と、比較用のRK-10出力 `未計上明細_218001_YYYYMMDD.csv`（同日のもの）を用意する。
3. 実行（元データの更新日が当日でないときは `-AllowStaleSource` が必要）:
```
powershell -NoProfile -ExecutionPolicy Bypass -File .\Make-ShipUnbilledCsv.ps1 -Date 20261022 -PrevBusinessDay 20261021 -SourceCsv "D:\test\in\未計上明細（全件）_PRONES.csv" -OutputDir "D:\test\out" -AllowStaleSource -CompareWith "D:\test\in\未計上明細_218001_20261022.csv"
```
   ログに「完全に一致（バイト単位）」と出れば合格。終了コード 0=作成、10=休日、20=元データなし、22=元データが当日でない、40=比較で差あり。
4. 一式の自動試験（`Run-LocalTest.ps1`）は、navicsv への読み取りができる環境で、`-Date` と `-PrevBusinessDay` を指定して実行する。

## 含まれるもの
- `Make-ShipUnbilledCsv.ps1` … 本体（UTF-8 BOM付き）
- `Run-LocalTest.ps1` … 試験の自動実行（7試験）
- `仕様書_Make-ShipUnbilledCsv_20261022.md` … 仕様書
- `Evidence\LocalTest_20261022_*` … 2026-10-22 の試験の証跡（試験条件・結果表・ログ・SHA256SUMS）
- `SHA256SUMS.txt` … 本体のハッシュ
業務データ（CSV）は含まない。証跡の出力CSVは、結果表のSHA256（基準 4F7B93B4…B018 と一致）で代えている。