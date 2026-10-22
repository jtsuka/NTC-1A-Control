# 船舶値付 FINALログ 実装キット v1.0

まず読む:
1. `実装手順書_FINALログ_v1.0.md`
2. `VBA最小挿入箇所チェックリスト.md`

ファイル:
- `modCheckLog.bas` : 検証コピーへImportするVBA logger
- `Print-ValueTag_..._CheckLogExt.ps1` : 正式v0.10.9のCheckLogだけ拡張した検証専用PS
- `Compare-FinalCheckLog_v1.0.ps1` : VBA Text ⇔ PS Value / LineNoを機械比較
- `REFERENCE_ONLY_...ps1` : 正式版参照用。変更禁止
- 原典xlsm : 参照用。変更禁止
- `SHA256SUMS.txt`

最初のゴールはT0 PASS。
T0がPASSするまでFINAL比較へ進まない。
