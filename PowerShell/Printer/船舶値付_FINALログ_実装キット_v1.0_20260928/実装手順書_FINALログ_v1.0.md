# 船舶値付 FINALログ 実装手順書 v1.0
日付: 2026-09-28
対象: 原典VBA → PowerShell v0.10.9 FINAL互換性検証
状態: Claude v0.4 最終レビュー「実装可」反映

## 0. 最重要
原典 `@(自動)企画室 船舶値付用紙出力(8).xlsm` は絶対に編集・保存しない。
最初にWindows Explorerでコピーを作り、例:
`@(検証用_FINALログ)企画室 船舶値付用紙出力_20260928.xlsm`
だけを編集する。

正式PSの基準:
`Print-ValueTag_Ship_v0.10.9_VBAFit_20260924.ps1`
SHA256:
`5608015c6898892a9f8f1a8e2c59b53b3e5f3c634993c286f75cc83ac8668de6`

このキットの `..._CheckLogExt.ps1` は、上記正式版から帳票ロジックを変えず、
CheckLog診断情報 DetailNo/LineNo/Area/Position/ActualCell を追加した検証専用版。

---

# Phase 1: 検証コピーを作る

1. Excelを完全終了。
2. 原典xlsmをコピー。
3. コピー名を `@(検証用_FINALログ)企画室 船舶値付用紙出力_20260928.xlsm` にする。
4. 原典のSHA256を保存しておく。
5. 以降は検証コピーだけを開く。

注意:
VBAプロジェクトにデジタル署名がある場合、編集で署名は無効になる。
検証コピーなので問題ないが、原典へ上書きしない。

# Phase 2: modCheckLogをImport

1. 検証コピーをExcelで開く。
2. `Alt + F11` でVBEを開く。
3. 左のProject Explorerで検証コピーのVBAProjectを選択。
4. `File > Import File...`
5. このキットの `modCheckLog.bas` を選択。
6. Modules配下に `modCheckLog` が出たことを確認。
7. `Debug > Compile VBAProject` を実行。
8. この時点でエラーが出たら先へ進まない。

# Phase 3: Run開始/終了を追加

## 3-1 Begin
`Sheet8.cls` の `Worksheet_SelectionChange` で、BT_NO=2が確定しPrintableShapesを呼ぶ直前に、
検証コピーだけ次を追加する。

```vb
If BT_NO = 2 Then
    CheckLog_Begin ThisWorkbook.Path & "\VBA_CheckLog", CStr(FILE_DATE)
End If
```

FILE_DATEがこのスコープから参照できない場合は第2引数を省略する。
FILE_DATEはFlush時にも渡せるため、ここで無理に原典変数の可視範囲を変えない。

## 3-2 End
`PrintableShapes`正常終了部、`KANRI_SKIP:`以降の正常フローで
`SetPrinter("DEF")` の後、Function/Subを抜ける直前に追加:

```vb
If CheckLog_IsEnabled() Then
    CheckLog_End
End If
```

原典へ包括的な `On Error` は追加しない。
途中異常終了では.completeが作られず、そのCSVは比較対象外となる。

# Phase 4: 1手配開始時に登録簿をリセット

`Process.cls` STEP_0 の
```vb
row_count = 0
```
直後へ追加:

```vb
If CheckLog_IsEnabled() Then
    CheckLog_ResetOrder
End If
```

# Phase 5: TOP / HEAD / BODY のTouch

原則:
**セルへ実際に代入した直後だけTouchする。**
空/0等のskip判定で代入しなかった場合はTouchしない。

Positionは必ずsetting表M列の基準アドレス。
RangeCalculation後の実セルはtarget RangeからActualCellとしてloggerが取得する。

## 5-1 TOP
STEP_1の実代入を、意味を変えず次の形にする。

既存:
```vb
WS_TEMP.Range(RangeCalculation(.Range("M" & myCell.Row), ...)) = DATA_DIC(dat)(...)
```

検証コピー:
```vb
Dim chkTarget As Range
Set chkTarget = WS_TEMP.Range(RangeCalculation(.Range("M" & myCell.Row), ...))
chkTarget.Value = DATA_DIC(dat)(...)

If CheckLog_IsEnabled() Then
    CheckLog_Touch "TOP", _
                   CStr(.Range("M" & myCell.Row).Value), _
                   CStr(.Range("N" & myCell.Row).Value), _
                   chkTarget, 0, "0"
End If
```

重要:
- 既存式の `...` 部分は原典のままコピーする。
- 値取得式 `DATA_DIC(dat)(...)` も原典のまま。
- 代入条件・skip条件は変更しない。
- `.Range("N"... )` がSourceId列であることを実コードで再確認してから貼る。
  Claude確認済みのsetting構造を優先し、列が異なる場合は原典のID列を使う。

## 5-2 HEAD
TOPと同じだがAreaだけ `"HEAD"`。

## 5-3 BODY
BODYはDetailNo/LineNoを持つ。

```vb
Dim chkTarget As Range
Set chkTarget = WS_TEMP.Range(RangeCalculation(.Range("M" & myCell.Row), ...))
chkTarget.Value = DATA_DIC(dat)(...)

If CheckLog_IsEnabled() Then
    CheckLog_Touch "BODY", _
                   CStr(.Range("M" & myCell.Row).Value), _
                   CStr(.Range("N" & myCell.Row).Value), _
                   chkTarget, _
                   row_count + 1, _
                   CStr(DATA_DIC(dat)(3))
End If
```

BJ12:
id=12とid=211が同じPositionをTouchしてもよい。
Dictionaryの同一キーが後勝ちし、PrintOut直前FlushでBJ12を一度だけ読み返す。
途中値はCSVへ出ない。

# Phase 6: STEP_5の単発Touch

## 6-1 BF1 取得ファイル日時
原典1867行目前後の
```vb
.Range(dic("取得ファイル日時")) = ...
```
の**代入直後**に追加:

```vb
If CheckLog_IsEnabled() Then
    CheckLog_Touch "OTHER", "BF1", "FILE_DATE", _
                   WS_TEMP.Range("BF1"), 0, "0"
End If
```

## 6-2 H1 原価あり/なし
GENKAUMUをFindし、H1へ実際に代入した直後だけ追加:

```vb
If CheckLog_IsEnabled() Then
    CheckLog_Touch "GENKAUMU", "H1", "FLAG_DIC.genka", _
                   WS_TEMP.Range("H1"), 0, "0"
End If
```

GENKAUMUが見つからずSTEP_6へ抜けた場合はTouchしない。
SPACE(A15)はTouchしない。

# Phase 7: PrintOut直前Flush

Claude確認:
STEP_5から `.PrintOut` の間にセル書込みはなく、
SetPrinter/PageSetupのみ。

したがってPageSetup設定が全部終わった後、`.PrintOut`の直前へ追加:

```vb
If CheckLog_IsEnabled() Then
    CheckLog_FlushOrder WS_TEMP, _
                        BT_NO, _
                        CStr(save_order), _
                        print_count + 1, _
                        CStr(FILE_DATE)
End If
```

注意:
`print_count`がPrintOut後に+1される原典なら `print_count + 1`。
もしPrintOut前に既に+1される実コードなら `print_count`。
これは診断列だけで比較キーではない。

# Phase 8: VBA Compile

すべて挿入後:
1. VBE `Debug > Compile VBAProject`
2. コンパイルエラー0件を確認
3. 保存
4. Excelを閉じる
5. 再度開き、マクロが通常起動できることを確認

ここで原典は開かない/保存しない。

# Phase 9: PS検証専用版

使用:
`Print-ValueTag_Ship_v0.10.9_VBAFit_20260924_CheckLogExt.ps1`

変更点はCheckLog診断だけ:
- DetailNo
- LineNo
- Area
- Position
- ActualCell

帳票描画ロジックは正式v0.10.9と同じ。

最初はPreviewOnly/DocuWorks等、これまで使っている安全な試験方法で実行し、
`-CheckLogPath` を指定する。

例:
```powershell
.\Print-ValueTag_Ship_v0.10.9_VBAFit_20260924_CheckLogExt.ps1 `
  <これまで使用している既存引数をそのまま> `
  -CheckLogPath "C:\HPDB\船舶値付用紙\PS_FINAL_CheckLog.csv"
```

既存引数は勝手に変更しない。

# Phase 10: T0 無侵襲試験【最初に必ず実施】

同一入力で:
A. 無変更原典VBA
B. FINALログ付き検証コピー

を同じDocuWorks経路で出力。

判定:
- 対象手配一致
- ページ数一致
- 印刷順一致
- 表示内容一致
- 改ページ一致
- Bだけ `VBA_CheckLog\*.csv` と `.complete` が生成

1つでも差があればSTOP。
VBA帳票ロジックを直して合わせるのではなく、ログ挿入位置/コードを見直す。

# Phase 11: FINAL値比較

VBA側:
`VBA_FINAL_CheckLog_*.csv`
および同名`.complete`

PS側:
`PS_FINAL_CheckLog.csv`

比較:
```powershell
.\Compare-FinalCheckLog_v1.0.ps1 `
  -VbaCsv "...\VBA_FINAL_CheckLog_YYYYMMDD_HHMMSS.csv" `
  -PsCsv "...\PS_FINAL_CheckLog.csv" `
  -OutDir "C:\HPDB\船舶値付用紙\FINAL_COMPARE"
```

初回は `-TrimEndForCompare` を付けない。

出力:
- FINAL_COMPARE_ALL.csv
- FINAL_COMPARE_DIFF.csv
- FINAL_COMPARE_DUPLICATES.csv

主判定:
- PASS
- VALUE_DIFF
- LINENO_DIFF
- VBA_ONLY
- PS_ONLY

AH1は比較対象外。
PS Page2以降のTOP/HEAD等は同一キー・同一値なら最初の1件へ正規化。

# Phase 12: 末尾空白の扱い

初回T0/FINAL観測で、Excel `.Text` の末尾空白が
NumberFormat `#,##0_);[Red](#,##0)` 等によって実際に出るか確認する。

出ることを確認し「表示上の意味がない書式由来空白」と確定した場合だけ、
比較規則としてTrimEndを固定し、以後:

```powershell
.\Compare-FinalCheckLog_v1.0.ps1 ... -TrimEndForCompare
```

を使う。

結果を見て都合よくON/OFFしない。
一度決めた規則を試験記録へ残す。

# Phase 13: 座標

このCompare v1.0は座標のPASS/FAILを**意図的に判定しない**。
理由:
- VBA = シート絶対座標
- PS = ページ相対 + OffsetX/Y
- PS列幅px丸め累積

まずログを取得して対応点の差分分布を観測。
原点変換式と許容差を決め、それを固定してから座標比較v1.1へ進む。
±0.5ptを最初から合格基準にはしない。

# Phase 14: T-PAGE

FINAL値比較とは別に:
- 10明細以下
- 11明細以上

をVBA/PSでDocuWorks出力。

確認:
- 物理ページ数
- 各ページDetailNo
- PrintTitleRows相当
- AH1/CenterHeader
- 改ページ位置

ここがPASSして初めて物理ページ互換性をCLOSEする。

# STOP条件

次の場合はその場で停止:
- VBA Compile error
- 原典と検証コピーのT0出力差
- `.complete`がない
- DUPLICATE_KEY
- LineNo不一致
- PS拡張版と正式版の帳票出力差
- 想定外のVBA_ONLY/PS_ONLY

ログ比較に合わせて原典VBAの業務ロジックを変更してはいけない。
