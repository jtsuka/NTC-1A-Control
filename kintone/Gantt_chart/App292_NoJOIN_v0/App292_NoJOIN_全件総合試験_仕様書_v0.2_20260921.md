# App292 NoJOIN 全件総合試験 仕様書 v0.2
作成日: 2026-09-21  
対象環境: **Windows PowerShell 5.1 / C:\HPDB**  
対象: kintone TEST App292  
レビュー先: Claude

## 0. v0.2変更点

Claude v0.1レビュー指摘を反映した。

- BLOCKER対応: 標準 `Import-Csv` を廃止し、`TextFieldParser` ベースの `Import-CsvAllowDuplicateHeaders` を採用
- CP932を明示
- 重複ヘッダは `__DUP2`, `__DUP3` のように一意化
- 100件単位POSTの途中失敗時は、App292を全削除して0件確認後に最初から再実行する方針を明文化
- PostVerifyへK系5項目を追加
- すべて **Windows PowerShell 5.1用 `.ps1`** とし、既定パスを `C:\HPDB` に統一

## 1. テスト用ファイル配置

以下を `C:\HPDB` に配置する。

```text
C:\HPDB\Clear-App292-TestData_v0.2.ps1
C:\HPDB\Prepare-App292-NoJoinMasterOptions_v0.2.ps1
C:\HPDB\Import-App292-NoJoin_v0.2.ps1
C:\HPDB\Run-App292-NoJoin-Test_v0.2.ps1
C:\HPDB\外注課納期管理_NoJOIN想定_全937件_App292_TEST_20260921.csv
C:\HPDB\Secrets\App292_Test.token
```

App292のAPIトークンは既存のDPAPIファイルをそのまま使う。

## 2. NoJOIN元データ

`NoJOIN-CSV(1).xlsx` を対応表に従って日本語列名へ変換した想定CSV。

- 937件
- 141列
- 発注番号重複: 0
- 手配書番号あり: 327
- 手配書番号なし: 610
- 注文伝票番号空欄: 0
- 重複ヘッダ `日本語未定（内部項目）`: 12列

## 3. 新I/Fのキー

| CSV | PRONES/SQL | App292 |
|---|---|---|
| 発注番号 | I_PO_DETAIL_NO | 数値 |
| 手配書番号 | 手配番号 | ODERNO |
| 注文伝票番号 | I_PO_SLIP_NO | 数値_44 |
| 注文伝票行番号 | I_PO_SLIP_LINE_NO | 数値_45 |
| 商品名 | I_ITEM_DESC | 商品名 |
| 発注数 | I_PO_QTY | 数値_10 |
| 手配日 | I_PO_DATE | 日付 |
| 納期 | I_DEL_DATE | 日付_1 |
| 指示先名 | 指示先名 | VENDOR_TEXT / VENDOR_DD |
| 手配担当者名 | 手配担当者名 | STAFF_TEXT / STAFF_DD |

使用しない:
- App270
- ルックアップ
- App270 APIトークン

## 4. スクリプト

### `Run-App292-NoJoin-Test_v0.2.ps1`
Windows環境のプリフライト専用。書込みなし。

### `Prepare-App292-NoJoinMasterOptions_v0.2.ps1`
CSVの外注先/担当者をApp292 dropdown optionと照合する。
既定DryRun。

### `Clear-App292-TestData_v0.2.ps1`
App292を空にする。
既定DryRun。

### `Import-App292-NoJoin_v0.2.ps1`
937件をApp292へ直接POSTする。
既定DryRun。

## 5. 部分POST失敗時

937件は100件単位でPOSTする。
途中で失敗した場合、成功済みバッチは残る。

**部分再開は禁止**。

復旧:
1. Clear DryRun
2. Clear Execute
3. App292=0件確認
4. Import DryRun
5. 937件を先頭から再Execute

## 6. K日程

v0.2はNoJOIN I/F試験と日程アルゴリズムを分離する。

TESTのみ:
- K加工着手日 = 手配日
- K完成検査日 = 納期
- K日程種別 = Batch
- K暫定設定 = ON
- K計算根拠 = 空欄

PostVerifyでもこの5項目を確認する。

## 7. 推奨順序

まず `C:\HPDB` で:

```powershell
.\Run-App292-NoJoin-Test_v0.2.ps1
.\Prepare-App292-NoJoinMasterOptions_v0.2.ps1 -DryRun
.\Clear-App292-TestData_v0.2.ps1 -DryRun
.\Import-App292-NoJoin_v0.2.ps1 -DryRun
```

**Claude再レビューPASS後にのみExecuteへ進む。**

## 8. 合格条件

- CSVRows=937
- UniquePO=937
- HandOrderBlank=610
- OrderSlipPresent=937
- App270アクセス=0
- ルックアップ書込み=0
- App292投入後=937
- PostVerifyMismatch=0
- ヒートマップで「手配書番号なし」＋注文書番号が表示される
