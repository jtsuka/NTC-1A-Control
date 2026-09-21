# App292 NoJOIN 全件総合試験 仕様書 v0.3 MasterAuto
作成日: 2026-09-22  
対象: Windows PowerShell 5.1 / kintone TEST App292

## 0. 訂正理由

v0.2では、NoJOIN I/Fそのものの成立性確認を優先し、K日程を全件

- K加工着手日 = 手配日
- K完成検査日 = 納期

として投入した。

これは当初仕様からの逸脱だった。**取込時点から `加工先別_日程暫定シート_20260910.xlsx` を参照し、該当加工先が「使用」の場合は前倒しルールを適用する**のが正しい。

実画面でも、例えば `㈱山和（八尾）` の案件が手配日→納期の全区間バーになっており、マスタ前倒しが未適用であることを確認した。

## 1. 流用する既存正式ロジック

`Import-App272_Phase1C_New_v0.7.4_MasterAuto.ps1` から以下を流用する。

- `Import-ScheduleMasterExcel`
- `Import-CompanyCalendar`
- `Import-HolidaySet`
- `Get-DayState`
- `Move-ToWorkingDay`
- `Calculate-ScheduleCompat`
- `Get-MasterKey`

アルゴリズム:

1. 完成検査日基準 = 納期 - 完成検査日前倒し暦日
2. 非営業日なら前営業日へ補正
3. 加工着手日基準 = 完成検査日 - 加工着手日前倒し暦日
4. 非営業日なら翌営業日へ補正
5. 会社カレンダー範囲内は `Company`
6. 範囲外は土日＋日本祝日で `Provisional`

## 2. 判定フロー

```text
CSV 指示先名
  ├─ マスタなし
  │    -> K着手=手配日 / K検査=納期 / K計算根拠=""
  │
  └─ マスタあり
       ├─ 使用
       │    ├─ 正常計算 -> 計算値をK日程へ / K計算根拠=Company|Provisional
       │    └─ ShortLeadTime -> ベースライン / K計算根拠=""
       │
       ├─ 要確認
       │    -> 計算値はPlanログへ出力
       │    -> 実書込みはベースライン / K計算根拠=""
       │
       ├─ 使用しない
       │    -> ベースライン / K計算根拠=""
       │
       └─ その他/不正日数
            -> SafetyStop
```

`K日程種別=Batch`、`K暫定設定=ON` は全937件共通。

## 3. NoJOIN I/Fは変更しない

- 主キー: 発注番号 (`数値`)
- App270: 使用しない
- `ルックアップ`: 書かない
- 手配書番号空欄: 正常値
- 注文伝票番号/行番号を保持

## 4. 現在投入済み937件

v0.2で投入した937件はマスタ未適用のため、**再試験対象**。

再試験手順はClaudeレビューPASS後に:

1. `Clear-App292-TestData_v0.2.ps1 -DryRun`
2. `Clear-App292-TestData_v0.2.ps1 -Execute -ConfirmExecute "APP292-CLEAR-ALL"`
3. App292=0確認
4. `Import-App292-NoJoin_v0.3_MasterAuto.ps1 -DryRun`
5. Planログで日程判定件数と代表案件を確認
6. Execute
7. 937件PostVerifyMismatch=0
8. ヒートマップ目視確認

## 5. 現在のマスタレビュー用コピーから確認できた構成

添付 `加工先別_日程暫定シート_20260910_レビュー用.xlsx` は `加工先別設定` シートを持つ。

NoJOIN 937件との単純な指示先名一致では、レビュー時点で:

- CSVユニーク指示先: 72
- マスタ一致ユニーク指示先: 29
- `使用` に一致するレコード: 299
- `使用しない` に一致するレコード: 2
- マスタ未一致レコード: 636

※これは日程計算前の名称一致集計。最終DryRunでは会社カレンダーと祝日を含めて判定する。

## 6. Planログ

DryRun/Executeとも、以下を `C:\HPDB\logs` に出す。

`App292_NoJOIN_MasterAuto_Plan_yyyyMMdd_HHmmss.csv`

主要列:

- 発注番号
- 指示先名
- RuleStatus
- 検日前倒し日数
- 着手前倒し日数
- PreviewKStart / PreviewKInspection / PreviewKBasis
- KStart / KInspection / KBasis
- ScheduleDecision

`要確認` はPreview値と実書込baselineを同時に比較できる。

## 7. 合格条件

- CSVRows=937
- UniquePO=937
- App270アクセス=0
- Lookup書込み=0
- Schedule不正=0
- Master `使用` の正常計算案件でK日程が前倒し値
- Masterなし/使用しない/要確認/ShortLeadTimeは仕様どおりbaseline
- 計算適用案件のK計算根拠がCompany/Provisional
- baseline案件のK計算根拠が空欄
- App292事後937
- PostVerifyMismatch=0
