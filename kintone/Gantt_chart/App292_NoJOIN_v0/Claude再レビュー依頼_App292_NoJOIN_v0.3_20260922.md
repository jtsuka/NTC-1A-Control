# Claude再レビュー依頼 — App292 NoJOIN v0.3 MasterAuto

v0.2はREST I/Fとして937件PASSしましたが、K日程仕様に漏れがありました。
今回のv0.3は、その訂正です。

## 最優先レビュー

`Import-App292-NoJoin_v0.3_MasterAuto.ps1` が、既存の
`Import-App272_Phase1C_New_v0.7.4_MasterAuto.ps1` の日程計算と整合しているか確認してください。

特に:

1. `Calculate-ScheduleCompat` 等7関数が既存Phase1Cから正しく流用されているか
2. マスタ `使用` のみ計算値を実書込みする設計が正しいか
3. `要確認` は計算値をPlanログに残し、実書込みはbaselineか
4. `使用しない` / マスタなしはbaselineか
5. ShortLeadTimeはPhase1C互換でbaseline fallbackか
6. `K計算根拠` は計算適用時だけCompany/Provisional、baseline時は空欄でよいか
7. 不正使用区分/不正日数/InvalidDateRelationをSafetyStopする設計に問題ないか
8. App270非依存・Lookup非書込みが維持されているか
9. PostVerifyが実際の予定K値まで検証しているか
10. Windows PowerShell 5.1互換性

## 添付

- `Import-App292-NoJoin_v0.3_MasterAuto.ps1`
- `Run-App292-NoJoin-Test_v0.3.ps1`
- `Clear-App292-TestData_v0.2.ps1`
- `Prepare-App292-NoJoinMasterOptions_v0.2.ps1`
- `App292_NoJOIN_全件総合試験_仕様書_v0.3_20260922.md`
- NoJOIN 937件CSV
- 加工先別日程暫定シート（レビュー用コピー）

**まだClear/Executeは行わず、まずレビューしてください。**

判定は BLOCKER / MUST / SHOULD / INFO でお願いします。
