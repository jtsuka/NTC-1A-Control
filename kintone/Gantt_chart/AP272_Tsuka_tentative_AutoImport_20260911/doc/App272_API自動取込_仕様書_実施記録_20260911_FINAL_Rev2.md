# App272 API自動取込 仕様書・実施記録

**確定版（Phase 1B / 1C PASS）**\
記録日: 2026-09-11

## 1. 目的

PRONES/RK-10側で正規化した外注課納期管理CSVを基準に、kintone
App272へ差分取込し、既存の手入力を保護しながら外注日程を管理する。

## 2. 対象

-   App272: 外注課発注データ-TEST
-   App270: Lookup参照台帳
-   入力CSV: `外注課納期管理_yyyyMMdd.csv`
-   日程マスタ: `App272_ScheduleMaster_Draft_20260911.csv`
-   会社カレンダー: `カレンダー(2023).xlsx`
-   日本祝日: `JapanHolidays_2026_2027.csv`

## 3. 確定した基本同期仕様

### App272キー

-   発注番号: field code `数値`
-   upsert判定は発注番号で行う。

### 基幹5項目

既存レコードは次の5項目だけを比較し、差分がある項目だけ更新する。 -
`ODERNO` - `VENDOR_TEXT` - `VENDOR_DD` - `日付` - `日付_1`

### 指示先名

CSV `指示先名` を正とし、次の2フィールドへ同期する。 - `VENDOR_TEXT` ---
旧来/基幹互換 - `VENDOR_DD` --- プラグイン/日程管理用

2026-09-11の実データ検証では、既存4件は `VENDOR_TEXT`
がCSVと一致し、`VENDOR_DD` のみ空欄だった。4件とも
`BaseDiffFields=VENDOR_DD` を確認後、実PUTした。

## 4. App270 / Lookup仕様

-   App270の手配書番号 field code: `文字列__1行_`
-   App272正式手配書番号: `ODERNO`
-   App272旧Lookup: `ルックアップ`
-   既存App270は更新しない。
-   App270にODERNOが存在することを確認してからApp272 Lookupを同期する。
-   App272新規POSTでLookupを設定する場合、App272トークンだけでは参照解決できなかった。
-   **App272 + App270 のAPIトークンを同一 `X-Cybozu-API-Token`
    ヘッダーに送信する**ことで正常POSTを確認した。
-   App270は今回、閲覧権限でLookup参照に使用。App270自体の編集は行っていない。

## 5. K日程保護仕様

`K日程種別` の優先順位: **Manual \> Batch \> Auto**

-   Manual: 人が直接編集。自動処理は上書き禁止。
-   Batch: PowerShell/定時処理。画面Autoより優先し保護。
-   Auto: kintone画面側JSによる自動計算。承認済みBatchで置換可能。
-   空欄: 条件を満たせばBatch初期設定可能。
-   未知の種別: 保護して自動更新しない。

日程関連: - `K加工着手日` - `K完成検査日` - `K日程種別` - `K暫定設定` -
`K計算根拠` (`Company` / `Provisional`)

## 6. 日程計算

-   完成検査日: 納期から指定暦日数を引き、非営業日なら前営業日へ補正。
-   加工着手日:
    補正後完成検査日から指定暦日数を引き、非営業日なら翌営業日へ補正。
-   `加工着手日 <= 完成検査日 <= 納期` を必須とする。
-   手配日より加工着手日が前なら `ShortLeadTime` とし書込禁止。
-   会社カレンダー範囲外は週末＋日本祝日で暫定計算し
    `K計算根拠=Provisional`。
-   マスタ `使用`: Batch書込可。
-   `要確認`: 計算表示のみ、書込禁止。
-   `使用しない`: 自動日程なし。
-   その他: 異常として保護。

## 7. CSV日付正規化

基幹システム由来の `yy-MM-dd` は `20yy-MM-dd` と明示変換する。  
.NET/OSの `TwoDigitYearMax` には依存しない。  
出力は `yyyy-MM-dd` に統一する。

### 7.1 2桁年の方針変更理由

初期検討（2026-09-08〜09）では、実運用経路で2桁年が実際に出力されるか未確認だったため、`yy-MM-dd` を検出した場合は世紀を推測せず、安全停止・要確認とする方針としていた。

その後、2026-09-11の実データ確認で、基幹システム自身が `26-06-15` のような2桁年を継続的に出力しており、258行中2346セルがこの正規化対象になることを確認した。これにより「2桁年が実運用データとして存在する」ことが確定したため、当該業務で扱う現行データは2000年代の日付であることを前提に、`20` を明示補完して `yyyy-MM-dd` へ正規化する方針へ変更した。

この変更は、OS/.NETの世紀推測に任せて動作させるものではなく、業務仕様として `20yy` を明示的に確定するものである。将来、1900年代等を扱う可能性が生じた場合は、この前提を見直し、範囲チェックまたは4桁年入力へ変更する。

2026-09-11実績: 258行、日付変換2346セル、列ずれ補正0。

## 8. 2026-09-11 実施結果

### Phase 1B --- 既存App272 PUT

DryRun: - CSV: 258 - App272: 782 - Update(BaseOnly): 4 - Batch日程候補:
25 - Planned PUT: 29 - 新規App272: 1（ブロック） - Record errors: 0 -
Warnings: 0

Execute: - 29件すべてPUT成功 - 25件: Batch日程5フィールド - 4件:
`VENDOR_DD` のみ - `EXECUTE COMPLETE: 29 件`

再DryRun: - Update(BaseOnly): 0 - NOCHANGE: 257 - Protected Batch: 25 -
Auto→Batch: 0 - Blank→Batch: 0 - Planned PUT: 0 - Record errors: 0 -
Warnings: 0

**判定: Phase 1B 正式PASS / 冪等性確認済み**

### Phase 1C --- 新規App272 POST

対象: - Row 194 - 発注番号: `2609248361` - ODERNO: `T268055630` -
指示先: `橋本産業㈱`

v0.1: - App272トークンのみでPOST - `GAIA_LO04` ---
Lookup参照先を解決できず失敗 - レコードは作成されず

v0.2: - App272 + App270トークンを同一ヘッダーで送信 - POST成功 - App272
record id: `3486` - revision: `1`

最終再DryRun: - App272Records: 783 - New candidates: 0 -
Update(BaseOnly): 0 - NOCHANGE: 258 - Lookup sync: 0 - Planned POST: 0 -
Record errors: 0 - Warnings: 0

**判定: Phase 1C 正式PASS / 新規作成・Lookup・冪等性確認済み**

## 9. 最終状態

2026-09-11入力CSV 258件について: - 258件すべてApp272に存在 -
基幹5項目の追加差分なし - Lookup追加同期なし - Batch 25件は保護状態 -
新規候補0 - Record errors 0 - Warnings 0

残件: - `Existing Auto schedule warnings = 1`
は以前から確認済みの既存Auto異常で、今回の取込未完了ではない。 -
日程マスタの `要確認` 162件は承認待ちであり、自動書込しない。 -
JS側のBatch保護は別途回帰確認する。

## 10. 本番統合版の安全原則

1.  既定はDryRun。
2.  Executeには明示確認文字列を要求。
3.  新規件数・更新件数に上限を設ける。
4.  App270追加候補、未知の日程種別、無効なマスタ状態、Record errors
    があれば安全停止。
5.  既存は差分項目だけPUT。
6.  新規は発注番号の重複を再確認してPOST。
7.  Lookupを伴うPOST/更新ではApp272 + App270トークンを送信。
8.  Manual/Batch保護を最優先。
9.  Execute後は必ず再GET/DryRunし、追加差分0を確認。
10. ログに計画・実行結果を残す。

## 11. 使用版

-   CSV正規化: `Format-KonyuCSV_RFC_yyyyMMdd_v3.ps1`
-   DryRun確定: `Import-App272_DryRun_v1.3.ps1`
-   既存PUT PASS: `Import-App272_Execute_v0.1.ps1`
-   新規POST PASS: `Import-App272_Phase1C_New_v0.2.ps1`

以上。
