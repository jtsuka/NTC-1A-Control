# 船舶値付用紙－FUJIFILM複合機ジョブ履歴API連携 詳細仕様書

作成日：2026年9月16日
版：Rev.5（ページング仕様確定版）
対象：船舶値付用紙印刷システム（Print-ValueTag.ps1、[[fixreorderpoint-rk10|RK-10]]連携）
対象複合機：FUJIFILM Apeos C3570（船舶機械部設置、IPアドレス 192.168.0.209）

Rev.1（概要仕様書）からの変更点は「9. 改訂履歴まとめ」にまとめてある。Rev.2はClaudeの実機DevTools調査結果、Rev.3・Rev.4はチャッピー（ChatGPT）によるクロスレビュー結果、Rev.5は実機での無限スクロール操作によりページング仕様（`offsetJobID`カーソル方式）が確定したことを反映している。

---

## 1. 目的

船舶値付用紙のVBA → PowerShell移行に伴い、RK-10およびWindows側での処理完了確認だけでなく、実際の出力先であるFUJIFILM複合機側でも印刷ジョブの実行結果を確認できる仕組みを設ける。

処理段階を次の3つに分離して確認する。

1. RK-10がPowerShellを正常起動した
2. PowerShellがWindows印刷スプールへジョブを渡した
3. FUJIFILM複合機がジョブを受信・処理した

第3段階の確認方法として、「人がブラウザーを見る」に加えて「PowerShellがAPIを直接叩いて自動照合する」を実装候補とする。ただしRev.3時点では、時刻仕様など未確定の要素が残るため、両方式を並行して残す設計とする。

---

## 2. 基本構成

```
基幹システム
  ↓
RK-10
  ↓
対象CSV生成
  ↓
\\192.168.200.5\navicsv
  ↓
Print-ValueTag.ps1
  ↓
Windows Print Spooler
  ↓
FUJIFILM Apeos C3570
  ↓
印刷処理
  ↓
複合機内部ジョブ履歴（jobs/api/job-list）
  ↓
PowerShellがHTTP POSTでポーリング照合（Rev.3で方式確定）
```

FUJIFILM複合機側の履歴確認は、RK-10およびPowerShellとは独立した確認経路とする。

---

## 3. 実機検証で判明したジョブ履歴API

### 3.1 エンドポイント

```
URL   : http://192.168.0.209/jobs/api/job-list?methodName=GET&limit=25&typeFilter=ALL
Method: POST （URLのmethodName=GETはアプリ内部の論理区分であり、実際のHTTPメソッドはPOST）
Content-Type（レスポンス）: application/json
Content-Length（リクエスト）: 0（ボディなしで送信されている）
```

DevToolsのRequest headersを確認した限り、**Cookieヘッダーは付与されていない**。未ログイン状態（画面右上が「ログイン」表示のまま）でこのAPIが200 OKで応答しており、機械管理者ログインや認証トークンなしでジョブ一覧を取得できている。

PowerShellからの呼び出し例：

```powershell
$uri = "http://192.168.0.209/jobs/api/job-list?methodName=GET&limit=25&typeFilter=ALL"
$result = Invoke-RestMethod -Uri $uri -Method Post -ContentType "application/json"
```

### 3.2 レスポンス構造（トップレベル）

```json
{
  "Jobs": [ { "JobInfo": { ... } }, ... ],
  "Next": true,
  "Supported": true,
  "TypeFilterSupported": ["ALL", "PRINT", "COPY", "SCAN", "FAX", "..."]
}
```

`Next: true` はさらに古いジョブが存在することを示している。ページングの詳細パラメータはRev.5で確定した（3.10参照）。

### 3.3 レスポンス構造（`JobInfo`、プリントジョブの例）

実際に取得できたJSON（一部抜粋、値は実機ログより）：

```json
"JobInfo": {
  "JobID": 677850,
  "RootJobID": -1,
  "UserJobType": "PRINT",
  "DocumentNumber": 5464,
  "UserName": "RPA-5F",
  "UserID": "RPA-5F",
  "UserType": "NOACCOUNT",
  "AppSubID": "",
  "DetailPermission": true,
  "State": "COMPLETED",
  "Created": "2026-09-16T02:49:28Z",
  "Completed": "2026-09-16T02:50:03Z",
  "RestrictionCancel": false,
  "RestrictionPromote": false,
  "SettingPrintColor": "MONOCHROME",
  "SettingPrintPlex": "SIMPLEX",
  "SettingMediumType": "unspecified",
  "SettingMediumColor": "WHITE",
  "SettingTrayName": "1 TRAY",
  "SettingMediumSize": "A4",
  "SettingMediumSizeDir": "LEF",
  "Collate": true,
  "IsPromoted": false,
  "CopiesRequested": 1,
  "IITOccupied": false,
  "NetInFilename": "@(自動)企画室 船舶値付用紙出力.xlsm",
  "OutputTrayNameId": "0_CENTER_LOWER",
  "FilingPrgPages": 1,
  "PrintInfoContained": true
}
```

### 3.4 各項目とRK-10照合の対応関係（Rev.3：優先順位を見直し）

| 優先度 | JSONキー | 内容 | RK-10照合での用途 |
|---|---|---|---|
| 1（主キー） | `NetInFilename` | 印刷文書名 | PowerShellが設定した一意な識別子との一致（3.8参照） |
| 2 | `UserName` / `UserID` | 送信元PC名（例：`RPA-5F`） | PC名との照合（3.5参照） |
| 3 | `UserJobType` | ジョブ種別 | `"PRINT"`であることの確認 |
| 4 | `CopiesRequested` | 部数 | PowerShellが生成した予定部数との整合確認 |
| 5（補助） | `Created`/`Completed` | ジョブ受信・完了時刻 | 時間帯の粗い絞り込み。時刻仕様は3.7で要確定のため主キーにはしない |
| 6（最終判定） | `State` | `COMPLETED`など | ブラウザー表示「正常終了」の実体（3.6参照） |
| 参考 | `JobID` | 複合機内部の一意なジョブ番号 | ログの一意キーとして保存推奨 |
| 参考 | `DocumentNumber` | 画面表示の「05464」等の番号 | 表示用途のみ、JobIDの方が安定 |
| 参考 | `UserType` | `NOACCOUNT`固定 | Windowsログオンユーザーではないことの裏付け |

Rev.2では時刻を主照合条件に置いていたが、Rev.3では**一意な`NetInFilename`を主キー、時刻は補助条件（粗い絞り込み用）に降格**する（チャッピー指摘・6章参照）。

### 3.5 PC名とユーザー名は別項目ではない

実機では`UserName`と`UserID`が両方とも`"RPA-5F"`（送信元PC名）で、`UserType: "NOACCOUNT"`となっている。この環境では、Windowsのログオンユーザー名は記録されておらず、**PC名がそのままユーザー識別子として使われている**。RK-10実行アカウント（ユーザー名）との突合はできない。実質的な照合キーは「PC名（RPA-5F）」のみである。

### 3.6 `State`の意味の厳密化（Rev.3で訂正）

ブラウザーのUIで「正常終了」と表示されているジョブは、JSON上では`State: "COMPLETED"`。

**この`State == "COMPLETED"`から確実に言えるのは、「Apeosが当該ジョブを完了状態としてジョブ履歴に記録した」ところまでである。** 物理的な紙の排出、出力物の品質、給紙の最終状態まで保証するものではない。「印刷できたことを確認」「紙が正常に排出されたことを保証」という表現は仕様書・ログ・アラート文言のいずれにも使わない。内部ステータス名も`PRINT_CONFIRMED`のような紙の排出を連想させる名称ではなく、`APEOS_JOB_COMPLETED`のように「Apeos側のジョブ履歴上の状態」であることが分かる名称にする（4.3参照）。

異常終了時の`State`値（`"ERROR"`等になるのか）は今回のデータでは未確認。実際に異常終了するジョブを意図的に発生させ、`State`欄がどう変化するかを確認しておく必要がある（8章参照）。

**Rev.4で明記**：`State`には`COMPLETED`以外にも、`WAITING`／`PROCESSING`／`PRINTING`のような**印刷が正常に進行中であることを示す途中状態**が存在する可能性が高い。「異常終了時の`State`値が未確認」という前提に立つ以上、判定ロジック側は`COMPLETED`以外を即座に異常とみなしてはならない。確認済みの異常終了State一覧が定まるまでは、「`COMPLETED`でも確認済みの異常終了Stateでもない値」を暫定的に「処理継続中」として扱う（4.3参照）。

### 3.7 【Rev.3で訂正】タイムスタンプのタイムゾーンは未確定・+9h補正は撤回

Rev.2では`"Created": "2026-09-16T02:49:28Z"`の`Z`をUTCとみなし、+9時間補正してJSTに変換する方針としていたが、**この補正は誤りだった可能性が高いため撤回する。**

根拠：添付スクリーンショットのタスクバー時計と、ブラウザー詳細モーダルの表示時刻を突き合わせると、以下の時系列になる。

```
04:32 → ジョブ一覧を初めて開いた時点（このジョブは既に一覧に存在＝印刷済み）
06:11 → 詳細モーダルで「開始日時: 2026/09/16 02:49」「終了日時: 2026/09/16 02:50」を確認
06:14〜06:20 → DevToolsでAPIの生データを取得。Created="2026-09-16T02:49:28Z"
```

ブラウザーの表示（02:49）とAPIの生値（02:49:28**Z**）は完全に一致しており、この一致自体はブラウザーのJSが値を変換せずそのまま表示していることを示す。さらに、もしこの`Z`を文字通りUTCとして解釈し+9hすると11:49 JSTとなるが、これは調査時点（04:32〜06:20）より未来であり、既に「印刷完了済み」としてジョブ一覧に存在していた事実と矛盾する。

したがって、**この複合機のAPIは実際にはJSTのローカル時刻を、末尾に`Z`を付けたまま（タイムゾーン表記が実態と食い違った状態で）返している可能性が高い**。あるいはブラウザー側のJSが`Z`を無視して数値をそのまま描画しているだけで、APIの生値自体の実際のタイムゾーンは依然不明という可能性も残る。

**Rev.3時点の結論：時刻を照合の主キーにはせず、「値の意味を厳密に確定させるまでは補正なしで扱う」。** 補正コード（`AddHours(9)`）はいったん削除し、`Created`/`Completed`を文字列としてはUTC表記のまま保持しつつ、実際の判定ロジックでは時刻を主キーとして使わない（3.4・4章参照）。

```powershell
# Rev.4：補正を入れない。[datetime]へキャストもしない（Zの解釈がまさに未確定の争点のため）。
# ログには文字列のまま保存し、意味の確定は8章のテストを待つ
$createdRaw   = $info.Created    # 例: "2026-09-16T02:49:28Z"（文字列のまま）
$completedRaw = $info.Completed  # 同上
```

**次に必ず行うべき実機テスト**：PowerShellから1件だけテスト印刷し、以下4つを秒単位で同時記録する。

```
① Windows側の印刷投入時刻（PowerShellのログ）
② Apeosブラウザー画面の表示時刻
③ API `Created`の生値
④ API `Completed`の生値
```

①〜④を突き合わせれば、`Z`の実態（本当にUTCなのか、JSTにZが誤付与されているだけなのか、あるいは別の規則があるのか）を一発で確定できる。この確認が終わるまで、時刻を使った自動判定ロジックは本番投入しない。

### 3.8 文書名の実体キーは`NetInFilename`

ブラウザー表示の「文書名」は、JSON上では`NetInFilename`キーに格納されている。今回のVBA版では、印刷対象のExcelブック名（`@(自動)企画室 船舶値付用紙出力.xlsm`）がそのまま複合機側に記録されていた。

これはRev.1で想定していた「PowerShell移行後にDocumentNameへ識別子を仕込めば複合機側の文書名として残る」という狙いが技術的に成立し得ることの裏付けになる。ただし、`PrintDocument.DocumentName`に設定した文字列が実際に`NetInFilename`へそのまま反映されるかは、PowerShell版でのプリント実行後に改めて実機確認する必要がある。

### 3.9 詳細情報用の別APIが存在する可能性（Rev.3で追加・チャッピー指摘）

Networkログに`job-list`とは別に`jobs-display-model`というリクエストが記録されている。ジョブ一覧画面の表示設定用と推測されるが、これとは別に、**詳細モーダルを開いたときに呼ばれる別のAPI（ジョブ詳細専用エンドポイント）が存在する可能性がある**（前回の調査で見えた`jobs_detail_modalview.html`が怪しい）。

一覧API（`job-list`）で取れる情報は`State`・`CopiesRequested`・`FilingPrgPages`程度だが、詳細APIには以下のようなより強い判定材料が含まれている可能性がある。

- 実出力ページ数
- 終了理由／エラー理由
- 給紙トレイ・排紙結果の詳細
- キャンセル理由

もしこれらが取得できれば、`State == "COMPLETED" AND 実出力ページ数 == 期待ページ数`というさらに強い判定条件を組める。詳細モーダルを開いた際のNetworkログを次回追加調査する（8章）。

### 3.10 ページング仕様（Rev.5で確定）

ブラウザーの［ジョブ］画面は「次へ」ボタンではなく**無限スクロール方式**で、スクロールして表示件数の下限に達するたびに新しい`job-list`リクエストが自動送信される。実機で2回分のリクエストを比較した結果、パラメータの仕組みが確定した。

```
1回目（初回表示）:
  http://192.168.0.209/jobs/api/job-list?methodName=GET&limit=25&typeFilter=ALL

2回目（1回スクロール後）:
  http://192.168.0.209/jobs/api/job-list?methodName=GET
    &offsetJobID=677790
    &limit=25
    &offsetJobIDType=COMPLETED
    &typeFilter=ALL

3回目（さらにスクロール後）:
  http://192.168.0.209/jobs/api/job-list?methodName=GET
    &offsetJobID=677330
    &limit=25
    &offsetJobIDType=COMPLETED
    &typeFilter=ALL
```

`offsetJobID`が677790→677330と実際に小さくなっている（＝より古いジョブへ遡っている）ことを確認した。これにより、**「直前に取得した25件のうち最後（最も古い）ジョブの`JobID`と`State`を、そのまま次のリクエストの`offsetJobID`／`offsetJobIDType`に渡すカーソル方式」**であることが確定した。単純な連番オフセット（`offset=25`のような）ではない。

**注意点**：`typeFilter=ALL`のまま遡ると、`PRINT`以外（`ファクス送信`／`ファクス転送`／`スキャン送信`等）のジョブも混在して返ってくる（2回目のレスポンスで`Content-Length`が22974→25240に増加しているのはこのため）。遡及照合を実装する際は、引き続き`UserJobType == "PRINT"`（またはリクエスト時点で`typeFilter=PRINT`を指定）でのフィルタが必須。

**PowerShellでの遡及取得サンプル**（監査・障害調査用。7章のポーリング用関数とは別のツールとして提供、8章参照）：

```powershell
function Get-ApeosJobHistoryPages {
    param(
        [string]$ApeosIP = "192.168.0.209",
        [int]$MaxPages = 10,
        [string]$TypeFilter = "PRINT"
    )
    $allJobs = New-Object System.Collections.Generic.List[object]
    $offsetJobID = $null
    $offsetJobIDType = $null

    for ($i = 0; $i -lt $MaxPages; $i++) {
        $uri = "http://$ApeosIP/jobs/api/job-list?methodName=GET&limit=25&typeFilter=$TypeFilter"
        if ($offsetJobID) {
            $uri += "&offsetJobID=$offsetJobID&offsetJobIDType=$offsetJobIDType"
        }
        $result = Invoke-RestMethod -Uri $uri -Method Post -ContentType "application/json"
        if ($result.Jobs.Count -eq 0) { break }

        $allJobs.AddRange($result.Jobs)

        if (-not $result.Next) { break }

        $lastInfo = $result.Jobs[-1].JobInfo
        $offsetJobID = $lastInfo.JobID
        $offsetJobIDType = $lastInfo.State
    }
    return $allJobs
}
```

`typeFilter=PRINT`を指定した場合でも`offsetJobID`／`offsetJobIDType`のカーソル方式自体は同じ挙動になる想定だが、`typeFilter`を絞った状態でのページング動作そのものは未検証（`typeFilter=ALL`での動作のみ実機確認済み）。本番の監査用途に使う前に一度、`typeFilter=PRINT`を指定した状態でも同様にページングできることを確認しておくとより安全。

---

## 4. 判定ロジック（Rev.3：ポーリング方式・4状態に変更）

### 4.1 なぜ1回の照会では不十分か

印刷処理は非同期であり、`PowerShell実行 → Windowsスプール → ネットワーク転送 → Apeos受付 → 印刷 → 履歴更新`の各段階に時間差がある。そのため「1回APIを読んで見つからなければ失敗」という実装は誤判定（実際には少し遅れて記録されるだけなのに「失敗」と判定してしまう）を招く。**タイムアウト付きポーリング方式**に変更する。

```
印刷投入
 ↓
2秒待機
 ↓
job-list取得 → 対象ジョブ（NetInFilename一致）を探す
 ↓ 見つからない
2秒待機 → 再取得（タイムアウトまで繰り返し）
```

初期値の目安：`PollingInterval = 2秒`、`Timeout = 60秒`。実測して調整する（8章）。

### 4.2 照合条件（Rev.4：CopiesRequestedの照合漏れを修正）

対象ジョブを「発見」したと判定する条件（＝ポーリングループの中でこの条件に一致したジョブを見つける）：

```
NetInFilename に一意な識別子が含まれる（主キー）
  AND UserJobType == "PRINT"
  AND UserName（PC名）が一致
  AND CopiesRequested が期待値と一致
```

Rev.3のサンプルコードでは、4.2に定義したこの4条件のうち`CopiesRequested`の照合が実装から漏れていた（結果オブジェクトに`Copies`／`ExpectedCopies`を含めていたにもかかわらず、比較条件には入れていなかった）。Rev.4のサンプル（7章）で修正済み。

「発見」した後、`State`の値によって4.3の5分類のいずれかを最終的に返す。時刻（`Created`/`Completed`）は「発見」の絞り込み条件には使わず、ログ保存のみに用いる（3.7参照）。

### 4.3 判定結果の5分類（Rev.4：処理継続中の扱いを追加）

Rev.3では「`State != COMPLETED`」を無条件に`FAILED`としていたが、これは3.6で追記した「途中状態が存在する可能性」および8章の「異常終了時のState値は未確認」という前提と矛盾していた。Rev.4では、対象ジョブを発見した後の`State`を次のように3方向に分岐させる。

```
対象ジョブを発見
  ├─ State == "COMPLETED"                    → APEOS_JOB_COMPLETED
  ├─ State が「確認済みの異常終了State一覧」に含まれる → APEOS_JOB_FAILED
  └─ それ以外（途中状態と推定、または未分類）        → ポーリング継続
        └─ タイムアウトに達した場合            → APEOS_JOB_STATE_TIMEOUT
```

「確認済みの異常終了State一覧」は8章のテストで判明するまで空リストとして扱う（＝現時点では`FAILED`に到達する経路が事実上存在しない設計にしておき、誤って正常進行中のジョブを異常と誤判定するリスクを避ける）。

| 結果コード | 条件 | 意味 |
|---|---|---|
| `APEOS_JOB_COMPLETED` | 対象ジョブが見つかり、`State == "COMPLETED"` | 複合機のジョブ履歴上で正常完了と記録された（紙の排出保証ではない、3.6参照） |
| `APEOS_JOB_FAILED` | 対象ジョブが見つかり、`State`が確認済みの異常終了State一覧に含まれる | 複合機側で異常が発生。JobID・Stateをログ保存し原因調査へ |
| `APEOS_JOB_STATE_TIMEOUT` | 対象ジョブは見つかったが、タイムアウトまで`COMPLETED`にも確認済みの異常終了Stateにもならなかった | Apeosには到達している。複合機内部の処理で止まっている可能性を疑う |
| `APEOS_JOB_NOT_FOUND_TIMEOUT` | タイムアウトまでに対象ジョブ自体が見つからない | Windowsスプーラー〜LAN〜Apeos到達までのどこかを疑う |
| `APEOS_API_ERROR` | `Invoke-RestMethod`自体が失敗（通信不可等） | 複合機との通信そのものに問題。ネットワーク障害等を疑う |

`STATE_TIMEOUT`（Apeosには届いたが状態が確定しない）と`NOT_FOUND_TIMEOUT`（そもそも届いていない）を分けることで、障害発生時に「Apeos側で止まっているのか」「そこまで到達していないのか」を切り分けられる。

---

## 5. 重要な責務分離

| 段階 | 確認内容 |
|---|---|
| 第1段階：RK-10／PowerShell | PowerShell ExitCode = 0 → 帳票生成およびWindows印刷処理への投入が正常終了 |
| 第2段階：Windows Print Spooler | 印刷ジョブをプリンターへ送信 |
| 第3段階：FUJIFILM Apeos C3570 | `job-list` APIのポーリングで4.3の結果コードを判定 |

最も強い確認は、RK-10正常 AND PowerShell ExitCode=0 AND `APEOS_JOB_COMPLETED` の三点一致とする。

---

## 6. ジョブを一意に識別するための識別子案（Rev.3：一意性強化）

基本形：

```
SHIP_VALUETAG_218001_20260916_054501
```

構成：業務識別子＋部門コード＋処理日＋処理開始時刻

同一秒に再実行される可能性まで潰す場合は、末尾に短いランダムID（またはRK-10の実行連番）を付与する。

```
SHIP_VALUETAG_218001_20260916_054501_A7F3
```

`PrintDocument.DocumentName`にこの文字列を設定し、実機の`NetInFilename`にどう記録されるかをPowerShell移行後に確認する。現行VBA版でブック名がそのまま`NetInFilename`に載っていることは確認済みのため、実現性は高いと見てよい。**この識別子が3.4で定めた「照合の主キー」になる。**

照合は完全一致ではなく、前後に文字列が付与される可能性を考慮し、

```powershell
$info.NetInFilename -like "*$ExpectedDocumentName*"
```

のような部分一致で実装する。

---

## 7. 運用フェーズ設計

### 第1段階（現行・フォールバックとして維持）
ブラウザーでの目視確認。［ジョブ］画面→対象ジョブをクリック→詳細モーダルで文書名・PC名・時刻・ページ数・「正常終了」を確認。API側に不具合があった場合の保険として手順は残す。

### 第2段階（本命：ポーリング自動照合）

Rev.4での修正点（チャッピー指摘）：
1. `State != "COMPLETED"`を即`FAILED`にせず、「確認済みの異常終了State一覧」（8章のテストで確定するまでは空リスト）に該当する場合のみ`FAILED`とし、それ以外は途中状態とみなしてポーリングを継続する
2. 4.2で定義した`CopiesRequested`の照合を条件式に追加
3. `Created`/`Completed`を`[datetime]`へキャストせず、文字列のまま保存する（キャスト時に`Z`がUTC指定として解釈され、3.7で保留中のタイムゾーン問題を実装側が先取りして誤判定するのを防ぐため）

```powershell
function Get-ApeosJobStatus {
    param(
        [string]$ApeosIP = "192.168.0.209",
        [string]$ExpectedPCName,
        [string]$ExpectedDocumentName,
        [int]$ExpectedCopies,
        [int]$PollingIntervalSec = 2,
        [int]$TimeoutSec = 60,
        # 8章のテストで確認できた異常終了Stateが分かり次第ここに追加する。現時点では空。
        [string[]]$KnownFailureStates = @()
    )
    $uri = "http://$ApeosIP/jobs/api/job-list?methodName=GET&limit=25&typeFilter=PRINT"
    $deadline = (Get-Date).AddSeconds($TimeoutSec)
    $foundJobSnapshot = $null

    while ((Get-Date) -lt $deadline) {
        try {
            $result = Invoke-RestMethod -Uri $uri -Method Post -ContentType "application/json"
        } catch {
            return [PSCustomObject]@{ Result = "APEOS_API_ERROR"; Error = $_.Exception.Message }
        }

        foreach ($job in $result.Jobs) {
            $info = $job.JobInfo
            if ($info.UserJobType -eq "PRINT" `
                -and $info.UserName -eq $ExpectedPCName `
                -and $info.NetInFilename -like "*$ExpectedDocumentName*" `
                -and ([int]$info.CopiesRequested -eq $ExpectedCopies)) {

                $foundJobSnapshot = $info

                if ($info.State -eq "COMPLETED") {
                    $resultCode = "APEOS_JOB_COMPLETED"
                } elseif ($KnownFailureStates -contains $info.State) {
                    $resultCode = "APEOS_JOB_FAILED"
                } else {
                    # COMPLETEDでも確認済みの異常終了Stateでもない＝途中状態とみなし、まだ確定させない
                    $resultCode = $null
                }

                if ($resultCode) {
                    return [PSCustomObject]@{
                        Result         = $resultCode
                        JobID          = $info.JobID
                        State          = $info.State
                        CreatedRaw     = $info.Created    # 文字列のまま保持。[datetime]キャストしない（3.7参照）
                        CompletedRaw   = $info.Completed  # 同上
                        FileName       = $info.NetInFilename
                        Copies         = $info.CopiesRequested
                        ExpectedCopies = $ExpectedCopies
                    }
                }
                # resultCodeがnull＝途中状態。ループを継続してポーリングし直す
                break
            }
        }
        Start-Sleep -Seconds $PollingIntervalSec
    }

    if ($foundJobSnapshot) {
        return [PSCustomObject]@{
            Result       = "APEOS_JOB_STATE_TIMEOUT"
            JobID        = $foundJobSnapshot.JobID
            State        = $foundJobSnapshot.State
            CreatedRaw   = $foundJobSnapshot.Created
            CompletedRaw = $foundJobSnapshot.Completed
        }
    }
    return [PSCustomObject]@{ Result = "APEOS_JOB_NOT_FOUND_TIMEOUT" }
}
```

### ログに残す項目（Rev.3で新設）

```
ExecutionID
DocumentName（期待値）
ExpectedCopies
ExpectedPC
WindowsPrintStart
JobID
ApeosCreated（生値。タイムゾーン解釈は8章確定まで保留）
ApeosCompleted（同上）
ApeosState
NetInFilename（実値）
CopiesRequested（実値）
ApiCheckStart / ApiCheckEnd
Result（4.3の5分類のいずれか：COMPLETED / FAILED / STATE_TIMEOUT / NOT_FOUND_TIMEOUT / API_ERROR）
```

これにより、後から「RK-10は動いた → PowerShellも正常終了 → ApeosにはJobID 677850として入り → COMPLETEDになった」までを追跡できる。

---

## 8. 未解決事項・次の実機テスト項目

1. **【最優先】時刻仕様の確定**：3.7の手順（①Windows投入時刻／②Apeos画面表示／③API Created／④API Completed を1回の単発テスト印刷で同時記録）を実施し、`Z`の実態を確定する。これが終わるまで時刻を使った自動判定は本番投入しない。
2. **異常終了時の`State`値**：`COMPLETED`以外にどんな文字列が入るか（`ERROR`、`CANCELED`、`ABORTED`等）を実機で意図的に発生させて確認する。判明した値は7章サンプルの`$KnownFailureStates`に追加する。それまでは`APEOS_JOB_FAILED`へは到達せず、`COMPLETED`にならないジョブはすべて`APEOS_JOB_STATE_TIMEOUT`として観測される設計になっている（4.3参照）。
3. **ジョブ詳細専用APIの有無**：詳細モーダルを開いた際のNetworkログを再取得し、`job-list`より詳細な情報（実出力ページ数、エラー理由等）を返すAPIがあるか確認する（3.9）。
4. **認証なしAPIであることのセキュリティ上の扱い**：現状Cookieなし・ログインなしでジョブ履歴（文書名・PC名を含む）が誰でも取得できる状態になっている。社内LANとはいえ、これを許容するか、複合機側のアクセス制限設定を見直すべきか方針を決める。
5. **PowerShell移行後の`PrintDocument.DocumentName`→`NetInFilename`の実継承確認**：現行VBA版での継承は確認できたが、.NET PrintDocument経由でも同様に継承されるかは別途実機テストが必要。
6. **API仕様の非公式性への耐性**：`jobs/api/job-list`は富士フイルムBIの公式ドキュメントに明記されたものではなく、DevToolsから発見した内部APIである。ファームウェア更新でURL・レスポンス構造が変わるリスクに備え、フォールバックとして公式の「ジョブ履歴配信」CSV機能（Apeos C3570は対応機種）とブラウザー目視確認の手順を維持する。
7. **ポーリング間隔・タイムアウト値の実測**：`PollingInterval = 2秒`、`Timeout = 60秒`は初期値の仮置き。実際の投入から履歴反映までの遅延を複数回実測し、余裕を持った値に調整する。
8. **`typeFilter=PRINT`指定時のページング動作**：3.10で確定したカーソル方式（`offsetJobID`／`offsetJobIDType`）は`typeFilter=ALL`での実機確認結果。`typeFilter=PRINT`に絞った状態でも同様に遡及できるかは未検証。監査用途で本番投入する前に確認する。

---

## 9. 改訂履歴まとめ

| 版 | 主な変更 |
|---|---|
| Rev.1 | 概要仕様書。ブラウザー目視確認を中心とした二段階設計を提案 |
| Rev.2 | Claudeが実機DevTools調査を実施。`jobs/api/job-list`エンドポイント・JSON構造・認証不要である点を発見。ただし時刻を+9h補正するUTC仮説を誤って採用 |
| Rev.3 | チャッピー（ChatGPT）がRev.2をクロスレビュー。以下を修正・追加<br>・+9h時刻補正を撤回（証拠と矛盾するため）。時刻仕様は単発実機テストで確定させる方針に変更<br>・`State == "COMPLETED"`の意味を「Apeosジョブ履歴上の完了」に厳密化し、紙の排出保証と混同しない文言に統一<br>・照合の主キーを時刻から一意な`NetInFilename`（DocumentName）へ変更、時刻は補助条件に降格<br>・単発照会からタイムアウト付きポーリング方式へ変更し、判定結果を4分類（COMPLETED/FAILED/NOT_FOUND_TIMEOUT/API_ERROR）に整理<br>・ジョブ詳細専用APIの存在可能性を追加指摘<br>・ページングは主フローの必須要件から外し、監査用途の将来課題に整理 |
| Rev.4 | チャッピーがRev.3のPowerShellサンプルと本文の整合性をレビュー。実装コードはまだ作成せず、仕様書内のサンプル疑似コードのみ以下を修正<br>・`State != "COMPLETED"`を即`FAILED`とする実装が「異常終了State未確認」という8章の前提と矛盾していたため撤回。`COMPLETED`／確認済み異常終了State（判明するまで空リスト）／それ以外（途中状態）の3方向分岐に変更し、判定結果を5分類（COMPLETED/FAILED/**STATE_TIMEOUT**/NOT_FOUND_TIMEOUT/API_ERROR）に拡張<br>・4.2で定義していた`CopiesRequested`の照合がサンプルコードから漏れていたため追加<br>・`Created`/`Completed`を`[datetime]`にキャストしていた箇所を、タイムゾーン仕様確定までは文字列のまま保持する実装に修正（3.7の方針と整合） |
| Rev.5 | 実機のブラウザー操作（無限スクロール）とDevTools調査により、ページング仕様を確定<br>・`offsetJobID`／`offsetJobIDType`によるカーソル方式であることを、2段階のスクロール（677790→677330とJobIDが減少）で実証<br>・`typeFilter=ALL`で遡るとPRINT以外のジョブ種別も混在することを確認し、遡及取得時のフィルタ必須要件を明記（3.10）<br>・監査・遡及照合用の`Get-ApeosJobHistoryPages`関数を仕様書に追加（Check-ApeosPrintJob.ps1本体とは別ツールとして`Get-ApeosJobHistory.ps1`に実装）<br>・8章の未解決事項からページング項目を解決済みとして除去し、`typeFilter=PRINT`指定時の未検証点を新規項目として追加 |

---

## 添付スクリーンショット一覧

| ファイル名 | 内容 |
|---|---|
| 01_ジョブ一覧_未ログイン表示.png | 複合機［ジョブ］画面、未ログイン状態での一覧表示（タスクバー時計：4:32） |
| 02_ジョブ詳細モーダル_初回.png | ジョブ行クリック時の詳細モーダル（正常終了、開始/終了日時、PC名、設定。タスクバー時計：6:11） |
| 03_DevTools_Consoleタブ.png | DevTools Consoleタブ（Networkタブへの切替前の状態。タスクバー時計：6:14） |
| 04_DevTools_Network一覧_Fetch-XHR.png | Networkタブ、Fetch/XHRフィルタ適用後のリクエスト一覧（job-list等を発見。タスクバー時計：6:15） |
| 05_ジョブ詳細モーダル_Network併記.png | 詳細モーダルとNetworkパネルを同時に表示した状態（タスクバー時計：6:16） |
| 06_job-list_APIリクエストHeaders.png | `job-list`リクエストのHeaders（URL、POSTメソッド、Cookieなしを確認。タスクバー時計：6:19） |
| 07_job-list_APIレスポンスResponse生JSON.png | `job-list`レスポンスのResponseタブ（JobInfo生データ。タスクバー時計：6:20） |
| 08_job-list_APIレスポンスPreview構造.png | `job-list`レスポンスのPreviewタブ（Jobs配列、TypeFilterSupported等の全体構造。タスクバー時計：6:20） |
| 09_ジョブ一覧_無限スクロール最下部.jpg | ［ジョブ］画面を無限スクロールで最下部まで送った状態。日付が前日（2026/09/15）まで遡れていることを確認 |
| 10_job-list_ページング1回目_Headers.jpg | 1回目のスクロール後の`job-list`リクエストHeaders（`offsetJobID=677790`） |
| 11_ジョブ詳細モーダル_一番下の行.jpg | 一覧最下部のジョブ（05285番）をクリックした詳細モーダル |
| 12_job-list_ページング2回目_Headers.jpg | さらにスクロール後の`job-list`リクエストHeaders（`offsetJobID=677330`。1回目より小さい値に変化したことを確認） |

※タスクバー時計の記録は、3.7の時刻仕様確定作業の傍証として付記した。
