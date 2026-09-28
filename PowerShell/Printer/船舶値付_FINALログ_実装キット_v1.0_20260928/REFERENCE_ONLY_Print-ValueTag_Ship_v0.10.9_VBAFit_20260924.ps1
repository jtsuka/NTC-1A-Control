<#
.SYNOPSIS
    船舶値付用紙 印刷 - Excel非依存版 (PoC / v0.10.3)

.DESCRIPTION
    label_layout_full.json (model/settingシートから抽出したレイアウト定義) を読み込み、
    System.Drawing(GDI+) でラベルを描画し、そのまま PrintDocument で印刷スプールへ送る。

    元VBA(Process.PrintableShapes)との対応:
      - MODEL_DIC("top"/"head"/"body")  → $layout.blocks
      - setting シートのTOP/HEAD/BODY/OTHERマッピング → $layout.field_mapping
      - .PrintOut                        → $printDoc.Print()

.NOTES
    ★これはPoC(概念実証)としてスタートしましたが、v0.9.5時点で
    CSV読込・改ページ/Zoom計算・注番グルーピング等の主要機能は実装済みです。
    以下、2026/08/11時点で未実装/要検証の項目:
      - BT_NO別プリンター切替 (元VBA SetPrinter相当) … 未実装、$PrinterName固定
        (船舶(BT_NO=2)専用スクリプトのため優先度低)
      - 原価未入力/売価未入力の警告表示 … v0.10.3 Step2でVBA原典に合わせて実装。
      - AS12のY受注金額注意 … v0.10.3 Step2でVBA原典に合わせて実装。
      - 出荷検収済み明細のGray8網掛け／全明細検収済み手配の除外 … v0.10.3で実装。
      - 一式金額の複合表示(請求/経由/受注をまとめた表示) … 未実装。
        書込み先「経由先２_一式金額」もfield_mappingに見当たらず、表示要否・
        表示位置ともに現場確認待ち。
      - 罫線の実機照合 … Excel列幅(文字単位)→pxの変換式は概算(MDW=7と仮定)。実際の印刷結果と必ず突合すること。

    (旧PoC時代の記述だった「改ページ/Zoom計算は未実装、1ページ内に全て描画する
    簡易版」「MakeData相当のCSV読込等は未実装、サンプルデータ埋め込み」は、
    v0.9.0(改ページ/Zoom)・初期バージョン(CSV読込)で既に実装済みのため、
    現状と矛盾する記述として2026/08/11(v0.9.6)に削除した。天野さんご指摘。)

    実行前提: Windows PowerShell 5.1、.NET Framework(System.Drawing.Common)
    v0.2での変更点:
      - 誤って指定していた Add-Type -AssemblyName System.Drawing.Printing を削除
        (PrintDocument等はSystem.Drawing.dllに含まれるため不要かつ無効なアセンブリ名だった)
      - Bitmap生成前にサイズを診断表示、失敗時は例外の詳細(内部例外・発生位置)を表示するよう変更
      - Bitmapサイズを明示的に[int]キャストして渡すよう変更
    v0.3での変更点:
      - 最終出荷日が yyyyMMdd (例:20260806) の場合も yyyy/MM/dd へ正規化
      - 数量1のとき請求/営業原価/製造原価の「合計」欄を非表示
      - 数量2以上の請求合計・営業原価合計をVBAと同じ注記形式へ整形
    v0.4での変更点:
      - AS12に誤表示されていたDL列32「計画必要数」(例: 1)を非表示化
        ※AS12は元VBAでは警告用の派生セルであり、生データを直接表示しない
    v0.5での変更点:
      - ページ順を明示ソート。既定はVBAの最終成果物（紙束/PDF）を意識した注番昇順(VbaFinal)。
        元VBAの船舶(BT_NO=2)内部では注番降順で個別PrintOutしているため、検証用にVbaSubmit(降順)も選択可能。
      - BODY転記時の「数値0は表示しない」を元VBAどおり全BODY項目へ適用
      - ヤンマー経由仕切り警告を元VBA条件でBJ14へ実装
        経由1仕切=受注仕切: ●〇経由仕切率注意！●〇
        経由1仕切>受注仕切: ■□受注仕切率注意！■□
    v0.6での変更点 (VBA出力とのPDF全38ページ再比較で新規発見):
      - 経由先コード(AQ4)・経由先名(AW4)が"0"のとき生値をそのまま印字していたのを非表示化
        (経由先未設定の受注で「経由先1」欄に単独の"0"が出ていた問題)
      - 分納出荷数(F13)が常に数量と同じ値で印字されていたのを、
        分納出荷数=数量(=分納なし)の場合は非表示にするよう修正
      - [要確認・未対応] 得意先/受注先コードの先頭ゼロ落ち(例: 07000014→7000014)を
        一部ページで確認。Print-ValueTag.ps1側はF4/F5/F6を数値変換していないため、
        CSV側(Import-Csvでの型認識、またはCSV生成元でのExcel数値化)を要確認。
    v0.7での変更点 (3-1 ページ順序ズレの原因について新知見。コード変更なし):
      - 元VBAの実際の送信ログを確認した結果、注番の完全な降順で1手配ずつ個別PrintOutして
        いることが判明(例: 06444→06404→06391→06373→06372→...→08580→08578→08577→08228)。
        紙プリンタ運用を前提に「後から出た紙が上に積まれる→最終的な紙束は概ね昇順になる」
        という設計だったと考えられる。
      - VBA版PDFで一部だけ崩れていた並び(08577→08228→08578、06444→06404→06391→06373等)は、
        個々のPrintOutジョブがDocuWorks/プリンタスプーラへ到着するタイミングの揺らぎによる
        ものであり、CSVの特定列(列179等)に基づく安定したソートキーの再現では説明できない。
        つまり「列179でソートすればVBAの出力順を再現できる」という仮説は誤りだった。
      - 上記の理由により、PS側でVBAの崩れた順序をそのまま再現することは「VBAの仕様の再現」
        ではなく「印刷キューの偶然の再現」になってしまうため、方針を変更しない
        (=既定の VbaFinal 昇順を維持する)。むしろ毎回同じ順序になるVbaFinalの方が
        業務上は安全という結論。
      - 1手配=1個別ジョブに分割する構成は(実験的にv0.6aで試したが)DocuWorks側で文書が
        分割される・PowerShellのクロージャでスコープ外変数の扱いを誤りやすい等のリスクが
        あるため不採用。本スクリプトは元から「全手配をまとめた単一PrintDocumentで
        HasMorePagesループする」構成のままであり、この変更の影響を受けていない。
      - [3-1 結論・確定] 実際のRK-10シナリオ(値付けデータ(船舶)for値付用紙書式VBマクロ版
        Ver4.0-自動)を確認した結果、VBAは「1手配=1個別PrintOut(降順送信)→DocuWorksが
        バラの1ページ文書として受信→RPAのブロック133「DocuWorksをまとめる」が後から
        Desk上の全ファイルを束ねる」という2段階構成だったと判明。つまりVBA運用でも
        束ねた後の最終ページ順は「その時々のPrintOutジョブ到着順」に依存しており、
        毎回同じ順序になる保証はもともと無い。
        利用側で「毎回同じ順番であること」が業務上必要とされているため、
        「VBAの(本来re再現性のない)出力順を追いかける」方針そのものを取り止め、
        PS版の「単一PrintDocumentで注番昇順固定」を正式な仕様として確定する。
        本番DL CSV列179等によるVBA出力順の再現検証は今後不要。
    v0.8での変更点 (2026/08/10データ, DocuWorksスプール印刷→PDF実機出力とVBA版PDFの
    28ページ全件比較で新規発見。3件とも本番RK-10実行ログのTopHead診断出力で裏取り済み):
      - [3-3 対応] 得意先番号(F4)・受注先番号(F5)・送り先番号(F6, 納入先番号)の
        先頭ゼロ落ち(例: 07000014→7000014、00203007→203007)を、8桁未満の数値文字列は
        PadLeftでゼロ埋めして復元するよう修正(28件中6件で再現していた問題)。
        原因はCSV(DL)側の数値変換であり、VBAは常に8桁ゼロ埋め表示のため、PS側で吸収する。
      - 経由先１_一式金額(source=185)が"0"のとき、AQ4/AW4と同じパターンで生値の"0"が
        そのまま印字されていたのを非表示化(28件中9件で再現していた問題)。
      - 参照手配書番号(source=179)を、注番(J2)と同じLeft3-Mid1-Mid5-Right1形式に整形し、
        「【参照注番：E26-8-03692-0】」の形式で表示するよう修正。
        (元は生値 E268036920 をそのまま出力しており、VBAのラベル付きハイフン整形と不一致だった)

    v0.8.1での変更点 (2026/08/10 v0.8実機再印刷PDFの検証で発見):
      - 「経由先１_一式金額」非表示化のsource_field_idを184→185に修正。
        label_layout_full.json(setting_mapping.json)では該当フィールド(一式金額表示,
        target_cell=AV5)のsource_field_idは185であり、184はどのフィールドにも
        割り当てられていなかった(タイプミス)。そのため v0.8 では条件が一度も一致せず、
        AV5の生値"0"が非表示化されずそのまま印字され続けていた(28件中9件で再現)。

    v0.8.2での変更点 (2026/08/07データのPDF確認で発見、2026/08/10報告):
      - PrintPage時の自動縮小(scaleX/scaleY)で、品目数が多く縦方向(scaleY)が
        縮尺を決めるページでは横方向に余白が生じるが、TranslateTransformが常に
        MarginBounds左上基準だったため左寄せになっていた。
        元VBA(Excelページ設定「水平方向で中央に印刷」相当)はセンタリングのため、
        縮小後の横幅とMarginBounds幅の差分を計算し、左右中央になるようオフセットを
        加える修正。縦方向は従来通り上詰めのまま。

    v0.8.3での変更点 (2026/08/10, C:\HPDB\直下のファイル整理に伴う変更。
    ロジックの変更は無し、JSON設定ファイル群の既定参照先のみ変更):
      - label_layout_full.json / shipping_code_mapping.json / inspection_code_mapping.json
        の既定パスを、スクリプトと同じ場所ではなく「スクリプトの場所を基準としたconfig\
        サブフォルダ」を参照するよう変更した。
      - 新設の -ConfigDir パラメータ(既定値: スクリプトと同じ場所の config\)で
        参照先フォルダをまとめて切り替え可能。カレントディレクトリ(実行時の
        作業フォルダ)に依存しないよう $PSScriptRoot基準の絶対パスにしたため、
        どこから実行してもフォルダ構成さえ合っていれば動作する。
      - 想定フォルダ構成:
          <スクリプト配置フォルダ>\Print-ValueTag.ps1
          <スクリプト配置フォルダ>\config\label_layout_full.json
          <スクリプト配置フォルダ>\config\model_layout.json        (参考/中間データ、本体は未読込)
          <スクリプト配置フォルダ>\config\setting_mapping.json     (参考/中間データ、本体は未読込)
          <スクリプト配置フォルダ>\config\dl_field_mapping.json    (参考/中間データ、本体は未読込)
          <スクリプト配置フォルダ>\config\shipping_code_mapping.json
          <スクリプト配置フォルダ>\config\inspection_code_mapping.json
      - 個別に -LayoutJsonPath 等を明示指定した場合はそちらが優先される(従来通り)。

    v0.8.4での変更点 (2026/08/10, 期待値JSONダンプ機能の追加。
    描画・印刷ロジックへの変更は無し):
      - 新設の -ExpectedDumpDir パラメータ(既定値: 空文字=出力しない)を指定すると、
        各ページで印字されるはずの値(labels配列の内容そのもの)を、セル番地に
        加えて項目名(fieldMappingのlabelから逆引き)付きでJSONダンプする。
      - 1ページ1ファイル(例: 001_E26-2-09369-0.json)に加えて、全ページをまとめた
        _all.json も出力する。前者はページ単位のAPI照合(Batch APIへの
        1リクエスト=1ページ)、後者は全ページ丸ごと1リクエストで照合する構成の
        どちらにも使えるようにするため。
      - この期待値JSONは、平行運用中に印刷結果(PDF/画像)をClaude API等で
        自動照合する際の「正解データ」として使うことを想定している。

    v0.9.0での変更点 (2026/08/10, VBA版と同じ「1手配11品目以上は複数ページに
    分割」ルールへの対応。会社の慣習として、1手配=1ページではなく10品目区切りで
    複数ページに分割するのが正式ルールであることが判明したため、元VBA
    (PrintableShapes()内、row_count Mod 10での改ページ・row_count>10でのZoom固定)
    に合わせてPowerShell版のロジックを変更した):
      - 1手配の品目数が10以下の場合は従来通り(1手配=1ページ、印字可能領域に
        自動縮小してぴったり収める)。
      - 1手配の品目数が11以上の場合、10品目ごとに複数ページへ分割する
        ($VBA_MAX_ROWS_PER_PAGE = 10)。分割後の各ページの縮尺は、元VBAの
        Zoom=59(通常)/Zoom=56(DocuWorks Printerの場合)に合わせて固定値とし、
        従来の自動縮小(scaleX/scaleYのmin)は使わない。
      - 2ページ目以降(継続ページ)は、元VBAのPrintTitleRows="$1:$3"相当として、
        通常のhead(2～8行目)ではなく新設のhead_titleブロック(2～3行目のみ)を
        描画する。4～8行目(得意先コード等)は継続ページには出ない。
      - -ExpectedDumpDir の出力も分割に対応。継続ページのファイル名は
        "_cont"サフィックス付き(例: 003_E26-2-00002-0_cont.json)。
        is_continuationフィールドを追加し、継続ページのtop_headは実際に
        描画される1～3行目のセルのみに絞り込んで出力する(4～8行目は
        実際には印字されないため、期待値からも除外)。
      - 診断ログの「手配件数」は受注単位の件数を、新設の「印刷ページ数」は
        分割後の実際のページ数を、それぞれ別々に表示するよう変更。

    v0.9.1での変更点 (2026/08/10, VBAソース全体(__自動_企画室_船舶値付用紙出力.xlsm)を
    通読し、label_layout_full.json等のJSONだけでは分からなかった書式ルールを追加で
    移植。GM()/MakeData()/PrintableShapes()を中心に確認した):
      - 最終出荷日(BO3): 元VBAは Format(...,"0000/00/00") の結果が未設定/未パース
        (="0000/00/00")の場合「未出荷」と表示する。この分岐が無かったため追加。
        手配日(BE3)・手配納期(BJ3)にはこの特別扱いは無い(元VBAでも無い)。
      - 分納数表示(F13): 元VBAは 受注数>1 かつ 分納出荷数>0 かつ 受注数<>分納出荷数
        の場合のみ「(分納数= X)」の括弧書式で表示する。これまでは「分納出荷数=受注数
        の場合に非表示」の判定しかなく、条件を満たす場合の括弧書式も無かったため追加。
      - 寸法記号(K9): 値がある場合のみ「寸法記号[ X ]」の括弧書式で表示する処理が
        未実装だったため追加。
      - 【参考・低優先】元VBAはCSV取込時、値の先頭が+-=@等だとExcelが数式と誤認し
        エラーになるのを防ぐため該当セルを空文字にするガード処理がある(HasFormulaチェック)。
        PowerShell版はCSVをテキストとして扱うためこの問題自体が起きないが、
        「先頭+-=@のデータは元VBAでは空欄になる」という差が理論上は残る。
        実データでの発生有無は未確認。
      - 上記のうち、既に一致を確認できた項目(参考): MODEL_DIC(TOP/HEAD/BODYブロック
        範囲)・注番/一式親番のハイフン整形・経由仕切率警告(BJ14)・締め日表示の「め」
        除去(A2)・原価有無表示の判定条件(H1)・船舶のみ受注番号降順ソート(v0.7で
        VbaSubmitモードとして対応済み)は、いずれもPS版の実装がVBAと一致していた。

    v0.9.2での変更点 (2026/08/11, 一式親番表示(AL5)を実装。後にv0.9.3で条件式を
    VBA完全準拠に修正):
      - 一式親番表示(AL5, source=182=計上先_手配書番号)に、注番と同じハイフン整形
        (XXX-X-XXXXX-X)＋「【一式親番 : ...】」の括弧書式を追加。
      - あわせて、v0.9.1で誤ってApply-BodyDerivedFields(品目単位/BODY用の関数)側に
        書いていたAL5関連のコメントを削除。AL5は手配単位(HEAD)のフィールドのため、
        実装はApply-DerivedFields(HEAD用の関数)側に正しく配置した。

    v0.9.3での変更点 (2026/08/11, 天野さんのレビューを反映):
      - 一式親番表示(AL5)の判定条件を、「AL5に値があるか」という簡略化から、
        VBA完全準拠の「一式計上区分(列180)=1 かつ AL5に値がある」に変更。
        列180が一式計上区分であることが判明したため、簡略化を使う理由が無くなった。
      - 軸径*長さ*厚み(P9)を実装。生データ列番号(216=軸径 217=長さ 218=厚み
        220=定尺区分 221=溝区分。219=寸法記号はK9として実装済み)が判明したため、
        v0.9.1～v0.9.2で保留にしていた複合表示(溝区分+軸径+厚み+長さ+定尺区分)を
        VBAのSelect Case/If分岐と同じ組み立て順で実装した。
      - 用紙サイズをA4に明示指定するよう変更。これまではプリンターの既定用紙サイズ
        任せだったが、業務ルール「A4横・1ページ最大10品目」に合わせ、
        PrinterSettings.PaperSizesからA4を検索して明示的に設定する
        (見つからない場合は827x1169(1/100インチ)のフォールバックを使用)。
      - ページ数表示(AH1)を、分割された手配では実際のページ位置("1/2"→"2/2"等)に
        なるよう修正。元VBAは .PageSetup.CenterHeader = "&P/&N" をその手配の
        印刷範囲に対して設定しており、Excelの組み込み機能でページごとに異なる値に
        なる。これまでは全ページ一律"1/1"固定だったため、10品目以下(常に1ページ)は
        従来通り"1/1"のまま、11品目以上(複数ページに分割)は手配内の相対ページ番号を
        算出してAH1に設定するよう変更した。TopHeadが全ページで同一のハッシュ
        テーブルを共有していたため、ページごとに書き換える前にCloneして独立させている。

    v0.9.4での変更点 (2026/08/11, 天野さんの2回目のレビューを反映。ロジック変更なし、
    コメントのみ修正):
      - AH1(ページ数表示)の初期化コメントが「システム依存のため常に1/1」という
        v0.9.2以前の古い記述のままだったため更新。実際はv0.9.3で実装した通り、
        ここでは全ページ共通の暫定値"1/1"を入れているだけで、11品目以上で
        分割される手配は後段のlabels構築ループでページごとに"1/2"等へ
        上書きされる、という実際の挙動に合わせた説明に修正した。
      - 現時点で未実装のまま残っている項目(参考、天野さんも確認済み):
        原価未入力/売価未入力の警告、AS12の本来の警告表示、一式金額の複合表示。
        AS12は現状 $override.Remove("AS12") とコード上明記した上で意図的に
        未実装としている(該当セルに表示すべき本来の警告文言は元VBAの判定条件を
        確認後に別途実装予定)。

    v0.9.5での変更点 (2026/08/11, 天野さんの3回目のレビューを反映。ロジック変更なし、
    コメントのみ追加):
      - P9(軸径*長さ*厚み)の溝区分が空欄の場合の扱いについて、疑問点があったため
        VBAソースを確認。VBAのSelect Caseは空欄セル(Empty)を数値比較すると
        自動的に0として評価される(VBAのVariant型の仕様上 Empty = 0 は True)ため、
        元VBAでも「溝区分が空欄」と「溝区分が明示的に0」は同じ Case 0 (="溝無し")に
        一致することを確認した。PowerShell版はTryParse失敗時に$grooveType=0の
        既定値のままとなる実装で、これは元VBAの挙動と一致している(ロジック変更は
        不要、根拠をコメントとして追記のみ)。

    v0.9.6での変更点 (2026/08/11, 天野さんの4回目のレビューを反映。ロジック変更なし、
    .NOTES整理のみ):
      - .NOTES冒頭の「未実装/要検証」一覧が、初期PoC時代のまま更新されずに残っており、
        現在の実装内容(v0.9.0で改ページ/Zoom計算を実装、初期バージョンからCSV読込・
        注番グルーピングを実装済み)と矛盾していたため整理した。
      - 現時点で本当に未実装の項目(原価未入力/売価未入力警告・AS12本来の警告表示・
        一式金額複合表示・BT_NO別プリンター切替)のみを.NOTESに残し、
        既に実装済みの項目の記述は削除した。

    v0.10.3での変更点 (2026/09/16, Step2人工試験T01～T05B＋VBA原典照合):
      - 出荷検収日(DL列256)が設定された明細をGray8相当で網掛け。
      - 手配内の全明細が検収済みなら、その手配自体を印刷対象から除外。
      - AS12へVBA原典の「▼Y受注金額注意▼」判定を実装。
      - BJ12へ「△▲△原価未入力▲△▲」「△▲△売価未入力▲△▲」を実装。
      - 売価未入力について、品目番号99999900＋運賃名「特別元払い」は警告除外。
      - T01～T05Bの実測で確認した差分だけを対象とし、既存レイアウト/印刷Fit処理は変更しない。

    v0.10.2での変更点 (2026/09/15, DocuWorks 167%でのVBA/PS実測比較反映):
      - 1明細・7明細ともPSの長い横罫線がVBAより約3.5%短いことを確認。
      - 印刷時はDraw-OneLabelをOffsetX=0/OffsetY=0で描画しているにもかかわらず、
        Fit計算だけcontentWidth/contentHeightへ+40pxの仮想余白を加えていたため、
        実際の描画領域より大きい矩形をFit対象として縮小していた。
      - 印刷時のFit計算からこの仮想+40pxを除去。データ処理・JSON・描画内容・
        11明細以上の10件分割/固定Zoom(59/56)には変更なし。

    v0.10.1での変更点 (2026/09/15, 元VBA PrintableShapes()追加調査反映):
      - 1～10明細: VBAの Zoom=False / FitToPagesWide=1 / FitToPagesTall=1 を再現するため、
        自動Fit倍率の上限1.0制限を撤廃し、必要時の拡大を許可。
      - 11明細以上: 従来どおり10明細単位で分割し、固定Zoom 59%、DocuWorks Printer 56%。
      - 余白: VBA原典(左0.25cm/右0cm/上0.5cm/下0cm)に合わせ Margins(10,0,20,0)へ修正。
      - 一時試験値0.586は正式仕様に採用しない。

    v0.10.0での変更点 (2026/08/11, VBAとの日次自動突合の第一歩として
    CheckLog機能を追加):
      - 新設の -CheckLogPath パラメータ(既定値: 空文字=出力しない)を指定すると、
        実際にセルへ描画した「値」と「位置(X/Y/Width/Height、単位:ポイント)」を
        CSV(Page,OrderNo,Cell,Value,X,Y,Width,Height)で出力する。
        対象は実データが書き込まれたセルのみ(固定ラベル文字は対象外)。
      - 位置はDraw-Block内で既に計算済みのpx座標を72/96倍してポイント単位に変換した
        ものを使用。VBA側のRange.Left/Top/Width/Heightと同じ単位になるようにし、
        別途作成予定のCompare-Output.ps1で直接比較できるようにする狙い。
      - VBA側にも同じ形式(Page,Value,X,Y,Width,Height)のCheckLog出力を追加する
        想定(PrintableShapes()内の「取得データを書込み」箇所、天野さんに別途
        コード案を提供)。
      - 【注意】この機能はDraw-Block内(実際のGDI+描画処理の中)に組み込んだため、
        Linux環境(PowerShell Core、System.Drawing.Commonが動作しない)では
        End-to-Endの動作確認ができなかった。CSV書き出し部分(Export-Csv等)は
        System.Drawing非依存のため単体で動作確認済みだが、実際の値・位置が
        正しく記録されるかは実機(Windows PowerShell 5.1)での確認が必要。
#>

# v0.10.8 最小修正 (2026/09/18):
#   HEAD AV5「一式金額複合表示」をVBA原典・人工試験T01-T04に合わせて実装。
#   DL183(請求先_一式金額)>0をゲートに、DL184(経由先1)>0、DL186(受注先)>0を任意追記し、
#   DL185(経由先2_一式金額)は材料には使わずAV5格納先として借用する。複数明細時は先頭明細を採用。
#   抽出条件・BODY・ページング・A7・BJ12・Gray8等の既存PASS領域は変更しない。
# v0.10.7 最小修正 (2026/09/18):
#   HEAD A7「標準表示」は DL列192「商談状況」を 6=標準 / 1=後報 の場合だけ表示し、
#   0/空欄/その他は空欄とする。抽出条件・BODY・ページング・船舶固有ロジックは変更しない。
# v0.10.3 Step2 VBA原典差分実装 (2026/09/16): 検収Gray8/全検収除外/Y受注金額注意/原価・売価未入力警告
param(
    # ---- 設定ファイル(JSON)の置き場所 ----
    # 既定ではスクリプト自身の場所(= $PSScriptRoot)を基準とした config\ サブフォルダを参照する。
    # C:\HPDB\船舶値付用紙\Print-ValueTag.ps1 の隣に config\ を置く構成を想定。
    # 個別のパスを明示指定した場合はそちらが優先される。
    [string]$ConfigDir = (Join-Path $PSScriptRoot "config"),
    [string]$LayoutJsonPath = (Join-Path $ConfigDir "label_layout_full.json"),
    [string]$ShippingCodeJsonPath = (Join-Path $ConfigDir "shipping_code_mapping.json"),   # 運送便コード→名称の対応表
    [string]$InspectionCodeJsonPath = (Join-Path $ConfigDir "inspection_code_mapping.json"), # 検査コード→名称の対応表

    [string]$PrinterName = "",              # 空ならデフォルトプリンターを使用
    [switch]$PreviewOnly,                    # 指定時は印刷せずPNGへ保存するだけ
    [string]$PreviewOutPath = ".\preview_output.png",

    # ---- 実データCSV関連 (未指定ならこれまで通りサンプルデータで1件のみ描画) ----
    [string]$CsvPath = "",                   # DLファイルの列並びをそのまま持つCSV(位置ベース)
    [switch]$CsvHasHeader = $true,           # 1行目がヘッダー文字列なら読み飛ばす
    [string]$CsvEncoding = "Default",        # "Default"=ANSI/ShiftJIS, "UTF8" 等
    [int]$MaxOrders = 0,                      # テスト用に処理する手配件数を制限(0=無制限)
    [ValidateSet("VbaFinal","VbaSubmit","Csv")]
    [string]$PageOrderMode = "VbaFinal",      # VbaFinal=最終紙束/PDF向け昇順, VbaSubmit=元VBA内部PrintOut順(船舶は降順), Csv=CSV出現順
    [string]$OrderNoFilter = "",              # 特定の受注番号のみ処理したい場合に指定(実機比較検証用)
    [string]$ExpectedDumpDir = "",             # 指定時、各ページの期待値(印字されるべき値)をJSONで出力する(平行運用の自動照合用)
    [string]$CheckLogPath = ""                # 指定時、実際に描画した値の位置(Page,OrderNo,Cell,Value,X,Y,Width,Height)をCSVで出力する。
                                                # VBA側にも同形式のCheckLog出力を追加し、Compare-Output.ps1で突合する想定(単位はポイント)。
)

Add-Type -AssemblyName System.Drawing
# 注: PrintDocument等のSystem.Drawing.PrintingはSystem.Drawing.dllに含まれるため、
#     "System.Drawing.Printing"という名前でのAdd-Typeは不要(そもそも存在しないアセンブリ名でエラーになる)

# ------------------------------------------------------------
# 1. レイアウト定義の読込み
# ------------------------------------------------------------
if (-not (Test-Path $LayoutJsonPath)) {
    throw "レイアウト定義が見つかりません: $LayoutJsonPath"
}
$layout = Get-Content $LayoutJsonPath -Raw -Encoding UTF8 | ConvertFrom-Json

# 運送便コード→名称の対応表 (元Excel「setting」シートR列より。元VBA: Application.Match(...,R:R,0)相当)
$shippingCodeMap = @{}
if (Test-Path $ShippingCodeJsonPath) {
    $raw = Get-Content $ShippingCodeJsonPath -Raw -Encoding UTF8 | ConvertFrom-Json
    foreach ($prop in $raw.PSObject.Properties) {
        $shippingCodeMap[$prop.Name] = $prop.Value
    }
    Write-Host "診断: 運送便コード表 $($shippingCodeMap.Count) 件を読込みました"
}
else {
    Write-Host "警告: 運送便コード表が見つかりません($ShippingCodeJsonPath)。前納/後納運送便はコードのまま表示されます。" -ForegroundColor Yellow
}

# 検査コード→名称の対応表 (元Excel「setting」シートP列より)
$inspectionCodeMap = @{}
if (Test-Path $InspectionCodeJsonPath) {
    $raw = Get-Content $InspectionCodeJsonPath -Raw -Encoding UTF8 | ConvertFrom-Json
    foreach ($prop in $raw.PSObject.Properties) {
        $inspectionCodeMap[$prop.Name] = $prop.Value
    }
    Write-Host "診断: 検査コード表 $($inspectionCodeMap.Count) 件を読込みました"
}

# ------------------------------------------------------------
# 2. Excel座標系のユーティリティ (列名<->インデックス, 幅/高さのpx換算)
# ------------------------------------------------------------
function ColLetterToIndex([string]$letter) {
    $letter = $letter.ToUpper()
    $idx = 0
    foreach ($ch in $letter.ToCharArray()) {
        $idx = $idx * 26 + ([int][char]$ch - [int][char]'A' + 1)
    }
    return $idx
}
function ColIndexToLetter([int]$idx) {
    $s = ""
    while ($idx -gt 0) {
        $rem = ($idx - 1) % 26
        $s = [char]([int][char]'A' + $rem) + $s
        $idx = [int](($idx - $rem - 1) / 26)
    }
    return $s
}
# Excel列幅(文字単位, MDW=7=Calibri11想定)→px 概算式
function ColWidthToPx([double]$w) {
    if (-not $w) { return 40 }
    return [math]::Round(((256 * $w + 128) / 256) * 7)
}
function PtToPx([double]$pt) {
    return [math]::Round($pt * 96 / 72)
}

$MAX_COL = ColLetterToIndex "BS"

# 列ごとのpx幅、X座標の累積
$colPx = @{}
$xPos = @{}
$x = 0
for ($c = 1; $c -le $MAX_COL; $c++) {
    $letter = ColIndexToLetter $c
    $w = $layout.column_widths.$letter
    $px = ColWidthToPx $w
    $colPx[$letter] = $px
    $xPos[$letter] = $x
    $x += $px
}
$TOTAL_W = $x

# 行ごとのpx高さ
$rowPx = @{}
foreach ($prop in $layout.row_heights.PSObject.Properties) {
    $rowPx[[int]$prop.Name] = PtToPx ([double]$prop.Value)
}
function YOfRow([int]$row) {
    $y = 0
    for ($r = 1; $r -lt $row; $r++) {
        if ($rowPx.ContainsKey($r)) { $y += $rowPx[$r] } else { $y += 15 }
    }
    return $y
}

# ------------------------------------------------------------
# 3. セル/結合セル/罫線情報を検索しやすい形に変換
# ------------------------------------------------------------
$cellMap = @{}
foreach ($c in $layout.cells) {
    $cellMap[$c.cell] = $c
}

# 結合範囲: "A1:B2" 形式をパース
function ParseRange([string]$rangeStr) {
    $parts = $rangeStr -split ":"
    $m1 = [regex]::Match($parts[0], "([A-Z]+)(\d+)")
    $m2 = [regex]::Match($parts[1], "([A-Z]+)(\d+)")
    return @{
        MinCol = ColLetterToIndex $m1.Groups[1].Value
        MinRow = [int]$m1.Groups[2].Value
        MaxCol = ColLetterToIndex $m2.Groups[1].Value
        MaxRow = [int]$m2.Groups[2].Value
    }
}
$mergeRanges = @()
foreach ($m in $layout.merged_cells) {
    $mergeRanges += ParseRange $m
}
function FindMerge([int]$col, [int]$row) {
    foreach ($mr in $mergeRanges) {
        if ($col -ge $mr.MinCol -and $col -le $mr.MaxCol -and $row -ge $mr.MinRow -and $row -le $mr.MaxRow) {
            return $mr
        }
    }
    return @{ MinCol = $col; MinRow = $row; MaxCol = $col; MaxRow = $row }
}

# ------------------------------------------------------------
# 4. ブロック定義 (top/head/body)
# ------------------------------------------------------------
$blocks = @{
    top  = @{ StartRow = 1; EndRow = 1 }
    head = @{ StartRow = 2; EndRow = 8 }
    body = @{ StartRow = 9; EndRow = 14 }
    # 元VBA PrintableShapes(): 1手配が11品目以上の場合、Excelは1手配の途中で
    # 改ページしつつ PrintTitleRows = "$1:$3" により1～3行目だけを継続ページの
    # 見出しとして自動的に繰り返し印字する(4～8行目は繰り返さない)。
    # PowerShell版はExcelのようにタイトル行の自動繰り返し機能を持たないため、
    # 継続ページではこのhead_title(2～3行目のみ)をheadの代わりに描画して再現する。
    head_title = @{ StartRow = 2; EndRow = 3 }
}

# field_mapping (setting_mapping.json相当) を block別に振り分け
# 元VBA: N列(source_field_id) = DLファイルの列番号。M列(target_cell) = WS_TEMP側の書込み座標。
$fieldMapping = $layout.field_mapping
$topHeadFields = $fieldMapping | Where-Object { ($_.block -eq "TOP" -or $_.block -eq "HEAD") -and $_.source_field_id }
$bodyFields    = $fieldMapping | Where-Object { $_.block -eq "BODY" -and $_.source_field_id }

# ------------------------------------------------------------
# 4-2. CSVから実データを読み込み、手配番号(DL列2=受注番号)でグルーピング
#      元VBA: .Range("A2:...").Sort Key1:=B列(手配番号) Key2:=C列(手配行番号)
#      工場番号(DL列1)="FAC11"以外は除外 (元VBA DEL_ROWと同じ)
# ------------------------------------------------------------
$orderGroups = @()

# データ取得日時 = CSVファイルの更新日時 (元VBA: FILE_DATE = DLファイルの更新日時)
$csvFileTimestamp = $null
if ($CsvPath -and (Test-Path $CsvPath)) {
    $csvFileTimestamp = (Get-Item $CsvPath).LastWriteTime
    Write-Host "診断: CSVファイル更新日時 = $csvFileTimestamp"
}

if ($CsvPath) {
    if (-not (Test-Path $CsvPath)) {
        throw "CSVファイルが見つかりません: $CsvPath"
    }

    # DL列番号(1,2,3...)をそのままヘッダー名として読み込む(位置ベース、テキストヘッダーには依存しない)
    $maxDlCol = ($fieldMapping | Where-Object { $_.source_field_id } | Measure-Object -Property source_field_id -Maximum).Maximum
    $csvHeaders = 1..$maxDlCol

    $allRows = Import-Csv -Path $CsvPath -Header $csvHeaders -Encoding $CsvEncoding
    if ($CsvHasHeader -and $allRows.Count -gt 0) {
        $allRows = $allRows | Select-Object -Skip 1
    }

    # 工場番号(列1)="FAC11"のみ残す (DLデータは固定長パディングのためTrim必須)
    $validRows = $allRows | Where-Object { $_."1".Trim() -eq "FAC11" }

    # 元VBA 2052行目相当: 船舶(BT_NO=2)ボタンでは受注番号(列2)が"E"始まりのみ対象
    # (元コード: If BT_NO = 2 And Left(受注番号, 1) <> "E" Then GoTo DEL_ROW)
    $beforeCount = $validRows.Count
    $validRows = $validRows | Where-Object { $_."2".Trim().StartsWith("E") }
    Write-Host "診断: 船舶(E始まり)絞込み: $beforeCount 件 → $($validRows.Count) 件"

    Write-Host "診断: CSV総行数=$($allRows.Count)  FAC11絞込後=$($validRows.Count)"

    # 特定の受注番号だけに絞り込みたい場合(実機との比較検証用)
    if ($OrderNoFilter) {
        $target = $OrderNoFilter.Trim()
        $validRows = $validRows | Where-Object { $_."2".Trim() -eq $target }
        Write-Host "診断: 受注番号フィルタ('$target')適用後 = $($validRows.Count) 行"
    }

    # 受注番号(列2)でグルーピング = 手配単位
    # 元VBA MakeData:
    #   初期段階は B(受注番号)昇順 → C(行番号)昇順
    #   フィルタ後、船舶(BT_NO=2)では B(受注番号)降順に並べ直してから個別PrintOutする。
    # ただし個別ジョブをFinePrint/PDF等で束ねた最終成果物は、紙のフェイスダウン排紙やスプール順の影響で
    # 実質的に昇順で扱われていたため、PSの単一PrintDocumentでは既定を VbaFinal(昇順) とする。
    $grouped = $validRows | Group-Object -Property { $_."2".Trim() }

    # Step2 / VBA原典: 手配内の全明細が出荷検収済みなら、その手配自体を印刷対象から除外する。
    # 出荷検収日(DL列256)は 0 / 空欄 = 未検収、それ以外 = 検収済みとして扱う。
    $beforeInspectionExclude = @($grouped).Count
    $grouped = $grouped | Where-Object {
        $allInspected = $true
        foreach ($r in $_.Group) {
            $v = [string]$r."256"
            if ($v) { $v = $v.Trim() }
            if (-not $v -or $v -eq "0") { $allInspected = $false; break }
        }
        -not $allInspected
    }
    if ($beforeInspectionExclude -ne @($grouped).Count) {
        Write-Host "診断: 全明細検収済み手配を除外: $beforeInspectionExclude 件 → $(@($grouped).Count) 件"
    }

    switch ($PageOrderMode) {
        "VbaFinal"  { $grouped = $grouped | Sort-Object Name }
        "VbaSubmit" { $grouped = $grouped | Sort-Object Name -Descending }
        "Csv"       { } # Group-Objectの出現順をそのまま使用
    }

    if ($MaxOrders -gt 0) { $grouped = $grouped | Select-Object -First $MaxOrders }

    Write-Host "診断: ページ順モード=$PageOrderMode"
    Write-Host ("診断: 注番順=" + (($grouped | ForEach-Object { $_.Name }) -join " -> "))

    foreach ($g in $grouped) {
        # 行番号(列3)昇順に整列
        $sortedRows = $g.Group | Sort-Object { [int]$_."3" }
        $orderGroups += , $sortedRows   # 1手配 = 複数行(品目)の配列
    }

    Write-Host "診断: 手配件数=$($orderGroups.Count)"
}
else {
    # CSV未指定時はサンプルデータ(動作確認用)
    Write-Host "診断: -CsvPath未指定のため、サンプルデータで1手配のみ描画します"
}

# DLの1行(PSCustomObject, プロパティ名="1","2",...,"268")からセル上書き辞書を作る
function Build-Override($row, $fields) {
    # Excelのエラー文字列がCSVにそのまま入っている場合、値なし扱いにする
    # (元VBA: セルにエラー値が入っていても印字対象から除外している挙動を再現)
    $ERROR_STRING_PATTERNS = @("#NAME?", "#N/A", "#REF!", "#VALUE!", "#DIV/0!", "#NULL!", "#NUM!")

    $result = @{}
    foreach ($f in $fields) {
        $val = $row.($f.source_field_id.ToString())
        if ($val) {
            $val = $val.Trim()
            if ($ERROR_STRING_PATTERNS -contains $val) { $val = "" }
            if ($val) { $result[$f.target_cell] = $val }
        }
    }
    return $result
}

# 部署コード→表示名の対応 (元Excel「設定」シートB6～B18より。"営業所"付与は推定のため要確認)
$DEPARTMENT_NAMES = @{
    "B1901" = "札幌営業所"
    "B1903" = "仙台営業所"
    "B1905" = "東京営業所"
    "B1907" = "福岡営業所"
    "B1911" = "大阪営業所"
    "B1913" = "特販"
    "B1915" = "船舶機械部"
}

# 元VBAのMakeDataが行っている「値の動的加工」を再現する
# (これらは元データをそのまま転記するのではなく、VBA側で計算してから書き込んでいる項目)
function Apply-DerivedFields($override, $row, $fields, $rows) {
    $orderNo = $row."2"
    if ($orderNo) { $orderNo = $orderNo.Trim() }

    if ($orderNo) {
        # バーコード用テキスト = "*" & 受注番号 & "*"
        # 元VBA: 品目区分(id=15)の値を上書きしてバーコードフォント(Code39)で表示させる仕組みのため、
        # source_field_id=15を参照している全セル(M1, AR2等)に同じ内容を反映する
        $barcodeText = "*$orderNo*"
        foreach ($f in $fields) {
            if ($f.source_field_id -eq 15) {
                $override[$f.target_cell] = $barcodeText
            }
        }

        # 注番(J2) = 受注番号をハイフン区切りに整形 (元VBA: Left3-Mid1-Mid5-Right1)
        if ($orderNo.Length -ge 10) {
            $p1 = $orderNo.Substring(0, 3)
            $p2 = $orderNo.Substring(3, 1)
            $p3 = $orderNo.Substring(4, 5)
            $p4 = $orderNo.Substring($orderNo.Length - 1, 1)
            $override["J2"] = "$p1-$p2-$p3-$p4"
        }
    }

    # 標準表示(A7) = 商談状況(列192)の値を変換 (元VBA: 1=後報, 6=標準)
    # v0.10.7: field_mapping転記で入った生値(例: "0")を必ず消してから、
    #           表示対象の 1/6 の場合だけ「後報/標準」を設定する。
    #           0/空欄/その他は元VBA実出力どおり空欄。
    $override.Remove("A7") | Out-Null
    $situation = [string]$row."192"
    if ($situation) { $situation = $situation.Trim() }
    if ($situation -eq "6") {
        $override["A7"] = "標準"
    }
    elseif ($situation -eq "1") {
        $override["A7"] = "後報"
    }

    # 部署名(B1, source=65=販売部門) / 責任部門(AH3, source=116) / 経由営業所(AM3, source=115)
    # をコードから表示名へ変換 (元VBAは3項目とも同じ$DEPARTMENT_NAMES変換テーブルを使用)
    foreach ($cellKey in @("B1", "AH3", "AM3")) {
        if ($override.ContainsKey($cellKey)) {
            $code = $override[$cellKey]
            if ($DEPARTMENT_NAMES.ContainsKey($code)) {
                $override[$cellKey] = $DEPARTMENT_NAMES[$code]
            }
        }
    }

    # 経由先コード(AQ4, source=107)・経由先名(AW4, source=108) : 経由先が設定されていない
    # 受注(コード=0)では、経由先名側にも生の"0"がそのまま印字されてしまう(全38ページで再現確認)。
    # 元VBAはこの場合どちらも空欄のため、値が"0"のときは非表示にする。
    foreach ($cellKey in @("AQ4", "AW4")) {
        if ($override.ContainsKey($cellKey) -and $override[$cellKey] -eq "0") {
            $override.Remove($cellKey) | Out-Null
        }
    }

    # AV5 一式金額複合表示 (source=185は格納先として借用)
    # 元VBA(Process.cls 1292-1313)＋2026/09/18人工試験T01-T04で実機確定:
    #   - 183(請求先_一式金額)>0 のときだけ生成
    #   - 184(経由先1_一式金額)>0 なら「 / 経由 : ￥...」を追記
    #   - 186(受注先_一式金額)>0 なら「/ 受注 : ￥...」を追記（経由側との間に空白なし）
    #   - 185(経由先2_一式金額)の生値は材料に使わず、常に一旦クリアして複合文字列の格納先にする
    #   - 複数明細時は先頭明細を採用。Apply-DerivedFieldsには$rows[0]が$rowとして渡されるため仕様一致。
    foreach ($f in $fields) {
        if ($f.source_field_id -eq 185) {
            $override.Remove($f.target_cell) | Out-Null

            $billing = 0.0
            [void][double]::TryParse([string]$row."183", [ref]$billing)
            if ($billing -gt 0) {
                $text = "【一式金額 / 請求 : ￥{0:N0}" -f $billing

                $via = 0.0
                [void][double]::TryParse([string]$row."184", [ref]$via)
                if ($via -gt 0) {
                    $text += (" / 経由 : ￥{0:N0}" -f $via)
                }

                $orderAmount = 0.0
                [void][double]::TryParse([string]$row."186", [ref]$orderAmount)
                if ($orderAmount -gt 0) {
                    $text += ("/ 受注 : ￥{0:N0}" -f $orderAmount)
                }

                $text += "】"
                $override[$f.target_cell] = $text
            }
        }
    }

    # 得意先番号(F4)・受注先番号(F5)・送り先番号(F6, 納入先番号) は、
    # CSV(DL)側で数値として書き出される際に先頭ゼロが欠落することがある
    # (例: 07000014→7000014、00203007→203007。2026/08/10データ 28件中6件で再現確認)。
    # 元VBAは常に8桁ゼロ埋めで表示しているため、数値のみ・8桁未満の場合は左ゼロ埋めして復元する。
    # (9999999999のような10桁の特殊コードは対象外)
    foreach ($cellKey in @("F4", "F5", "F6")) {
        if ($override.ContainsKey($cellKey)) {
            $code = $override[$cellKey].Trim()
            if ($code -match '^\d+$' -and $code.Length -lt 8) {
                $override[$cellKey] = $code.PadLeft(8, '0')
            }
        }
    }

    # 参照手配書番号(source=179) : 一式計上等で元手配書を参照する場合に設定される。
    # 元VBAは注番(J2)と同じLeft3-Mid1-Mid5-Right1のハイフン整形を行い、
    # 「【参照注番：E26-8-03692-0】」の形式でヘッダー右上に表示する。
    # PS側はこれまで生値(例: E268036920)をそのまま出力していた(2026/08/10データで発見)。
    foreach ($f in $fields) {
        if ($f.source_field_id -eq 179) {
            if ($override.ContainsKey($f.target_cell)) {
                $refNo = $override[$f.target_cell].Trim()
                if ($refNo.Length -ge 10) {
                    $rp1 = $refNo.Substring(0, 3)
                    $rp2 = $refNo.Substring(3, 1)
                    $rp3 = $refNo.Substring(4, 5)
                    $rp4 = $refNo.Substring($refNo.Length - 1, 1)
                    $override[$f.target_cell] = "【参照注番：$rp1-$rp2-$rp3-$rp4】"
                }
            }
        }
    }

    # 原価あり/なし表示(H1) : 手配内の品目行に1つでも
    # 「品目番号<>COMENT かつ 仮単価区分=仮(=1) かつ 営業原価_単価=0」があれば《原価なし》
    # (元VBA: FLAG_DIC(save_order)("genka")の判定と同じ)
    $genkaNashi = $false
    foreach ($r in $rows) {
        $itemNo = $r."10"; if ($itemNo) { $itemNo = $itemNo.Trim() }
        $kariKubun = $r."25"; if ($kariKubun) { $kariKubun = $kariKubun.Trim() }
        $genkaTanka = 0; [void][double]::TryParse($r."208", [ref]$genkaTanka)
        if ($itemNo -ne "COMENT" -and $kariKubun -eq "1" -and $genkaTanka -eq 0) {
            $genkaNashi = $true
            break
        }
    }
    $override["H1"] = if ($genkaNashi) { "《原価なし》" } else { "《原価あり》" }

    # 一式親番表示(AL5, source=182=計上先_手配書番号)
    # 元VBAは「一式計上区分(列180)=1」の場合のみ、値を注番と同じハイフン整形をした上で
    # 「【一式親番 : XXX-X-XXXXX-X】」の括弧書式にする。
    # 2026/08/11 天野さん確認により、列180が一式計上区分であることが判明したため、
    # 簡略化(AL5に値があるかどうかだけで判定)ではなくVBAと同じ条件式に修正した。
    $isPackageOrder = ([string]$row."180").Trim() -eq "1"
    if ($isPackageOrder -and $override.ContainsKey("AL5")) {
        $parentNo = $override["AL5"].Trim()
        if ($parentNo.Length -ge 10) {
            $override["AL5"] = "【一式親番 : {0}-{1}-{2}-{3}】" -f `
                $parentNo.Substring(0, 3), $parentNo.Substring(3, 1), $parentNo.Substring(4, 5), $parentNo.Substring(9, 1)
        }
        # 10桁に満たないが空でもない値は、想定外の形式として整形せずそのまま残す
        # (元VBAのtehaifun関数も同様に、Left(str,3)等が空文字になる場合は無変換のまま)
    }
    else {
        $override.Remove("AL5") | Out-Null
    }

    # ページ数表示(AH1) : いったん"1/1"で初期化する。
    # 元VBAは .PageSetup.CenterHeader = "&P/&N" (Excel組み込みのページ番号/その手配の
    # 総ページ数)を使っており、11品目以上で複数ページに分割される手配では
    # "1/2"→"2/2"のようにページごとに異なる値になる。この初期値はここでは
    # 全ページ共通の暫定値に過ぎず、実際の分割判定後(labels構築ループ内、
    # TopHeadをCloneした上で)"1/2"等へページ単位に上書きされる。
    # 10品目以下(常に1ページ)の手配はこの初期値"1/1"のまま確定する。
    $override["AH1"] = "1/1"

    # データ取得日時(BF1) = CSVファイルの更新日時
    if ($csvFileTimestamp) {
        $override["BF1"] = "データ取得日時: " + $csvFileTimestamp.ToString("yyyy年MM月dd日HH時mm分")
    }

    # 前納運送便(BJ8)・後納運送便(BO8) をコードから名称へ変換
    # (元VBA: Application.Match(コード, setting!R:R, 0) でsetting!S列の名称に置換)
    foreach ($cellKey in @("BJ8", "BO8")) {
        if ($override.ContainsKey($cellKey)) {
            $code = $override[$cellKey].Trim()
            if ($shippingCodeMap.ContainsKey($code)) {
                $override[$cellKey] = $shippingCodeMap[$code]
            }
        }
    }

    # 検査(U8) をコードから名称へ変換 (元VBA: Application.Match(コード, setting!P:P, 0))
    if ($override.ContainsKey("U8")) {
        $code = $override["U8"].Trim()
        if ($inspectionCodeMap.ContainsKey($code)) {
            $override["U8"] = $inspectionCodeMap[$code]
        }
    }

    # 締め日表示(A2, source=91=得意先種別1名称) から「め」を除去
    # (元VBA: Replace(値, "め", "") ※「31日締め」ではなく「31日締」が正しい表示)
    if ($override.ContainsKey("A2")) {
        $override["A2"] = $override["A2"].Replace("め", "")
    }

    # 手配日(BE3)・手配納期(BJ3)・最終出荷日(BO3) は yyyy/MM/dd (ゼロ埋め) 形式に統一する
    # DLの最終出荷日は "20260806" のような8桁形式もあるため TryParse だけに頼らず ParseExact も行う。
    foreach ($cellKey in @("BE3", "BJ3", "BO3")) {
        if ($override.ContainsKey($cellKey)) {
            $dateText = [string]$override[$cellKey]
            $d = [datetime]::MinValue
            $parsed = $false

            if ($dateText -match '^\d{8}$') {
                $parsed = [datetime]::TryParseExact(
                    $dateText,
                    "yyyyMMdd",
                    [System.Globalization.CultureInfo]::InvariantCulture,
                    [System.Globalization.DateTimeStyles]::None,
                    [ref]$d
                )
            }
            if (-not $parsed) {
                $parsed = [datetime]::TryParse($dateText, [ref]$d)
            }
            if ($parsed) {
                $override[$cellKey] = $d.ToString("yyyy/MM/dd")
            }
            elseif ($cellKey -eq "BO3") {
                # 元VBA MakeData(): 最終出荷日は Format(...,"0000/00/00") の結果が
                # "0000/00/00"(=未設定/未パース)なら「未出荷」に置き換える。
                # この特別扱いは最終出荷日(BO3)のみで、手配日(BE3)・手配納期(BJ3)には無い。
                $override[$cellKey] = "未出荷"
            }
        }
    }

    return $override
}

# BODY行(品目明細)ごとの派生フィールド計算
function Apply-BodyDerivedFields($override, $row) {
    $orderQty = 0
    [void][int]::TryParse($row."19", [ref]$orderQty)

    # AS12 は setting 上では source_field_id=32(計画必要数) が割り当てられているが、
    # 元VBAでは生データの値をそのまま表示するセルではなく、警告表示用の派生セル。
    # そのため Build-Override で入った「1」等の計画必要数は必ず除去する。
    # Step2 / VBA原典: Y受注金額注意。AS12は生データではなく警告専用セル。
    $override.Remove("AS12") | Out-Null
    $viaPrice = 0.0; $orderPrice = 0.0
    [void][double]::TryParse([string]$row."200", [ref]$viaPrice)
    [void][double]::TryParse([string]$row."206", [ref]$orderPrice)
    $viaNoY = [string]$row."107"; if ($viaNoY) { $viaNoY = $viaNoY.Trim() }
    $orderNoY = [string]$row."111"; if ($orderNoY) { $orderNoY = $orderNoY.Trim() }

    # v0.10.5 / VBA原典 Y_DIC.exists(得意先番号) のガードを復元。
    # 元VBA settingシートT列の15件。Step2 Claude独立レビュー＋E1実機再現で欠落を確認。
    $yanmarCustomers = @(
        "1000152", "11000240", "26000246", "34000247", "43000241",
        "51000242", "59000241", "65000245", "71000249", "77013520",
        "77013539", "77013547", "77013555", "77013571", "77013563"
    )
    $customerNoY = [string]$row."5"; if ($customerNoY) { $customerNoY = $customerNoY.Trim() }
    $isYanmarCustomer = ($yanmarCustomers -contains $customerNoY)

    $yWarning = $false
    if ($isYanmarCustomer -and $viaPrice -gt 0 -and $orderPrice -gt 0) {
        if ($viaNoY -eq $orderNoY) {
            # 同一相手先なのに経由単価と受注単価が不一致
            $yWarning = ([math]::Abs($viaPrice - $orderPrice) -gt 0.0000001)
        }
        elseif ($viaPrice -gt $orderPrice) {
            # 経由先が別で、経由単価が受注単価を上回る
            $yWarning = $true
        }
    }
    if ($yWarning) { $override["AS12"] = "▼Y受注金額注意▼" }

    # 経由仕切率警告(BJ14) - 元VBAは「経由先２_仕切率」列(id=202)を警告表示用に借用
    # 条件:
    #   経由先1仕切率 > 0
    #   経由先1番号 <> 受注先番号
    #   運賃行(99999900)・コメント行(COMENT)ではない
    #   同率 → ●〇経由仕切率注意！●〇
    #   経由1 > 受注 → ■□受注仕切率注意！■□
    # 生データの経由先2仕切率そのものは帳票へ出さず、条件成立時だけ警告文を出す。
    $override.Remove("BJ14") | Out-Null
    $viaRate = 0.0
    $orderRate = 0.0
    [void][double]::TryParse([string]$row."199", [ref]$viaRate)
    [void][double]::TryParse([string]$row."205", [ref]$orderRate)
    $viaNo = [string]$row."107"; if ($viaNo) { $viaNo = $viaNo.Trim() }
    $orderDestNo = [string]$row."111"; if ($orderDestNo) { $orderDestNo = $orderDestNo.Trim() }
    $warningItemNo = [string]$row."10"; if ($warningItemNo) { $warningItemNo = $warningItemNo.Trim() }

    if ($viaRate -gt 0 -and
        $viaNo -ne $orderDestNo -and
        $warningItemNo -ne "99999900" -and
        $warningItemNo -ne "COMENT") {

        if ([math]::Abs($viaRate - $orderRate) -lt 0.0000001) {
            $override["BJ14"] = "●〇経由仕切率注意！●〇"
        }
        elseif ($viaRate -gt $orderRate) {
            $override["BJ14"] = "■□受注仕切率注意！■□"
        }
    }

    # 仮単価区分(F10) = 0または空→"正", 1→"仮" (元VBA: If...=1 Then "仮" Else "正")
    if ($override.ContainsKey("F10")) {
        $override["F10"] = if ($override["F10"].Trim() -eq "1") { "仮" } else { "正" }
    }

    # 分納出荷数(F13, source=257)
    # 元VBA: 受注数>1 かつ 分納出荷数>0 かつ 受注数<>分納出荷数 の場合のみ
    # 「(分納数= X)」の括弧書式で表示。それ以外(分納が発生していない通常行、
    # または分納出荷数が0)は非表示。
    if ($override.ContainsKey("F13")) {
        $splitQty = 0
        $hasSplitQty = [double]::TryParse($override["F13"], [ref]$splitQty)
        if ($orderQty -gt 1 -and $hasSplitQty -and $splitQty -gt 0 -and $splitQty -ne $orderQty) {
            $override["F13"] = "(分納数= {0})" -f $override["F13"]
        }
        else {
            $override.Remove("F13") | Out-Null
        }
    }

    # 寸法記号(K9, source=219) : 値がある場合のみ「寸法記号[ X ]」形式で表示
    # (元VBA: If Len(Trim(値))>0 Then 値="寸法記号[ " & 値 & " ]")
    if ($override.ContainsKey("K9")) {
        $dimSymbol = $override["K9"].Trim()
        if ($dimSymbol.Length -gt 0) {
            $override["K9"] = "寸法記号[ {0} ]" -f $dimSymbol
        }
        else {
            $override.Remove("K9") | Out-Null
        }
    }

    # 軸径*長さ*厚み(P9) : 溝区分(0=溝無し/1=溝入り/2～9=X本溝入り)+軸径+
    # (厚み>0ならx厚み)+(長さ>0ならx長さ)+(定尺区分=2なら"寸法切り")を1セルに組み立てる複合表示。
    # 2026/08/11 天野さん確認により生データ列番号が判明: 216=軸径 217=長さ 218=厚み 220=定尺区分 221=溝区分
    # (219=寸法記号はK9として別途上で処理済み)
    $override.Remove("P9") | Out-Null
    # 溝区分が空欄の場合、TryParse失敗により$grooveType=0(既定値)のままとなり、
    # "溝無し"扱いになる。これは元VBAの実際の仕様と一致している: VBAのSelect Caseは
    # 空欄セル(Empty)を数値比較すると自動的に0として評価されるため
    # (VBAのVariant型の仕様上 Empty = 0 は True)、元VBAでも「空欄」と「明示的な0」は
    # どちらも同じ Case 0 (="溝無し") に一致する。2026/08/11 天野さんのご指摘を受けVBA
    # ソースで確認済み。
    $grooveType = 0
    [void][int]::TryParse([string]$row."221", [ref]$grooveType)
    $shaftDia = [string]$row."216"; if ($shaftDia) { $shaftDia = $shaftDia.Trim() }
    $thickness = 0.0; [void][double]::TryParse([string]$row."218", [ref]$thickness)
    $length = 0.0; [void][double]::TryParse([string]$row."217", [ref]$length)
    $stdLenType = [string]$row."220"; if ($stdLenType) { $stdLenType = $stdLenType.Trim() }

    $dimStr = switch ($grooveType) {
        0 { "溝無し　" }
        1 { "溝入り　" }
        { $_ -ge 2 -and $_ -le 9 } { "{0}本溝入り　" -f $grooveType }
        default { "" }
    }
    $shaftDiaNum = 0.0
    if ([double]::TryParse($shaftDia, [ref]$shaftDiaNum) -and $shaftDiaNum -gt 0) { $dimStr += $shaftDia }
    if ($thickness -gt 0) { $dimStr += " x " + $row."218".Trim() }
    if ($length -gt 0) { $dimStr += " x " + $row."217".Trim() }
    if ($stdLenType -eq "2") { $dimStr += "　寸法切り" }

    if ($dimStr) { $override["P9"] = $dimStr }

    # 一式親番表示(AL5) は手配単位(HEAD)のフィールドのため Apply-DerivedFields 側で処理する

    # 明細納期(AH9) は 手配納期(id=127) と異なる場合のみ「明細納期=[ yyyy/mm/dd ]」形式で表示
    # (元VBA: 一致すれば空文字。今回は生データをそのまま出すと常に表示されてしまうため条件判定を追加)
    if ($override.ContainsKey("AH9")) {
        $lineDate = [datetime]::MinValue
        $headerDate = [datetime]::MinValue
        $lineDateStr = $row."18"
        $headerDateStr = $row."127"
        $lineOk = [datetime]::TryParse($lineDateStr, [ref]$lineDate)
        $headerOk = [datetime]::TryParse($headerDateStr, [ref]$headerDate)
        if ($lineOk -and $headerOk -and $lineDate.Date -eq $headerDate.Date) {
            $override.Remove("AH9") | Out-Null
        }
        elseif ($lineOk) {
            $override["AH9"] = "明細納期=[ " + $lineDate.ToString("yyyy/MM/dd") + " ]"
        }
    }

    # 金額系フィールドは3桁区切りカンマを付与して表示する。0円の場合は非表示にする。
    # VBA出力では「合計」は数量が2以上の時だけ表示し、単なる数値ではなく
    #   (請求合計:￥45,800) / (原価合計:￥35,300) / (製造原価合計:￥30,000)
    # のような注記形式になる。数量1の時は合計欄を出さない。
    $unitMoneyCells = @("BE9", "O13", "AI13", "AS13", "BC13", "BC12")
    foreach ($cellKey in $unitMoneyCells) {
        if ($override.ContainsKey($cellKey)) {
            $numVal = 0
            if ([double]::TryParse($override[$cellKey], [ref]$numVal)) {
                if ($numVal -eq 0) {
                    $override.Remove($cellKey) | Out-Null
                }
                else {
                    $override[$cellKey] = "{0:N0}" -f $numVal
                }
            }
        }
    }

    # 請求先合計(V13)
    if ($orderQty -gt 1 -and $override.ContainsKey("V13")) {
        $total = 0
        if ([double]::TryParse($override["V13"], [ref]$total) -and $total -ne 0) {
            $override["V13"] = "(請求合計:￥{0:N0})" -f $total
        }
        else {
            $override.Remove("V13") | Out-Null
        }
    }
    else {
        $override.Remove("V13") | Out-Null
    }

    # 営業原価合計(BJ13)
    if ($orderQty -gt 1 -and $override.ContainsKey("BJ13")) {
        $total = 0
        if ([double]::TryParse($override["BJ13"], [ref]$total) -and $total -ne 0) {
            $override["BJ13"] = "(原価合計:￥{0:N0})" -f $total
        }
        else {
            $override.Remove("BJ13") | Out-Null
        }
    }
    else {
        $override.Remove("BJ13") | Out-Null
    }

    # 粗利率(V12)は生データではなく都度計算し直す
    # (元VBA: 請求先_単価>0 かつ 営業原価_単価>0 の場合のみ (請求先単価-営業原価単価)/請求先単価 を%表示。
    #  それ以外は空欄。元データの粗利率列(id=233)は実は使われていない)
    $override.Remove("V12") | Out-Null
    $billingPrice = 0; [void][double]::TryParse($row."197", [ref]$billingPrice)
    $costPrice = 0; [void][double]::TryParse($row."208", [ref]$costPrice)
    if ($billingPrice -gt 0 -and $costPrice -gt 0) {
        $margin = ($billingPrice - $costPrice) / $billingPrice * 100
        $override["V12"] = "{0:N1}%" -f $margin
    }

    # Step2 / VBA原典: 原価未入力 / 売価未入力警告。
    # v0.10.6: setting由来のBJ12(id=211: 製造原価_金額)をいったん除去する。
    # VBAでは数量1の通常明細に製造原価合計(BJ12)は表示しない。
    # 警告を先にBJ12へ仮設定し、後段で数量>1かつ製造原価単価>0の場合だけ
    # 製造原価合計をBJ12へ再設定する。
    $override.Remove("BJ12") | Out-Null
    $itemNo = $row."10"
    if ($itemNo) { $itemNo = $itemNo.Trim() }

    $packageKubun = [string]$row."180"; if ($packageKubun) { $packageKubun = $packageKubun.Trim() }
    $kariPriceKubun = [string]$row."25"; if ($kariPriceKubun) { $kariPriceKubun = $kariPriceKubun.Trim() }
    $billingUnit = 0.0; $salesCostUnit = 0.0
    [void][double]::TryParse([string]$row."197", [ref]$billingUnit)
    [void][double]::TryParse([string]$row."208", [ref]$salesCostUnit)
    if ($packageKubun -eq "0" -and $kariPriceKubun -eq "0") {
        if ($billingUnit -gt 0 -and $salesCostUnit -eq 0) {
            $override["BJ12"] = "△▲△原価未入力▲△▲"
        }
        elseif ($billingUnit -eq 0 -and $salesCostUnit -gt 0) {
            # 運賃行(99999900)の「特別元払い」はVBA原典の除外例外。
            $freightName = [string]$row."245"; if ($freightName) { $freightName = $freightName.Trim() }
            if (-not ($itemNo -eq "99999900" -and $freightName -eq "特別元払い")) {
                $override["BJ12"] = "△▲△売価未入力▲△▲"
            }
        }
    }

    # v0.10.9 / 2026-09-24 正式採用: 原価割れ注意。
    # VBA原典のAND条件を省略せず再現する。船舶機械部(B1915)は業務仕様として警告対象外。
    # この警告はBJ12へ設定するが、後段の製造原価合計条件が成立した場合は製造原価合計が後勝ちする。
    $responsibleDept = [string]$row."116"; if ($responsibleDept) { $responsibleDept = $responsibleDept.Trim() }
    if ($packageKubun -eq "0" -and
        $responsibleDept -ne "B1915" -and
        $billingUnit -gt 0 -and
        $salesCostUnit -gt 0 -and
        $billingUnit -lt $salesCostUnit) {
        $override["BJ12"] = "≪≪≪原価割れ注意！≫≫≫"
    }

    # 製造原価=表示(AY12,BC12,BJ12) は船舶出力のみ・運賃行/コメント行を除いて表示
    # (元VBA: BT_NO=2 かつ 品目番号<>99999900 かつ <>COMENT の場合のみ)
    # v0.10.5: 製造原価合計条件成立時だけBJ12を後勝ちで上書き。
    # 条件不成立時は前段の原価/売価未入力警告を消さない。
    if ($itemNo -and $itemNo -ne "99999900" -and $itemNo -ne "COMENT") {
        $override["AY12"] = "製造原価="

        $unitCost = 0; [void][double]::TryParse($row."210", [ref]$unitCost)

        if ($orderQty -gt 1 -and $unitCost -gt 0) {
            $totalCost = 0; [void][double]::TryParse($row."211", [ref]$totalCost)
            $override["BJ12"] = "(製造原価合計:￥{0:N0})" -f $totalCost
        }
    }
    else {
        # 船舶以外(または運賃/コメント行)は製造原価表示のみ非表示。
        # BJ12は前段の警告値を保持する（売価未入力の特別元払い例外は前段で処理済み）。
        $override.Remove("BC12") | Out-Null
    }

    # Step2 / VBA原典: 出荷検収済み明細はGray8網掛け。描画用の内部フラグを付ける。
    $inspectionDate = [string]$row."256"; if ($inspectionDate) { $inspectionDate = $inspectionDate.Trim() }
    if ($inspectionDate -and $inspectionDate -ne "0") { $override["__Gray8"] = $true }

    # 元VBA STEP_3:
    #   If DATA_DIC(dat)(...) = 0 Then GoTo CONTINUE_3
    # BODYへの最終転記では「数値0は表示しない」が全項目共通。
    # 派生処理(F10の"正"/"仮"など)を済ませた後、残っている純粋な数値0だけを除去する。
    foreach ($cellKey in @($override.Keys)) {
        $rawText = [string]$override[$cellKey]
        $zeroCheck = 0.0
        if ([double]::TryParse($rawText, [ref]$zeroCheck) -and $zeroCheck -eq 0) {
            $override.Remove($cellKey) | Out-Null
        }
    }

    return $override
}

# 元VBA PrintableShapes(): 1手配内の品目数(row_count)が10を超えると、
# 10行ごとに強制改ページ(.Rows(now_row).PageBreak)し、かつ縮尺は自動計算ではなく
# 固定値(通常59%、DocuWorks Printerは56%)になる。会社の慣習として、
# 1手配=1ページではなく「10品目区切りで複数ページに分割」が正式ルールのため、
# PowerShell版もこれに合わせる。
$VBA_MAX_ROWS_PER_PAGE = 10

# CSVがあれば実データから、なければサンプルから top/head/body 上書き辞書のリストを作る
# $labels の各要素: @{ OrderNo=""; TopHead=@{}; BodyRows=@(@{},...); IsContinuation=$bool; FixedScale=$bool }
# 1手配が11品目以上の場合、複数要素(=複数ページ)に分割される。IsContinuation=$trueの要素は
# 2ページ目以降を表し、Draw-OneLabel側でhead_title(2～3行目のみ)を使う。
$labels = @()

if ($orderGroups.Count -gt 0) {
    foreach ($rows in $orderGroups) {
        $orderNo = $rows[0]."2".Trim()
        $topHeadOverride = Build-Override $rows[0] $topHeadFields   # 手配の先頭行から取得
        $topHeadOverride = Apply-DerivedFields $topHeadOverride $rows[0] $topHeadFields $rows
        $bodyOverrides = @()
        foreach ($r in $rows) {
            $bodyOverride = Build-Override $r $bodyFields
            $bodyOverride = Apply-BodyDerivedFields $bodyOverride $r
            $bodyOverrides += , $bodyOverride
        }

        if ($bodyOverrides.Count -le $VBA_MAX_ROWS_PER_PAGE) {
            # 10品目以下: 従来通り1手配=1ページ、縮尺は自動計算
            $labels += , @{
                OrderNo = $orderNo; TopHead = $topHeadOverride; BodyRows = $bodyOverrides
                IsContinuation = $false; FixedScale = $false
            }
        }
        else {
            # 11品目以上: 10品目ごとに分割。1ページ目は通常のhead、2ページ目以降はhead_titleのみ。
            # 縮尺はページ全体を通して固定(VBAのZoom=56/59相当、自動計算しない)。
            # 元VBAは .PageSetup.CenterHeader = "&P/&N" (Excel組み込みのページ番号/その手配の
            # 総ページ数)をPrintArea=その手配の行範囲に対して設定しているため、分割された
            # 手配では「1/2」「2/2」のようにページごとに異なる値になる。2026/08/11 天野さん指摘。
            # (TopHeadは全ページで同じハッシュテーブルを共有しているため、AH1を書き換える前に
            # 必ずCloneしてページごとに独立させる)
            $totalPagesInOrder = [Math]::Ceiling($bodyOverrides.Count / $VBA_MAX_ROWS_PER_PAGE)
            $pageInOrder = 0
            for ($chunkStart = 0; $chunkStart -lt $bodyOverrides.Count; $chunkStart += $VBA_MAX_ROWS_PER_PAGE) {
                $pageInOrder++
                $chunkEnd = [Math]::Min($chunkStart + $VBA_MAX_ROWS_PER_PAGE, $bodyOverrides.Count) - 1
                $chunk = $bodyOverrides[$chunkStart..$chunkEnd]
                $topHeadForThisPage = $topHeadOverride.Clone()
                $topHeadForThisPage["AH1"] = "{0}/{1}" -f $pageInOrder, $totalPagesInOrder
                $labels += , @{
                    OrderNo = $orderNo; TopHead = $topHeadForThisPage; BodyRows = $chunk
                    IsContinuation = ($chunkStart -gt 0); FixedScale = $true
                }
            }
        }
    }

    # 診断: 1件目の手配で、想定通り値が取れているか主要項目をダンプ
    if ($labels.Count -gt 0) {
        Write-Host "----- 診断: 1件目手配のTopHead内容 -----"
        $labels[0].TopHead.GetEnumerator() | Sort-Object Name | ForEach-Object {
            Write-Host "  $($_.Name) = $($_.Value)"
        }
        Write-Host "----------------------------------------"
    }

    # ------------------------------------------------------------
    # 期待値JSONダンプ (平行運用の自動照合用、v0.8.4で追加)
    # $labels[$i] が印刷$i+1ページ目そのものに対応する(1手配=1ページ)ことを
    # 利用し、印字されるべき値をそのままJSON化する。セル番地(例:F4)は
    # fieldMapping(label_layout_full.jsonのfield_mapping、setting_mapping.json由来)
    # のlabelで人間可読な項目名に変換して併記する。
    # ------------------------------------------------------------
    if ($ExpectedDumpDir -and $labels.Count -gt 0) {
        if (-not (Test-Path $ExpectedDumpDir)) { New-Item -ItemType Directory -Path $ExpectedDumpDir -Force | Out-Null }

        $cellToLabel = @{}
        foreach ($f in $fieldMapping) {
            if ($f.target_cell -and $f.label) { $cellToLabel[$f.target_cell] = $f.label }
        }

        $allDumps = @()
        for ($di = 0; $di -lt $labels.Count; $di++) {
            $pageNo  = $di + 1
            $orderNo = $labels[$di].OrderNo   # 分割ページ対応: orderGroupsではなくlabels自身が持つ値を使う

            $topHeadNamed = [ordered]@{}
            foreach ($kv in ($labels[$di].TopHead.GetEnumerator() | Sort-Object Name)) {
                # 継続ページ(is_continuation)は実際にはTOP(1行目)+head_title(2～3行目)しか
                # 描画されない(4～8行目はPrintTitleRows相当の対象外)ため、期待値も同じ範囲に絞る
                if ($labels[$di].IsContinuation) {
                    $rowMatch = [regex]::Match($kv.Name, '\d+$')
                    if ($rowMatch.Success -and [int]$rowMatch.Value -gt 3) { continue }
                }
                $name = if ($cellToLabel.ContainsKey($kv.Name)) { $cellToLabel[$kv.Name] } else { $kv.Name }
                $topHeadNamed["$name ($($kv.Name))"] = $kv.Value
            }

            $bodyNamed = @()
            foreach ($body in $labels[$di].BodyRows) {
                $rowNamed = [ordered]@{}
                foreach ($kv in ($body.GetEnumerator() | Sort-Object Name)) {
                    $name = if ($cellToLabel.ContainsKey($kv.Name)) { $cellToLabel[$kv.Name] } else { $kv.Name }
                    $rowNamed["$name ($($kv.Name))"] = $kv.Value
                }
                $bodyNamed += , $rowNamed
            }

            $dump = [ordered]@{
                order_no        = $orderNo
                page_no         = $pageNo
                is_continuation = $labels[$di].IsContinuation   # $true=同一手配の2ページ目以降(11品目以上で分割)
                generated_at    = (Get-Date).ToString("s")
                top_head        = $topHeadNamed
                body_rows       = $bodyNamed
            }
            $allDumps += , $dump

            $safeOrderNo = $orderNo -replace '[\\/:*?"<>|]', '_'
            $suffix = if ($labels[$di].IsContinuation) { "_cont" } else { "" }
            $dumpPath = Join-Path $ExpectedDumpDir ("{0:D3}_{1}{2}.json" -f $pageNo, $safeOrderNo, $suffix)
            $dump | ConvertTo-Json -Depth 6 | Out-File -FilePath $dumpPath -Encoding UTF8
        }

        # パターンB(全ページ丸ごと1リクエスト)用に、全ページ分をまとめた配列も出力
        $allDumps | ConvertTo-Json -Depth 6 | Out-File -FilePath (Join-Path $ExpectedDumpDir "_all.json") -Encoding UTF8

        Write-Host "診断: 期待値JSONを $ExpectedDumpDir に $($labels.Count) 件出力しました"
    }
}
elseif (-not $CsvPath) {
    # サンプル帳票は -CsvPath 未指定時の手動描画確認専用。
    # 実CSV指定時に業務条件で0件になった場合はサンプルへフォールバックしない。
    $sampleTopHead = @{
        "B1" = "企画室"; "M1" = "T-000123"; "AH1" = "1/1"; "AS1" = "T-000123"
        "A2" = "20260810"; "H2" = "A"; "H3" = "B"; "J2" = "T-000123"
        "R3" = "山田商事"; "Z3" = "田中"; "AH3" = "営業一課"; "AM3" = "本社"
        "F4" = "株式会社サンプル造船"; "F5" = "サンプル受注先"; "F6" = "サンプル送り先"
        "F7" = "発注者A"; "K7" = "発注者B"; "P7" = "発注者C"; "U7" = "検査済"
        "Z7" = "サンプル海運"; "AG7" = "さくら丸"; "AP7" = "SB-001"; "AW7" = "サンプル造船所"
        "BE7" = "元払い"; "BJ7" = "10便"; "BO7" = "20便"
    }
    $sampleBodyRows = @(
        @{ "A9" = "1"; "F9" = "掛"; "K10" = "サンプル商品A"; "A11" = "PRD-0001"; "F12" = "10"; "M12" = "本" }
        @{ "A9" = "2"; "F9" = "掛"; "K10" = "サンプル商品B"; "A11" = "PRD-0002"; "F12" = "5";  "M12" = "個" }
    )
    $labels += , @{ OrderNo = "SAMPLE"; TopHead = $sampleTopHead; BodyRows = $sampleBodyRows; IsContinuation = $false; FixedScale = $false }
}

# ------------------------------------------------------------
# 5. 描画本体 (Preview/印刷 共通)
# ------------------------------------------------------------

# CheckLog: -CheckLogPath指定時のみ収集する。VBA側の対応する出力とCompare-Output.ps1で突合する。
$script:CheckLogEnabled = [bool]$CheckLogPath
$script:checkLog = New-Object System.Collections.Generic.List[object]

function Write-CheckLogCsv {
    if (-not $script:CheckLogEnabled) { return }
    $dir = Split-Path -Parent $CheckLogPath
    if ($dir -and -not (Test-Path $dir)) { New-Item -ItemType Directory -Path $dir -Force | Out-Null }
    $script:checkLog | Export-Csv -Path $CheckLogPath -NoTypeInformation -Encoding UTF8
    Write-Host "診断: CheckLogを $CheckLogPath に $($script:checkLog.Count) 件出力しました"
}

function Draw-Block {
    param($Graphics, $Block, $YOffset, $DataOverride, $FontCache, $OffsetX, $OffsetY, $PenHair, $PenThin, $PageNo, $OrderNo)

    # Step2: 検収済みBODY明細はExcel/VBAのGray8相当で明細ブロック全体を網掛けする。
    if ($DataOverride -and $DataOverride.ContainsKey("__Gray8") -and $DataOverride["__Gray8"]) {
        $gray8 = [System.Drawing.Color]::FromArgb(217, 217, 217)
        $grayBrush = New-Object System.Drawing.SolidBrush($gray8)
        try {
            $bx0 = $OffsetX
            $by0 = $OffsetY + $YOffset
            $bx1 = $OffsetX + $TOTAL_W
            $by1 = $OffsetY + $YOffset + (YOfRow ($Block.EndRow + 1)) - (YOfRow $Block.StartRow)
            $Graphics.FillRectangle($grayBrush, $bx0, $by0, ($bx1 - $bx0), ($by1 - $by0))
        } finally { $grayBrush.Dispose() }
    }

    for ($r = $Block.StartRow; $r -le $Block.EndRow; $r++) {
        for ($c = 1; $c -le $MAX_COL; $c++) {
            $letter = ColIndexToLetter $c
            $cellKey = "$letter$r"
            $info = $cellMap[$cellKey]
            $hasOverride = $DataOverride -and $DataOverride.ContainsKey($cellKey)

            # レイアウト定義(罫線/書式)が無いセルでも、実データの上書き値があれば描画対象にする
            # (元Excelで罫線も既定値も持たない「データ書込み専用セル」は抽出時に記録されていないため)
            if (-not $info -and -not $hasOverride) { continue }

            $merge = FindMerge $c $r
            if ($merge.MinCol -ne $c -or $merge.MinRow -ne $r) { continue }  # 結合セルの左上のみ描画

            $x0 = $OffsetX + $xPos[$letter]
            $y0 = $OffsetY + $YOffset + (YOfRow $r) - (YOfRow $Block.StartRow)
            $maxColLetter = ColIndexToLetter $merge.MaxCol
            $x1 = $OffsetX + $xPos[$maxColLetter] + $colPx[$maxColLetter]
            $y1 = $OffsetY + $YOffset + (YOfRow ($merge.MaxRow + 1)) - (YOfRow $Block.StartRow)

            # 線種(hair=極細/thin=通常)に応じてペンを選択。元Excelの見た目の太さの違いを再現する。
            function Get-Pen($side) {
                if (-not $side) { return $null }
                if ($side.style -eq "thin") { return $PenThin }
                return $PenHair
            }

            if ($info -and $info.border) {
                $pBottom = Get-Pen $info.border.bottom
                $pTop    = Get-Pen $info.border.top
                $pLeft   = Get-Pen $info.border.left
                $pRight  = Get-Pen $info.border.right
                if ($pBottom) { $Graphics.DrawLine($pBottom, $x0, $y1 - 1, $x1, $y1 - 1) }
                if ($pTop)    { $Graphics.DrawLine($pTop, $x0, $y0, $x1, $y0) }
                if ($pLeft)   { $Graphics.DrawLine($pLeft, $x0, $y0, $x0, $y1) }
                if ($pRight)  { $Graphics.DrawLine($pRight, $x1 - 1, $y0, $x1 - 1, $y1) }
            }

            $value = if ($hasOverride) { $DataOverride[$cellKey] } elseif ($info) { $info.value } else { $null }

            # CheckLog記録: 実データが書き込まれたセル(静的ラベルは対象外)の
            # 値と位置(ポイント単位。VBAのRange.Left/Top/Width/Heightと同じ単位にして
            # Compare-Output.ps1側で直接比較できるようにする。px→pt = px×72/96)
            if ($hasOverride -and $value -and $script:CheckLogEnabled) {
                $script:checkLog.Add([PSCustomObject]@{
                    Page    = $PageNo
                    OrderNo = $OrderNo
                    Cell    = $cellKey
                    Value   = [string]$value
                    X       = [math]::Round(($x0) * 72 / 96, 1)
                    Y       = [math]::Round(($y0) * 72 / 96, 1)
                    Width   = [math]::Round(($x1 - $x0) * 72 / 96, 1)
                    Height  = [math]::Round(($y1 - $y0) * 72 / 96, 1)
                }) | Out-Null
            }

            if ($value) {
                # セルごとの実フォント(游ゴシック/Code39等)・サイズ・太字指定をそのまま使う
                # (レイアウト抽出漏れのセル=データ書込み専用セルは、シート標準の11ptにフォールバック)
                $fontName = if ($info -and $info.font -and $info.font.name) { $info.font.name } else { "Yu Gothic" }
                $fontSize = if ($info -and $info.font -and $info.font.size) { $info.font.size } else { 11 }
                $isBold   = $info -and $info.font -and $info.font.bold
                $fontKey  = "$fontName|$fontSize|$isBold"

                if (-not $FontCache.ContainsKey($fontKey)) {
                    $style = if ($isBold) { [System.Drawing.FontStyle]::Bold } else { [System.Drawing.FontStyle]::Regular }
                    try {
                        $FontCache[$fontKey] = New-Object System.Drawing.Font($fontName, $fontSize, $style)
                    }
                    catch {
                        # 指定フォント(Code39等)が端末に無い場合は代替フォントにフォールバック
                        $FontCache[$fontKey] = New-Object System.Drawing.Font("Yu Gothic", $fontSize, $style)
                    }
                }
                $font = $FontCache[$fontKey]

                # セルの矩形で厳密にクリップしてから描画する。バーコード用フォント(Code39)は行の高さより
                # 大きいサイズが指定されているため、クリップしないと下の行の文字と重なってしまう
                # (元Excelでもセルの外にはみ出す部分は表示されない=クリップされているのと同じ見た目)
                # ※以前は縦方向に余裕を持たせていたが、下の行(J2等)に重なる不具合が出たため
                #   厳密なセル矩形に戻す。読み取り精度はアンチエイリアシング無効化側で確保する。
                # ※ただし静的ラベル文字(データ上書きではない固定テキスト)は、Excel側の「右セルが
                #   空なら文字があふれて表示される」仕様を再現するため、クリップをスキップする
                $isStaticLabel = -not $hasOverride
                if (-not $isStaticLabel) {
                    $clipRect = New-Object System.Drawing.RectangleF($x0, $y0, ($x1 - $x0), ($y1 - $y0))
                    $prevClip = $Graphics.Clip
                    $Graphics.SetClip($clipRect)
                }

                # バーコードフォント(Code39)はアンチエイリアシングによる滲みでスキャナーが読み取れなくなるため、
                # 白黒がくっきり分かれる描画モードに切り替える(日本語テキスト部分は通常の高品質描画のまま)
                $isBarcodeFont = $fontName -match "(?i)code ?39"
                if ($isBarcodeFont) {
                    $prevTextHint = $Graphics.TextRenderingHint
                    $prevSmoothing = $Graphics.SmoothingMode
                    $Graphics.TextRenderingHint = [System.Drawing.Text.TextRenderingHint]::SingleBitPerPixelGridFit
                    $Graphics.SmoothingMode = [System.Drawing.Drawing2D.SmoothingMode]::None
                }

                $Graphics.DrawString([string]$value, $font, [System.Drawing.Brushes]::Black, ($x0 + 3), ($y0 + 1))

                if ($isBarcodeFont) {
                    $Graphics.TextRenderingHint = $prevTextHint
                    $Graphics.SmoothingMode = $prevSmoothing
                }

                if (-not $isStaticLabel) {
                    $Graphics.Clip = $prevClip
                }
            }
        }
    }
}

# 1手配分(top+head+body×品目数)を描画。$Label = @{ TopHead=@{}; BodyRows=@(@{},...) }
# $IsContinuation=$true の場合、元VBAのPrintTitleRows="$1:$3"相当として
# headの代わりにhead_title(2～3行目のみ)を描画する(11品目以上の手配の2ページ目以降)。
# $PageNo/$OrderNoはCheckLog記録用(-CheckLogPath指定時のみ使用、それ以外は無視される)。
function Draw-OneLabel {
    param($Graphics, $Label, $OffsetX = 20, $OffsetY = 20, [bool]$IsContinuation = $false, $PageNo = $null, $OrderNo = $null)

    # セルごとに実際のフォント(游ゴシック/Code39等)を都度生成すると遅いのでキャッシュする
    $fontCache = @{}

    # 線種の太さ (Excel上の "hair"=極細線 / "thin"=通常の細線 の見た目の差を再現)
    $penHair = New-Object System.Drawing.Pen([System.Drawing.Color]::Black, 0.5)
    $penThin = New-Object System.Drawing.Pen([System.Drawing.Color]::Black, 1.25)

    $headBlock = if ($IsContinuation) { $blocks.head_title } else { $blocks.head }

    $yCursor = 0
    Draw-Block -Graphics $Graphics -Block $blocks.top  -YOffset $yCursor -DataOverride $Label.TopHead -FontCache $fontCache -OffsetX $OffsetX -OffsetY $OffsetY -PenHair $penHair -PenThin $penThin -PageNo $PageNo -OrderNo $OrderNo
    $yCursor += (YOfRow ($blocks.top.EndRow + 1)) - (YOfRow $blocks.top.StartRow)

    Draw-Block -Graphics $Graphics -Block $headBlock -YOffset $yCursor -DataOverride $Label.TopHead -FontCache $fontCache -OffsetX $OffsetX -OffsetY $OffsetY -PenHair $penHair -PenThin $penThin -PageNo $PageNo -OrderNo $OrderNo
    $yCursor += (YOfRow ($headBlock.EndRow + 1)) - (YOfRow $headBlock.StartRow)

    foreach ($item in $Label.BodyRows) {
        Draw-Block -Graphics $Graphics -Block $blocks.body -YOffset $yCursor -DataOverride $item -FontCache $fontCache -OffsetX $OffsetX -OffsetY $OffsetY -PenHair $penHair -PenThin $penThin -PageNo $PageNo -OrderNo $OrderNo
        $yCursor += (YOfRow ($blocks.body.EndRow + 1)) - (YOfRow $blocks.body.StartRow)
    }

    return $yCursor
}

# 1手配分の推定描画高さ(px)を、実際に描画せず概算する(ページ割付/プレビューの縦サイズ計算用)
function Estimate-LabelHeight($Label, [bool]$IsContinuation = $false) {
    $headBlock = if ($IsContinuation) { $blocks.head_title } else { $blocks.head }
    $h = (YOfRow ($blocks.top.EndRow + 1)) - (YOfRow $blocks.top.StartRow)
    $h += (YOfRow ($headBlock.EndRow + 1)) - (YOfRow $headBlock.StartRow)
    $h += $Label.BodyRows.Count * ((YOfRow ($blocks.body.EndRow + 1)) - (YOfRow $blocks.body.StartRow))
    return $h
}

# ------------------------------------------------------------
# 6. 実行: プレビュー(PNG保存) or 印刷スプールへ送信
# ------------------------------------------------------------
Write-Host "診断: TOTAL_W = $TOTAL_W  (型: $($TOTAL_W.GetType().Name))"
$splitCount = @($labels | Where-Object { $_.IsContinuation }).Count
Write-Host "診断: 出力対象の手配件数 = $($orderGroups.Count)  / 印刷ページ数 = $($labels.Count)$(if ($splitCount -gt 0) { "(11品目以上での分割継続ページ $splitCount 枚を含む)" })"

# v0.10.4 / Step2 T02修正:
# 実CSVを指定して処理した結果、印刷対象が0件なら正常な「印刷なし」として終了する。
# 旧来のサンプル帳票は -CsvPath 未指定時だけ使用し、業務データ0件時には生成・送信しない。
if ($CsvPath -and $labels.Count -eq 0) {
    Write-Host "診断: 実CSVの出力対象が0件のため、印刷ジョブは送信せず正常終了します。"
    Write-CheckLogCsv
    exit 0
}

$bmpWidth  = [int]([math]::Ceiling($TOTAL_W)) + 40

# プレビューは1枚のPNGに縦連結する仕様のため、件数が多いとGDI+のBitmap上限(高さ)を超えて
# "GDI+ で汎用エラーが発生しました" になる。-MaxOrders未指定かつ大量件数の場合は自動的に絞る。
$PREVIEW_SAFE_LIMIT = 15
if ($PreviewOnly -and $MaxOrders -eq 0 -and $labels.Count -gt $PREVIEW_SAFE_LIMIT) {
    Write-Host "警告: プレビューは1枚のPNGに全件縦連結するため、$($labels.Count)件のままだと画像が巨大になりGDI+エラーになります。" -ForegroundColor Yellow
    Write-Host "      先頭 $PREVIEW_SAFE_LIMIT 件のみに自動的に絞り込みます(全件確認したい場合は -PreviewOnly を外して実際に印刷してください)。" -ForegroundColor Yellow
    $labels = $labels | Select-Object -First $PREVIEW_SAFE_LIMIT
}

if ($PreviewOnly) {
    try {
        # 全手配を縦に連結した高さを見積もる(プレビューは1枚のPNGに全件並べる簡易仕様)
        $totalHeight = 40
        foreach ($lbl in $labels) { $totalHeight += (Estimate-LabelHeight $lbl $lbl.IsContinuation) + 20 }
        Write-Host "診断: Bitmapサイズ = ${bmpWidth} x ${totalHeight}"

        $bmp = New-Object System.Drawing.Bitmap($bmpWidth, $totalHeight)
        $g = [System.Drawing.Graphics]::FromImage($bmp)
        $g.Clear([System.Drawing.Color]::White)

        $yOffset = 20
        $previewPageNo = 0
        foreach ($lbl in $labels) {
            $previewPageNo++
            $used = Draw-OneLabel -Graphics $g -Label $lbl -OffsetX 20 -OffsetY $yOffset -IsContinuation $lbl.IsContinuation -PageNo $previewPageNo -OrderNo $lbl.OrderNo
            $yOffset += $used + 20   # 手配間の余白
        }

        # 絶対パスへ変換 (相対パスだと作業ディレクトリ次第で書込み失敗することがあるため)
        $fullOutPath = [System.IO.Path]::GetFullPath((Join-Path (Get-Location) $PreviewOutPath))
        Write-Host "診断: 保存先(絶対パス) = $fullOutPath"

        if (Test-Path $fullOutPath) {
            Remove-Item $fullOutPath -Force -ErrorAction SilentlyContinue
        }

        # Save(path)直呼び出しはGDI+の汎用エラーが出ることがあるため、FileStream経由で保存
        $fs = New-Object System.IO.FileStream($fullOutPath, [System.IO.FileMode]::Create, [System.IO.FileAccess]::Write)
        try {
            $bmp.Save($fs, [System.Drawing.Imaging.ImageFormat]::Png)
        }
        finally {
            $fs.Close()
        }

        $g.Dispose(); $bmp.Dispose()
        Write-Host "プレビュー画像を保存しました: $fullOutPath"
        Write-CheckLogCsv
    }
    catch {
        Write-Host "===== エラー詳細 =====" -ForegroundColor Red
        Write-Host "メッセージ: $($_.Exception.Message)"
        Write-Host "種類      : $($_.Exception.GetType().FullName)"
        if ($_.Exception.InnerException) {
            Write-Host "内部例外  : $($_.Exception.InnerException.Message)"
        }
        Write-Host "発生位置  : $($_.InvocationInfo.PositionMessage)"
        throw
    }
}
else {
    try {
        $printDoc = New-Object System.Drawing.Printing.PrintDocument

        # プリンター名指定時のみ切替。未指定ならOS既定のデフォルトプリンターのまま。
        if ($PrinterName) {
            $printDoc.PrinterSettings.PrinterName = $PrinterName
            if (-not $printDoc.PrinterSettings.IsValid) {
                throw "指定されたプリンター '$PrinterName' が見つからないか無効です。Get-Printer で名前を確認してください。"
            }
        }

        # A4横向き・余白を最小化 (元VBA: Orientation=xlLandscape, 上0.5cm/下0cm/左0.25cm/右0cm相当)
        # 2026/08/11 天野さん指摘: 用紙サイズはプリンターの既定値任せだったため、
        # 業務ルール「A4横・1ページ最大10品目」に合わせてA4を明示指定するよう修正。
        $a4PaperSize = $printDoc.PrinterSettings.PaperSizes | Where-Object { $_.Kind -eq [System.Drawing.Printing.PaperKind]::A4 } | Select-Object -First 1
        if ($a4PaperSize) {
            $printDoc.DefaultPageSettings.PaperSize = $a4PaperSize
        }
        else {
            # プリンタードライバーがA4を返さない場合のフォールバック(A4 = 8.27x11.69インチ = 1/100インチ単位で827x1169)
            Write-Warning "プリンターの用紙サイズ一覧にA4が見つかりませんでした。固定サイズ(827x1169)でフォールバックします。"
            $printDoc.DefaultPageSettings.PaperSize = New-Object System.Drawing.Printing.PaperSize("A4", 827, 1169)
        }
        $printDoc.DefaultPageSettings.Landscape = $true
        $printDoc.DefaultPageSettings.Margins = New-Object System.Drawing.Printing.Margins(10, 0, 20, 0)  # VBA: 左0.25cm/右0cm/上0.5cm/下0cm 相当 (1/100インチ)

        Write-Host "診断: 使用プリンター = $($printDoc.PrinterSettings.PrinterName)"
        Write-Host "診断: デフォルトプリンターか = $($printDoc.PrinterSettings.IsDefaultPrinter)"
        Write-Host "診断: プリンターは有効か     = $($printDoc.PrinterSettings.IsValid)"
        Write-Host "診断: 用紙サイズ = $($printDoc.DefaultPageSettings.PaperSize.PaperName)  横向き=$($printDoc.DefaultPageSettings.Landscape)"

        # 1手配 = 1ページ、ただし11品目以上の手配は10品目ごとに複数ページへ分割
        # (元VBA PrintableShapes()の会社の慣習ルールに合わせる)。
        # 10品目以下のページは印字可能領域に収まるよう自動縮小(scaleX/scaleY)、
        # 11品目以上で分割されたページ群は元VBAのZoom=59/56相当の固定縮尺を使う。
        $script:pageIndex = 0
        $printDoc.add_PrintPage({
            param($sender, $e)
            $lbl = $labels[$script:pageIndex]

            $contentWidth  = $TOTAL_W
            $contentHeight = (Estimate-LabelHeight $lbl $lbl.IsContinuation)
            # v0.10.2: PrintPageではDraw-OneLabelをOffsetX=0/OffsetY=0で描画するため、
            # Fit計算にもプレビュー用の仮想余白(+40px)を含めない。

            # PrintPageのGraphics既定座標単位は"1/100インチ"だが、contentWidth/Heightは
            # 96dpi換算のピクセル値のため、比率計算前に単位を1/100インチへ変換する
            $contentWidthHundredths  = $contentWidth  * 100 / 96
            $contentHeightHundredths = $contentHeight * 100 / 96

            $marginBounds = $e.MarginBounds

            if ($lbl.FixedScale) {
                # 元VBA STEP_6: row_count > 10 の場合、自動縮小(FitToPagesTall)ではなく
                # 固定Zoom(通常59%、DocuWorks Printerのみ56%)を使う。ここでも同じ値を固定適用する。
                $scale = if ($printDoc.PrinterSettings.PrinterName -eq "DocuWorks Printer") { 0.56 } else { 0.59 }
            }
            else {
                # 元VBA: Zoom=False / FitToPagesWide=1 / FitToPagesTall=1。
                # そのため「縮小のみ」ではなく、印刷範囲が小さい場合は拡大も許可する。
                # 旧PSの scale<=1 制限はVBA Fit再現を妨げるため撤廃する。
                $scaleX = $marginBounds.Width  / $contentWidthHundredths
                $scaleY = $marginBounds.Height / $contentHeightHundredths
                $scale = [Math]::Min($scaleX, $scaleY)
            }

            # 元VBA(Excelのページ設定「水平方向で中央に印刷」相当)に合わせ、
            # 縮尺が縦方向(scaleY)で決まり横方向に余白が生じるケースでは、
            # 左詰めではなく左右中央に配置する(2026/08/10 天野さん指摘、縦は従来通り上詰め)。
            # 固定縮尺(FixedScale)ページも同じ中央寄せを適用する。
            $scaledContentWidth = $contentWidthHundredths * $scale
            $offsetXHundredths = $marginBounds.Left + [Math]::Max(0, ($marginBounds.Width - $scaledContentWidth) / 2)

            $pageKind = if ($lbl.FixedScale) { if ($lbl.IsContinuation) { "分割継続" } else { "分割1枚目" } } else { "通常" }
            Write-Host "診断: ページ$($script:pageIndex + 1)/$($labels.Count) [$($lbl.OrderNo) / $pageKind]  内容サイズ(px)=${contentWidth}x${contentHeight}  MarginBounds(1/100inch)=$($marginBounds.Width)x$($marginBounds.Height)  縮小率=$([math]::Round($scale,3))  水平オフセット(1/100inch)=$([math]::Round($offsetXHundredths - $marginBounds.Left,1))"

            $e.Graphics.TranslateTransform($offsetXHundredths, $marginBounds.Top)
            $e.Graphics.ScaleTransform($scale, $scale)

            Draw-OneLabel -Graphics $e.Graphics -Label $lbl -OffsetX 0 -OffsetY 0 -IsContinuation $lbl.IsContinuation -PageNo ($script:pageIndex + 1) -OrderNo $lbl.OrderNo | Out-Null

            $script:pageIndex++
            $e.HasMorePages = ($script:pageIndex -lt $labels.Count)
        })

        Write-Host "印刷スプールへ送信します... (全 $($labels.Count) 枚)"
        $printDoc.Print()
        Write-Host "印刷ジョブを送信しました。プリンターのキュー(印刷キュー画面)で状態を確認してください。"
        Write-CheckLogCsv
    }
    catch {
        Write-Host "===== エラー詳細 =====" -ForegroundColor Red
        Write-Host "メッセージ: $($_.Exception.Message)"
        Write-Host "種類      : $($_.Exception.GetType().FullName)"
        if ($_.Exception.InnerException) {
            Write-Host "内部例外  : $($_.Exception.InnerException.Message)"
        }
        Write-Host "発生位置  : $($_.InvocationInfo.PositionMessage)"
        throw
    }
}
