# TC106 LA解析ツール `tc106_la_analyzer.py` v1.1.0 — 仕様・検証記録

- 作成日: 2026-09-28(v1.1.0: 独立レビュー反映)
- 対応仕様: `B_TC106_EvidenceV3_Rerun_LA_TestSpec_v1.3`(§4.6 / §6.6 / §6.7 / §8)
- **状態: REVIEW CANDIDATE(RUN_READYではない)。** 昇格の条件は§9。
- ツールSHA256: `fec62c08f841fe44fbf0e167c2e11902b6fa6294b1b1534aea68510267c557b3`
  (実行するたび、レポートJSONに自動記録される。1文字でも変えればこの値は変わる。旧v1.0.0の `f7c50665…c7ec` は破棄)

### 変更履歴 v1.0.0 → v1.1.0(ChatGPT独立レビュー REVISE への対応)

| 指摘 | 対応 |
|---|---|
| Evidence境界余裕を ±7ms → ±10ms に同期(**Blocking**) | `Params.BOUNDARY_MARGIN_MS = 10.0`。READMEの記述も同期。試験で数値を固定し(`TestMarginPinnedToSpec`)、7msへ戻すバグ注入を検出できることを確認 |
| ZIP内の `evidence/` 構造と `SHA256SUMS` の不一致 | ZIPを1本にまとめ、`SHA256SUMS` の相対パスとZIP内の構造を一致させた。**展開して検証コマンドを実行し、全件OK**を確認(下記「検証方法」) |
| 任意強化: `raw_hex` 3 packet完全一致と `checksum7_ok` | **`V3-CMD` を追加(判定対象)**。packetの完全一致、checksum7の**再計算**、ロガーの `checksum7_ok` フラグ、重複送信を確認 |

`V3-CMD` はBの§8の判定表にまだ無いID(ツール側で定義)です。**§8に追加してください。**

### ZIPの検証方法

```bash
unzip TC106_LA_Analyzer_v1.1.zip && cd TC106_LA_Analyzer_v1.1
shasum -a 256 -c SHA256SUMS.txt        # macOS   (Linux: sha256sum -c SHA256SUMS.txt)
```

個別ファイルを別々にダウンロードすると `evidence/` が平らになり、`SHA256SUMS` と一致しなくなります。**ZIPを使ってください。**

---

## 1. 何をするツールか

PulseViewの `.sr`(sigrokコンテナ)を**直接**読み、B仕様書の「機械判定できる項目」を判定します。

- PulseViewのデコーダや、その保存状態には依存しません(コンテナ・metadata・生ロジックを直接読む)
- Python標準ライブラリのみ(numpy等なし)。Python 3.7以上、3.12で試験済み
- 200Mサンプルを約0.7秒で走査します(エッジが疎なため、`bytes.translate`と`bytes.find`で処理)
- **閾値は `Params` クラスの定数で、コマンドラインで変更できません。** 変更すれば版とハッシュが変わります
- 宣言が欠けているとき(オフセットモデル未宣言など)は、PASSではなく**UNRESOLVED**になります
- 入力ファイルの不備は終了コード3で、**FAIL(1)と混同されません**
- `--out-json` の出力先に同名ファイルがあれば、上書きせず中止します

## 2. 判定項目

| ID | 判定内容(仕様の出典) | 判定方法 |
|---|---|---|
| LA-01 | capture範囲(§3.3, §7) | 窓が捕捉内に収まる / RESULT到着から後余裕≧10s / loggerの開始遅れD≦19s |
| LA-02 | sample rate | ≧1 MHz(metadataから) |
| LA-03 | TXとTP5の同時記録 | 両chがメタデータに存在し、両方にエッジがある |
| LA-04 | TX frame数 | SEND=1, RESET=1, SENS.ADJ=1、未識別・余分なTX eventは0 |
| LA-05 | 途中切断なし | 各frameが期待byte列に復号でき、edge数が期待と一致 |
| LA-06 | step/byte構造 | 期待byte列+markerの一致、隣接byte開始間隔=13slot(42.9ms)±165µs |
| LA-07 | TP5連続性 | 窓の中核で、Direction-Bフレーム間隔とedge間隔が≦505ms |
| LA-08 | LA↔Pi対応 | 3コマンドが一意に対応、**宣言したモデル**のoffset spread≦10ms |
| LA-09 | TP5 edge数 と `c1_isr_edges` | `E_LA_core ≦ E_ESP ≦ E_LA_core+E_start+E_end`(§4.6.5) |
| LA-10 | `.sr` 再オープン | **手動**(PulseViewでの確認。ツールは判定しない) |
| LA-11 | TX edge timing | 連続edge間隔 T で `n=round(T/3300µs)`, `|T−n×3300µs|≦165µs`(絶対値)、nが期待byte列の継続slot数と一致 |
| V3-01〜17, V3-BUSY | Evidence JSONの判定 | V3仕様§7どおり(BUSYは補助) |
| **V3-CMD**(v1.1追加) | Piが送った3 packet | `SEND=02 00 00 00 00 02` / `RESET=01 00…00 01`(12byte) / `SENS.ADJ=03 00 00 00 00 00 00 03` に**完全一致**、各packetのchecksum7を**再計算**して一致、ロガーの `checksum7_ok` が true、各コマンド1回のみ。全データ値0という試験の適用範囲もここで担保する |

期待するTXのbyte列(全データ値0):

| コマンド | byte列 | 構造 |
|---|---|---|
| SEND | `00 F3 00 FB` | 13slot×4byte |
| RESET | `00 00 00 00 00 F1 00 00 00 00 00 F9` | 13slot×12byte |
| SENS.ADJ | `F2` | 11slot×1byte |

### 観測できないため、PASSとは言えない部分(レポートの `not_observable` に明記)

- **SENS.ADJの11-step**: 1byteのみで、末尾のguardがidleのHIGHと区別できません。確認できるのはbyte値F2とmarkerだけです
- SEND/RESETの**最後のbyteの末尾guard**
- LA-06のPASSは、この範囲での「整合」を意味します

## 3. 使い方

```bash
# (a) 実ファイルの互換確認・チャンネル統計(判定ではない)
python3 tc106_la_analyzer.py info FILE.sr --compare D0 D1 --dirb D1 --progress

# (b) 試行解析(閾値は同じ、--formalなし)
python3 tc106_la_analyzer.py analyze --sr RUN.sr --evidence-json evidence_v3_report_STAMP.json \
    --tx D0 --rx D1 --offset-model WIRE_CORRECTED --out-json analysis_STAMP.json

# (c) Formal判定(§7手順13)
python3 tc106_la_analyzer.py analyze --formal --sr RUN.sr --evidence-json evidence_v3_report_STAMP.json \
    --tx <TX ch名> --rx <TP5 ch名> --offset-model <相関予行で宣言したモデル> \
    --out-json analysis_STAMP.json
```

- `--tx` と `--rx` は**必須で、既定値はありません**(チャンネルの意味をハードコードしない)
- TXを **TP4(Q2ゲート側)** で測る場合は `--tx-invert` を付けます。TP3(ドレイン=バス側)なら不要です。使ったかどうかは、レポートに `tx_inverted` として記録されます
- `--formal` は、`--offset-model` 未指定と `duration_ms≠120000` を拒否/FAILにします
- 終了コード: 0=自動判定項目すべてPASS、1=FAILあり、2=FAILは無いがUNRESOLVEDあり、3=入力/使い方の誤り

### 入力の前提

- TX線: アイドルHIGH、STARTがLOW(論理レベル=バスの電圧)。`TcMainTx` の `writeLogical()` とQ2(NMOS)の反転から、TP3で論理と電圧が一致する、というのが私の読みです
- Evidence JSON: `counters.*.delta`、`scripted_commands[].{name,sent_monotonic_s,raw_hex}`、`evidence_monotonic_s`、`duration_ms`、`normal_telemetry`(V3 loggerの出力そのまま)
- Pi時刻の原点は、最初のEVIDENCE_START送信の直前(`PI_START_TIMESTAMP_S=0`)
- エッジ時刻は「新しいレベルの最初のsampleの番号 / sample rate」

## 4. 検証結果(すべて `evidence/` に記録)

### 4.1 実物の `.sr`(`２００Mサンプル１Mz.sr`)での再現 — preflight報告書の数値と一致

| 項目 | 報告書 | ツール |
|---|---|---|
| sigrok / rate / probes / unitsize / chunks | 0.5.2 / 1MHz / 8 / 1 / 20 | 一致 |
| raw samples / duration | 200,000,000 / 200.000 s | 一致 |
| edge数 D0, D1 | 7,142 (3,571↑/3,571↓) | 一致 |
| first / last edge | 20.072 ms / 199.998627 s | 一致 |
| 最大 / 最小edge間隔 | 52.744 / 3.275 ms | 一致 |
| D1−D0のずれ | 0µs: 6,611、+1µs: 531 | 一致(最大1µs) |
| `.sr` SHA256 | 592c0cd6…0dbb | 一致 |

### 4.2 独立に書いたDirection-Bデコーダを、実Nano波形にかけた結果(新規)

D1を復号したところ、**594フレームがすべて `00 00 00 00 00`、同期ミス0**でした。byte間隔は55.969〜56.054ms(公称56.1ms)、フレーム間隔は335.886〜336.223msで、警告は0です。これはESPの実装とは独立の復号で、実波形と整合しました。

### 4.3 テスト63件(標準+実ファイル+実JSON+境界条件。150秒の実寸試験は `TC106_FULLSIZE=1` を付けて別実行)

- TXテンプレートの数値を、`TcMainTx` のスロット動作から独立に導出した値と照合(最後に見えるedge: 141.9 / 485.1 / 16.5 ms)
- 正常runがPASSし、**故障を注入した13種類(波形・カウンタ)**と、**Piコマンドの故障5種類**(データ値≠0、checksum不正、ロガーのフラグ偽り、packetは正しいがフラグFalse、二重送信)が正しくFAILすること(edge遅延300µs、途中切断、byte値違い、SENS.ADJ欠落、余分TX、グリッチ、フレーム内グリッチ、TP5の700ms欠落、ESPカウンタの過大/過小、offset spread超過、capture不足、logger開始遅れ)
- 各edge間隔は範囲内なのに隣接byteの開始間隔だけが外れる波形を、検出できること
- LA側クロックが200ppmずれた正常runを、誤ってFAILにしないこと
- 境界帯にedgeが入る位相で、LA-09の許容が「両端を含み、1つ超えたらFAIL」であること、位相を掃引しても真値が帯に収まること
- 未宣言・timeout・`--formal`違反が、PASSにならないこと
- コンテナ: チャンク境界ちょうどのエッジ、チャンクをまたぐレベル継続、`unitsize=2`の上位ch
- CLI: 極性違い、入力不備が終了コード3であること、上書き拒否

### 4.3.1 ミューテーション検査(テストが空振りしていないことの確認)

ツールに1行ずつバグを入れ、テストが落ちるかを調べました。**29件中28件を検出、見逃し(穴)は0件、残り1件は等価ミュータント**(理由付き)です。

この検査で、これまでに**4件の穴**が見つかり、すべて塞ぎました。v1.0の3件(下記)に加え、v1.1では「ロガーの `checksum7_ok` フラグを無視しても通る」が見つかりました(再計算でも同じ判定になり、フラグ単独の効果が見えなかったため、「packetは正しいのにフラグだけFalse」の試験を足しました)。v1.0の3件は次のとおりです。

1. LA-09の許容帯を無視しても通る → 合成シナリオの帯にedgeが入っていなかったのが原因で、位相探索の試験を追加しました
2. 隣接byte開始間隔の判定を無効化しても通る → 試験を追加しました。あわせて実装を修正しました(§7の判断1)
3. スロット数判定を消しても通る → 等価ミュータントです。「復号が期待byte列に一致し、edge数も一致」なら、スロット数は必ず一致します。二重の安全策として残しています

### 4.4 性能

150秒/1.5億サンプルの合成captureを `--formal` で解析: 生成0.4秒、解析0.3秒、終了コード0。

### 4.5 実データでの挙動

| 入力 | 結果 |
|---|---|
| 実 `.sr`(TX波形なし)+ 9/27のPASS run JSON | V3-01〜17 と **V3-CMD** がすべてPASS(私の手検算と一致)、LA-04/05/06 FAIL、その他UNRESOLVED、総合 `HAS_FAIL`、終了コード1 |
| 同 `.sr` + 9/28のtimeout JSON | V3-01〜15 UNRESOLVED、**V3-CMD はPASS**(Piの送信packetは正しかった) |

## 5. この検証で言えないこと(重要)

- **実際のTX波形での検証は、まだ一度もありません。** 期待波形(テンプレート)と合成波形は、どちらも `TcMainTx` のスロット動作の同じモデルから作っています。**同じ前提の思い込みが両方に入っていれば、テストはそれを検出できません。**(例: 極性、STARTの位置、markerの扱い)
- 対策: §6.7の相関予行で取る**実物の `.sr` を、`--formal` なしで解析**し、`tx.frames[].decode_diag` と `interval_rule.bad` を人間が見てください。差異があれば版を上げ、ハッシュを取り直してからFormal runに進みます
- Direction-Bデコーダは、実Nano波形(データ値0のみ)で検証しています。**非ゼロpayloadと実TC106は合成でのみ**確認しています
- ±10ms(境界余裕。v1.1で7→10に変更)は仕様書のDERIVED値です。START受信(約4.2ms)、Core1/Core0のポーリング(約2ms)、クロック差を見込んだ事前固定値で、**実測値ではありません**
- 私は実機のPulseViewや、実TX波形を持っていません。文書とコードと計算に基づく検証です

## 6. 仕様との対応で、私が判断した点(レビューで確認してください)

1. **隣接byte開始間隔(LA-06)**: 仕様は「隣接byte start間隔は§4.6.3のtiming規則で判定する」です。
   - 私は**隣接間隔を判定に使いました**(範囲: 42.9ms±165µs)
   - フレーム先頭からの累積偏差は**参考値**にしました。RESETは約0.5秒あり、LAの水晶誤差だけで約100µsずれるため、判定に使うと正常でも誤ってFAILする恐れがあります
2. **LA-11のスロット数一致(nが期待と一致)**: 仕様どおり実装しましたが、実質的には二重の安全策です(§4.3.1)
3. **TXフレームの分離**: 100ms超のidleで区切ります(仕様に無い解析パラメータ。フレーム内で最長の無edge区間は33msなので、約3倍の余裕)
4. **Direction-Bのbyte境界**: 26ms以上のHIGHの後の立下り(ESPのC1と同じ考え方の別実装)
5. **LA-01の前余裕**: 仕様に数値が無いので、窓が捕捉内に収まる(≧0)ことだけを見ています
6. **LA-09の窓の対応**: START時刻は `PI_START_TIMESTAMP_S=0` として、3コマンドのoffsetのmin/maxに±10msを加えた帯を使います(§4.6.5どおり)

## 7. Formal runでの使い方

1. 相関予行(§6.7)の `.sr` で、`--formal` なしで解析します(`decode_diag`を確認し、offset spreadの両モデルを見て、**採用モデルを宣言**)
2. ESPをリセットし、Formal runを実施します
3. 手順13: `analyze --formal ... --offset-model <宣言したモデル>`
4. レポートJSONを保存します(ツールのSHA256、入力のSHA256、全閾値が記録されます)
5. LA-10(PulseViewでの再オープン確認)を、手動で§8に記入します

## 8. クロスレビューの依頼点(仕様書が求める「解析ツールの独立クロスレビュー」)

- §6の判断1〜6が、仕様の意図に沿っているか
- `template_slot_levels()` の波形モデルが、実際の `TcMainTx::poll()` と一致しているか(特に、START=LOW、データ8bitのLSB first、marker=command byteでHIGH/data byteでLOW、guard=HIGH)
- `analyze_edges()` の各判定式が、B仕様書§3.3/§4.6/§8と一致しているか
- `Params` の定数が、仕様書の数値と一致しているか

## 9. RUN_READYに上げる条件

1. 独立クロスレビューの完了(ChatGPTと人間)
2. 相関予行の実 `.sr` を、`--formal` なしで解析し、`decode_diag` に想定外がないこと
3. TX観測点(TP3/TP4)、LAのチャンネル割当、LA型番と入力定格、電圧実測の確定
4. ツールのSHA256を、仕様書の§6.1に記録して固定

## 10. ファイル

| ファイル | 内容 |
|---|---|
| `tc106_la_analyzer.py` | ツール本体(Formalで固定するのはこのファイルだけ) |
| `tc106_la_synth.py` | 試験用の合成波形・`.sr`書き出し(Formalのハッシュ対象外) |
| `test_tc106_la_analyzer.py` | テスト63件 |
| `mutation_check.py` | ミューテーション検査 |
| `evidence/test_run.txt` | テストの実行ログ(標準/実ファイル/150秒/終了コード) |
| `evidence/mutation_check.txt` | ミューテーション検査の結果 |
| `evidence/preflight_info_report.json` | 実 `.sr` の `info` 出力(4.1の数値の元) |
| `SHA256SUMS.txt` | 上記ファイルのハッシュ(相対パス。ZIPを展開した直下で検証) |
