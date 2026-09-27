# TCBridge Evidence V2 仕様書

**版:** v2.0 Review Candidate\
**日付:** 2026-09-27\
**対象:** TC106代替機 / ESP32-S3 Second Poll Evidence\
**基準ソース:**
`ESP32_TC_Bridge_RTOS_Final_Silent_BootMagic_20260907.ino`\
**実装名:** `TCBridge_EvidenceV2.ino`\
**Piロガー:** `pi_logger_evidence_v2.py`

## 1. 目的

Second Poll formal
gateのため、ESP32-S3内部のPoll/C1/C2/C3/C4カウンタと、Piから見たDirection-B受信連続性を**同一の120秒測定窓**で取得する。

Evidence V1では120秒終了後のUSB CDC `Serial.print()`をCore1
`taskPiUart()`から自動実行したため、USBホスト未読時にCore1が約76.143秒停止し、Pi転送も同時に停止した。復帰時に`QUEUE_DEPTH_TC_TO_PI=8`と一致する8フレームが急速排出されたため、V1は測定系が対象系を破壊した
**INVALID** 試験として扱う。

V2ではEvidence経路からUSB CDCを完全に除外し、Pi UART (`SerialPi`)
のみで開始同期と結果回収を行う。

## 2. 設計原則

1.  20260907オリジナルの通常TC通信仕様を変更しない。
2.  `C4_USB_DEBUG=0`を維持し、Evidence目的では`Serial.begin()`/`Serial.print()`を使用しない。
3.  PiがEvidence STARTを送信して初めて120秒測定を開始する。
4.  start/end snapshotはCore0 `taskTcBus()`だけが取得する。
5.  END判定は必ず`gEvidenceStarted == true`でgateする。ESP
    uptimeだけでENDしてはならない。
6.  120秒終了時はend
    snapshotを先にRAMへ確定し、その後だけRESULTを送信する。
7.  RESULT送信はformal 120秒windowの外である。
8.  通常Direction-B 6byte telemetryとEvidence RESULTをPi
    parserで明確に分離する。
9.  実TC106接続Gateは、本V2実機試験がPASSするまでHOLDとする。

## 3. Evidence STARTプロトコル

Pi→ESPの診断専用4byte列。

``` text
EVIDENCE_START_MAGIC = E5 1D 53 54
```

`53 54`はASCIIの`ST`に相当する。既存Pi→ESPコマンド`01/02/03/10/11`とは一致しない。

### 3.1 ESPでの検知条件

START検知はCore1
`taskPiUart()`の**`WAIT_CMD`状態だけ**で行う。`COLLECTING`中のRESET/SEND/SENS.ADJ等のpayloadはEvidence判定に使用しない。

直近4byteのshift
registerで`E5 1D 53 54`を検出し、検出時はCore1からCore0へ

``` cpp
gEvidenceStartRequested.store(true);
```

だけを通知する。snapshotはCore1では取得しない。

通常parserが`WAIT_CMD -> COLLECTING`へ遷移するとき、および`COLLECTING -> WAIT_CMD`へ戻るときは`evidenceStartShiftBuf=0`とし、状態境界をまたいだ疑似MAGIC成立を禁止する。

### 3.2 複数STARTへの耐性

Pi
loggerはSTARTを3回、既定30ms間隔で送信する。ESPは`gEvidenceStarted`により初回だけを採用する。2回目以降は測定を再開始しない。

## 4. Core間同期と120秒window

共有状態:

``` cpp
std::atomic<bool> gEvidenceStartRequested{false}; // Core1 -> Core0
std::atomic<bool> gEvidenceStarted{false};        // Core0確定
std::atomic<bool> gEvidenceReady{false};          // Core0 END確定
```

Core0側の必須ロジック:

``` cpp
if (!gEvidenceStarted.load() && gEvidenceStartRequested.load()) {
    captureEvidenceSnapshot(true);
    gEvidenceStartMs = millis();
    gEvidenceStarted.store(true);
}

if (gEvidenceStarted.load() &&
    !gEvidenceReady.load() &&
    (uint32_t)(millis() - gEvidenceStartMs) >= TC_EVIDENCE_DURATION_MS) {
    captureEvidenceSnapshot(false);
    gEvidenceReady.store(true);
}
```

**最重要条件:**
2番目のEND条件から`gEvidenceStarted.load()`を削除してはならない。START前の`gEvidenceStartMs=0`を基準にESP起動120秒後に誤ENDするのを防止する。

## 5. snapshot項目

start/endそれぞれ以下14項目を保持する。

    No. Field
  ----- --------------------------
      1 `poll_max_interval_us`
      2 `poll_over1000_count`
      3 `c1_isr_edges`
      4 `c1_edge_overflow`
      5 `c2_decoded_bytes`
      6 `c2_decode_errors`
      7 `c2_timing_errors`
      8 `c2_local_edge_overflow`
      9 `c3_valid_frames`
     10 `c3_frame_errors`
     11 `c3_sync_misses`
     12 `c3_resyncs`
     13 `c4_queued_frames`
     14 `c4_queue_drops`

C1 ISR共有値は`c1RingMux` critical
section内で取得する。C2/C3/C4はCore0所有値なのでCore0から取得する。Poll値はatomicから取得する。

## 6. Evidence RESULTフレーム

ESP→Pi。全multi-byte integerは**little-endian**。

  --------------------------------------------------------------------------------------------------------
                Offset                 Size Field           内容
  -------------------- -------------------- --------------- ----------------------------------------------
                     0                    4 MAGIC           `E5 1D E7 C5`

                     4                    1 VERSION         `01`

                     5                    1 LENGTH          `74` hex = 116 decimal

                     6                    4 duration_ms     uint32 = 120000

                    10                  112 counters        14項目 × (start,end) × uint32

                   122                    2 CHECKSUM16      offset
                                                            0..121の全byte加算和&`0xFFFF`、little-endian

                                    **124**                 **総byte数**
  --------------------------------------------------------------------------------------------------------

`LENGTH`はpayloadのみで、`duration_ms(4)+counters(112)=116`。

counter
payload順は§5の順序で、各項目を`start(uint32), end(uint32)`の順に格納する。

CHECKSUM16はCRCではない。名称はESP/Python/JSON/仕様書すべて`checksum`で統一する。

## 7. ESP RESULT送信条件

Core1は通常`qTcToPi`
telemetryをドレインした後、`gEvidenceReady==true`かつ未送信なら124byte
RESULTを1回だけ送信する。

``` cpp
SerialPi.write(evidenceFrame, 124);
SerialPi.flush();
```

RESULTは1回の連続writeで送信し、途中へ通常telemetryを割り込ませない。`SerialPi`を書き込むのは同一Core1
taskであるため、この124byte内のinterleaveを発生させない。

9600 baudではRESULT送信に約129msを要するが、end
snapshot確定後なのでformal 120秒windowの評価値には含めない。

## 8. Pi logger仕様

標準実行:

``` bash
python3 pi_logger_evidence_v2.py --port /dev/ttyUSB0
```

既定値:

-   baud: 9600
-   START送信: 3回
-   START間隔: 30ms
-   timeout: 150秒
-   telemetry CSV: `pi_rx_evidence_v2.csv`
-   Evidence JSON: `evidence_report_YYYYMMDD_HHMMSS.json`

### 8.1 parser優先順位

1.  Evidence RESULT MAGIC `E5 1D E7 C5`
2.  Boot Magic `DE AD BE EF 01 7F`
3.  通常6byte Direction-B telemetry
4.  それ以外は1byteずつresync

Evidence MAGICを認識したら、VERSION/LENGTH/PAYLOAD/CHECKSUMをEvidence
parserで完結させ、そのbyte列を6byte telemetry parserへ渡さない。

通常telemetry判定は従来と同じ:

-   6byte固定
-   byte5=`0x7F`
-   byte0..4=`0..99`

### 8.2 CSV

通常telemetryだけを記録する。

``` text
FrameNo, Timestamp, Monotonic_s, Raw_Hex,
Interval_ms, Footer_OK, Wind_Length, Actual_Tension
```

### 8.3 JSON

Evidence RESULTが正常なら最低限以下を記録する。

-   `magic_ok`
-   `version`, `version_ok`
-   `length`, `length_ok`
-   `checksum_type`
-   `checksum_received`
-   `checksum_calculated`
-   `checksum_ok`
-   `duration_ms`, `duration_ok`
-   14 counterの`start/end/delta`
-   `evidence_received_at`
-   `evidence_monotonic_s`
-   normal telemetryの受信frame数、footer正常数、最大interval、Boot
    Magic数、resync数

checksum-validでもVERSIONが1以外ならV2正常終了には使用しない。

### 8.4 timeout

最初のSTART送信直前をPi側測定時刻0とする。150秒以内にchecksum/version/length/duration正常なRESULTを受信しなければ、`evidence_received=false`をJSONへ保存しexit
code 2で終了する。無限待ちは禁止する。

## 9. Formal Gate判定項目

Second Pollの主要ゼロdelta条件:

``` text
poll_over1000_delta = 0
c1_edge_overflow_delta = 0
c2_decode_errors_delta = 0
c2_timing_errors_delta = 0
c2_local_edge_overflow_delta = 0
c3_frame_errors_delta = 0
c4_queue_drops_delta = 0
```

加えて`c3_sync_misses_delta`、`c3_resyncs_delta`、Pi
CSVのframe連続性/最大gapを確認する。

V2コードが動作しただけではPASSとしない。実測JSON+CSVを解析して初めてGate判定する。

## 10. 実機試験手順

1.  Nano
    Everyは現行`NanoEvery_TCEmulator_V2_8_Original250201.ino`、Direction-B連続送信のまま。
2.  ESPへ`TCBridge_EvidenceV2.ino`を書き込む。
3.  Pi GUIを終了し、`/dev/ttyUSB0`を解放する。
4.  Piで`pi_logger_evidence_v2.py --port /dev/ttyUSB0`を実行する。
5.  SEND/RESET/SENS.ADJ操作はしない。
6.  loggerがSTARTを送信し、ESPがCore0でstart snapshotを確定する。
7.  120秒間、通常Direction-BをCSVへ記録する。
8.  ESPがend snapshotを確定後、124byte RESULTをPiへ送る。
9.  Piがchecksum等を検証し、CSV+JSONを保存して終了する。
10. CSV+JSONを解析してFormal Gateを判定する。

## 11. 実装レビュー必須チェック

-   [ ] 基準が20260907オリジナルである
-   [ ] `C4_USB_DEBUG=0`
-   [ ] Evidence経路にUSB CDC `Serial.print/println`がない
-   [ ] START=`E5 1D 53 54`
-   [ ] START検知が`WAIT_CMD`限定
-   [ ] parser状態境界でSTART shift bufferをclear
-   [ ] Core1はSTART requestだけを立てる
-   [ ] start snapshotはCore0
-   [ ] **END条件に`gEvidenceStarted` gateがある**
-   [ ] end snapshotがRESULT送信より先
-   [ ] RESULT=`E5 1D E7 C5`, version=1, payload=116, total=124
-   [ ] 14 counter順序が仕様と一致
-   [ ] CHECKSUM16対象がoffset 0..121
-   [ ] RESULTが1回の`SerialPi.write()`で送られる
-   [ ] PythonがEvidenceをtelemetry parserへ流さない
-   [ ] Python timeoutがある
-   [ ] 実TC106 GateはHOLD

## 12. 今回作成コードの検証状態

-   Python: 構文チェック済み。
-   Evidence RESULT parser: 124byte合成データでpayload/counter
    decodeを確認済み。
-   ESP: 20260907オリジナルからのdiffを同梱し、START
    gate、USB非使用、RESULT送信先、payload長static_assertを機械チェック済み。
-   **Arduino IDE/ESP32-S3実コンパイルはこの作成環境では未実施。**
    実機書込み前にArduino
    IDEでcompileを行い、その結果を次のレビュー対象とする。

------------------------------------------------------------------------

**状態:** Review Candidate / NOT FROZEN / Real TC106 Gate HOLD
