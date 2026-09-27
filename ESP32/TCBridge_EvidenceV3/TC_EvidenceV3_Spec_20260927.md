# TC106代替機 Evidence V3 仕様書 --- RX/TX共存 Second Poll Formal Gate

作成日: 2026-09-27\
状態: **実機試験前 / コードレビュー待ち**\
実TC106 Gate: **HOLD**

## 1. 目的

Evidence V2でDirection-B RX単体120秒はPASSした。一方、Second
Poll本来の目的である 「Direction-Bを連続受信しながらMain→TCのSEND /
RESET / SENS.ADJを送信しても、 TcMainTxのdeadline
overrunやRX欠落を生じないか」は未試験である。

V3は通信ロジックを変更せず、計測と試験入力だけを追加してRX/TX共存を120秒同期窓で検証する。

## 2. V2からの判断変更

V2で `poll_over1000_count` は +409だったが `poll_max_interval_us`
は試験前後とも1001usだった。 現コードでは `vTaskDelay(1)` は
`!tcMainTx.busy()` を含むidle条件でのみ実行されるため、
idleを含む全poll間隔の `>1000us` を即FAILとする旧Gateは廃止する。

V3では次を分離する。

-   legacy all-poll: 比較・監視用
-   idle poll: 情報指標
-   TX busy poll: Formal timing指標
-   `tcMainTx.overrunCount()`: TcMainTx自身が `lateBy > 1000us`
    でABORTした一次証拠

## 3. Pi→ESP試験コマンドは現行GUIと同一形式

原典: `NTC_1A_serial_comm_v1_3_4.py`

-   `USE_LSB=False`
-   checksum7 = `sum(payload) & 0x7F`
-   `send_packet()`はpayload末尾へchecksum7を追加し、そのまま送信
-   RESET payload:
    `01 + CH1 length(3) + CH1 tension + CH2 length(3) + CH2 tension + 00 00`
-   SEND payload: `02 + CH1 tension + CH2 tension + 00 00`
-   SENS.ADJ payload: `03 + 00 00 00 00 00 00`

V3自動試験では動作設定値を新たに持ち込まないため、全データ値を0とする。

実送信bytes:

``` text
SEND     02 00 00 00 00 02
RESET    01 00 00 00 00 00 00 00 00 00 00 01
SENS.ADJ 03 00 00 00 00 00 00 03
```

## 4. 120秒スケジュール

Pi loggerが `/dev/ttyUSB0` を単独所有する。GUI/CoolTerm/Serial
Monitorは同時使用しない。

``` text
t=0s    EVIDENCE_START E5 1D 53 54 ×3（30ms間隔）
t=20s   SEND
t=50s   RESET
t=80s   SENS.ADJ
t=120s  ESP Core0がEND snapshot
その後  ESP→Pi Evidence RESULT
```

コマンド時刻は最初のSTART送信を基準とする。ESPの正式window開始とのずれはSTART処理の短い通信/処理遅延のみで、
20/50/80秒は十分内側に置く。

## 5. ESP側追加計測

通信処理そのものは変更しない。追加するのは診断カウンタのみ。

### 5.1 poll busy/idle分離

`pollDiagBeforePoll(tcMainTx.busy())` とし、直前のinter-poll intervalを
現在のTX stateによりbusy/idleへ分類する。

Evidence START時にV3 window-localの以下4値だけを0へresetする。

-   `poll_busy_max_interval_us_window`
-   `poll_busy_over1000_count_window`
-   `poll_idle_max_interval_us_window`
-   `poll_idle_over1000_count_window`

legacyのboot-lifetime `poll_max_interval_us` / `poll_over1000_count`
はresetしない。

### 5.2 TcMainTx一次証拠

既存getter `tcMainTx.overrunCount()` をstart/end snapshotへ追加する。

TcMainTx本体の判定は変更しない:

``` cpp
lateBy = now - _nextDueUs;
if (lateBy > 1000) {
    _overrunCount++;
    ... ABORT ...
}
```

### 5.3 コマンド実行証拠

120秒window中のみ以下を計数する。

-   `tx_reset_started_count`: normal `sendReset()` 成功
-   `tx_send_started_count`: `sendTension()` 成功
-   `tx_sens_started_count`: `sendSensAdj()` 成功
-   `tx_done_count`: `consumeDoneFlag()` completion
-   `tx_aborted_done_count`: completionで`aborted=true`

これにより「Piが送った」だけでなく「ESPが3種を実際にTcMainTxへ開始させ、完了した」ことを確認する。

## 6. Evidence V3 RESULT

Version: `0x02`

24 counter pairs。payloadは:

``` text
duration                         4 bytes
24 counters × start/end × u32  192 bytes
----------------------------------------
payload                         196 bytes (0xC4)
```

全frame:

``` text
MAGIC       4  E5 1D E7 C5
VERSION     1  02
LENGTH      1  C4
PAYLOAD   196
CHECKSUM    2  uint16 additive, little-endian
----------------------------------------
TOTAL     204 bytes
```

counter order:

1.  poll_max_interval_us
2.  poll_over1000_count
3.  poll_busy_max_interval_us_window
4.  poll_busy_over1000_count_window
5.  poll_idle_max_interval_us_window
6.  poll_idle_over1000_count_window
7.  tc_main_tx_overrun_count
8.  tx_reset_started_count
9.  tx_send_started_count
10. tx_sens_started_count
11. tx_done_count
12. tx_aborted_done_count
13. c1_isr_edges
14. c1_edge_overflow
15. c2_decoded_bytes
16. c2_decode_errors
17. c2_timing_errors
18. c2_local_edge_overflow
19. c3_valid_frames
20. c3_frame_errors
21. c3_sync_misses
22. c3_resyncs
23. c4_queued_frames
24. c4_queue_drops

window-local poll
4項目はSTARTでresetするためstart=0、end=window実測値として格納する。

## 7. Formal判定案

### 必須PASS

-   Evidence Magic/Version/Length/Checksum/Durationすべて正常
-   loggerがSEND/RESET/SENS.ADJを各1回送信
-   `tx_send_started_count delta = 1`
-   `tx_reset_started_count delta = 1`
-   `tx_sens_started_count delta = 1`
-   `tx_done_count delta = 3`
-   `tx_aborted_done_count delta = 0`
-   `tc_main_tx_overrun_count delta = 0`
-   `c1_edge_overflow delta = 0`
-   `c2_decode_errors delta = 0`
-   `c2_timing_errors delta = 0`
-   `c2_local_edge_overflow delta = 0`
-   `c3_frame_errors delta = 0`
-   `c3_sync_misses delta = 0`
-   `c3_resyncs delta = 0`
-   `c4_queue_drops delta = 0`
-   Pi telemetry footer不良なし
-   Pi側に異常な長時間gapなし

### TX busy poll

`poll_busy_over1000_count_window = 0` を設計目標とする。 0であれば「TX
busy中は1000us以内にpollへ戻れた」という強い補助証拠になる。

もし0でなくても `tc_main_tx_overrun_count=0` だけで自動PASSにはせず、
busy max値・発生数・LA波形と併せて再レビューする。
この扱いにより「1000usという同じ数値の別指標」を再び混同しない。

idle側の `poll_idle_over1000_count_window`
はGate対象外で情報記録とする。

## 8. 実行前条件

-   Nano Every: 現行TC emulatorのまま
-   実TC106: 未接続
-   ESP: `TCBridge_EvidenceV3.ino`
-   Pi GUI: 停止
-   CoolTerm / Arduino Serial Monitor: 開かない
-   Pi port: `/dev/ttyUSB0`
-   可能ならLAでMain→TC TXとDirection-B RXを同時記録

Pi:

``` bash
cd ~/NTC-1A-Control/Python
python3 pi_logger_evidence_v3.py --port /dev/ttyUSB0
```

## 9. Gate

V3コードレビュー → NanoでV3実機120秒 → Evidence解析。

このV3がFormal PASSするまでは実TC106 GateはHOLD。
