# TCBridge_EvidenceV2

Evidence V1のUSB CDC回収経路による試験破壊を除去した、Second Poll 120秒同期EvidenceのReview Candidateです。

## 内容

- `TCBridge_EvidenceV2.ino` — ESP32-S3 Evidence V2
- `tc_message.hpp` / `tc_packet_phase3.hpp` — 現行ヘッダー
- `pi_logger_evidence_v2.py` — Pi側同期logger/parser
- `TC_EvidenceV2_Spec_20260927.md` — 仕様書
- `DIFF_vs_20260907.patch` — オリジナルとの差分
- `SHA256SUMS.txt` — ハッシュ

## 実行

ESPをArduino IDEでcompile/write後、Pi GUIを停止して:

```bash
python3 pi_logger_evidence_v2.py --port /dev/ttyUSB0
```

正常終了時は通常telemetry CSVとEvidence JSONが生成されます。timeout時はJSONを残してexit code 2です。

## 注意

- 実TC106接続GateはHOLDです。
- 本コードはNOT FROZENです。
- まずESPのcompile結果を確認し、その後Nano Everyで120秒実機試験を行ってください。
