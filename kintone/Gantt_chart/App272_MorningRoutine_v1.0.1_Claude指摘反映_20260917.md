# App272 Morning Routine v1.0.1 — Claudeレビュー指摘反映
作成日: 2026-09-17

## Claude判定
`Run-App272-MorningRoutine_v1.0.ps1` に対する判定は **GO WITH CONDITIONS（軽微）**。

唯一の推奨事項:
- タスクスケジューラから成否を明確に判定できるよう、成功時 `exit 0`、失敗時 `exit 1` を明示する。

## v1.0.1での対応
- `$script:ExitCode = 1` を初期値に設定
- DryRun成功時: `$script:ExitCode = 0`
- Execute + Final Verify成功時: `$script:ExitCode = 0`
- 例外時: `catch` で `$script:ExitCode = 1`
- TMP cleanup失敗時: 業務処理が成功していても `$script:ExitCode = 1`
- 最後に `exit $script:ExitCode`

タスクスケジューラでは
- `0x0` = 正常終了
- 非0 = 異常、またはcleanup異常
として扱える。

## 既存安全設計は維持
- App270 READ ONLY
- App272 PROD
- App292 TEST token不使用
- TMP成功時削除／失敗時保持
- 同日TMP残存時SafetyStop
- Phase1C / PhaseB事前DryRun
- Phase1C POST後のPhaseB再DryRun
- Protected > 0 で無人Execute停止
- Updates > 50 で停止
- Expected PO集合一致
- revision/current-before/changeHash
- 最終 Phase1C NEW=0
- 最終 PhaseB Updates=0 / Protected=0

## Formatterレビューは未完了
MorningRoutineの正式I/F:
```powershell
-InputPath <raw csv>
-OutputPath <formatted csv>
```

Formatter実ファイル提示後に以下を確認する:
1. `InputPath` / `OutputPath` I/F一致
2. CP932 / RFC4180処理
3. 出力ファイル生成
4. エラー時の終了コード
5. 入出力同一ファイル誤指定防止
6. `C:\HPDB\TMP`運用との整合

## SHA256
`f9cbeb5d9159ac057bf65e55efe0f102631ae756d3440d12e4d95cca9dd0bbb1`
