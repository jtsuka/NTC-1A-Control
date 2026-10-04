# App272 Evidence v0.1.3.6 FINAL CANDIDATE

## 目的
Claude最終レビューでPhase 1はPASS/CLOSED可。基準版固定前に残ったLow 2点だけを修正する。

## v0.1.3.5 → v0.1.3.6 変更
### Low-1: catch側Summary上書き防止
通常出力の衝突でthrowした場合も、既存Summaryを上書きしない。
既存なら `_FAIL_<8桁GUID>.txt` の別名へ失敗Summaryを書き出す。
別名も衝突しないことをdo/whileで確認する。

### Low-2: Overdue Class UNKNOWNをFail Closed
ExportVerifyでClass空欄に加え `UNKNOWN` もthrowする。

## 非変更
- GET ONLY / WRITE=0
- App293/App272 token split
- v0.1.3.5のF1〜F6意味論
- Class分類ロジック
- EvidenceStatusロジック
- ReceiptDateFieldCode既定空欄 / NOT_EVALUATED
- Overdue `< AsOfDate`
- CurrentAcceptanceDate / Before / After分離
- CSV列構成
- Source accounting
- SHA256 / ExportVerify

## 次のGate
1. Windows PowerShell 5.1 Parser = 0
2. 同日境界テスト
   - スナップショットが同一なら AsOfDate=2026-09-30 → Overdue=352
   - AsOfDate=2026-10-01 → Overdue=377
   - データ変化後は D+1 - D と期限D当日集合を比較
3. 上書き衝突テスト
4. Receipt DATEフィールド実機確認
5. 可能ならAcceptanceSync Class突合
