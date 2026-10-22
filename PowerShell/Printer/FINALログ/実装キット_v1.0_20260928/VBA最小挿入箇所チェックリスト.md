# VBA 最小挿入箇所チェックリスト

この文書は「どこへ何を入れたか」を実装時にチェックするためのもの。
行番号はClaudeが展開した原典に基づくため、VBE表示では前後のコード文字列で確認する。

| No | 原典箇所 | 追加内容 | 帳票ロジック変更 |
|---|---|---|---|
| 1 | Sheet8 Worksheet_SelectionChange / BT_NO=2 / PrintableShapes呼出直前 | CheckLog_Begin | なし |
| 2 | Process STEP_0 / `row_count = 0`直後 | CheckLog_ResetOrder | なし |
| 3 | STEP_1 TOP / 実代入直後 | CheckLog_Touch TOP | なし |
| 4 | STEP_2 HEAD / 実代入直後 | CheckLog_Touch HEAD | なし |
| 5 | STEP_3 BODY / skip通過・実代入直後 | CheckLog_Touch BODY, DetailNo=row_count+1, LineNo=DATA_DIC(dat)(3) | なし |
| 6 | STEP_5 / BF1代入直後 | Touch OTHER/BF1 | なし |
| 7 | STEP_5 / GENKAUMU H1代入直後 | Touch GENKAUMU/H1 | なし |
| 8 | PageSetup完了後 / `.PrintOut`直前 | CheckLog_FlushOrder | なし |
| 9 | KANRI_SKIP正常終端 / SetPrinter("DEF")後 | CheckLog_End | なし |

## 実装後の静的確認
- `CheckLog_Touch`がFind/FindNextの「書込み前」には存在しない
- skipされたセルをTouchしていない
- BJ12を特別に上書き/修正するログコードを入れていない
- 原典If条件を変更していない
- Find/FindNext順を変更していない
- PrintArea/Zoom/FitToPagesTall/PrintTitleRows/PrintOutを変更していない
- 新しい包括 `On Error` をProcess/Sheet8へ追加していない
- 原典xlsmを保存していない
