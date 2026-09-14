# App272 外注負荷ヒートマップ Ver0.6.1.6 Claude diffクロスレビュー依頼

Ver0.6.1.5を安定母体として、外注先クイックフィルタ（複数選択）を実装しました。
設計仕様Rev.2と、v0.6.1.5→v0.6.1.6のJS/CSS diffを基準にレビューしてください。

## 判定してほしい項目

1. `state.records` が標準絞り込み後の原母集団として保持され、クイックフィルタで破壊・上書きされていないか。
2. `visibleRecords` が Summary / Heatmap / Risk / Detail の共通描画母集団になっているか。
3. `renderHeader()` の対象件数が `visibleRecords.length` になっているか。
4. `renderSummary()` の日付矛盾／要注意集計が `visibleRecords` になっているか。
5. 入荷操作ハンドラの `state.records.find(...)` が維持されているか。
6. 複数外注先が OR 条件、0社選択が全社表示になっているか。2社専用ではなく3社以上でも一般化されているか。
7. kintone標準絞り込みとの関係が `標準条件 AND クイック外注先条件` になっているか。
8. 標準絞り込みで母集団が変わった際にクイック選択が安全にリセットされるか。
9. `selectedVendor` がクイック条件から外れた場合の再選択が安全か。
10. クイック操作だけでkintone API再取得や更新APIが発生しないか。
11. `render(root, false)` と既存scroll generation guard / capture / restoreを壊していないか。
12. 今日マーカー、Windows Shift+wheel、Mac横スクロール、表示サイズ、Manual表示、レコードリンクに差分混入がないか。
13. 入荷登録・修正・取消への回帰リスクがないか。特にPUT中の再描画抑止が妥当か。
14. 候補検索、チップ解除、全解除、外側クリック、Escapeのイベント処理に二重バインドやDOM再描画由来の問題がないか。
15. 外注先名の `escapeHtml` / `escapeAttr` 使用が安全か。
16. CSSが既存ヒートマップ・sticky列・今日線・下段ガントへ副作用を与えないか。

## 期待する回答形式

- Verdict: GO / GO WITH CONDITIONS / STOP
- Major findings
- Minor findings
- Observation
- 上記16項目それぞれ PASS / CONDITION / FAIL
- 実機で重点確認すべき項目
