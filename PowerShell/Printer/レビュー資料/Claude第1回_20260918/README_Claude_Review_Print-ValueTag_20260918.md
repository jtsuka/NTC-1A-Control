# Claudeレビュー依頼｜Print-ValueTag 船舶・購買 2026-09-18

以下4ファイルをセットでレビューしてください。

1. `Print-ValueTag_Ship_v0.10.7_VBAFit_20260918.ps1`
2. `SPEC_Print-ValueTag_Ship_v0.10.7_20260918.md`
3. `Print-ValueTag_Purchase_v0.2.1_20260918.ps1`
4. `SPEC_Print-ValueTag_Purchase_v0.2.1_20260918.md`

## 重要

- 両PS1は既存の実証済みファイルを**内容変更せずファイル名だけ変更したコピー**です。
- Ship新旧SHA256: `b5cd0956a4d4b586a3eb68afb48088cbfdab5c856cac9f17628de47c419244ee`
- Purchase新旧SHA256: `97777a565c8cf8db9adbf21d53f2d02eaa6963571cf4574068dc54828a3d55cb`
- ShipとPurchaseを混同せず、対象抽出条件を別々に監査してください。
- 実証済み版を保護するため、大規模リファクタリングより先に「仕様不一致の有無」をレビューしてください。

## 期待するレビュー出力

### A. Ship
- 仕様一致点
- 仕様不一致/潜在バグ
- Purchase条件との混線の有無
- v0.10.6→v0.10.7でA7以外に実質差分がないか
- AV5/原価割れ等の未CLOSED項目がコード上どう見えるか

### B. Purchase
- 購買固有抽出条件の原典整合性
- kou/genkaの手配単位判定
- 在庫抜き/担当手配抜き/日付/送料無料除外
- 11明細3ページ化
- A7局所修正
- 自由記述先頭`=`保持
- Ship条件との混線の有無

### C. 共通
- 同じ帳票Engine部分で両版に不整合がないか
- 今後共通化するとしても、現行PASS版を壊さず切り出せる境界
- 修正が必要なら「最小修正案」と「将来リファクタ案」を分離

重大度は `Critical / High / Medium / Low / Info` で示し、指摘ごとに該当コード位置・根拠仕様・再現条件を添えてください。
