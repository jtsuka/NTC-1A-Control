# Ver0.6.0.3 Claudeクロスレビュー反映メモ

Ver0.6.0.2レビュー指摘2点を修正。

## 修正1
上段「要注意／リスク一覧」も、
`K手配書番号 → 旧Lookup → #レコード番号`
の3段フォールバックへ統一。

normalize時に `displayKey` を生成し、
上段・下段の双方で参照する。

## 修正2
旧 `--eh-left-width` ベースの2列grid定義を削除。

未使用の `--eh-left-width` 宣言もすべて削除し、
4列grid定義へ `display:grid / gap / width / min-width` を集約。

## 再レビュー確認点
- `displayKey`共通化による副作用がないか
- 旧2列grid削除後、必要プロパティが4列定義へ漏れなく移っているか
- `--eh-left-width` がCSSに残っていないか
- 標準/大/特大・sticky・Manual表示・絞り込み連動に回帰がないか
