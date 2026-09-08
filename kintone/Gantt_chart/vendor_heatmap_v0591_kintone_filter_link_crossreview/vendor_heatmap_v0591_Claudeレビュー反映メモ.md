# Ver0.5.9.1 変更メモ

Claudeクロスレビューで指摘された `root.dataset.initialized` ガードの問題を修正。

## 修正前
`initialized=1` なら無条件で return していたため、
kintoneが絞り込み変更時に同じDOMを再利用した場合、
新しい絞り込み条件で再取得されない可能性があった。

## 修正後
1. `kintone.app.getQueryCondition()` を `index.show` 冒頭で取得
2. `root.dataset.listQueryCondition` に前回条件を保持
3. 初期化済みかつ条件が同じ場合だけスキップ
4. 条件が変わった場合は必ず再フェッチ
5. APIエラー時は `initialized=0` に戻し、再試行可能にする
6. `fetchActiveRecords()` へ取得済み条件を引数で渡し、同一イベント内の条件を固定

その他のVer0.5.9仕様は変更なし。
