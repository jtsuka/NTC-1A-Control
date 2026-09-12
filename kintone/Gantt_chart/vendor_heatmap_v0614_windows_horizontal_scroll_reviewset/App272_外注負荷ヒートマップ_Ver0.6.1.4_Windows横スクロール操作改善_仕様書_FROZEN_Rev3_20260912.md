# App272 外注負荷ヒートマップ Ver0.6.1.4

## Windows 横スクロール操作改善 修正仕様書（クロスレビュー版）

**作成日:** 2026-09-12\
**対象:** kintone App272「外注課発注データ-TEST」\
**母体:** Ver0.6.1.3 `vendor_heatmap_v0613_receipt_undo`\
**状態:** SPEC REVIEW / 実装禁止\
**目的:** Claude / ChatGPT
クロスレビュー後に仕様をFROZENし、最小差分で実装する。

------------------------------------------------------------------------

## 1. 背景

現行の外注負荷ヒートマップは、上段ヒートマップ・下段ガントとも横方向に長い表示を持つ。

Macのトラックパッドや横スクロール可能なポインティングデバイスでは自然に左右へ移動できるが、職場で一般的なWindows
PC＋縦ホイールのみのマウスでは、横スクロールバーをドラッグする操作が中心となり、日常利用では操作負担が大きい。

外注課担当者は多忙であり、「新機能を増やす」だけでなく、既存画面を普通のWindowsマウスで短時間に扱えることを重視する。

そのため Ver0.6.1.4
では、**Shift＋マウスホイールによる横スクロール操作を明示的に補助する**。

本改修は表示・操作性だけを対象とし、データ取得、日程計算、入荷実績、取消、kintone
PUT、ヒートマップ計算には一切影響させない。

------------------------------------------------------------------------

## 2. 現行 Ver0.6.1.3 で維持する機能

以下は変更禁止とする。

-   kintone標準一覧の絞り込み条件を母集団として使用する。
-   500件単位のGETページング。
-   過去90日を含む表示範囲。
-   30 / 60 / 90日表示切替。
-   初回表示時の「今日」付近への自動スクロール。
-   UI再描画時のスクロール位置保持。
-   `renderGeneration` による古い遅延スクロール処理の抑止。
-   上段ヒートマップ。
-   下段ガント・髭・各種マーカー。
-   Manual表示、リスク表示、sticky列。
-   入荷実績登録。
-   入荷済みグレー表示。
-   body直下 `position:fixed` の入荷日編集ポップアップ。
-   入荷実績取消。
-   `$revision` 競合ガード。
-   保存・取消中の二重PUT防止。
-   保存・取消成功後の該当行だけのDOM更新。
-   App272へのPUT対象を `外注入荷実績日` のみに限定する安全仕様。

**Ver0.6.1.4 の横スクロール操作を理由に `render()`、REST
API、レコード正規化、日程計算、入荷処理へ変更を加えてはならない。**

------------------------------------------------------------------------

## 3. 対象スクロール領域

現行画面には独立した横スクロール領域がある。

### 3.1 上段ヒートマップ

対象は、実際のヒートマップを内包する `.eh-table-scroll`。

ただし `.eh-table-scroll`
は将来または他表示部でも使用される可能性があるため、単純な最初の
`.eh-table-scroll` 決め打ちは避ける。

**上段対象は `.eh-heatmap` を内包する `.eh-table-scroll` に限定する。**

例:

``` js
const heatmap = root.querySelector('.eh-heatmap');
const upperScroll = heatmap ? heatmap.closest('.eh-table-scroll') : null;
```

### 3.2 下段ガント

対象は `.eh-schedule-scroll`。

### 3.3 独立操作

上段と下段の `scrollLeft` は同期させない。

-   上段にマウスポインタがあるときは上段だけ移動。
-   下段にマウスポインタがあるときは下段だけ移動。

現行 Ver0.6.1.1
以降の「上段・下段を独立したスクロールコンテナとして扱う」設計を維持する。

------------------------------------------------------------------------

## 4. 操作仕様

### 4.1 Windows普通マウス

横スクロール対象領域上で:

-   通常ホイール → **従来どおり縦スクロール。横移動へ変換しない。**
-   `Shift + ホイール` → **対象領域を横スクロールする。**

これにより、上下に案件を探す通常操作を奪わない。

### 4.2 Mac / 横スクロール対応デバイス

Macトラックパッド、Magic Mouse、横ホイール等から既に `deltaX`
が発生している場合は、ブラウザ本来の横スクロールを優先する。

**既に横方向入力があるイベントを、JavaScriptで二重に横移動させない。**

### 4.3 ブラウザ標準 Shift＋ホイールとの二重移動防止

ブラウザ自身が Shift＋ホイールを横スクロールとして処理できる環境がある。

したがって実装では次の考え方を採用する。

1.  `Shift` が押されていなければ何もしない。
2.  `deltaX`
    が主成分のイベントは、既に横入力として扱われている可能性が高いため、原則としてブラウザ標準動作へ任せる。
3.  `Shift` が押され、かつ縦ホイール入力 `deltaY`
    が主成分の場合だけ、`deltaY` を `scrollLeft` へ変換する。
4.  JavaScriptで横移動を実行した場合だけ `event.preventDefault()` する。

判定例:

``` js
function handleShiftWheelHorizontal(event, scrollEl) {
  if (!event.shiftKey) return;

  const absX = Math.abs(event.deltaX || 0);
  const absY = Math.abs(event.deltaY || 0);

  // 既に横入力として届いている場合はブラウザ標準に任せる。
  if (absX >= absY && absX > 0) return;

  if (absY <= 0) return;

  event.preventDefault();
  scrollEl.scrollLeft += normalizeWheelDelta(event, scrollEl);
}
```

※このコードは仕様上の参考例であり、クロスレビュー前の確定実装ではない。

------------------------------------------------------------------------

## 5. wheel delta の扱い

`WheelEvent.deltaMode`
はブラウザ・デバイスによって異なる可能性があるため、単純に `deltaY`
を常にピクセルとして扱う実装は避ける。

候補:

``` js
function normalizeWheelDelta(event, scrollEl) {
  const value = event.deltaY || 0;

  if (event.deltaMode === WheelEvent.DOM_DELTA_LINE) {
    return value * 16;
  }

  if (event.deltaMode === WheelEvent.DOM_DELTA_PAGE) {
    return value * Math.max(1, scrollEl.clientWidth * 0.9);
  }

  return value;
}
```

クロスレビューでは、Chrome / Edge / Firefox
の実用上の操作感を含め、この正規化が必要十分か確認する。

過度な加速係数は付けない。まずOS・ブラウザから渡されるホイール量を尊重する。

------------------------------------------------------------------------

## 6. イベント登録仕様

`preventDefault()` を使用する可能性があるため、対象の `wheel` listener
は `passive:false` で登録する。

``` js
scrollEl.addEventListener('wheel', handler, { passive: false });
```

### 6.1 登録タイミング

現行 `render()` は `root.innerHTML` を再構築した後に `bindEvents(root)`
を呼ぶ。

横スクロール補助も、**再描画後の現在のDOMに対して `bindEvents(root)`
内、またはそこから呼ぶ専用関数で登録する**。

古いスクロールDOMは `root.innerHTML`
の置換で破棄されるため、document/windowへの恒久的なwheel
listener追加は原則行わない。

推奨:

``` js
function bindHorizontalWheelSupport(root) {
  const heatmap = root.querySelector('.eh-heatmap');
  const upperScroll = heatmap ? heatmap.closest('.eh-table-scroll') : null;
  const lowerScroll = root.querySelector('.eh-schedule-scroll');

  [upperScroll, lowerScroll].forEach(function (scrollEl) {
    if (!scrollEl) return;
    // wheel listenerを登録
  });
}
```

------------------------------------------------------------------------

## 7. スクロール端での扱い

Claudeクロスレビュー結果を正式採用し、**本機能が「Shift＋縦ホイールを横スクロールとして扱う」と判定したイベントは、左右端であっても対象コンテナの操作として一貫して消費する。**

確定条件は以下とする。

- `Shift` が押されている。
- `deltaY` が主成分である。
- 本JavaScriptが縦ホイール→横スクロール変換の対象と判定した。

上記3条件を満たす場合は、左右端かどうかに関係なく `event.preventDefault()` を実行し、正規化した `deltaY` を `scrollLeft` へ加算する。

`scrollLeft` はブラウザにより0〜最大値へ自然にクランプされるため、左右端専用の分岐は追加しない。

一方で、次の場合はブラウザ標準動作へ任せる。

- `Shift` が押されていない通常ホイール。
- `deltaX` が主成分のネイティブ横スクロール入力。
- 横スクロール補助の対象と判定しないイベント。

これにより、「中央では横スクロールするが端だけ別の挙動になる」という予測しにくいUXを避ける。

## 8. UI表示

Ver0.6.1.4では、新しい「今日へ戻る」ボタンや左右移動ボタンは追加しない。

必要なら下段の既存説明文へ、目立ちすぎない補助文を追加する候補とする。

候補:

> Windows: Shift＋ホイールで左右にスクロールできます。

ただし画面を煩雑にしないことを優先し、説明文追加の要否も実機確認で判断する。

------------------------------------------------------------------------

## 9. CSS

原則として **CSS変更なし** を第一候補とする。

Shift＋ホイール横スクロールはJavaScriptの操作補助だけで成立させる。

CSSを変更する場合は、必要性をクロスレビューで説明できるものに限定する。

------------------------------------------------------------------------

## 10. 書込み安全性

Ver0.6.1.4の横スクロール操作では、kintone REST APIを呼ばない。

### 絶対条件

-   GET追加なし。
-   PUT追加なし。
-   POSTなし。
-   DELETEなし。
-   `外注入荷実績日` を変更しない。
-   K加工着手日を変更しない。
-   K完成検査日を変更しない。
-   K暫定設定を変更しない。
-   K日程種別を変更しない。
-   K計算根拠を変更しない。
-   注文日・納期・ODERNO・Lookup等を変更しない。

**横スクロールイベントから業務データへ到達するコードパスを作らない。**

------------------------------------------------------------------------

## 10.1 既存スクロール要素選択の遡及修正【クロスレビュー必須条件】

Claudeクロスレビューにより、現行DOMには `.eh-table-scroll` が少なくとも2つ存在することが確認された。

- 上段ヒートマップ本体を包む `.eh-table-scroll`
- 「日程リスク」一覧を包む別の `.eh-table-scroll`

さらにVer0.6.1.3実コードをGitHub上で再照合した結果、既存の `captureScrollPositions()` は現在も `root.querySelector('.eh-table-scroll')` を使用しており、同系統の要素選択がスクロール保持処理に残っていることを確認した。

したがってVer0.6.1.4では、新設wheel処理だけでなく、次の既存3関数も同じ安全な要素特定方式へ遡及修正する。

- `captureScrollPositions()`
- `restoreScrollPositions()`
- `jumpToTodayAfterRender()`

上段スクロール要素は必ず次の考え方で特定する。

```js
function getHeatmapScrollContainer(root) {
  const heatmap = root.querySelector('.eh-heatmap');
  return heatmap ? heatmap.closest('.eh-table-scroll') : null;
}
```

可能であれば、この要素特定を共通ヘルパーへまとめ、新設wheel処理と既存3関数で同じロジックを再利用する。

### 禁止

```js
root.querySelector('.eh-table-scroll')
```

だけで「上段ヒートマップのスクロールコンテナ」とみなしてはならない。

この修正は新機能追加とは別に、既存Ver0.6.1系の潜在的なDOM依存を解消する**予防的バグ修正**としてVer0.6.1.4へ含める。


## 11. 実装差分の原則

Ver0.6.1.3を正式母体とし、別フォルダ / 別ファイルで Ver0.6.1.4
を作成する。

推奨フォルダ名:

`kintone/Gantt_chart/vendor_heatmap_v0614_windows_horizontal_scroll/`

推奨JS名:

`vendor_heatmap_v0614_windows_horizontal_scroll.js`

CSS変更が不要なら、Ver0.6.1.3の正式CSSをそのまま使用するか、配布単位を分かりやすくするため同一内容をVer0.6.1.4フォルダへコピーする。どちらにするかは実装時に明記する。

**Ver0.6.1.3を上書きしない。**

ロールバックはVer0.6.1.3 JS/CSSへ戻すだけで完了できること。

------------------------------------------------------------------------

## 12. テスト項目

| ID | 試験 | 期待結果 |
|---|---|---|
| T00 | Windows Chrome/Edgeで、カスタムwheel処理を入れない現行Ver0.6.1.3のShift＋ホイール挙動を確認 | ネイティブ横スクロールの有無、`deltaX` / `deltaY` の傾向、二重移動の有無を記録し、Ver0.6.1.4実装前の基準とする |
| T01 | Windows + 普通マウス、下段で通常ホイール | ページ/一覧が従来どおり上下。下段が勝手に横移動しない |
| T02 | Windows + 普通マウス、下段でShift＋ホイール | 下段ガントだけ左右移動 |
| T03 | Windows + 普通マウス、上段でShift＋ホイール | 上段ヒートマップだけ左右移動 |
| T04 | 上段Shift＋ホイール | 下段の`scrollLeft`は変化しない |
| T05 | 下段Shift＋ホイール | 上段の`scrollLeft`は変化しない |
| T06 | Macトラックパッド横スワイプ | 従来どおり自然に横移動。二重移動・過加速なし |
| T07 | Mac/横対応デバイスで`deltaX`入力 | JavaScriptによる二重変換なし |
| T08 | 30/60/90切替後 | 横スクロール補助が引き続き有効 |
| T09 | 外注先切替後 | 横スクロール補助が引き続き有効 |
| T10 | 表示サイズ 標準/大/特大 | 横スクロール補助が有効 |
| T11 | 初回表示 | 従来の今日付近への自動スクロールを維持 |
| T12 | UI再描画 | 従来のスクロール位置保持を維持 |
| T13 | 素早い外注先切替 | `renderGeneration`の既存動作を破壊しない |
| T14 | 入荷しました | 従来どおり登録可能 |
| T15 | 入荷日修正 | 従来どおり修正可能 |
| T16 | 入荷取消 | 従来どおり取消可能 |
| T17 | 取消→再入荷 | revision競合なく従来どおり動作 |
| T18 | Shift＋ホイール中 | kintone PUT/POST/DELETEが発生しない |
| T19 | 横スクロール左端/右端でShift＋縦ホイール | 対象コンテナ操作として消費され、端専用の挙動変化や二重移動がない |
| T20 | Edge / Chrome | Windowsで操作感に大きな差がない |
| T21 | Firefox（使用環境にある場合） | `deltaMode`差で極端に遅い/速い横移動にならない |
| T22 | 入荷ポップアップ表示中 | Shift＋ホイール改修がポップアップ操作を破壊しない |


## 13. FAIL条件

以下のいずれかが発生した場合は Ver0.6.1.4 FAIL
とし、Ver0.6.1.3へロールバックする。

-   通常ホイールが横移動に奪われる。
-   Macの既存横スクロールが悪化する。
-   Shift＋ホイールで上段・下段が同時に動く。
-   1回のホイール入力で二重移動・異常な加速が起きる。
-   30/60/90切替や外注先切替後に操作できなくなる。
-   初回今日位置、スクロール保持、renderGenerationに回帰が出る。
-   入荷登録・修正・取消に回帰が出る。
-   横スクロール操作からREST書込みが発生する。
-   横スクロール端でページ全体の操作が著しく不自然になる。
-   JavaScript例外が発生する。

------------------------------------------------------------------------

## 14. クロスレビュー結果

ClaudeがVer0.6.1.3実コードと本仕様を突き合わせてレビューした。

**判定: GO WITH CONDITIONS**

唯一の必須条件は、`.eh-heatmap.closest('.eh-table-scroll')` による安全な対象特定を、新設wheel処理だけでなく、既存の `captureScrollPositions()` / `restoreScrollPositions()` / `jumpToTodayAfterRender()` にも遡及適用すること。

この条件は10.1章へ反映済みであり、ChatGPT側でもGitHub現行コードに `root.querySelector('.eh-table-scroll')` の決め打ちが残っていることを再確認したため、正式採用した。

### 14.1 確定した判断

1. **対象DOM**
   - 上段は `.eh-heatmap.closest('.eh-table-scroll')` で特定する。
   - 下段は `.eh-schedule-scroll` を使用する。
   - `.eh-table-scroll` の最初の1件を決め打ちしない。

2. **イベント登録場所**
   - `render()` 後に呼ばれる `bindEvents(root)` 内、またはそこから呼ぶ専用関数で、その時点のDOMへwheel listenerを登録する。
   - `root.innerHTML` 再構築により古いDOMとlistenerは破棄されるため、document/windowへの恒久listenerは追加しない。

3. **Shift＋wheel判定**
   - `Shift` が押され、かつ `deltaY` が主成分の場合のみ自前で横スクロールへ変換する。
   - `deltaX` が主成分の場合はネイティブ横入力とみなし、ブラウザ標準動作へ任せる。
   - Windows Chrome/EdgeではShift＋ホイールが既にネイティブ横スクロールへ変換される可能性があるため、T00で実機確認する。

4. **deltaMode正規化**
   - pixel / line / page の違いを考慮する。
   - lineは16px/行、pageは対象コンテナ幅のおおむね0.9倍という近似を採用候補とする。
   - 過度な加速係数は付けない。

5. **preventDefault**
   - JavaScript側が縦ホイールを横スクロールとして扱うと判定した場合のみ `preventDefault()` する。
   - `deltaX` 主成分のネイティブ横入力には適用しない。

6. **左右端**
   - カスタムShift＋縦ホイールとして扱うと決めたイベントは、左右端でも一貫して消費する。
   - 端専用の条件分岐は追加しない。
   - `scrollLeft` のブラウザ標準クランプに任せる。

7. **既存スクロール保持との干渉**
   - wheel操作自体は `scrollLeft` を変更するだけなので、正しい上段対象要素を使えば `captureScrollPositions()` / `restoreScrollPositions()` と整合する。
   - 既存3関数のDOM特定も10.1章の方式へ統一する。

8. **入荷ポップアップとの干渉**
   - 入荷編集ポップアップは `document.body` 直下に配置され、`.eh-schedule-scroll` の外にあるため、下段wheel listenerとは構造的に干渉しない。

9. **CSS**
   - 本改修はJavaScriptのみで完結させる。
   - CSS変更は不要とする。

10. **最小差分**
    - 対象2スクロール要素へのwheel listener追加と、既存3関数の安全な上段DOM特定への遡及修正が最小差分。
    - 日程計算、REST書込み、入荷処理、描画ロジック本体には変更を加えない。

### 14.2 クロスレビュー結論

必須条件を10.1章へ反映し、7章・12章・本14章の未決定表現も解消したため、仕様上の矛盾は解消済み。

**最終判定: GO / SPEC FROZEN / IMPLEMENTATION AUTHORIZED**

## 15. 実装開始条件

現時点では **実装しない**。

1.  Claudeクロスレビュー
2.  指摘事項を本仕様へ反映
3.  ChatGPT再レビュー
4.  仕様FROZEN
5.  Ver0.6.1.4実装
6.  逆側AIによる実コードレビュー
7.  Windows実機テスト
8.  Mac回帰テスト
9.  PASS後にFORMAL PASS / CLOSED

この順序を守る。

------------------------------------------------------------------------

## 16. 今回の設計意図

本改修は「横スクロール対応マウスを買ってもらう」ことを前提にしない。

**普通のWindowsマウスでも、忙しい外注課担当者が既存ガントを楽に使えること**を目的とする。

一方で、通常ホイールによる縦移動は日常操作として重要なので奪わない。

したがって、

**通常ホイール＝縦 / Shift＋ホイール＝横 /
Macのネイティブ横操作＝そのまま**

を基本UXとする。

---

## 17. Revision

### Rev.1 / 2026-09-12
初版クロスレビュー仕様。

### Rev.2 / 2026-09-12
Claudeクロスレビュー `GO WITH CONDITIONS` を反映。

反映内容:

- 現行DOMに `.eh-table-scroll` が複数存在する事実を正式記録。
- 上段対象特定を `.eh-heatmap.closest('.eh-table-scroll')` に統一。
- `captureScrollPositions()` / `restoreScrollPositions()` / `jumpToTodayAfterRender()` へ遡及修正することを必須化。
- 左右端でもカスタムShift＋縦ホイールイベントは一貫して消費する仕様に確定。
- 実装前T00として、Windows Chrome/EdgeのネイティブShift＋ホイール挙動確認を追加。
- SPEC FROZEN / IMPLEMENTATION AUTHORIZEDへ移行。

### Rev.3 / 2026-09-12
Claude再確認で指摘されたFROZEN文書内の内部矛盾を解消。

- 7章を未決定の質問文から、「カスタムShift＋縦ホイールは左右端でも一貫して `preventDefault()` する」確定仕様へ修正。
- 12章テスト表へT00を実体として追加。
- 14章から旧「依頼事項」「判定依頼」の残骸を削除し、クロスレビューで確定した10項目の判断へ全面置換。
- 本改訂をもって、文書本文・テスト表・改訂履歴の整合を確認し、正式FROZENとする。
