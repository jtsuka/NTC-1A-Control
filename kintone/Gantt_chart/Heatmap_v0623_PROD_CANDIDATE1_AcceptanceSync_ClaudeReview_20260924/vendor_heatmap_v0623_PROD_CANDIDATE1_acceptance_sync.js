/*
 * 外注課 外注先負荷ヒートマップ / 日程リスク可視化 Ver0.6.2.3-PROD-CANDIDATE1
 * 対象: kintone App 272 (外注課発注データ-TEST)
 *
 * 前提:
 *  - カスタムビュー名: 外注負荷ヒートマップ
 *  - カスタムビューHTML: <div id="vendor-heatmap-root"></div>
 *
 * 方針:
 *  - 閲覧専用。日付編集はkintone標準レコード画面で行う。
 *  - K加工着手日/K完成検査日は自動更新しない。
 *  - 注文日～納期の細線と、K加工着手日～K完成検査日の太線を重ねて表示。
 *  - 4日付矛盾、納期余裕不足、外注先の日別同時案件数を可視化。
 *
 * Ver0.6.1.2 追加機能:
 *  - Ver0.6.1.1 (scroll preserve + render generation guard) を母体にする。
 *  - 外注入荷実績日を案件単位で記録する。
 *  - 保存成功後はrender()全体を呼ばず、該当行だけを完了表示へ更新する。
 *  - 入荷済み案件は下段ガントのみグレー表示し、上段負荷計算は変更しない。
 *  - PUT対象は外注入荷実績日のみに限定し、$revisionで競合更新を防ぐ。
 *
 * Ver0.6.1.3 追加機能:
 *  - 入荷済み案件の外注入荷実績日を「取り消し」で空欄へ戻せる。
 *  - body直下ポップアップも保存/取消中はdisableし、二重PUTを防ぐ。
 *  - 取消成功後はrender()せず、該当行だけ未入荷表示へ戻す。
 *
 * Ver0.6.1.4 追加機能:
 *  - Windows普通マウス向けに、Shift＋縦ホイールを横スクロールへ変換する。
 *  - deltaX主成分のネイティブ横入力はブラウザ標準動作へ任せる。
 *  - 上段ヒートマップのスクロール要素特定を .eh-heatmap.closest('.eh-table-scroll') に統一する。
 *  - capture/restore/jumpToToday の既存スクロール処理も同じ安全な要素特定へ遡及修正する。
 *


 * Ver0.6.2.0-TEST4（番号クリック起動導線追加 / Claudeレビュー前）:
 *  - 下段案件詳細に「日程編集」ボタンを追加。
 *  - K加工着手日 / K完成検査日 / K暫定設定 / 手配担当者をbody直下ポップアップで編集。
 *  - 日付変更時はK日程種別を自動的にManualへ変更。
 *  - ポップアップ内に「その他の項目を標準画面で編集」ボタンを追加し、
 *    kintone標準レコード編集画面へ遷移できる逃げ道を用意する。
 *  - STAFF_DD候補は /k/v1/app/form/fields.json のフィールド設定を正本として取得する。
 *    取得失敗時はポップアップを開かずFail Closedとする。
 *  - 下段案件詳細の手配書番号表示と注文書番号表示を、同じ日程編集ポップアップの
 *    起動トリガーとして追加する。既存の「日程編集」ボタンはTEST中は残す。
 *  - 手配書番号なし表示も主番号セルの起動トリガーとして扱う。
 *    注文書番号が空欄（—）の場合はリンク化しない。
 *  - 新設DATEフィールド「受入れ日」を日程編集ポップアップへ追加。外注入荷実績日とは
 *    独立管理とし、自動連動させない。フォーム設定APIで存在/type=DATEをFail Closed確認。
 *  - 日程リスク表の手配書番号 / 注文書番号も、下段ガントと同じ
 *    日程編集ポップアップ起動トリガーへ統一する。
 *  - 日程編集ポップアップ本文の先頭に、手配書番号 / 注文書番号を
 *    「案件情報（表示のみ）」として明示する。保存・PUT対象には含めない。
 *  - 日程編集ポップアップへ外注入荷実績日を追加し、既存の入荷専用ボタンと同じ
 *    フィールドを同じrevision付きPUTで更新できるようにする。TEST中は既存入荷ボタンを残す。
 *  - 標準編集へ遷移する直前にsessionStorageへ強制再取得フラグを立て、
 *    一覧へ戻った最初のindex.showでは一覧条件が同一でも1回だけ再GETする。
 *    フラグは再GET成功後にのみ削除し、失敗時は残して次回再試行可能とする。
 *
 *
 *
 * Ver0.6.2.3-PROD-CANDIDATE1:
 *  - App272 PRODだけControlled Executeを許可。
 *  - App292はAnalysis Only。
 *  - CANDIDATE_EMPTY_SINGLEから人が手動選択し、1回最大50件。
 *  - 候補総数が50件を超えていても一括実行はせず、選択件数だけMaxWrites判定する。
 *  - Execute直前に全Analysisを再実行し、選択POのClass / CandidateReceiptDate /
 *    RecordId / Revision / CurrentAcceptanceDateがAnalysis時点と完全一致することを確認。
 *    1件でも変化があればSafetyStopし、WRITE=0。
 *  - 確認ダイアログ後、APP272-ACCEPTANCE-EXECUTE:<件数> の手入力を要求。
 *  - revision付きPUT、受入れ日のみ更新、PostVerify、No Auto Retryを維持。
 *
 * Ver0.6.2.3-PROD-CANDIDATE1 追加機能（App293受入実績同期β）:
 *  - App293（受入情報App / AppID=293）をREAD ONLYで全件取得し、発注番号単位で集約する。
 *  - App272/App292側も全件を別GETし、7分類でAnalysisする。
 *  - CANDIDATE_EMPTY_SINGLEだけを自動更新候補とする。
 *  - 複数受入、負数、既存受入れ日あり、ソースなしは書込み対象外。
 *  - 「App293に無い = 未受入」とは表示しない。NO_SOURCE_DATA_UNKNOWNを固定する。
 *  - App272/App292側で同一発注番号が2件以上あれば SafetyStop。
 *    重複発注番号は分類対象から除外し、Executeを全面禁止する。
 *  - 受入番号の空欄/重複もSafetyStopとしてExecuteを全面禁止する。
 *  - Analysis → 確認 → revision付きPUT → reGET PostVerify の2段階。
 *  - MaxWrites=50。超過時はWRITE=0。
 *  - App292 TEST / App272 PRODの両方で同一コードを使えるが、その他AppではFail Closed。
 *
 * Ver0.6.1.8.2 変更:
 *  - 上部の固定案内「現在の暫定日程ルール（検収日: 納期の3日前 / 仕掛日: 検収日の5日前）」を削除。
 *  - 外注先別日程マスタ運用へ移行済みのため、固定3日/5日の表示は実態と合わず誤解を招くため廃止。
 *  - 日程リスク判定、負荷計算、ODERNO表示、入荷更新、各種フィルタ等の業務ロジックは変更しない。
 *
 * Ver0.6.1.8 追加機能:
 *  - 手配書番号が空欄の案件で、kintoneレコード番号（例: #4）を代替表示しない。
 *  - 代わりに「手配書番号なし」と明示し、営業手配起因ではない案件であることをオペレーターが判別しやすくする。
 *  - 注文書番号の表示はVer0.6.1.7のまま維持し、手配書番号なし案件は注文書番号で追跡できる。
 *  - 日程判定・負荷計算・入荷更新ロジックは変更しない。
 * Ver0.6.1.7 追加機能:
 *  - オペレーター追跡用に注文書番号（内部フィールド: 注文伝票番号 / code=数値_44）を表示。
 *  - 手配書番号が空欄でも注文書番号で追跡できるよう、日程リスク一覧と案件詳細の左固定列へ併記。
 *  - 既存の日程判定・負荷計算・入荷更新ロジックは変更しない。
 *
 * Ver0.6.1.6 追加機能:
 *  - kintone標準一覧条件を母集団として維持したまま、外注先クイックフィルタを追加する。
 *  - 外注先はチェックボックスで複数選択し、選択社間はOR条件、0社選択は全社表示とする。
 *  - state.recordsは原母集団として保持し、描画用visibleRecordsだけを派生させる。
 *  - 入荷操作系のstate.records.find(...)は変更せず、既存更新処理を保護する。
 */
(function () {
  'use strict';

  const CONFIG = {
    targetViewName: '外注負荷ヒートマップ',
    rootId: 'vendor-heatmap-root',

    fields: {
      recordId: '$id',
      key: 'ODERNO',
      keyFallback: 'ルックアップ',
      orderSlipNo: '数値_44', // 表示名: 注文伝票番号（画面上は『注文書番号』として表示）
      vendor: 'VENDOR_DD',
      vendorFallback: 'VENDOR_TEXT',
      staff: 'STAFF_DD',
      staffFallback: 'STAFF_TEXT',
      product: '商品名',
      orderDate: '日付',
      startDate: 'K加工着手日',
      inspectionDate: 'K完成検査日',
      dueDate: '日付_1',
      scheduleType: 'K日程種別',
      provisionalFlag: 'K暫定設定',
      receiptDate: '外注入荷実績日',
      acceptanceDate: '受入れ日',
      revision: '$revision'
    },

    initialDays: 60,
    rangeOptions: [30, 60, 90],
    pastDays: 90,
    minInspectionToDueDays: 3,

    acceptanceSync: {
      sourceAppId: 293,
      allowedTargetAppIds: [292, 272],
      maxWrites: 50,
      prodExecute: {
        targetAppId: 272,
        maxSelected: 50,
        confirmationPrefix: 'APP272-ACCEPTANCE-EXECUTE'
      },
      sourceFields: {
        recordId: '$id',
        revision: '$revision',
        receiptNo: '受入番号',
        po: '発注番号',
        receiptDate: '受入日',
        receiptQty: '受入数',
        orderNo: '手配書番号',
        orderSlipNo: '注文伝票番号'
      },
      targetFields: {
        recordId: '$id',
        revision: '$revision',
        po: '数値',
        acceptanceDate: '受入れ日',
        orderNo: 'ODERNO',
        orderSlipNo: '数値_44'
      }
    },

    // 「危険」の業務基準は未確定なので、Ver0.1では単純な件数色分けのみ。
    heatLevels: {
      lowMax: 2,
      mediumMax: 4
      // 5件以上は high 表示
    }
  };


  const STANDARD_EDIT_REFRESH_KEY =
    'externalVendorHeatmap.forceRefreshAfterStandardEdit';

  function shouldForceRefreshAfterStandardEdit() {
    try {
      return sessionStorage.getItem(STANDARD_EDIT_REFRESH_KEY) === '1';
    } catch (e) {
      console.warn('[Heatmap][Edit] sessionStorage read failed', e);
      return false;
    }
  }

  function markForceRefreshAfterStandardEdit() {
    try {
      sessionStorage.setItem(STANDARD_EDIT_REFRESH_KEY, '1');
      return true;
    } catch (e) {
      console.warn('[Heatmap][Edit] sessionStorage write failed', e);
      return false;
    }
  }

  function clearForceRefreshAfterStandardEdit() {
    try {
      sessionStorage.removeItem(STANDARD_EDIT_REFRESH_KEY);
    } catch (e) {
      console.warn('[Heatmap][Edit] sessionStorage remove failed', e);
    }
  }

  const state = {
    records: [],
    rangeDays: CONFIG.initialDays,
    selectedVendor: null,
    detailSize: loadDetailSize(),
    listQueryCondition: '',
    listFilterApplied: false,
    selectedQuickVendors: new Set(),
    quickFilterOpen: false,
    quickVendorSearch: ''
  };

  function loadDetailSize() {
    try {
      const saved = localStorage.getItem('externalVendorHeatmap.detailSize');
      return ['standard', 'large', 'xlarge'].includes(saved) ? saved : 'standard';
    } catch (e) {
      return 'standard';
    }
  }

  function saveDetailSize(value) {
    try {
      localStorage.setItem('externalVendorHeatmap.detailSize', value);
    } catch (e) {
      // ブラウザ設定等で保存できない場合は、その画面内だけで切替を継続する。
    }
  }

  kintone.events.on('app.record.index.show', async function (event) {
    if (event.viewName !== CONFIG.targetViewName) return event;

    const root = document.getElementById(CONFIG.rootId);
    if (!root) {
      console.error('[Heatmap] root element not found:', CONFIG.rootId);
      return event;
    }

    /*
     * Ver0.5.9.1:
     * 同じDOMが再利用された状態で index.show が再発火しても、
     * kintone標準一覧の絞り込み条件が変わっていれば必ず再取得する。
     * 同一条件での重複発火だけを抑止する。
     */
    const currentListCondition =
      String(kintone.app.getQueryCondition() || '').trim();
    const previousListCondition =
      String(root.dataset.listQueryCondition || '');
    const forceRefreshAfterStandardEdit =
      shouldForceRefreshAfterStandardEdit();

    if (
      root.dataset.initialized === '1' &&
      previousListCondition === currentListCondition &&
      !forceRefreshAfterStandardEdit
    ) {
      return event;
    }

    root.dataset.initialized = '1';
    root.dataset.listQueryCondition = currentListCondition;

    root.innerHTML = '<div class="eh-loading">外注先負荷データを取得しています...</div>';

    try {
      state.records = await fetchActiveRecords(currentListCondition);
      // Ver0.6.1.6: kintone標準絞り込みで母集団が変わったら、
      // 以前のクイック選択を持ち越さず全社表示へ安全に戻す。
      state.selectedQuickVendors.clear();
      state.quickFilterOpen = false;
      state.quickVendorSearch = '';
      state.selectedVendor = chooseInitialVendor(state.records);

      // Ver0.6.2.0-TEST:
      // 標準編集から戻った強制再取得フラグは、再GET成功後にだけ削除する。
      // 失敗時はcatchへ入りフラグを残すため、次回index.showで再試行できる。
      if (forceRefreshAfterStandardEdit) {
        clearForceRefreshAfterStandardEdit();
      }

      // Ver0.6.1: 初回表示・母集団再取得のときだけ今日位置へ移動する。
      render(root, true);
    } catch (err) {
      // 同一条件でも再試行できるよう、失敗時は初期化済みフラグを戻す。
      root.dataset.initialized = '0';

      console.error('[Heatmap] error', err);
      root.innerHTML =
        '<div class="eh-error"><strong>データ取得に失敗しました。</strong><br>' +
        escapeHtml(err && err.message ? err.message : String(err)) +
        '</div>';
    }

    return event;
  });

  async function fetchActiveRecords(listConditionArg) {
    const appId = kintone.app.getId();

    /*
     * Ver0.5.9:
     * kintone標準一覧の「絞り込み条件」だけを引き継ぐ。
     *
     * getQueryCondition() は order by / limit / offset を含まないため、
     * 画面の表示件数（20件等）に制限されず、絞り込み対象の全件を
     * このヒートマップ用に再取得できる。
     *
     * Ver0.6.1:
     * 業務母集団はkintone標準一覧の絞り込み条件だけで決定する。
     * JS内部の「納期 >= 今日」固定条件は追加しない。
     */
    const listCondition =
      typeof listConditionArg === 'string'
        ? listConditionArg
        : String(kintone.app.getQueryCondition() || '').trim();
    state.listQueryCondition = listCondition;
    state.listFilterApplied = listCondition.length > 0;

    let allRecords = [];
    let lastId = 0;

    while (true) {
      const conditions = [];

      if (listCondition) {
        conditions.push(`(${listCondition})`);
      }

      // 500件ずつ安全に全件取得するためのページング条件
      conditions.push(`$id > ${lastId}`);

      const query =
        `${conditions.join(' and ')} ` +
        `order by $id asc limit 500`;

      const resp = await kintone.api(
        kintone.api.url('/k/v1/records.json', true),
        'GET',
        { app: appId, query: query }
      );

      allRecords = allRecords.concat(resp.records || []);

      if (!resp.records || resp.records.length < 500) {
        break;
      }

      lastId = Number(resp.records[resp.records.length - 1].$id.value);
    }

    return allRecords.map(normalizeRecord);
  }

  function normalizeRecord(r) {
    const f = CONFIG.fields;
    return {
      id: fieldValue(r, f.recordId),
      key: fieldValue(r, f.key),
      keyFallback: fieldValue(r, f.keyFallback),
      orderSlipNo: fieldValue(r, f.orderSlipNo) || '',
      arrangementNo: fieldValue(r, f.key) || fieldValue(r, f.keyFallback) || '',
      displayKey:
        fieldValue(r, f.key) ||
        fieldValue(r, f.keyFallback) ||
        '手配書番号なし',
      vendor: fieldValue(r, f.vendor) || fieldValue(r, f.vendorFallback) || '(外注先未設定)',
      staff: fieldValue(r, f.staff) || fieldValue(r, f.staffFallback) || '',
      product: fieldValue(r, f.product) || '',
      orderDate: parseYmd(fieldValue(r, f.orderDate)),
      startDate: parseYmd(fieldValue(r, f.startDate)),
      inspectionDate: parseYmd(fieldValue(r, f.inspectionDate)),
      dueDate: parseYmd(fieldValue(r, f.dueDate)),
      scheduleType: String(fieldValue(r, f.scheduleType) || '').trim(),
      receiptDate: parseYmd(fieldValue(r, f.receiptDate)),
      revision: String(fieldValue(r, f.revision) || ''),

      isManual: String(fieldValue(r, f.scheduleType) || '').trim() === 'Manual'
    };
  }

  function getAvailableVendors(records) {
    return Array.from(new Set((records || []).map(r => String(r.vendor || ''))))
      .filter(Boolean)
      .sort((a, b) => a.localeCompare(b, 'ja'));
  }

  function getQuickFilteredRecords(records) {
    if (!state.selectedQuickVendors || state.selectedQuickVendors.size === 0) {
      return records;
    }
    return records.filter(r => state.selectedQuickVendors.has(String(r.vendor || '')));
  }

  function reconcileSelectedVendor(vendorStats) {
    const visibleVendors = new Set(vendorStats.map(x => x.vendor));
    if (!state.selectedVendor || !visibleVendors.has(state.selectedVendor)) {
      state.selectedVendor = vendorStats.length ? vendorStats[0].vendor : null;
    }
  }

  function render(root, jumpToToday) {
    /*
     * Ver0.6.1:
     * jumpToTodayAfterRender / restoreScrollPositions はrAF・setTimeoutで
     * 少し遅れて実行される。短時間に連続してrender()が呼ばれると、古い
     * render()の遅延処理が、後から来た新しいrender()の結果を上書きして
     * しまう恐れがあるため、世代番号で「自分が最新のrender()か」を
     * 遅延処理の実行時に確認できるようにする。
     */
    state.renderGeneration = (state.renderGeneration || 0) + 1;
    const myGeneration = state.renderGeneration;

    const today = startOfDay(new Date());
    const viewStart = addDays(today, -CONFIG.pastDays);
    const end = addDays(today, state.rangeDays - 1);
    const dates = makeDateRange(viewStart, end);

    // Ver0.6.1.6: state.recordsは標準絞り込み後の原母集団として保持し、
    // クイックフィルタは描画用の派生配列visibleRecordsにだけ適用する。
    const visibleRecords = getQuickFilteredRecords(state.records);
    const vendorStats = buildVendorStats(visibleRecords, dates);
    reconcileSelectedVendor(vendorStats);

    /*
     * Ver0.6.1:
     * render()はroot.innerHTMLを丸ごと作り直すため、既存のスクロール要素は
     * 再描画のたびに新しいDOMへ置き換わり、scrollLeft=0へ戻ってしまう。
     * 初回表示・母集団再取得（jumpToToday=true）のときだけ今日位置へ移動し、
     * それ以外のUI内再描画（外注先クリック・表示期間/サイズ変更）では、
     * 書き換え直前のスクロール位置を退避しておいて再描画後に復元する。
     */
    const savedScroll = jumpToToday ? null : captureScrollPositions(root);

    root.innerHTML = `
      <div class="eh-wrap eh-display-${state.detailSize}">
        ${renderHeader(vendorStats, visibleRecords)}
        ${renderVendorQuickFilter(state.records)}
        ${renderSummary(vendorStats, visibleRecords)}
        ${renderHeatmap(vendorStats, dates)}
        ${renderRiskSummary(visibleRecords)}
        ${renderVendorDetail(visibleRecords, state.selectedVendor, viewStart, end)}
      </div>
    `;

    bindEvents(root);

    if (jumpToToday) {
      jumpToTodayAfterRender(root, today, myGeneration);
    } else {
      restoreScrollPositions(root, savedScroll, myGeneration);
    }
  }

  function renderHeader(vendorStats, visibleRecords) {
    return `
      <div class="eh-header">
        <div>
          <h2>外注先 負荷ヒートマップ / 日程リスク</h2>
          <div class="eh-subtitle">
            注文日→加工着手日、完成検査日→納期を細いヒゲ、加工期間を太線で表示します。
            K日程・担当者は日程編集ポップアップ、その他はkintone標準編集画面で編集します。
          </div>
          <div class="eh-top-alert"><div class="eh-gantt-status-legends"><span class="eh-legend-alert">赤＝日付矛盾・要注意</span><span class="eh-legend-manual"><span class="eh-manual-badge eh-manual-badge-legend">M</span>＝手動調整</span></div></div>
        </div>
        <div class="eh-controls">
          <label>表示期間
            <select id="eh-range">
              ${CONFIG.rangeOptions.map(n =>
                `<option value="${n}" ${n === state.rangeDays ? 'selected' : ''}>${n}日</option>`
              ).join('')}
            </select>
          </label>
          <label>表示サイズ
            <select id="eh-display-size">
              <option value="standard" ${state.detailSize === 'standard' ? 'selected' : ''}>標準</option>
              <option value="large" ${state.detailSize === 'large' ? 'selected' : ''}>大</option>
              <option value="xlarge" ${state.detailSize === 'xlarge' ? 'selected' : ''}>特大</option>
            </select>
          </label>
          <button type="button" id="eh-acceptance-sync-button" class="eh-acceptance-sync-button">
            受入実績同期（β）
          </button>
          <span class="eh-count">
            対象 ${visibleRecords.length}件 / 外注先 ${vendorStats.length}社
            ${state.listFilterApplied
              ? '<span class="eh-list-filter-status eh-list-filter-status-on">一覧絞込：適用中</span>'
              : '<span class="eh-list-filter-status">一覧絞込：なし</span>'}
          </span>
        </div>
      </div>
    `;
  }

  function renderVendorQuickFilter(records) {
    const vendors = getAvailableVendors(records);
    const selected = state.selectedQuickVendors;
    const selectedCount = selected.size;
    const buttonLabel = selectedCount === 0
      ? 'すべての外注先'
      : (selectedCount === 1 ? Array.from(selected)[0] : `${selectedCount}社選択中`);
    const search = String(state.quickVendorSearch || '').trim().toLocaleLowerCase('ja');

    const options = vendors.map(vendor => {
      const checked = selected.has(vendor);
      const matches = !search || vendor.toLocaleLowerCase('ja').includes(search);
      return `
        <label class="eh-quick-filter-option${matches ? '' : ' eh-quick-filter-option-hidden'}">
          <input type="checkbox" class="eh-quick-vendor-checkbox" value="${escapeAttr(vendor)}" ${checked ? 'checked' : ''}>
          <span>${escapeHtml(vendor)}</span>
        </label>
      `;
    }).join('');

    const chips = Array.from(selected).map(vendor => `
      <span class="eh-quick-filter-chip">
        <span>${escapeHtml(vendor)}</span>
        <button type="button" class="eh-quick-filter-chip-remove" data-vendor="${escapeAttr(vendor)}" aria-label="${escapeAttr(vendor)}を解除">×</button>
      </span>
    `).join('');

    return `
      <div class="eh-quick-filter-bar">
        <div class="eh-quick-filter-main">
          <span class="eh-quick-filter-title">外注先クイック絞り込み</span>
          <div class="eh-quick-filter-dropdown">
            <button type="button" id="eh-quick-filter-toggle" class="eh-quick-filter-toggle" aria-expanded="${state.quickFilterOpen ? 'true' : 'false'}">
              <span>${escapeHtml(buttonLabel)}</span><span class="eh-quick-filter-caret">▼</span>
            </button>
            <div id="eh-quick-filter-panel" class="eh-quick-filter-panel" ${state.quickFilterOpen ? '' : 'hidden'}>
              <input type="search" id="eh-quick-filter-search" class="eh-quick-filter-search" placeholder="外注先名を入力" value="${escapeAttr(state.quickVendorSearch || '')}" autocomplete="off">
              <div class="eh-quick-filter-options">
                ${options || '<div class="eh-quick-filter-empty">外注先がありません</div>'}
                <div class="eh-quick-filter-no-match" ${vendors.length && search && !vendors.some(v => v.toLocaleLowerCase('ja').includes(search)) ? '' : 'hidden'}>該当する外注先がありません</div>
              </div>
              <div class="eh-quick-filter-footer">
                <span>選択中 ${selectedCount}社</span>
                <button type="button" class="eh-quick-filter-clear" ${selectedCount ? '' : 'disabled'}>全解除</button>
              </div>
            </div>
          </div>
          <span class="eh-quick-filter-status">${selectedCount ? `${selectedCount}社で表示中` : '全社表示'}</span>
          ${selectedCount ? '<button type="button" class="eh-quick-filter-clear eh-quick-filter-clear-inline">全解除</button>' : ''}
        </div>
        ${selectedCount ? `<div class="eh-quick-filter-chips">${chips}</div>` : ''}
      </div>
    `;
  }

  function renderSummary(vendorStats, visibleRecords) {
    const issues = visibleRecords.filter(r => analyzeRisk(r).severity === 'danger').length;
    const cautions = visibleRecords.filter(r => analyzeRisk(r).severity === 'warning').length;
    const max = vendorStats.reduce((best, x) => (!best || x.maxConcurrent > best.maxConcurrent ? x : best), null);

    return `
      <div class="eh-cards">
        <div class="eh-card">
          <div class="eh-card-label">日付矛盾 / 納期超過</div>
          <div class="eh-card-value eh-danger-text">${issues}</div>
        </div>
        <div class="eh-card">
          <div class="eh-card-label">要注意</div>
          <div class="eh-card-value eh-warning-text">${cautions}</div>
        </div>
        <div class="eh-card">
          <div class="eh-card-label">最大同時案件</div>
          <div class="eh-card-value">${max ? max.maxConcurrent : 0}</div>
          <div class="eh-card-note">${max ? escapeHtml(max.vendor) : '-'}</div>
        </div>
        <div class="eh-card">
          <div class="eh-card-label">判定ルール</div>
          <div class="eh-card-note">完成検査→納期の余裕 ${CONFIG.minInspectionToDueDays}日未満を注意</div>
        </div>
      </div>
    `;
  }

  function renderHeatmap(vendorStats, dates) {
    // Ver0.6.1.5: 上段ヒートマップでも「今日」を一目で判別できるようにする。
    // 背景色（負荷色）は変更せず、専用classによる縦基準線とヘッダ強調だけを追加する。
    const todayKey = formatDate(startOfDay(new Date()));

    const dateHeader = dates.map(d => {
      const dow = d.getDay();
      const cls = dow === 0 ? 'eh-sun' : (dow === 6 ? 'eh-sat' : '');
      const todayClass = formatDate(d) === todayKey ? ' eh-today-column eh-today-header' : '';
      return `<th class="eh-date ${cls}${todayClass}" title="${formatDate(d)}">${d.getMonth()+1}/${d.getDate()}</th>`;
    }).join('');

    const rows = vendorStats.map(v => {
      const selected = v.vendor === state.selectedVendor ? ' eh-selected' : '';
      const cells = v.counts.map((count, i) => {
        const level = heatLevel(count);
        const title = `${v.vendor} / ${formatDate(dates[i])} / 同時案件 ${count}件`;
        const dow = dates[i].getDay();
        const weekendClass = dow === 0 ? ' eh-col-sun' : (dow === 6 ? ' eh-col-sat' : '');
        const todayClass = formatDate(dates[i]) === todayKey ? ' eh-today-column' : '';
        return `<td class="eh-heat eh-${level}${weekendClass}${todayClass}" title="${escapeHtml(title)}">${count || ''}</td>`;
      }).join('');

      return `
        <tr class="eh-vendor-row${selected}" data-vendor="${escapeAttr(v.vendor)}">
          <th class="eh-vendor">
            <span class="eh-vendor-name">${escapeHtml(v.vendor)}</span>
            <span class="eh-vendor-meta">案件 ${v.recordCount} / 最大 ${v.maxConcurrent}</span>
          </th>
          ${cells}
        </tr>
      `;
    }).join('');

    return `
      <section class="eh-section">
        <div class="eh-section-title">
          <strong>外注先別 日次同時案件数</strong>
          <span>※ 色は件数の多さを示すだけで、外注先ごとの処理能力判定ではありません。</span>
        </div>
        <div class="eh-table-scroll">
          <table class="eh-heatmap">
            <thead><tr><th class="eh-vendor eh-sticky">外注先</th>${dateHeader}</tr></thead>
            <tbody>${rows || '<tr><td>対象データなし</td></tr>'}</tbody>
          </table>
        </div>
      </section>
    `;
  }

  function renderRiskSummary(records) {
    const riskRows = records
      .map(r => ({ record: r, risk: analyzeRisk(r) }))
      .filter(x => x.risk.severity !== 'ok')
      .sort((a, b) => severityRank(b.risk.severity) - severityRank(a.risk.severity))
      .slice(0, 30);

    const rows = riskRows.map(x => {
      const r = x.record;
      const risk = x.risk;
      return `
        <tr>
          <td><span class="eh-badge eh-badge-${risk.severity}">${risk.severity === 'danger' ? '異常' : '注意'}</span></td>
          <td>
            <a href="#"
               class="eh-record-link eh-schedule-edit-trigger${r.arrangementNo ? '' : ' eh-no-arrangement'}"
               data-record-id="${escapeAttr(r.id)}"
               title="手配書番号: ${escapeAttr(r.displayKey)}（クリックで日程編集）"
               aria-label="${escapeAttr(r.displayKey)} の日程編集を開く">${escapeHtml(r.displayKey)}</a>
          </td>
          <td>
            ${r.orderSlipNo
              ? `<a href="#"
                    class="eh-risk-order-slip-link eh-schedule-edit-trigger"
                    data-record-id="${escapeAttr(r.id)}"
                    title="注文書番号: ${escapeAttr(r.orderSlipNo)}（クリックで日程編集）"
                    aria-label="注文書番号 ${escapeAttr(r.orderSlipNo)} の日程編集を開く">${escapeHtml(r.orderSlipNo)}</a>`
              : '<span class="eh-muted">—</span>'}
          </td>
          <td>${escapeHtml(r.vendor)}</td>
          <td>${escapeHtml(r.staff)}</td>
          <td>${escapeHtml(risk.messages.join(' / '))}</td>
          <td>${fmt(r.orderDate)}</td>
          <td>${fmt(r.startDate)}</td>
          <td>${fmt(r.inspectionDate)}</td>
          <td>${fmt(r.dueDate)}</td>
        </tr>
      `;
    }).join('');

    return `
      <section class="eh-section">
        <div class="eh-section-title">
          <strong>日程リスク</strong>
          <span>異常・注意のみ最大30件表示</span>
        </div>
        <div class="eh-table-scroll">
          <table class="eh-risk-table">
            <thead>
              <tr>
                <th>判定</th><th>手配書番号</th><th>注文書番号</th><th>外注先</th><th>担当</th><th>理由</th>
                <th>注文日</th><th>加工着手</th><th>完成検査</th><th>納期</th>
              </tr>
            </thead>
            <tbody>${rows || '<tr><td colspan="10" class="eh-ok-message">現在、日付矛盾・要注意案件はありません。</td></tr>'}</tbody>
          </table>
        </div>
      </section>
    `;
  }

  function renderScheduleDateHeader(viewStart, viewEnd, totalDays) {
    const cells = [];
    for (let d = new Date(viewStart.getTime()); d <= viewEnd; d = addDays(d, 1)) {
      const dow = d.getDay();
      const cls = dow === 0 ? ' eh-schedule-date-sun' : (dow === 6 ? ' eh-schedule-date-sat' : '');
      cells.push(
        `<div class="eh-schedule-date-cell${cls}" title="${formatDate(d)}">` +
          `<span class="eh-schedule-date-day">${d.getDate()}</span>` +
          `<span class="eh-schedule-date-dow">${['日','月','火','水','木','金','土'][dow]}</span>` +
        `</div>`
      );
    }
    return `<div class="eh-schedule-date-header">${cells.join('')}</div>`;
  }

  function renderVendorDetail(records, vendor, viewStart, viewEnd) {
    if (!vendor) return '';

    const list = records
      .filter(r => r.vendor === vendor)
      .sort((a, b) => compareDate(a.dueDate, b.dueDate));

    const totalDays = Math.max(1, diffDays(viewStart, viewEnd) + 1);

    const rows = list.map(r => {
      const leftWhisker = clippedCellSpan(r.orderDate, r.startDate, viewStart, viewEnd);
      const workbar = clippedCellSpan(r.startDate, r.inspectionDate, viewStart, viewEnd);
      const rightWhisker = clippedCellSpan(r.inspectionDate, r.dueDate, viewStart, viewEnd);
      const risk = analyzeRisk(r);

      const leftBad = !!(r.orderDate && r.startDate && r.startDate < r.orderDate);
      const workBad = !!(r.startDate && r.inspectionDate && r.inspectionDate < r.startDate);
      const rightBad = !!(r.inspectionDate && r.dueDate && r.inspectionDate > r.dueDate);

      const displayKey = r.displayKey;

      return `
        <div class="eh-gantt-row${r.isManual ? ' eh-manual-row' : ''}${r.receiptDate ? ' eh-completed' : ''}" data-record-id="${escapeAttr(r.id)}">
          <div class="eh-gantt-cell eh-gantt-col-order">
            <div class="eh-order-key-line">
              ${r.isManual ? '<span class="eh-manual-badge" title="手動調整された日程">M</span>' : ''}
              <a href="#" class="eh-record-link eh-schedule-edit-trigger${r.arrangementNo ? '' : ' eh-no-arrangement'}"
                 data-record-id="${escapeAttr(r.id)}"
                 title="手配書番号: ${escapeAttr(displayKey)}（クリックで日程編集）"
                 aria-label="${escapeAttr(displayKey)} の日程編集を開く">${escapeHtml(displayKey)}</a>
            </div>
            ${r.orderSlipNo
              ? `<a href="#" class="eh-order-slip-no eh-order-slip-edit-link eh-schedule-edit-trigger"
                    data-record-id="${escapeAttr(r.id)}"
                    title="注文書番号: ${escapeAttr(r.orderSlipNo)}（クリックで日程編集）"
                    aria-label="注文書番号 ${escapeAttr(r.orderSlipNo)} の日程編集を開く">注文書 ${escapeHtml(r.orderSlipNo)}</a>`
              : '<div class="eh-order-slip-no eh-muted" title="注文書番号">注文書 —</div>'}
            <div class="eh-dates" title="手配日 → 仕掛開始 → 検収日 → 納期">${fmt(r.orderDate)} → ${fmt(r.startDate)} → ${fmt(r.inspectionDate)} → ${fmt(r.dueDate)}</div>
            ${risk.severity !== 'ok'
              ? `<div class="eh-inline-risk eh-${risk.severity}-text" title="${escapeAttr(risk.messages.join(' / '))}">⚠ ${escapeHtml(risk.messages.join(' / '))}</div>`
              : ''}
            <div class="eh-row-action-wrap">
              ${renderReceiptControl(r)}
              <button type="button" class="eh-schedule-edit-button eh-schedule-edit-trigger" data-record-id="${escapeAttr(r.id)}">日程編集</button>
            </div>
          </div>
          <div class="eh-gantt-cell eh-gantt-col-vendor" title="${escapeAttr(r.vendor || '')}">
            <span class="eh-cell-text">${escapeHtml(r.vendor || '')}</span>
          </div>
          <div class="eh-gantt-cell eh-gantt-col-staff" title="${escapeAttr(r.staff || '')}">
            <span class="eh-cell-text">${escapeHtml(r.staff || '')}</span>
          </div>
          <div class="eh-track">
            ${renderTrackBands(viewStart, viewEnd, totalDays)}
            ${leftWhisker ? `<div class="eh-whisker eh-whisker-left${leftBad ? ' eh-alert-segment' : ''}" style="${cellSpanStyle(leftWhisker, 0)}"></div>` : ''}
            ${workbar ? `<div class="eh-workbar eh-plugin-clip eh-work-${risk.severity}${workBad ? ' eh-alert-segment' : ''}${r.isManual ? ' eh-manual-workbar' : ''}" style="${cellSpanStyle(workbar, 2)}" title="${escapeAttr(r.product || '')}"><span class="eh-plugin-start-cap"></span><span class="eh-workbar-label">${escapeHtml(r.product || '')}</span><span class="eh-plugin-end-cap"></span></div>` : ''}
            ${rightWhisker ? `<div class="eh-whisker eh-whisker-right${rightBad ? ' eh-alert-segment' : ''}" style="${cellSpanStyle(rightWhisker, 0)}"></div>` : ''}
            ${makePluginMarker(r.orderDate, viewStart, viewEnd, 'order', leftBad, '手配日')}
            ${makePluginMarker(r.startDate, viewStart, viewEnd, 'start', leftBad || workBad, '仕掛開始')}
            ${makePluginMarker(r.inspectionDate, viewStart, viewEnd, 'inspection', workBad || rightBad, '検収日')}
            ${makePluginMarker(r.dueDate, viewStart, viewEnd, 'due', rightBad, '納期')}
          </div>
        </div>
      `;
    }).join('');

    return `
      <section class="eh-section">
        <div class="eh-section-title eh-detail-title">
          <div class="eh-detail-title-text">
            <strong>${escapeHtml(vendor)}：案件詳細</strong>
            <span>外注先行をクリックするとここが切り替わります。下段は1日1マスの予定表表示です。</span>
          </div>
        </div>
        <div class="eh-gantt-guide">
          <div class="eh-gantt-guide-visual" aria-label="日程ひげチャート凡例">
            <span class="eh-guide-symbol eh-guide-order-symbol">○</span>
            <span class="eh-guide-line eh-guide-left"></span>
            <span class="eh-guide-symbol eh-guide-start-symbol">▶</span>
            <span class="eh-guide-bar"></span>
            <span class="eh-guide-symbol eh-guide-inspection-symbol">●</span>
            <span class="eh-guide-line eh-guide-right"></span>
            <span class="eh-guide-symbol eh-guide-due-symbol">◆</span>
          </div>
          <div class="eh-gantt-guide-labels">
            <span>○ 手配日</span><span>▶ 仕掛開始</span><span>● 検収日</span><span>◆ 納期</span>
          </div>
          <div class="eh-gantt-status-legends"><span class="eh-legend-alert">赤＝日付矛盾・要注意</span><span class="eh-legend-manual"><span class="eh-manual-badge eh-manual-badge-legend">M</span>＝手動調整</span></div>
        </div>
        <div class="eh-schedule-note">
          左側に手配書番号・注文書番号・指示先名・手配担当者を固定表示し、右側を1日1マスの予定表として表示します。M表示・太枠・背景色変更は手動調整（K日程種別=Manual）を示します。
        </div>
        <div class="eh-schedule-scroll eh-size-${state.detailSize}" style="--eh-total-days:${totalDays};">
          <div class="eh-schedule-head-row">
            <div class="eh-schedule-col-head eh-head-order">手配書番号 / 注文書番号</div>
            <div class="eh-schedule-col-head eh-head-vendor">指示先名</div>
            <div class="eh-schedule-col-head eh-head-staff">手配担当者</div>
            <div class="eh-schedule-timeline">
              ${renderScheduleDateHeader(viewStart, viewEnd, totalDays)}
            </div>
          </div>
          <div class="eh-gantt-list">${rows || '<div class="eh-empty">対象案件なし</div>'}</div>
        </div>
      </section>
    `;
  }

  function buildVendorStats(records, dates) {
    const byVendor = new Map();

    records.forEach(r => {
      if (!byVendor.has(r.vendor)) byVendor.set(r.vendor, []);
      byVendor.get(r.vendor).push(r);
    });

    return Array.from(byVendor.entries()).map(([vendor, list]) => {
      const counts = dates.map(d => list.reduce((n, r) => {
        if (!r.startDate || !r.inspectionDate) return n;
        return (r.startDate <= d && d <= r.inspectionDate) ? n + 1 : n;
      }, 0));

      return {
        vendor,
        recordCount: list.length,
        counts,
        maxConcurrent: Math.max.apply(null, counts.concat([0]))
      };
    }).sort((a, b) =>
      b.maxConcurrent - a.maxConcurrent ||
      b.recordCount - a.recordCount ||
      a.vendor.localeCompare(b.vendor, 'ja')
    );
  }

  function analyzeRisk(r) {
    const messages = [];
    let severity = 'ok';

    if (!r.orderDate || !r.startDate || !r.inspectionDate || !r.dueDate) {
      messages.push('4日付のいずれかが未設定');
      severity = 'warning';
    }

    if (r.orderDate && r.startDate && r.startDate < r.orderDate) {
      messages.push('加工着手日が注文日より前');
      severity = 'danger';
    }

    if (r.startDate && r.inspectionDate && r.inspectionDate < r.startDate) {
      messages.push('完成検査日が加工着手日より前');
      severity = 'danger';
    }

    if (r.inspectionDate && r.dueDate && r.inspectionDate > r.dueDate) {
      messages.push('完成検査日が納期を超過');
      severity = 'danger';
    } else if (r.inspectionDate && r.dueDate) {
      const buffer = diffDays(r.inspectionDate, r.dueDate);
      if (buffer < CONFIG.minInspectionToDueDays) {
        messages.push(`完成検査→納期の余裕が${buffer}日`);
        if (severity !== 'danger') severity = 'warning';
      }
    }

    return { severity, messages };
  }

  function chooseInitialVendor(records) {
    const today = startOfDay(new Date());
    const dates = makeDateRange(today, addDays(today, CONFIG.initialDays - 1));
    const stats = buildVendorStats(records, dates);
    return stats.length ? stats[0].vendor : null;
  }

  /*
   * Ver0.6.1:
   * kintoneの描画タイミング（レイアウト確定前）に対する保険として、
   * 二重rAF＋setTimeoutの両方で一度だけ実処理を走らせる。
   * jumpToTodayAfterRender / restoreScrollPositions の共通処理。
   */
  function runAfterRenderSettled(fn) {
    requestAnimationFrame(function () {
      requestAnimationFrame(fn);
    });
    setTimeout(fn, 120);
  }

  /*
   * Ver0.6.1.4:
   * .eh-table-scroll は上段ヒートマップ以外（日程リスク一覧）にも存在する。
   * 「先頭の .eh-table-scroll」を決め打ちせず、必ず .eh-heatmap を起点に
   * 本来の上段スクロールコンテナを特定する。
   */
  function getHeatmapScrollContainer(root) {
    const heatmap = root.querySelector('.eh-heatmap');
    return heatmap ? heatmap.closest('.eh-table-scroll') : null;
  }

  /*
   * Ver0.6.1.4:
   * WheelEvent.deltaMode を考慮して、縦ホイール量を横スクロール量へ正規化する。
   * 0=pixel, 1=line, 2=page。
   */
  function normalizeHorizontalWheelDelta(event, scrollEl) {
    const value = Number(event.deltaY || 0);

    if (event.deltaMode === 1) {
      return value * 16;
    }

    if (event.deltaMode === 2) {
      return value * Math.max(1, scrollEl.clientWidth * 0.9);
    }

    return value;
  }

  /*
   * Ver0.6.1.4:
   * 通常ホイールは従来どおり縦操作のまま残し、
   * Shift＋「deltaY主成分」のホイールだけを横スクロールへ変換する。
   * deltaX主成分はMacトラックパッドやChrome/Edge等のネイティブ横入力として
   * ブラウザ標準動作へ任せ、二重移動を防ぐ。
   */
  function bindHorizontalWheelSupport(root) {
    const upperScroll = getHeatmapScrollContainer(root);
    const lowerScroll = root.querySelector('.eh-schedule-scroll');

    [upperScroll, lowerScroll].forEach(function (scrollEl) {
      if (!scrollEl || scrollEl.dataset.ehHorizontalWheelBound === '1') return;

      scrollEl.dataset.ehHorizontalWheelBound = '1';

      scrollEl.addEventListener('wheel', function (event) {
        if (!event.shiftKey) return;

        const absX = Math.abs(Number(event.deltaX || 0));
        const absY = Math.abs(Number(event.deltaY || 0));

        // 既に横入力として届いている場合はブラウザ標準動作へ任せる。
        if (absX >= absY && absX > 0) return;

        // Shift＋縦ホイールとして扱える入力が無ければ何もしない。
        if (absY <= 0) return;

        // 左右端でも本機能の操作として一貫して消費する。
        event.preventDefault();
        scrollEl.scrollLeft += normalizeHorizontalWheelDelta(event, scrollEl);
      }, { passive: false });
    });
  }

  function jumpToTodayAfterRender(root, today, myGeneration) {
    const targetDate = formatDate(today);

    const scrollOne = function (container, target, fixedWidth) {
      if (!container || !target) return;
      const cRect = container.getBoundingClientRect();
      const tRect = target.getBoundingClientRect();
      const desired = container.scrollLeft + (tRect.left - cRect.left) - Math.max(0, fixedWidth || 0);
      container.scrollLeft = Math.max(0, desired);
    };

    const run = function () {
      // 自分より新しいrender()が既に走っていたら、古い結果で上書きしない。
      if (state.renderGeneration !== myGeneration) return;

      const upper = getHeatmapScrollContainer(root);
      const upperToday = upper
        ? upper.querySelector(`.eh-date[title="${targetDate}"]`)
        : null;
      const upperSticky = upper
        ? upper.querySelector('.eh-vendor.eh-sticky')
        : null;
      const upperFixedWidth = upperSticky
        ? upperSticky.getBoundingClientRect().width
        : 0;
      scrollOne(upper, upperToday, upperFixedWidth);

      const lower = root.querySelector('.eh-schedule-scroll');
      const lowerToday = lower
        ? lower.querySelector(`.eh-schedule-date-cell[title="${targetDate}"]`)
        : null;
      const lowerFixedWidth = lower
        ? Array.from(lower.querySelectorAll('.eh-schedule-col-head'))
            .slice(0, 3)
            .reduce((sum, el) => sum + el.getBoundingClientRect().width, 0)
        : 0;
      scrollOne(lower, lowerToday, lowerFixedWidth);
    };

    runAfterRenderSettled(run);
  }

  /*
   * Ver0.6.1:
   * root.innerHTML を書き換える直前に、上段/下段スクロール要素の
   * scrollLeft を退避する。要素が無ければ null を保持する。
   */
  function captureScrollPositions(root) {
    const upper = getHeatmapScrollContainer(root);
    const lower = root.querySelector('.eh-schedule-scroll');
    return {
      upper: upper ? upper.scrollLeft : null,
      lower: lower ? lower.scrollLeft : null
    };
  }

  /*
   * Ver0.6.1:
   * 再描画後、新しく作られたスクロール要素へ退避しておいた位置を戻す。
   * 退避時に要素が無かった（値がnull）場合はその軸には触れない。
   */
  function restoreScrollPositions(root, saved, myGeneration) {
    if (!saved) return;

    const run = function () {
      // 自分より新しいrender()が既に走っていたら、古い結果で上書きしない。
      if (state.renderGeneration !== myGeneration) return;

      const upper = getHeatmapScrollContainer(root);
      if (upper && saved.upper !== null) {
        upper.scrollLeft = saved.upper;
      }

      const lower = root.querySelector('.eh-schedule-scroll');
      if (lower && saved.lower !== null) {
        lower.scrollLeft = saved.lower;
      }
    };

    runAfterRenderSettled(run);
  }

  function bindVendorQuickFilterEvents(root) {
    const prepareQuickFilterRender = function () {
      // 入荷PUT中は既存更新処理を優先し、描画DOMを差し替えない。
      if (activeReceipt && activeReceipt.saving) return false;
      if (activeEdit && activeEdit.saving) return false;
      if (activeReceipt) closeActiveReceiptEditor();
      // 日程編集ポップアップはdocument.body直下にあるため、通常のrender()では保持できる。
      return true;
    };

    const toggle = root.querySelector('#eh-quick-filter-toggle');
    const panel = root.querySelector('#eh-quick-filter-panel');
    const searchInput = root.querySelector('#eh-quick-filter-search');

    if (toggle && panel) {
      toggle.addEventListener('click', function (event) {
        event.preventDefault();
        state.quickFilterOpen = !state.quickFilterOpen;
        panel.hidden = !state.quickFilterOpen;
        toggle.setAttribute('aria-expanded', state.quickFilterOpen ? 'true' : 'false');
        if (state.quickFilterOpen && searchInput) {
          window.setTimeout(() => searchInput.focus(), 0);
        }
      });

    }

    if (searchInput) {
      searchInput.addEventListener('input', function () {
        state.quickVendorSearch = this.value || '';
        const q = state.quickVendorSearch.trim().toLocaleLowerCase('ja');
        let visibleCount = 0;
        root.querySelectorAll('.eh-quick-filter-option').forEach(option => {
          const checkbox = option.querySelector('.eh-quick-vendor-checkbox');
          const vendor = checkbox ? String(checkbox.value || '') : '';
          const matches = !q || vendor.toLocaleLowerCase('ja').includes(q);
          option.classList.toggle('eh-quick-filter-option-hidden', !matches);
          if (matches) visibleCount += 1;
        });
        const noMatch = root.querySelector('.eh-quick-filter-no-match');
        if (noMatch) noMatch.hidden = visibleCount !== 0;
      });
    }

    root.querySelectorAll('.eh-quick-vendor-checkbox').forEach(checkbox => {
      checkbox.addEventListener('change', function () {
        const vendor = String(this.value || '');
        if (!vendor) return;
        if (!prepareQuickFilterRender()) {
          this.checked = state.selectedQuickVendors.has(vendor);
          return;
        }
        if (this.checked) state.selectedQuickVendors.add(vendor);
        else state.selectedQuickVendors.delete(vendor);
        state.quickFilterOpen = true;
        render(root, false);
      });
    });

    root.querySelectorAll('.eh-quick-filter-clear').forEach(button => {
      button.addEventListener('click', function (event) {
        event.preventDefault();
        if (state.selectedQuickVendors.size === 0) return;
        if (!prepareQuickFilterRender()) return;
        state.selectedQuickVendors.clear();
        state.quickVendorSearch = '';
        state.quickFilterOpen = false;
        render(root, false);
      });
    });

    root.querySelectorAll('.eh-quick-filter-chip-remove').forEach(button => {
      button.addEventListener('click', function (event) {
        event.preventDefault();
        const vendor = String(this.dataset.vendor || '');
        if (!vendor) return;
        if (!prepareQuickFilterRender()) return;
        state.selectedQuickVendors.delete(vendor);
        render(root, false);
      });
    });

    if (!bindVendorQuickFilterEvents.documentBound) {
      bindVendorQuickFilterEvents.documentBound = true;
      document.addEventListener('click', function (event) {
        if (!state.quickFilterOpen) return;
        const currentRoot = document.getElementById(CONFIG.rootId);
        if (!currentRoot) return;
        const dropdown = currentRoot.querySelector('.eh-quick-filter-dropdown');
        if (dropdown && dropdown.contains(event.target)) return;
        state.quickFilterOpen = false;
        const currentPanel = currentRoot.querySelector('#eh-quick-filter-panel');
        const currentToggle = currentRoot.querySelector('#eh-quick-filter-toggle');
        if (currentPanel) currentPanel.hidden = true;
        if (currentToggle) currentToggle.setAttribute('aria-expanded', 'false');
      });
      document.addEventListener('keydown', function (event) {
        if (event.key !== 'Escape' || !state.quickFilterOpen) return;
        state.quickFilterOpen = false;
        const currentRoot = document.getElementById(CONFIG.rootId);
        if (!currentRoot) return;
        const currentPanel = currentRoot.querySelector('#eh-quick-filter-panel');
        const currentToggle = currentRoot.querySelector('#eh-quick-filter-toggle');
        if (currentPanel) currentPanel.hidden = true;
        if (currentToggle) {
          currentToggle.setAttribute('aria-expanded', 'false');
          currentToggle.focus();
        }
      });
    }
  }

  function bindEvents(root) {
    bindReceiptEvents(root);
    bindScheduleEditEvents(root);
    bindAcceptanceSyncEvents(root);
    bindHorizontalWheelSupport(root);
    bindVendorQuickFilterEvents(root);

    const range = root.querySelector('#eh-range');
    if (range) {
      range.addEventListener('change', function () {
        state.rangeDays = Number(this.value) || CONFIG.initialDays;
        // Ver0.6.1: UI内再描画。直前のスクロール位置を維持する。
        render(root, false);
      });
    }

    const displaySize = root.querySelector('#eh-display-size');
    if (displaySize) {
      displaySize.addEventListener('change', function () {
        const value = ['standard', 'large', 'xlarge'].includes(this.value)
          ? this.value
          : 'standard';
        state.detailSize = value;
        saveDetailSize(value);
        // Ver0.6.1: UI内再描画。直前のスクロール位置を維持する。
        render(root, false);
      });
    }

    root.querySelectorAll('.eh-vendor-row').forEach(row => {
      row.addEventListener('click', function () {
        state.selectedVendor = this.dataset.vendor;
        // Ver0.6.1: UI内再描画。直前のスクロール位置を維持する。
        render(root, false);
        const detail = root.querySelector('.eh-gantt-list');
        if (detail) detail.scrollIntoView({ behavior: 'smooth', block: 'start' });
      });
    });
  }


  /* =====================================================================
   * Ver0.6.2.3-PROD-CANDIDATE1: App293受入実績同期（β）
   * ===================================================================== */

  let activeAcceptanceSync = null; // { overlay, dialog, analysis, executing }

  function syncValue(record, code) {
    return record && record[code] && record[code].value != null
      ? String(record[code].value).trim()
      : '';
  }

  function isValidYmd(value) {
    if (!/^\d{4}-\d{2}-\d{2}$/.test(String(value || ''))) return false;
    const [y, m, d] = String(value).split('-').map(Number);
    const dt = new Date(y, m - 1, d);
    return dt.getFullYear() === y && dt.getMonth() === m - 1 && dt.getDate() === d;
  }

  function toNumberOrNull(value) {
    const s = String(value == null ? '' : value).trim();
    if (!s) return null;
    const n = Number(s);
    return Number.isFinite(n) ? n : null;
  }

  function uniqSorted(values) {
    return Array.from(new Set(values.filter(v => v != null && String(v) !== ''))).sort();
  }

  function maxYmd(values) {
    const arr = values.filter(isValidYmd).sort();
    return arr.length ? arr[arr.length - 1] : '';
  }

  function minYmd(values) {
    const arr = values.filter(isValidYmd).sort();
    return arr.length ? arr[0] : '';
  }

  async function fetchAllRecordsForAcceptanceSync(appId, fields) {
    let all = [];
    let lastId = 0;

    while (true) {
      const query = `$id > ${lastId} order by $id asc limit 500`;
      const response = await kintone.api(
        kintone.api.url('/k/v1/records.json', true),
        'GET',
        { app: appId, query: query, fields: fields }
      );
      const batch = (response && response.records) || [];
      all = all.concat(batch);
      if (batch.length < 500) break;
      lastId = Number(syncValue(batch[batch.length - 1], '$id'));
      if (!Number.isFinite(lastId) || lastId <= 0) {
        throw new Error(`App ${appId}: $idページングに失敗しました。`);
      }
    }

    return all;
  }

  async function fetchFormFieldsForAcceptanceSync(appId) {
    const response = await kintone.api(
      kintone.api.url('/k/v1/app/form/fields.json', true),
      'GET',
      { app: appId }
    );
    return (response && response.properties) || {};
  }

  function assertAcceptanceSyncPreflight(targetAppId, targetProps, sourceProps) {
    const c = CONFIG.acceptanceSync;
    if (!c.allowedTargetAppIds.includes(Number(targetAppId))) {
      throw new Error(
        `SafetyStop: この同期機能は App292 TEST / App272 PROD 専用です。現在 App${targetAppId} です。`
      );
    }

    const tf = c.targetFields;
    const sf = c.sourceFields;

    [tf.po, tf.acceptanceDate, tf.orderNo, tf.orderSlipNo].forEach(code => {
      if (!targetProps[code]) {
        throw new Error(`SafetyStop: 対象Appにフィールド ${code} がありません。`);
      }
    });
    if (targetProps[tf.acceptanceDate].type !== 'DATE') {
      throw new Error(
        `SafetyStop: ${tf.acceptanceDate} の型がDATEではありません（${targetProps[tf.acceptanceDate].type}）。`
      );
    }

    [sf.receiptNo, sf.po, sf.receiptDate, sf.receiptQty, sf.orderNo, sf.orderSlipNo].forEach(code => {
      if (!sourceProps[code]) {
        throw new Error(`SafetyStop: App293にフィールド ${code} がありません。`);
      }
    });
    if (sourceProps[sf.receiptDate].type !== 'DATE') {
      throw new Error(
        `SafetyStop: App293.${sf.receiptDate} の型がDATEではありません。`
      );
    }
    if (sourceProps[sf.receiptQty].type !== 'NUMBER') {
      throw new Error(
        `SafetyStop: App293.${sf.receiptQty} の型がNUMBERではありません。`
      );
    }
    if (!sourceProps[sf.receiptNo].unique) {
      throw new Error(
        `SafetyStop: App293.${sf.receiptNo} の重複禁止設定が有効ではありません。`
      );
    }
  }

  function buildAcceptanceSyncAnalysis(sourceRecords, targetRecords) {
    const c = CONFIG.acceptanceSync;
    const sf = c.sourceFields;
    const tf = c.targetFields;

    const safetyStops = [];
    const sourceIssues = {
      missingPO: [],
      invalidReceiptDate: [],
      blankReceiptNo: [],
      duplicateReceiptNos: []
    };

    // App293 受入番号の空欄/重複を再確認（設計前提を実行時にも検証）
    const receiptNoMap = new Map();
    sourceRecords.forEach(r => {
      const receiptNo = syncValue(r, sf.receiptNo);
      if (!receiptNo) {
        sourceIssues.blankReceiptNo.push(syncValue(r, sf.recordId));
        return;
      }
      if (!receiptNoMap.has(receiptNo)) receiptNoMap.set(receiptNo, []);
      receiptNoMap.get(receiptNo).push(r);
    });
    receiptNoMap.forEach((list, receiptNo) => {
      if (list.length > 1) sourceIssues.duplicateReceiptNos.push(receiptNo);
    });

    if (sourceIssues.blankReceiptNo.length > 0) {
      safetyStops.push(
        `SafetyStop: App293 受入番号空欄 ${sourceIssues.blankReceiptNo.length}件`
      );
    }
    if (sourceIssues.duplicateReceiptNos.length > 0) {
      safetyStops.push(
        `SafetyStop: App293 受入番号重複 ${sourceIssues.duplicateReceiptNos.length}件`
      );
    }

    // App272/App292側 発注番号一意性を毎回再確認（Claude指摘C反映）
    const targetByPO = new Map();
    let targetMissingPOCount = 0;
    targetRecords.forEach(r => {
      const po = syncValue(r, tf.po);
      if (!po) {
        targetMissingPOCount += 1;
        return;
      }
      if (!targetByPO.has(po)) targetByPO.set(po, []);
      targetByPO.get(po).push(r);
    });

    const duplicateTargetPOs = [];
    targetByPO.forEach((list, po) => {
      if (list.length > 1) duplicateTargetPOs.push({ po, count: list.length });
    });
    if (duplicateTargetPOs.length > 0) {
      safetyStops.push(
        `SafetyStop: App272 duplicate order number ${duplicateTargetPOs.length}発注番号`
      );
    }
    const duplicateTargetPOSet = new Set(duplicateTargetPOs.map(x => x.po));

    // App293を発注番号単位でグループ化
    const sourceByPO = new Map();
    sourceRecords.forEach(r => {
      const po = syncValue(r, sf.po);
      const receiptDate = syncValue(r, sf.receiptDate);

      if (!po) {
        sourceIssues.missingPO.push({
          id: syncValue(r, sf.recordId),
          receiptNo: syncValue(r, sf.receiptNo)
        });
        return;
      }

      if (!sourceByPO.has(po)) sourceByPO.set(po, []);
      sourceByPO.get(po).push(r);

      if (!isValidYmd(receiptDate)) {
        sourceIssues.invalidReceiptDate.push({
          po,
          receiptNo: syncValue(r, sf.receiptNo),
          receiptDate
        });
      }
    });

    const rows = [];
    const sourcePOs = new Set(sourceByPO.keys());
    const targetPOs = new Set(targetByPO.keys());

    // App293にある発注番号を分類
    sourceByPO.forEach((group, po) => {
      // App272側重複POは同期分類から除外し、SafetyStop詳細としてのみ保持
      if (duplicateTargetPOSet.has(po)) return;

      const dates = group.map(r => syncValue(r, sf.receiptDate));
      const validDates = dates.filter(isValidYmd);
      const invalidDateCount = dates.length - validDates.length;
      const negativeCount = group.reduce((acc, r) => {
        const q = toNumberOrNull(syncValue(r, sf.receiptQty));
        return acc + (q != null && q < 0 ? 1 : 0);
      }, 0);

      const candidate = maxYmd(validDates);
      const first = minYmd(validDates);
      const last = maxYmd(validDates);
      const distinctDates = uniqSorted(validDates).length;
      const receiptNumbers = uniqSorted(group.map(r => syncValue(r, sf.receiptNo)));

      const targetList = targetByPO.get(po) || [];
      if (targetList.length === 0) {
        rows.push({
          po,
          targetId: '',
          targetRevision: '',
          currentAcceptanceDate: '',
          sourceRowCount: group.length,
          distinctReceiptDateCount: distinctDates,
          firstReceiptDate: first,
          lastReceiptDate: last,
          candidateReceiptDate: candidate,
          negativeReceiptCount: negativeCount,
          receiptNumbers,
          className: 'SOURCE_ONLY_NO_APP272',
          futureAutoEligible: 'NO',
          reason: 'App293に受入実績あり / 対象Appに同一発注番号なし'
        });
        return;
      }

      const target = targetList[0];
      const current = syncValue(target, tf.acceptanceDate);

      let className;
      let futureAutoEligible = 'NO';
      let reason;

      if (negativeCount > 0 || invalidDateCount > 0 || !candidate) {
        className = 'SOURCE_REVIEW_REQUIRED';
        reason = [
          negativeCount > 0 ? `負数受入 ${negativeCount}件` : '',
          invalidDateCount > 0 ? `受入日不正/空欄 ${invalidDateCount}件` : '',
          !candidate ? '有効な候補受入日なし' : ''
        ].filter(Boolean).join(' / ');
      } else if (current) {
        if (current === candidate) {
          className = 'ALREADY_SAME';
          reason = '既存受入れ日と候補日が一致';
        } else {
          className = 'EXISTING_DIFFERENT';
          reason = '既存受入れ日を保護（候補日と不一致）';
        }
      } else if (group.length === 1) {
        className = 'CANDIDATE_EMPTY_SINGLE';
        futureAutoEligible = 'YES';
        reason = '受入れ日空欄 / 単一受入 / レビュー要因なし';
      } else {
        className = 'CANDIDATE_EMPTY_MULTI';
        reason = '受入れ日空欄 / 複数受入。MAX(受入日)は完納日と定義しない';
      }

      rows.push({
        po,
        targetId: syncValue(target, tf.recordId),
        targetRevision: syncValue(target, tf.revision),
        currentAcceptanceDate: current,
        sourceRowCount: group.length,
        distinctReceiptDateCount: distinctDates,
        firstReceiptDate: first,
        lastReceiptDate: last,
        candidateReceiptDate: candidate,
        negativeReceiptCount: negativeCount,
        receiptNumbers,
        className,
        futureAutoEligible,
        reason
      });
    });

    // App272/App292にあるがApp293に無い発注番号
    targetByPO.forEach((targetList, po) => {
      if (duplicateTargetPOSet.has(po)) return;
      if (sourcePOs.has(po)) return;
      const target = targetList[0];
      rows.push({
        po,
        targetId: syncValue(target, tf.recordId),
        targetRevision: syncValue(target, tf.revision),
        currentAcceptanceDate: syncValue(target, tf.acceptanceDate),
        sourceRowCount: 0,
        distinctReceiptDateCount: 0,
        firstReceiptDate: '',
        lastReceiptDate: '',
        candidateReceiptDate: '',
        negativeReceiptCount: 0,
        receiptNumbers: [],
        className: 'NO_SOURCE_DATA_UNKNOWN',
        futureAutoEligible: 'NO',
        reason: 'Prones受入実績として確認できない（未受入とは判定しない）'
      });
    });

    rows.sort((a, b) =>
      a.className.localeCompare(b.className, 'en') ||
      a.po.localeCompare(b.po, 'ja')
    );

    const classCounts = {};
    rows.forEach(r => {
      classCounts[r.className] = (classCounts[r.className] || 0) + 1;
    });

    let candidates = rows.filter(r =>
      r.className === 'CANDIDATE_EMPTY_SINGLE' &&
      r.futureAutoEligible === 'YES'
    );

    const targetAppId = Number(kintone.app.getId());
    const isControlledProdTarget =
      targetAppId === Number(c.prodExecute && c.prodExecute.targetAppId);

    let bulkWriteBlocked = false;
    let bulkWriteReason = '';

    if (candidates.length > c.maxWrites) {
      if (isControlledProdTarget) {
        bulkWriteBlocked = true;
        bulkWriteReason =
          `PROD CONTROLLED MODE: 更新候補は全${candidates.length}件。` +
          `一括実行は禁止し、1回最大${c.prodExecute.maxSelected}件を手動選択してください。`;
      } else {
        bulkWriteBlocked = true;
        bulkWriteReason =
          `Analysis Only: 更新候補 ${candidates.length}件 / MaxWrites ${c.maxWrites}件`;
      }
    }

    // 構造異常SafetyStop発生時だけ書込み候補を強制0件にする。
    // 候補総数>MaxWritesはPRODでは「選択式バッチ」に切り替え、全件一括を禁止する。
    if (safetyStops.length > 0) {
      candidates = [];
    }

    return {
      generatedAt: new Date().toISOString(),
      sourceRecordCount: sourceRecords.length,
      targetRecordCount: targetRecords.length,
      sourceMissingPOCount: sourceIssues.missingPO.length,
      sourceInvalidReceiptDateCount: sourceIssues.invalidReceiptDate.length,
      sourceBlankReceiptNoCount: sourceIssues.blankReceiptNo.length,
      sourceDuplicateReceiptNoCount: sourceIssues.duplicateReceiptNos.length,
      targetMissingPOCount,
      duplicateTargetPOs,
      safetyStops,
      rows,
      classCounts,
      candidates,
      maxWrites: c.maxWrites,
      bulkWriteBlocked,
      bulkWriteReason,
      controlledProdMode: isControlledProdTarget,
      controlledProdMaxSelected:
        isControlledProdTarget ? Number(c.prodExecute.maxSelected || 0) : 0
    };
  }

  async function runAcceptanceSyncAnalysis() {
    const c = CONFIG.acceptanceSync;
    const targetAppId = Number(kintone.app.getId());
    const sourceAppId = Number(c.sourceAppId);

    const [targetProps, sourceProps] = await Promise.all([
      fetchFormFieldsForAcceptanceSync(targetAppId),
      fetchFormFieldsForAcceptanceSync(sourceAppId)
    ]);
    assertAcceptanceSyncPreflight(targetAppId, targetProps, sourceProps);

    const tf = c.targetFields;
    const sf = c.sourceFields;

    const [sourceRecords, targetRecords] = await Promise.all([
      fetchAllRecordsForAcceptanceSync(sourceAppId, [
        sf.recordId, sf.revision, sf.receiptNo, sf.po,
        sf.receiptDate, sf.receiptQty, sf.orderNo, sf.orderSlipNo
      ]),
      fetchAllRecordsForAcceptanceSync(targetAppId, [
        tf.recordId, tf.revision, tf.po, tf.acceptanceDate,
        tf.orderNo, tf.orderSlipNo
      ])
    ]);

    return buildAcceptanceSyncAnalysis(sourceRecords, targetRecords);
  }

  function acceptanceClassCount(analysis, className) {
    return Number((analysis.classCounts && analysis.classCounts[className]) || 0);
  }

  function acceptanceSyncCsv(analysis) {
    const headers = [
      'Timestamp', 'PO', 'App272RecordId', 'App272Revision',
      'CurrentAcceptanceDate', 'SourceRowCount', 'DistinctReceiptDateCount',
      'FirstReceiptDate', 'LastReceiptDate', 'CandidateReceiptDate',
      'NegativeReceiptCount', 'Class', 'FutureAutoEligible', 'Reason'
    ];

    const q = value => {
      const s = String(value == null ? '' : value);
      return `"${s.replace(/"/g, '""')}"`;
    };

    const lines = [headers.map(q).join(',')];
    analysis.rows.forEach(r => {
      lines.push([
        analysis.generatedAt, r.po, r.targetId, r.targetRevision,
        r.currentAcceptanceDate, r.sourceRowCount, r.distinctReceiptDateCount,
        r.firstReceiptDate, r.lastReceiptDate, r.candidateReceiptDate,
        r.negativeReceiptCount, r.className, r.futureAutoEligible, r.reason
      ].map(q).join(','));
    });
    return '\uFEFF' + lines.join('\r\n');
  }

  function downloadAcceptanceSyncCsv(analysis) {
    const blob = new Blob([acceptanceSyncCsv(analysis)], {
      type: 'text/csv;charset=utf-8'
    });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    const stamp = new Date().toISOString().replace(/[-:TZ.]/g, '').slice(0, 14);
    a.href = url;
    a.download = `App272_AcceptanceSync_Analysis_${stamp}.csv`;
    document.body.appendChild(a);
    a.click();
    a.remove();
    setTimeout(() => URL.revokeObjectURL(url), 0);
  }

  function closeAcceptanceSyncDialog() {
    if (!activeAcceptanceSync) return;
    if (activeAcceptanceSync.executing) return;
    const overlay = activeAcceptanceSync.overlay;
    activeAcceptanceSync = null;
    if (overlay && overlay.parentNode) overlay.parentNode.removeChild(overlay);
  }

  function renderAcceptanceSyncDialog(analysis) {
    closeAcceptanceSyncDialog();

    const overlay = document.createElement('div');
    overlay.className = 'eh-acceptance-sync-overlay';

    const safetyHtml = analysis.safetyStops.length
      ? `
        <div class="eh-acceptance-sync-safety">
          <strong>SafetyStop：書込み禁止</strong>
          ${analysis.safetyStops.map(x => `<div>${escapeHtml(x)}</div>`).join('')}
          ${analysis.duplicateTargetPOs.length
            ? `<div class="eh-acceptance-sync-small">重複発注番号: ${
                analysis.duplicateTargetPOs.slice(0, 20)
                  .map(x => `${escapeHtml(x.po)} (${x.count}件)`).join(', ')
              }${analysis.duplicateTargetPOs.length > 20 ? ' …' : ''}</div>`
            : ''}
        </div>`
      : '';

    const classRows = [
      ['CANDIDATE_EMPTY_SINGLE', '自動更新候補'],
      ['CANDIDATE_EMPTY_MULTI', '複数受入・保留'],
      ['ALREADY_SAME', '既に一致'],
      ['EXISTING_DIFFERENT', '既存値保護'],
      ['SOURCE_REVIEW_REQUIRED', 'ソース要確認'],
      ['SOURCE_ONLY_NO_APP272', 'App293のみ'],
      ['NO_SOURCE_DATA_UNKNOWN', 'Prones受入実績として確認不能']
    ].map(([key, label]) => `
      <tr>
        <td>${escapeHtml(key)}</td>
        <td>${escapeHtml(label)}</td>
        <td class="eh-acceptance-sync-num">${acceptanceClassCount(analysis, key)}</td>
      </tr>
    `).join('');

    const candidateRows = analysis.rows
      .filter(r => r.className === 'CANDIDATE_EMPTY_SINGLE')
      .slice(0, 100)
      .map(r => `
        <tr>
          <td>
            ${analysis.controlledProdMode
              ? `<input type="checkbox"
                   class="eh-acceptance-sync-select"
                   data-po="${escapeAttr(r.po)}"
                   aria-label="発注番号 ${escapeAttr(r.po)} をPROD選択">`
              : '—'}
          </td>
          <td>${escapeHtml(r.po)}</td>
          <td>${escapeHtml(r.currentAcceptanceDate || '—')}</td>
          <td>${escapeHtml(r.candidateReceiptDate || '—')}</td>
          <td>${r.sourceRowCount}</td>
          <td>${escapeHtml(r.reason)}</td>
        </tr>
      `).join('');

    const dialog = document.createElement('div');
    dialog.className = 'eh-acceptance-sync-dialog';
    dialog.innerHTML = `
      <div class="eh-acceptance-sync-titlebar">
        <div>
          <div class="eh-acceptance-sync-title">受入実績同期（β） Analysis</div>
          <div class="eh-acceptance-sync-subtitle">
            App293 → App${escapeHtml(kintone.app.getId())} / 書込み前解析
          </div>
        </div>
        <button type="button" class="eh-acceptance-sync-close" aria-label="閉じる">×</button>
      </div>

      ${safetyHtml}

      <div class="eh-acceptance-sync-summary">
        <div><span>App293受入イベント</span><strong>${analysis.sourceRecordCount}</strong></div>
        <div><span>App293 発注番号空欄</span><strong>${analysis.sourceMissingPOCount}</strong></div>
        <div><span>対象Appレコード</span><strong>${analysis.targetRecordCount}</strong></div>
        <div><span>対象App 発注番号空欄</span><strong>${analysis.targetMissingPOCount}</strong></div>
        <div><span>更新候補</span><strong>${analysis.candidates.length}</strong></div>
        <div><span>MaxWrites</span><strong>${analysis.maxWrites}</strong></div>
      </div>

      <table class="eh-acceptance-sync-table eh-acceptance-sync-class-table">
        <thead><tr><th>Class</th><th>意味</th><th>件数</th></tr></thead>
        <tbody>${classRows}</tbody>
      </table>

      <div class="eh-acceptance-sync-section-title">
        CANDIDATE_EMPTY_SINGLE（最大100件表示）
      </div>
      <div class="eh-acceptance-sync-table-wrap">
        <table class="eh-acceptance-sync-table">
          <thead>
            <tr>
              <th>PROD選択</th><th>発注番号</th><th>現在</th><th>候補受入日</th><th>件数</th><th>理由</th>
            </tr>
          </thead>
          <tbody>
            ${candidateRows || '<tr><td colspan="6">候補なし</td></tr>'}
          </tbody>
        </table>
      </div>

      <div class="eh-acceptance-sync-note">
        NO_SOURCE_DATA_UNKNOWN は「未受入」を意味しません。Prones入力遅れ・未入力・履歴範囲外を含み得ます。
      </div>

      ${analysis.controlledProdMode ? `
        <div class="eh-acceptance-sync-testnote">
          <strong>App272 PROD Controlled Execute</strong><br>
          CANDIDATE_EMPTY_SINGLEから最大
          ${analysis.controlledProdMaxSelected}件だけ手動選択して実行します。
          全件一括実行は行いません。
          Execute直前に再Analysisし、選択POの状態が1件でも変わっていれば停止します。
          ${analysis.bulkWriteBlocked ? `<br>${escapeHtml(analysis.bulkWriteReason)}` : ''}
        </div>
      ` : `
        <div class="eh-acceptance-sync-testnote eh-acceptance-sync-prodlock">
          App292ではPROD候補版はAnalysis Onlyです。Executeは強制禁止です。
        </div>
      `}

      <div class="eh-acceptance-sync-actions">
        <span class="eh-acceptance-sync-selected-count">
          PROD選択: <strong>0</strong> / ${analysis.controlledProdMaxSelected || 0}
        </span>
        <button type="button" class="eh-acceptance-sync-csv">Analysis CSV</button>
        <button type="button" class="eh-acceptance-sync-cancel">閉じる</button>
        <button type="button" class="eh-acceptance-sync-execute" disabled>
          0件を受入れ日に反映
        </button>
      </div>
    `;

    overlay.appendChild(dialog);
    document.body.appendChild(overlay);

    activeAcceptanceSync = {
      overlay,
      dialog,
      analysis,
      executing: false
    };

    dialog.querySelector('.eh-acceptance-sync-close').addEventListener('click', closeAcceptanceSyncDialog);
    dialog.querySelector('.eh-acceptance-sync-cancel').addEventListener('click', closeAcceptanceSyncDialog);
    dialog.querySelector('.eh-acceptance-sync-csv').addEventListener('click', () => {
      downloadAcceptanceSyncCsv(analysis);
    });

    const execute = dialog.querySelector('.eh-acceptance-sync-execute');
    const selectedCount = dialog.querySelector('.eh-acceptance-sync-selected-count strong');
    const selectionBoxes = Array.from(dialog.querySelectorAll('.eh-acceptance-sync-select'));

    function updateControlledSelectionUi() {
      const checked = selectionBoxes.filter(x => x.checked);
      const maxSelected = Number(analysis.controlledProdMaxSelected || 0);

      if (checked.length > maxSelected) {
        const last = checked[checked.length - 1];
        last.checked = false;
        window.alert(`PRODでは1回最大${maxSelected}件までです。`);
      }

      const current = selectionBoxes.filter(x => x.checked);
      if (selectedCount) selectedCount.textContent = String(current.length);

      execute.textContent = `${current.length}件を受入れ日に反映`;
      execute.disabled =
        !analysis.controlledProdMode ||
        analysis.safetyStops.length > 0 ||
        current.length < 1 ||
        current.length > maxSelected;
    }

    selectionBoxes.forEach(box => {
      box.addEventListener('change', updateControlledSelectionUi);
    });
    updateControlledSelectionUi();

    execute.addEventListener('click', async () => {
      if (execute.disabled) return;
      const selectedPOs = selectionBoxes
        .filter(x => x.checked)
        .map(x => String(x.dataset.po || ''));
      await executeAcceptanceSync(selectedPOs);
    });

    overlay.addEventListener('click', event => {
      if (event.target === overlay) closeAcceptanceSyncDialog();
    });
  }

  async function executeAcceptanceSync(selectedPOs) {
    if (!activeAcceptanceSync || activeAcceptanceSync.executing) return;

    const originalAnalysis = activeAcceptanceSync.analysis;
    const allCandidates = originalAnalysis.candidates || [];
    const c = CONFIG.acceptanceSync;
    const tf = c.targetFields;
    const targetAppId = Number(kintone.app.getId());

    if (targetAppId !== Number(c.prodExecute.targetAppId)) {
      window.alert('SafetyStop: PROD ExecuteはApp272でのみ許可されています。');
      return;
    }

    if (originalAnalysis.safetyStops.length > 0) {
      window.alert('SafetyStop中のため書込みできません。');
      return;
    }

    const selectedSet = new Set((selectedPOs || []).map(String));
    const selectedOriginal = allCandidates.filter(r => selectedSet.has(String(r.po)));

    if (selectedOriginal.length === 0) {
      window.alert('PROD実行対象が選択されていません。');
      return;
    }

    const maxSelected = Math.min(
      Number(c.maxWrites),
      Number(c.prodExecute.maxSelected)
    );

    if (selectedOriginal.length > maxSelected) {
      window.alert(
        `SafetyStop: PROD選択 ${selectedOriginal.length}件 > 1回上限 ${maxSelected}件`
      );
      return;
    }

    if (selectedOriginal.some(r =>
      r.className !== 'CANDIDATE_EMPTY_SINGLE' ||
      r.futureAutoEligible !== 'YES'
    )) {
      window.alert('SafetyStop: 選択対象に自動更新対象外のClassが含まれています。');
      return;
    }

    const ok = window.confirm(
      `App272 PRODの「受入れ日」を ${selectedOriginal.length}件更新します。\n\n` +
      `対象: 手動選択した CANDIDATE_EMPTY_SINGLE のみ\n` +
      `既存の受入れ日は上書きしません。\n` +
      `複数受入・負数・要確認データは更新しません。\n` +
      `Execute直前に再Analysisし、状態が変わっていれば停止します。\n\n` +
      `続行しますか？`
    );
    if (!ok) return;

    const confirmationText =
      `${c.prodExecute.confirmationPrefix}:${selectedOriginal.length}`;
    const typed = window.prompt(
      `最終確認です。\n次を正確に入力してください。\n\n${confirmationText}`
    );
    if (typed !== confirmationText) {
      window.alert('SafetyStop: 最終確認文字列が一致しません。書込みは行いません。');
      return;
    }

    activeAcceptanceSync.executing = true;
    const buttons = activeAcceptanceSync.dialog.querySelectorAll('button');
    buttons.forEach(b => { b.disabled = true; });

    try {
      const freshAnalysis = await runAcceptanceSyncAnalysis();

      if (freshAnalysis.safetyStops.length > 0) {
        throw new Error(
          `PreExecute SafetyStop: ${freshAnalysis.safetyStops.join(' / ')}`
        );
      }

      const freshByPO = new Map(
        (freshAnalysis.rows || []).map(r => [String(r.po), r])
      );

      const stale = [];
      const candidates = [];

      selectedOriginal.forEach(oldRow => {
        const fresh = freshByPO.get(String(oldRow.po));

        if (!fresh) {
          stale.push(`${oldRow.po}: fresh row not found`);
          return;
        }

        const same =
          fresh.className === 'CANDIDATE_EMPTY_SINGLE' &&
          fresh.futureAutoEligible === 'YES' &&
          String(fresh.targetId) === String(oldRow.targetId) &&
          String(fresh.targetRevision) === String(oldRow.targetRevision) &&
          String(fresh.currentAcceptanceDate || '') === String(oldRow.currentAcceptanceDate || '') &&
          String(fresh.candidateReceiptDate) === String(oldRow.candidateReceiptDate) &&
          Number(fresh.sourceRowCount) === Number(oldRow.sourceRowCount) &&
          Number(fresh.negativeReceiptCount) === Number(oldRow.negativeReceiptCount);

        if (!same) {
          stale.push(`${oldRow.po}: Analysis後に状態変化`);
          return;
        }

        candidates.push(fresh);
      });

      if (stale.length > 0 || candidates.length !== selectedOriginal.length) {
        throw new Error(
          `PreExecute reAnalysis mismatch: ${stale.slice(0, 10).join(' / ')}`
        );
      }

      const records = candidates.map(r => ({
        id: r.targetId,
        revision: r.targetRevision,
        record: {
          [tf.acceptanceDate]: { value: r.candidateReceiptDate }
        }
      }));

      await kintone.api(
        kintone.api.url('/k/v1/records.json', true),
        'PUT',
        { app: targetAppId, records: records }
      );

      const idList = candidates.map(r => Number(r.targetId)).filter(Number.isFinite);
      const verifyResponse = await kintone.api(
        kintone.api.url('/k/v1/records.json', true),
        'GET',
        {
          app: targetAppId,
          query: `$id in (${idList.join(',')}) order by $id asc limit 100`,
          fields: [tf.recordId, tf.acceptanceDate]
        }
      );
      const verifyRecords = (verifyResponse && verifyResponse.records) || [];
      const actualById = new Map(
        verifyRecords.map(r => [syncValue(r, tf.recordId), syncValue(r, tf.acceptanceDate)])
      );

      const mismatches = candidates.filter(r =>
        actualById.get(String(r.targetId)) !== r.candidateReceiptDate
      );

      if (verifyRecords.length !== candidates.length || mismatches.length > 0) {
        throw new Error(
          `PostVerify failed: expected=${candidates.length}, fetched=${verifyRecords.length}, mismatch=${mismatches.length}`
        );
      }

      window.alert(
        `受入実績同期 PROD PASS\n\n` +
        `更新成功: ${candidates.length}件\n` +
        `PostVerify NG: 0件`
      );

      state.records = await fetchActiveRecords(state.listQueryCondition);
      const currentRoot = document.getElementById(CONFIG.rootId);
      if (currentRoot) render(currentRoot, false);

      activeAcceptanceSync.executing = false;
      closeAcceptanceSyncDialog();
    } catch (error) {
      console.error('[Heatmap][AcceptanceSync][PROD] Execute failed', error);
      activeAcceptanceSync.executing = false;
      buttons.forEach(b => { b.disabled = false; });
      window.alert(
        '受入実績同期 PRODに失敗しました。\n' +
        '自動再試行はしません。\n\n' +
        (error && error.message ? error.message : String(error))
      );
    }
  }

  function bindAcceptanceSyncEvents(root) {
    const button = root.querySelector('#eh-acceptance-sync-button');
    if (!button) return;

    button.addEventListener('click', async event => {
      event.preventDefault();
      event.stopPropagation();

      if (activeAcceptanceSync && activeAcceptanceSync.executing) return;
      if (activeEdit && activeEdit.saving) {
        window.alert('日程編集の保存処理中です。完了後に実行してください。');
        return;
      }
      if (activeReceipt && activeReceipt.saving) {
        window.alert('入荷実績の保存処理中です。完了後に実行してください。');
        return;
      }

      button.disabled = true;
      const original = button.textContent;
      button.textContent = '受入実績を解析中…';

      try {
        const analysis = await runAcceptanceSyncAnalysis();
        renderAcceptanceSyncDialog(analysis);
      } catch (error) {
        console.error('[Heatmap][AcceptanceSync] Analysis failed', error);
        window.alert(
          '受入実績同期のAnalysisに失敗しました。\n' +
          '書込みは行っていません。\n\n' +
          (error && error.message ? error.message : String(error))
        );
      } finally {
        button.disabled = false;
        button.textContent = original;
      }
    });
  }


  /* ------------------------------------------------------------------
   * Ver0.6.1.3: 外注入荷実績日 / 下段ガント完了表示 / 入荷取消
   * ------------------------------------------------------------------ */

  function renderReceiptControl(r) {
    if (r.receiptDate) {
      return `
        <div class="eh-receipt-control" data-receipt-control="${escapeAttr(r.id)}">
          <span class="eh-completed-badge">✓ 入荷済 ${escapeHtml(formatDateSlash(r.receiptDate))}</span>
          <button type="button" class="eh-receipt-button eh-receipt-edit-button" data-record-id="${escapeAttr(r.id)}">入荷日修正</button>
        </div>
      `;
    }

    return `
      <div class="eh-receipt-control" data-receipt-control="${escapeAttr(r.id)}">
        <button type="button" class="eh-receipt-button" data-record-id="${escapeAttr(r.id)}">入荷しました</button>
      </div>
    `;
  }

  /*
   * Ver0.6.1.2 修正:
   * position:sticky を基準にした position:absolute は、ブラウザによって
   * 「stickyの見た目上の位置」ではなく「stickyが無かった場合の静的な位置」
   * を基準に計算されることがあり、クリックした行と無関係な場所に
   * ポップアップが表示される不具合の原因になっていた。
   *
   * エディタは document.body 直下に置き、クリックされたボタンの実際の
   * 画面座標（getBoundingClientRect）をJavaScriptで計算してposition:fixedで
   * 直接指定する。同時に開けるエディタは1つだけとする。
   *
   * Ver0.6.1.3:
   * 入荷済み案件では「取り消し」を表示し、外注入荷実績日だけを空文字でPUTする。
   * 保存・取消中は body 直下のポップアップも activeReceipt 経由でdisableする。
   */
  // =====================================================================
  // Ver0.6.2.0-TEST4: K日程・担当者 編集ポップアップ + 番号クリック起動
  // =====================================================================
  let activeEdit = null; // { overlay, popup, recordId, revision, initial, saving, sourceButton }

  function bindScheduleEditEvents(root) {
    root.querySelectorAll('.eh-schedule-edit-trigger').forEach(trigger => {
      trigger.addEventListener('click', async function (event) {
        event.preventDefault();
        event.stopPropagation();

        if (activeEdit && activeEdit.saving) return;
        if (activeReceipt && activeReceipt.saving) return;

        const recordId = String(this.dataset.recordId || '');
        if (!recordId) return;

        try {
          await openScheduleEditPopup(root, recordId, this);
        } catch (error) {
          console.error('[Heatmap][Edit] open failed', error);
          window.alert(
            '編集用の最新データ取得に失敗しました。\n' +
            (error && error.message ? error.message : String(error))
          );
        }
      });
    });
  }

  async function fetchLatestRecordForEdit(recordId) {
    const response = await kintone.api(
      kintone.api.url('/k/v1/record.json', true),
      'GET',
      { app: kintone.app.getId(), id: recordId }
    );
    if (!response || !response.record) {
      throw new Error('対象レコードを取得できませんでした。');
    }
    return response.record;
  }

  function valueOfRawRecord(record, fieldCode) {
    return record && record[fieldCode] && record[fieldCode].value != null
      ? record[fieldCode].value
      : '';
  }

  async function fetchEditFormMetadata(currentValue) {
    const response = await kintone.api(
      kintone.api.url('/k/v1/app/form/fields.json', true),
      'GET',
      { app: kintone.app.getId() }
    );

    const properties = response && response.properties;

    // TEST4: 受入れ日は新設DATEフィールドとして厳密確認する。
    const acceptanceField = properties && properties[CONFIG.fields.acceptanceDate];
    if (!acceptanceField) {
      throw new Error(
        `フォーム設定に受入れ日（${CONFIG.fields.acceptanceDate}）が見つかりません。` +
        ' App292にDATEフィールドとして作成してからTESTしてください。'
      );
    }
    if (acceptanceField.type !== 'DATE') {
      throw new Error(
        `受入れ日（${CONFIG.fields.acceptanceDate}）の型がDATEではありません。` +
        ` 現在の型: ${acceptanceField.type || '不明'}`
      );
    }

    const field = properties && properties[CONFIG.fields.staff];
    if (!field) {
      throw new Error(
        `フォーム設定にSTAFF_DD（${CONFIG.fields.staff}）が見つかりません。`
      );
    }

    const optionMap = field.options;
    if (!optionMap || typeof optionMap !== 'object') {
      throw new Error(
        `STAFF_DD（${CONFIG.fields.staff}）の選択肢を取得できません。`
      );
    }

    const options = Object.keys(optionMap)
      .map(key => {
        const item = optionMap[key] || {};
        return {
          value: String(item.label != null ? item.label : key),
          index: Number(item.index)
        };
      })
      .filter(item => item.value.trim().length > 0)
      .sort((a, b) => {
        const ai = Number.isFinite(a.index) ? a.index : Number.MAX_SAFE_INTEGER;
        const bi = Number.isFinite(b.index) ? b.index : Number.MAX_SAFE_INTEGER;
        return ai !== bi
          ? ai - bi
          : a.value.localeCompare(b.value, 'ja');
      });

    if (options.length === 0) {
      throw new Error(
        `STAFF_DD（${CONFIG.fields.staff}）に選択肢が登録されていません。`
      );
    }

    // 現在値が設定候補から外れている場合でも、未変更保存で値を壊さないため
    // 表示専用の現在値として先頭へ残す。変更候補の正本はフォーム設定のみ。
    const current = String(currentValue || '').trim();
    const exists = current && options.some(item => item.value === current);
    if (current && !exists) {
      options.unshift({
        value: current,
        index: -1,
        legacyCurrent: true
      });
    }

    return {
      staffOptions: options,
      acceptanceFieldType: acceptanceField.type
    };
  }

  function normalizeProvisionalFlag(value) {
    const v = String(value == null ? '' : value).trim();
    if (v === 'ON' || v === 'OFF') return v;
    return v;
  }

  async function openScheduleEditPopup(root, recordId, sourceButton) {
    // 後から開いたポップアップを優先。保存中だけは切替禁止。
    if (activeReceipt) {
      if (activeReceipt.saving) return;
      closeActiveReceiptEditor();
    }
    if (activeEdit) {
      if (activeEdit.saving) return;
      closeActiveEditPopup();
    }

    const sourceOriginalText = sourceButton.textContent;
    const sourceIsButton =
      String(sourceButton.tagName || '').toUpperCase() === 'BUTTON';

    if (sourceIsButton) {
      sourceButton.disabled = true;
    } else {
      sourceButton.setAttribute('aria-disabled', 'true');
      sourceButton.classList.add('eh-schedule-edit-trigger-loading');
    }
    sourceButton.setAttribute('aria-busy', 'true');
    sourceButton.textContent = '取得中...';

    let raw;
    let editFormMetadata;
    try {
      raw = await fetchLatestRecordForEdit(recordId);

      const fForStaff = CONFIG.fields;
      const currentStaffForOptions = String(
        valueOfRawRecord(raw, fForStaff.staff) ||
        valueOfRawRecord(raw, fForStaff.staffFallback) ||
        ''
      ).trim();

      // Claude再レビュー R01/R02:
      // STAFF_DD候補はフォーム設定APIを正本にし、失敗時はポップアップを開かない。
      editFormMetadata = await fetchEditFormMetadata(
        currentStaffForOptions
      );
    } finally {
      if (document.body.contains(sourceButton)) {
        if (sourceIsButton) {
          sourceButton.disabled = false;
        } else {
          sourceButton.removeAttribute('aria-disabled');
          sourceButton.classList.remove('eh-schedule-edit-trigger-loading');
        }
        sourceButton.removeAttribute('aria-busy');
        sourceButton.textContent = sourceOriginalText;
      }
    }

    const staffOptions = editFormMetadata.staffOptions;

    const f = CONFIG.fields;
    const start = String(valueOfRawRecord(raw, f.startDate) || '').trim();
    const inspection = String(valueOfRawRecord(raw, f.inspectionDate) || '').trim();
    const provisional = normalizeProvisionalFlag(valueOfRawRecord(raw, f.provisionalFlag));
    const receipt = String(valueOfRawRecord(raw, f.receiptDate) || '').trim();
    const acceptance = String(valueOfRawRecord(raw, f.acceptanceDate) || '').trim();
    const due = String(valueOfRawRecord(raw, f.dueDate) || '').trim();
    const staff = String(
      valueOfRawRecord(raw, f.staff) || valueOfRawRecord(raw, f.staffFallback) || ''
    ).trim();
    const scheduleType = String(valueOfRawRecord(raw, f.scheduleType) || '').trim();
    const revision = String(valueOfRawRecord(raw, f.revision) || '').trim();
    const vendor = String(
      valueOfRawRecord(raw, f.vendor) || valueOfRawRecord(raw, f.vendorFallback) || ''
    ).trim();
    const displayKey = String(
      valueOfRawRecord(raw, f.key) ||
      valueOfRawRecord(raw, f.keyFallback) ||
      '手配書番号なし'
    ).trim() || '手配書番号なし';
    const orderSlipNo = String(valueOfRawRecord(raw, f.orderSlipNo) || '').trim();

    if (!revision) throw new Error('$revisionを取得できませんでした。');

    const staffOptionsHtml = [
      '<option value="">-- 未設定 --</option>',
      ...staffOptions.map(item => {
        const v = item.value;
        const label = item.legacyCurrent
          ? `${v}（現在値・候補外）`
          : v;
        return `<option value="${escapeAttr(v)}" ${v === staff ? 'selected' : ''}>${escapeHtml(label)}</option>`;
      })
    ].join('');

    const provisionalOptions = ['', 'ON', 'OFF'];
    if (provisional && !provisionalOptions.includes(provisional)) provisionalOptions.push(provisional);
    const provisionalHtml = provisionalOptions.map(v => {
      const label = v || '-- 未設定 --';
      return `<option value="${escapeAttr(v)}" ${v === provisional ? 'selected' : ''}>${escapeHtml(label)}</option>`;
    }).join('');

    const overlay = document.createElement('div');
    overlay.className = 'eh-edit-overlay';

    const popup = document.createElement('div');
    popup.className = 'eh-edit-popup';
    popup.dataset.recordId = recordId;
    popup.innerHTML = `
      <div class="eh-edit-popup-header">
        <div class="eh-edit-popup-title">
          <strong>${escapeHtml(vendor || '(外注先未設定)')}</strong>
          <span> / ${escapeHtml(displayKey)}</span>
        </div>
        <button type="button" class="eh-edit-close" aria-label="閉じる">×</button>
      </div>
      <div class="eh-edit-popup-body">
        <div class="eh-edit-readonly-card" aria-label="案件情報（表示のみ）">
          <div class="eh-edit-readonly-heading">案件情報（表示のみ）</div>
          <div class="eh-edit-readonly-row">
            <span class="eh-edit-readonly-label">手配書番号</span>
            <span class="eh-edit-readonly-value">${escapeHtml(displayKey)}</span>
          </div>
          <div class="eh-edit-readonly-row">
            <span class="eh-edit-readonly-label">注文書番号</span>
            <span class="eh-edit-readonly-value">${orderSlipNo ? escapeHtml(orderSlipNo) : '—'}</span>
          </div>
        </div>
        <label class="eh-edit-field">
          <span>K加工着手日</span>
          <input type="date" class="eh-edit-start" value="${escapeAttr(start)}">
        </label>
        <label class="eh-edit-field">
          <span>K完成検査日</span>
          <input type="date" class="eh-edit-inspection" value="${escapeAttr(inspection)}">
        </label>
        <label class="eh-edit-field">
          <span>K暫定設定</span>
          <select class="eh-edit-provisional">${provisionalHtml}</select>
        </label>
        <label class="eh-edit-field">
          <span>手配担当者名</span>
          <select class="eh-edit-staff">${staffOptionsHtml}</select>
        </label>
        <label class="eh-edit-field">
          <span>外注入荷日</span>
          <input type="date" class="eh-edit-receipt" value="${escapeAttr(receipt)}">
        </label>
        <div class="eh-edit-receipt-note">
          ${due
            ? `客先納期: ${escapeHtml(due.replace(/-/g, '/'))}`
            : '客先納期: 未設定'}
        </div>
        <div class="eh-edit-receipt-warning" aria-live="polite"></div>
        <label class="eh-edit-field">
          <span>受入れ日</span>
          <input type="date" class="eh-edit-acceptance" value="${escapeAttr(acceptance)}">
        </label>
        <div class="eh-edit-acceptance-note">
          ※ 外注入荷日とは別管理の受入れ実績日です。自動連動しません。
        </div>
        <div class="eh-edit-manual-note">※ K日程を変更すると、K日程種別は自動的に「Manual」になります。</div>
        <div class="eh-edit-error" aria-live="polite"></div>
      </div>
      <div class="eh-edit-popup-actions">
        <button type="button" class="eh-edit-standard">その他の項目を標準画面で編集</button>
        <div class="eh-edit-popup-actions-right">
          <button type="button" class="eh-edit-cancel">キャンセル</button>
          <button type="button" class="eh-edit-save">保存</button>
        </div>
      </div>
    `;

    overlay.appendChild(popup);
    document.body.appendChild(overlay);

    activeEdit = {
      overlay,
      popup,
      root,
      recordId,
      revision,
      initial: { start, inspection, provisional, staff, receipt, acceptance, due, scheduleType },
      saving: false,
      sourceButton
    };

    bindActiveEditPopupEvents();
    const firstInput = popup.querySelector('.eh-edit-start');
    if (firstInput) firstInput.focus();
  }

  function collectActiveEditValues() {
    if (!activeEdit || !activeEdit.popup) return null;
    const popup = activeEdit.popup;
    return {
      start: String((popup.querySelector('.eh-edit-start') || {}).value || '').trim(),
      inspection: String((popup.querySelector('.eh-edit-inspection') || {}).value || '').trim(),
      provisional: String((popup.querySelector('.eh-edit-provisional') || {}).value || '').trim(),
      staff: String((popup.querySelector('.eh-edit-staff') || {}).value || '').trim(),
      receipt: String((popup.querySelector('.eh-edit-receipt') || {}).value || '').trim(),
      acceptance: String((popup.querySelector('.eh-edit-acceptance') || {}).value || '').trim()
    };
  }

  function activeEditIsDirty() {
    if (!activeEdit) return false;
    const v = collectActiveEditValues();
    const i = activeEdit.initial;
    return !!v && (
      v.start !== i.start ||
      v.inspection !== i.inspection ||
      v.provisional !== i.provisional ||
      v.staff !== i.staff ||
      v.receipt !== i.receipt ||
      v.acceptance !== i.acceptance
    );
  }

  function validateActiveEdit(values) {
    if (values.start && values.inspection && values.start > values.inspection) {
      return 'K加工着手日はK完成検査日以前の日付にしてください。';
    }
    if (values.receipt) {
      if (!/^\d{4}-\d{2}-\d{2}$/.test(values.receipt)) {
        return '外注入荷日を YYYY-MM-DD 形式で指定してください。';
      }
      const parsedReceipt = parseYmd(values.receipt);
      if (!parsedReceipt || formatDate(parsedReceipt) !== values.receipt) {
        return '有効な外注入荷日を指定してください。';
      }
    }
    if (values.acceptance) {
      if (!/^\d{4}-\d{2}-\d{2}$/.test(values.acceptance)) {
        return '受入れ日を YYYY-MM-DD 形式で指定してください。';
      }
      const parsedAcceptance = parseYmd(values.acceptance);
      if (!parsedAcceptance || formatDate(parsedAcceptance) !== values.acceptance) {
        return '有効な受入れ日を指定してください。';
      }
    }
    return '';
  }

  function refreshActiveEditReceiptWarning() {
    if (!activeEdit || !activeEdit.popup) return;
    const popup = activeEdit.popup;
    const input = popup.querySelector('.eh-edit-receipt');
    const warning = popup.querySelector('.eh-edit-receipt-warning');
    if (!input || !warning) return;

    const value = String(input.value || '').trim();
    const due = activeEdit.initial ? String(activeEdit.initial.due || '').trim() : '';
    if (value && due && value > due) {
      warning.textContent =
        '客先納期を超えています。実際の入荷日であれば、そのまま保存できます。';
    } else {
      warning.textContent = '';
    }
  }

  function setActiveEditSaving(saving) {
    if (!activeEdit || !activeEdit.popup) return;
    activeEdit.saving = !!saving;
    activeEdit.popup.querySelectorAll('input, select, button').forEach(el => {
      el.disabled = !!saving;
    });
    const save = activeEdit.popup.querySelector('.eh-edit-save');
    if (save) save.textContent = saving ? '保存中...' : '保存';
  }

  function closeActiveEditPopup() {
    if (!activeEdit) return;
    if (activeEdit.saving) return;
    const sourceButton = activeEdit.sourceButton;
    if (activeEdit.overlay && activeEdit.overlay.parentNode) {
      activeEdit.overlay.parentNode.removeChild(activeEdit.overlay);
    }
    activeEdit = null;
    if (sourceButton && document.body.contains(sourceButton)) sourceButton.focus();
  }

  function recordEditUrl(id) {
    return `${location.origin}/k/${kintone.app.getId()}/show#record=${encodeURIComponent(id)}&mode=edit`;
  }

  function goToStandardEdit(recordId) {
    if (!recordId) return;
    if (activeEdit && activeEdit.saving) return;

    if (activeEditIsDirty()) {
      const ok = window.confirm(
        'ポップアップで入力中の未保存変更は保存されません。\n' +
        'kintone標準編集画面へ移動しますか？'
      );
      if (!ok) return;
    }

    // Claude再レビュー R03-R06:
    // 同一タブの標準編集から一覧へ戻った最初のindex.showで、
    // 一覧条件が同一でも1回だけ強制再GETさせる。
    // sessionStorage書込不能でも標準編集自体は妨げず、警告ログを残す。
    markForceRefreshAfterStandardEdit();
    location.href = recordEditUrl(recordId);
  }

  function bindActiveEditPopupEvents() {
    if (!activeEdit || !activeEdit.popup) return;
    const popup = activeEdit.popup;
    const overlay = activeEdit.overlay;
    const errorBox = popup.querySelector('.eh-edit-error');
    const cancel = popup.querySelector('.eh-edit-cancel');
    const close = popup.querySelector('.eh-edit-close');
    const save = popup.querySelector('.eh-edit-save');
    const standard = popup.querySelector('.eh-edit-standard');
    const receiptInput = popup.querySelector('.eh-edit-receipt');

    if (receiptInput) {
      receiptInput.addEventListener('change', refreshActiveEditReceiptWarning);
      refreshActiveEditReceiptWarning();
    }

    popup.addEventListener('click', event => event.stopPropagation());
    overlay.addEventListener('click', function (event) {
      if (event.target !== overlay) return;
      if (!activeEdit || activeEdit.saving) return;
      if (activeEditIsDirty() && !window.confirm('未保存の変更を破棄して閉じますか？')) return;
      closeActiveEditPopup();
    });

    const closeRequested = function (event) {
      event.preventDefault();
      event.stopPropagation();
      if (!activeEdit || activeEdit.saving) return;
      if (activeEditIsDirty() && !window.confirm('未保存の変更を破棄して閉じますか？')) return;
      closeActiveEditPopup();
    };
    if (cancel) cancel.addEventListener('click', closeRequested);
    if (close) close.addEventListener('click', closeRequested);

    if (standard) {
      standard.addEventListener('click', function (event) {
        event.preventDefault();
        event.stopPropagation();
        goToStandardEdit(activeEdit ? activeEdit.recordId : '');
      });
    }

    if (save) {
      save.addEventListener('click', async function (event) {
        event.preventDefault();
        event.stopPropagation();
        if (!activeEdit || activeEdit.saving) return;

        const values = collectActiveEditValues();
        const validation = validateActiveEdit(values);
        if (errorBox) errorBox.textContent = validation;
        if (validation) return;

        const i = activeEdit.initial;
        const dateChanged = values.start !== i.start || values.inspection !== i.inspection;
        const payloadRecord = {};

        if (values.start !== i.start) payloadRecord[CONFIG.fields.startDate] = { value: values.start };
        if (values.inspection !== i.inspection) payloadRecord[CONFIG.fields.inspectionDate] = { value: values.inspection };
        if (dateChanged) payloadRecord[CONFIG.fields.scheduleType] = { value: 'Manual' };
        if (values.provisional !== i.provisional) payloadRecord[CONFIG.fields.provisionalFlag] = { value: values.provisional };
        if (values.staff !== i.staff) {
          payloadRecord[CONFIG.fields.staffFallback] = { value: values.staff };
          payloadRecord[CONFIG.fields.staff] = { value: values.staff };
        }

        if (values.receipt !== i.receipt) {
          // 既存の入荷取消と同じ意味になるため、入荷済み→空欄は確認を入れる。
          if (i.receipt && !values.receipt) {
            const ok = window.confirm(
              `入荷実績を取り消して未入荷の状態に戻します。\n` +
              `現在の外注入荷日: ${i.receipt.replace(/-/g, '/')}\n\nよろしいですか？`
            );
            if (!ok) return;
          }
          payloadRecord[CONFIG.fields.receiptDate] = { value: values.receipt };
        }

        if (values.acceptance !== i.acceptance) {
          if (i.acceptance && !values.acceptance) {
            const ok = window.confirm(
              `受入れ日を空欄に戻します。\n` +
              `現在の受入れ日: ${i.acceptance.replace(/-/g, '/')}\n\nよろしいですか？`
            );
            if (!ok) return;
          }
          payloadRecord[CONFIG.fields.acceptanceDate] = { value: values.acceptance };
        }

        if (Object.keys(payloadRecord).length === 0) {
          closeActiveEditPopup();
          return;
        }

        setActiveEditSaving(true);
        try {
          const current = activeEdit;
          const response = await kintone.api(
            kintone.api.url('/k/v1/record.json', true),
            'PUT',
            {
              app: kintone.app.getId(),
              id: current.recordId,
              revision: current.revision,
              record: payloadRecord
            }
          );

          const local = state.records.find(r => String(r.id) === String(current.recordId));
          if (local) {
            if (values.start !== i.start) local.startDate = parseYmd(values.start);
            if (values.inspection !== i.inspection) local.inspectionDate = parseYmd(values.inspection);
            if (dateChanged) {
              local.scheduleType = 'Manual';
              local.isManual = true;
            }
            if (values.staff !== i.staff) local.staff = values.staff;
            if (values.receipt !== i.receipt) {
              local.receiptDate = values.receipt ? parseYmd(values.receipt) : null;
            }
            // 受入れ日は現行ガント描画には未使用だが、同一stateレコードへ保持して
            // 次回の局所参照と整合させる。描画ルールへの自動影響は持たせない。
            if (values.acceptance !== i.acceptance) {
              local.acceptanceDate = values.acceptance ? parseYmd(values.acceptance) : null;
            }
            if (response && response.revision != null) local.revision = String(response.revision);
          }

          // 日程変更は上段ヒートマップ・リスク判定にも影響するため、
          // 仮実装では安全優先で全体renderし、既存のスクロール復元を利用する。
          current.saving = false;
          if (current.overlay && current.overlay.parentNode) current.overlay.parentNode.removeChild(current.overlay);
          activeEdit = null;
          render(root, false);
        } catch (error) {
          console.error('[Heatmap][Edit] update failed', error);
          setActiveEditSaving(false);
          if (errorBox) {
            errorBox.textContent = isRevisionConflict(error)
              ? '他の変更と競合しました。最新の内容を再取得してやり直してください。'
              : '保存に失敗しました: ' + (error && error.message ? error.message : String(error));
          }
        }
      });
    }
  }

  if (!window.__ehEditEscapeBound) {
    window.__ehEditEscapeBound = true;
    document.addEventListener('keydown', function (event) {
      if (event.key !== 'Escape' || !activeEdit || activeEdit.saving) return;
      if (activeEditIsDirty() && !window.confirm('未保存の変更を破棄して閉じますか？')) return;
      closeActiveEditPopup();
    });
  }

  let activeReceipt = null; // { editor, row, recordId, saving }

  function closeActiveReceiptEditor() {
    if (activeReceipt && activeReceipt.editor && activeReceipt.editor.parentNode) {
      activeReceipt.editor.parentNode.removeChild(activeReceipt.editor);
    }
    activeReceipt = null;
  }

  function positionReceiptEditor(editor, anchorEl) {
    const rect = anchorEl.getBoundingClientRect();
    const margin = 4;
    const editorWidth = 220;

    let left = rect.left;
    if (left + editorWidth > window.innerWidth - 8) {
      left = Math.max(8, window.innerWidth - editorWidth - 8);
    }
    editor.style.left = `${Math.round(left)}px`;
    editor.style.top = `${Math.round(rect.bottom + margin)}px`;

    // 画面下端でエディタが切れる場合は、ボタンの上側へ表示し直す。
    requestAnimationFrame(function () {
      if (!document.body.contains(editor)) return;
      const editorRect = editor.getBoundingClientRect();
      if (editorRect.bottom > window.innerHeight - 8) {
        const flippedTop = rect.top - editorRect.height - margin;
        editor.style.top = `${Math.round(Math.max(8, flippedTop))}px`;
      }
    });
  }

  function ensureOutsideClickHandlerBound() {
    if (ensureOutsideClickHandlerBound.bound) return;
    ensureOutsideClickHandlerBound.bound = true;
    document.addEventListener('click', function (event) {
      if (!activeReceipt) return;
      if (activeReceipt.editor.contains(event.target)) return;
      // PUT中は外側クリックでDOMを消さない。完了/失敗ハンドラに閉じ処理を任せる。
      if (activeReceipt.saving) return;
      closeActiveReceiptEditor();
    });
  }

  function bindReceiptEvents(root) {
    ensureOutsideClickHandlerBound();

    root.querySelectorAll('.eh-receipt-button').forEach(button => {
      button.addEventListener('click', function (event) {
        event.preventDefault();
        event.stopPropagation();

        const recordId = String(this.dataset.recordId || '');
        const record = state.records.find(r => String(r.id) === recordId);
        if (!record) {
          window.alert('対象レコードを画面データから取得できませんでした。画面を再読込してください。');
          return;
        }

        const row = this.closest('.eh-gantt-row');
        if (!row) return;

        openReceiptEditor(row, record, this);
      });
    });
  }

  function openReceiptEditor(row, record, sourceButton) {
    // 日程編集ポップアップが開いていれば、保存中でない限り後から開く入荷操作を優先する。
    if (activeEdit) {
      if (activeEdit.saving) return;
      closeActiveEditPopup();
    }

    // 同時に開けるエディタは1つだけ。同じ案件なら既存を再利用する。
    if (activeReceipt) {
      if (activeReceipt.recordId === String(record.id)) {
        const input = activeReceipt.editor.querySelector('.eh-receipt-date-input');
        if (input) input.focus();
        return;
      }
      if (activeReceipt.saving) return;
      closeActiveReceiptEditor();
    }

    const control = row.querySelector(`[data-receipt-control="${cssEscape(record.id)}"]`);
    if (!control) return;

    const initialDate = record.receiptDate
      ? formatDate(record.receiptDate)
      : formatDate(startOfDay(new Date()));

    const editor = document.createElement('div');
    editor.className = 'eh-receipt-editor';
    editor.dataset.recordId = String(record.id);
    editor.innerHTML = `
      <label class="eh-receipt-editor-label">
        <span class="eh-receipt-editor-label-text">外注入荷日</span>
        <input type="date" class="eh-receipt-date-input" value="${escapeAttr(initialDate)}">
      </label>
      ${record.dueDate
        ? `<div class="eh-receipt-due">客先納期: ${escapeHtml(formatDateSlash(record.dueDate))}</div>`
        : '<div class="eh-receipt-due">客先納期: 未設定</div>'}
      <div class="eh-receipt-warning" aria-live="polite"></div>
      <div class="eh-receipt-editor-actions">
        <button type="button" class="eh-receipt-save">保存</button>
        ${record.receiptDate
          ? '<button type="button" class="eh-receipt-undo">取り消し</button>'
          : ''}
        <button type="button" class="eh-receipt-cancel">キャンセル</button>
      </div>
    `;

    document.body.appendChild(editor);
    positionReceiptEditor(editor, sourceButton);
    activeReceipt = {
      editor: editor,
      row: row,
      recordId: String(record.id),
      saving: false
    };

    const input = editor.querySelector('.eh-receipt-date-input');
    const saveButton = editor.querySelector('.eh-receipt-save');
    const undoButton = editor.querySelector('.eh-receipt-undo');
    const cancelButton = editor.querySelector('.eh-receipt-cancel');
    const warning = editor.querySelector('.eh-receipt-warning');

    const stop = function (event) {
      event.stopPropagation();
    };
    editor.addEventListener('click', stop);

    const refreshWarning = function () {
      if (!warning) return;
      const value = input ? input.value : '';
      if (value && record.dueDate && value > formatDate(record.dueDate)) {
        warning.textContent = '客先納期を超えています。実際の入荷日であれば、そのまま保存できます。';
      } else {
        warning.textContent = '';
      }
    };

    if (input) {
      input.addEventListener('click', stop);
      input.addEventListener('change', refreshWarning);
      refreshWarning();
      input.focus();
    }

    if (cancelButton) {
      cancelButton.addEventListener('click', function (event) {
        event.preventDefault();
        event.stopPropagation();
        if (activeReceipt && activeReceipt.saving) return;
        closeActiveReceiptEditor();
        if (sourceButton && document.body.contains(sourceButton)) sourceButton.focus();
      });
    }

    if (saveButton) {
      saveButton.addEventListener('click', async function (event) {
        event.preventDefault();
        event.stopPropagation();

        if (!activeReceipt || activeReceipt.saving) return;

        const value = input ? String(input.value || '').trim() : '';
        if (!/^\d{4}-\d{2}-\d{2}$/.test(value)) {
          window.alert('外注入荷日を YYYY-MM-DD 形式で指定してください。');
          if (input) input.focus();
          return;
        }

        const parsed = parseYmd(value);
        if (!parsed || formatDate(parsed) !== value) {
          window.alert('有効な外注入荷日を指定してください。');
          if (input) input.focus();
          return;
        }

        activeReceipt.saving = true;
        setReceiptRowSaving(row, true);
        saveButton.textContent = '保存中...';

        try {
          const response = await updateReceiptDate(record, value);
          record.receiptDate = parsed;
          updateLocalRevision(record, response);

          applyCompletedStateToRow(row, record);
          closeActiveReceiptEditor();
        } catch (error) {
          console.error('[Heatmap][Receipt] update failed', error);
          if (activeReceipt) activeReceipt.saving = false;
          setReceiptRowSaving(row, false);
          saveButton.textContent = '保存';

          if (isRevisionConflict(error)) {
            window.alert(
              'このレコードは他の操作で更新されています。上書きせず停止しました。\n' +
              '画面を再読込して最新状態を確認してください。'
            );
          } else {
            const detail = error && error.message ? error.message : String(error);
            window.alert('外注入荷日の保存に失敗しました。\n' + detail);
          }
        }
      });
    }

    if (undoButton) {
      undoButton.addEventListener('click', async function (event) {
        event.preventDefault();
        event.stopPropagation();

        if (!activeReceipt || activeReceipt.saving) return;
        if (!record.receiptDate) return;

        const currentDate = formatDateSlash(record.receiptDate);
        const ok = window.confirm(
          `入荷実績を取り消して未入荷の状態に戻します。\n` +
          `現在の外注入荷日: ${currentDate}\n\nよろしいですか？`
        );
        if (!ok) return;

        activeReceipt.saving = true;
        setReceiptRowSaving(row, true);
        undoButton.textContent = '取消中...';

        try {
          const response = await updateReceiptDate(record, '');
          record.receiptDate = null;
          updateLocalRevision(record, response);

          applyUncompletedStateToRow(row, record);
          closeActiveReceiptEditor();
        } catch (error) {
          console.error('[Heatmap][Receipt] undo failed', error);
          if (activeReceipt) activeReceipt.saving = false;
          setReceiptRowSaving(row, false);
          undoButton.textContent = '取り消し';

          if (isRevisionConflict(error)) {
            window.alert(
              'このレコードは他の操作で更新されています。取り消さず停止しました。\n' +
              '画面を再読込して最新状態を確認してください。'
            );
          } else {
            const detail = error && error.message ? error.message : String(error);
            window.alert('外注入荷実績の取り消しに失敗しました。\n' + detail);
          }
        }
      });
    }
  }

  async function updateReceiptDate(record, ymd) {
    const appId = kintone.app.getId();
    const revision = String(record.revision || '').trim();

    if (!record.id) {
      throw new Error('レコードIDが取得できません。');
    }
    if (!revision) {
      throw new Error('$revisionが取得できません。画面を再読込してください。');
    }

    const payload = {
      app: appId,
      id: record.id,
      revision: revision,
      record: {}
    };

    // 書込み対象は外注入荷実績日のみに限定する。
    // 取消時は ymd='' を送り、日付フィールドを空欄化する。
    payload.record[CONFIG.fields.receiptDate] = { value: ymd };

    return kintone.api(
      kintone.api.url('/k/v1/record.json', true),
      'PUT',
      payload
    );
  }

  function updateLocalRevision(record, response) {
    if (response && response.revision != null) {
      record.revision = String(response.revision);
    } else if (/^\d+$/.test(record.revision)) {
      record.revision = String(Number(record.revision) + 1);
    }
  }

  function applyCompletedStateToRow(row, record) {
    // 保存成功後のピンポイントDOM更新。render()全体は呼ばない。
    row.classList.add('eh-completed');

    const control = row.querySelector(`[data-receipt-control="${cssEscape(record.id)}"]`);
    if (!control) return;

    control.innerHTML = `
      <span class="eh-completed-badge">✓ 入荷済 ${escapeHtml(formatDateSlash(record.receiptDate))}</span>
      <button type="button" class="eh-receipt-button eh-receipt-edit-button" data-record-id="${escapeAttr(record.id)}">入荷日修正</button>
    `;

    bindReceiptButton(control, row, record);
    setReceiptRowSaving(row, false);
  }

  function applyUncompletedStateToRow(row, record) {
    // 取消成功後のピンポイントDOM更新。render()全体は呼ばない。
    row.classList.remove('eh-completed');

    const control = row.querySelector(`[data-receipt-control="${cssEscape(record.id)}"]`);
    if (!control) return;

    // renderReceiptControl() が返す外側divは差し込まず、中身だけを更新する。
    control.innerHTML = `
      <button type="button" class="eh-receipt-button" data-record-id="${escapeAttr(record.id)}">入荷しました</button>
    `;

    bindReceiptButton(control, row, record);
    setReceiptRowSaving(row, false);
  }

  function bindReceiptButton(control, row, record) {
    const button = control.querySelector('.eh-receipt-button');
    if (!button) return;

    button.addEventListener('click', function (event) {
      event.preventDefault();
      event.stopPropagation();
      openReceiptEditor(row, record, this);
    });
  }

  function setReceiptRowSaving(row, saving) {
    // 行内に残る入荷しました／入荷日修正ボタンをdisableする。
    row.querySelectorAll('.eh-receipt-button, .eh-receipt-edit-button')
      .forEach(el => {
        el.disabled = !!saving;
      });

    // 日付確認UIはdocument.body直下にあるため、activeReceipt経由でdisableする。
    if (activeReceipt && activeReceipt.row === row && activeReceipt.editor) {
      activeReceipt.editor
        .querySelectorAll('.eh-receipt-save, .eh-receipt-cancel, .eh-receipt-date-input, .eh-receipt-undo')
        .forEach(el => {
          el.disabled = !!saving;
        });
    }
  }

  function isRevisionConflict(error) {
    if (!error) return false;
    if (error.code === 'GAIA_CO02') return true;
    const text = `${error.code || ''} ${error.message || ''}`.toLowerCase();
    return text.includes('revision') || text.includes('リビジョン');
  }

  function cssEscape(value) {
    const s = String(value == null ? '' : value);
    if (window.CSS && typeof window.CSS.escape === 'function') {
      return window.CSS.escape(s);
    }
    return s.replace(/[^a-zA-Z0-9_-]/g, function (ch) {
      return '\\' + ch;
    });
  }

  function formatDateSlash(d) {
    if (!d) return '';
    const y = d.getFullYear();
    const m = String(d.getMonth() + 1).padStart(2, '0');
    const day = String(d.getDate()).padStart(2, '0');
    return `${y}/${m}/${day}`;
  }

  function renderTrackBands(viewStart, viewEnd, totalDays) {
    let html = '';
    for (let d = new Date(viewStart.getTime()); d <= viewEnd; d = addDays(d, 1)) {
      const dow = d.getDay();
      if (dow !== 0 && dow !== 6) continue;
      const left = diffDays(viewStart, d) / totalDays * 100;
      const width = 1 / totalDays * 100;
      html += `<span class="eh-track-weekend ${dow === 0 ? 'eh-track-sun' : 'eh-track-sat'}" style="left:${left}%;width:${width}%"></span>`;
    }
    const today = startOfDay(new Date());
    if (today >= viewStart && today <= viewEnd) {
      const left = diffDays(viewStart, today) / totalDays * 100;
      html += `<span class="eh-track-today" style="left:${left}%" title="今日"></span>`;
    }
    return html;
  }

  function clippedSpan(a, b, viewStart, viewEnd, totalDays) {
    if (!a || !b) return null;
    let start = a < viewStart ? viewStart : a;
    let end = b > viewEnd ? viewEnd : b;
    if (end < viewStart || start > viewEnd || end < start) return null;

    const left = diffDays(viewStart, start) / totalDays * 100;
    const width = Math.max(0.8, (diffDays(start, end) + 1) / totalDays * 100);
    return { left, width };
  }


  // Ver0.5.0: 標準プラグイン風に、日付セル幅を基準にpx相当でバー位置を決める。
  // 実際の幅はCSS変数 --eh-day-width に追従するため、表示サイズ変更でもずれない。
  function clippedCellSpan(a, b, viewStart, viewEnd) {
    if (!a || !b) return null;
    const start = a < viewStart ? viewStart : a;
    const end = b > viewEnd ? viewEnd : b;
    if (end < viewStart || start > viewEnd || end < start) return null;

    return {
      startIndex: Math.max(0, diffDays(viewStart, start)),
      dayCount: Math.max(1, diffDays(start, end) + 1)
    };
  }

  function cellSpanStyle(span, insetPx) {
    if (!span) return '';
    const inset = Number(insetPx) || 0;
    return `left:calc(${span.startIndex} * var(--eh-day-width) + ${inset}px);` +
      `width:calc(${span.dayCount} * var(--eh-day-width) - ${inset * 2}px)`;
  }

  function cellMarkerStyle(d, viewStart, viewEnd) {
    if (!d || d < viewStart || d > viewEnd) return '';
    const index = diffDays(viewStart, d);
    return `left:calc(${index} * var(--eh-day-width) + (var(--eh-day-width) / 2))`;
  }

  function makePluginMarker(d, viewStart, viewEnd, type, alert, label) {
    if (!d || d < viewStart || d > viewEnd) return '';
    const cls = `eh-plugin-marker eh-plugin-marker-${type}${alert ? ' eh-alert-symbol' : ''}`;
    return `<span class="${cls}" style="${cellMarkerStyle(d, viewStart, viewEnd)}" title="${escapeAttr(label + ': ' + formatDate(d))}"></span>`;
  }

  function makeSymbolMarker(d, viewStart, viewEnd, totalDays, symbol, cls, label) {
    if (!d) return '';
    // r.orderDate / r.startDate / r.inspectionDate / r.dueDate は既に Date オブジェクト。
    // 文字列のときだけ parseYmd() する。
    const date = d instanceof Date ? d : parseYmd(d);
    if (!date || Number.isNaN(date.getTime()) || date < viewStart || date > viewEnd) return '';
    const left = diffDays(viewStart, date) / totalDays * 100;
    const shapeClass =
      symbol === '○' ? 'eh-shape-order' :
      symbol === '▶' ? 'eh-shape-start' :
      symbol === '●' ? 'eh-shape-inspection' :
      'eh-shape-due';
    return `<span class="eh-symbol-marker ${shapeClass} ${cls}" style="left:${left}%" title="${escapeHtml(label)}"></span>`;
  }

  function makeMarker(d, viewStart, viewEnd, totalDays, cls, label) {
    if (!d || d < viewStart || d > viewEnd) return '';
    const left = diffDays(viewStart, d) / totalDays * 100;
    return `<span class="eh-marker ${cls}" style="left:${left}%" title="${label}: ${formatDate(d)}"></span>`;
  }

  function heatLevel(count) {
    if (!count) return 'zero';
    if (count <= CONFIG.heatLevels.lowMax) return 'low';
    if (count <= CONFIG.heatLevels.mediumMax) return 'medium';
    return 'high';
  }

  function severityRank(s) {
    return s === 'danger' ? 2 : (s === 'warning' ? 1 : 0);
  }

  function fieldValue(record, code) {
    return record && record[code] ? record[code].value : null;
  }

  function parseYmd(s) {
    if (!s) return null;
    const m = /^(\d{4})-(\d{2})-(\d{2})$/.exec(s);
    if (!m) return null;
    return new Date(Number(m[1]), Number(m[2]) - 1, Number(m[3]));
  }

  function startOfDay(d) {
    return new Date(d.getFullYear(), d.getMonth(), d.getDate());
  }

  function addDays(d, n) {
    const x = new Date(d.getTime());
    x.setDate(x.getDate() + n);
    return x;
  }

  function diffDays(a, b) {
    return Math.round((startOfDay(b) - startOfDay(a)) / 86400000);
  }

  function makeDateRange(a, b) {
    const arr = [];
    for (let d = new Date(a.getTime()); d <= b; d = addDays(d, 1)) arr.push(d);
    return arr;
  }

  function compareDate(a, b) {
    if (!a && !b) return 0;
    if (!a) return 1;
    if (!b) return -1;
    return a - b;
  }

  function formatDate(d) {
    if (!d) return '';
    const y = d.getFullYear();
    const m = String(d.getMonth() + 1).padStart(2, '0');
    const day = String(d.getDate()).padStart(2, '0');
    return `${y}-${m}-${day}`;
  }

  function fmt(d) {
    return d ? `${d.getMonth()+1}/${d.getDate()}` : '-';
  }

  function recordUrl(id) {
    return `${location.origin}/k/${kintone.app.getId()}/show#record=${encodeURIComponent(id)}`;
  }

  function escapeHtml(s) {
    return String(s == null ? '' : s)
      .replace(/&/g, '&amp;')
      .replace(/</g, '&lt;')
      .replace(/>/g, '&gt;')
      .replace(/"/g, '&quot;')
      .replace(/'/g, '&#39;');
  }

  function escapeAttr(s) {
    return escapeHtml(s);
  }
})();