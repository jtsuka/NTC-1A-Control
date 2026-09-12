/*
 * 外注課 外注先負荷ヒートマップ / 日程リスク可視化 Ver0.6.1.2
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
 */
(function () {
  'use strict';

  const CONFIG = {
    targetViewName: '外注負荷ヒートマップ',
    rootId: 'vendor-heatmap-root',

    fields: {
      recordId: '$id',
      key: 'K手配書番号',
      keyFallback: 'ルックアップ',
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
      receiptDate: '外注入荷実績日',
      revision: '$revision'
    },

    initialDays: 60,
    rangeOptions: [30, 60, 90],
    pastDays: 90,
    minInspectionToDueDays: 3,

    // 暫定日程設定ボタンで現在使用している標準日程ルール
    // 将来はこの値をアプリ設定/共通設定から読める形に変更予定。
    scheduleRule: {
      inspectionDaysBeforeDue: 3,
      startDaysBeforeInspection: 5,
      adjustWeekendHoliday: true
    },

    // 暫定日程トグルJSと共有する設定。
    scheduleRuleStorageKey: 'schedulePreview.offsetDays',
    scheduleRuleInputIds: {
      inspection: 'schedule_preview_inspection_days',
      start: 'schedule_preview_start_days'
    },

    // 「危険」の業務基準は未確定なので、Ver0.1では単純な件数色分けのみ。
    heatLevels: {
      lowMax: 2,
      mediumMax: 4
      // 5件以上は high 表示
    }
  };

  const state = {
    records: [],
    rangeDays: CONFIG.initialDays,
    selectedVendor: null,
    detailSize: loadDetailSize(),
    listQueryCondition: '',
    listFilterApplied: false
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

    if (
      root.dataset.initialized === '1' &&
      previousListCondition === currentListCondition
    ) {
      return event;
    }

    root.dataset.initialized = '1';
    root.dataset.listQueryCondition = currentListCondition;

    root.innerHTML = '<div class="eh-loading">外注先負荷データを取得しています...</div>';

    try {
      state.records = await fetchActiveRecords(currentListCondition);
      state.selectedVendor = chooseInitialVendor(state.records);
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
      displayKey:
        fieldValue(r, f.key) ||
        fieldValue(r, f.keyFallback) ||
        ('#' + fieldValue(r, f.recordId)),
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

    const vendorStats = buildVendorStats(state.records, dates);
    if (!state.selectedVendor && vendorStats.length) {
      state.selectedVendor = vendorStats[0].vendor;
    }

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
        ${renderHeader(vendorStats)}
        ${renderSummary(vendorStats)}
        ${renderHeatmap(vendorStats, dates)}
        ${renderRiskSummary(state.records)}
        ${renderVendorDetail(state.records, state.selectedVendor, viewStart, end)}
      </div>
    `;

    bindEvents(root);

    if (jumpToToday) {
      jumpToTodayAfterRender(root, today, myGeneration);
    } else {
      restoreScrollPositions(root, savedScroll, myGeneration);
    }
  }

  function renderHeader(vendorStats) {
    return `
      <div class="eh-header">
        <div>
          <h2>外注先 負荷ヒートマップ / 日程リスク</h2>
          <div class="eh-subtitle">
            注文日→加工着手日、完成検査日→納期を細いヒゲ、加工期間を太線で表示します。
            日付の修正はkintone標準レコード画面で行います。
          </div>
          <div class="eh-top-alert"><div class="eh-gantt-status-legends"><span class="eh-legend-alert">赤＝日付矛盾・要注意</span><span class="eh-legend-manual"><span class="eh-manual-badge eh-manual-badge-legend">M</span>＝手動調整</span></div></div>
          ${renderScheduleRule()}
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
          <span class="eh-count">
            対象 ${state.records.length}件 / 外注先 ${vendorStats.length}社
            ${state.listFilterApplied
              ? '<span class="eh-list-filter-status eh-list-filter-status-on">一覧絞込：適用中</span>'
              : '<span class="eh-list-filter-status">一覧絞込：なし</span>'}
          </span>
        </div>
      </div>
    `;
  }

  function renderScheduleRule() {
    const rule = getCurrentScheduleRule();
    return `
      <div class="eh-rule-box">
        <span class="eh-rule-title">現在の暫定日程ルール</span>
        <span class="eh-rule-item">検収日：納期の<strong id="eh-rule-inspection-days">${rule.inspectionDaysBeforeDue}日前</strong></span>
        <span class="eh-rule-item">仕掛日：検収日の<strong id="eh-rule-start-days">${rule.startDaysBeforeInspection}日前</strong></span>
        <span class="eh-rule-note">${rule.adjustWeekendHoliday ? '※ 土日・祝日は調整' : '※ 暦日で計算'}</span>
      </div>
    `;
  }

  function getCurrentScheduleRule() {
    let inspection = null;
    let start = null;

    // 同一画面にトグル入力欄があれば、現在の入力値を最優先。
    const inspectionInput = document.getElementById(CONFIG.scheduleRuleInputIds.inspection);
    const startInput = document.getElementById(CONFIG.scheduleRuleInputIds.start);

    if (inspectionInput && startInput) {
      const i = Number(inspectionInput.value);
      const s = Number(startInput.value);
      if (Number.isInteger(i) && i >= 0) inspection = i;
      if (Number.isInteger(s) && s >= 0) start = s;
    }

    // 入力欄が無い、または値が不正ならトグルJSが保存したlocalStorageを読む。
    if (inspection === null || start === null) {
      try {
        const raw = localStorage.getItem(CONFIG.scheduleRuleStorageKey);
        if (raw) {
          const saved = JSON.parse(raw);
          const i = Number(saved.inspectionDaysBeforeDue);
          const s = Number(saved.startDaysBeforeInspection);
          if (inspection === null && Number.isInteger(i) && i >= 0) inspection = i;
          if (start === null && Number.isInteger(s) && s >= 0) start = s;
        }
      } catch (e) {
        // 保存値が読めなくても既定値で表示を継続。
      }
    }

    return {
      inspectionDaysBeforeDue:
        inspection !== null ? inspection : CONFIG.scheduleRule.inspectionDaysBeforeDue,
      startDaysBeforeInspection:
        start !== null ? start : CONFIG.scheduleRule.startDaysBeforeInspection,
      adjustWeekendHoliday: CONFIG.scheduleRule.adjustWeekendHoliday
    };
  }

  function refreshScheduleRuleDisplay(root) {
    const rule = getCurrentScheduleRule();
    const inspection = root.querySelector('#eh-rule-inspection-days');
    const start = root.querySelector('#eh-rule-start-days');

    if (inspection) inspection.textContent = `${rule.inspectionDaysBeforeDue}日前`;
    if (start) start.textContent = `${rule.startDaysBeforeInspection}日前`;
  }

  function bindScheduleRuleInputs(root) {
    const ids = [
      CONFIG.scheduleRuleInputIds.inspection,
      CONFIG.scheduleRuleInputIds.start
    ];

    ids.forEach(function (id) {
      const input = document.getElementById(id);
      if (!input || input.dataset.ehRuleBound === '1') return;

      input.dataset.ehRuleBound = '1';
      input.addEventListener('input', function () {
        // トグルJS側がchange時に保存する前でも、表示だけは即追従。
        refreshScheduleRuleDisplay(root);
      });
      input.addEventListener('change', function () {
        refreshScheduleRuleDisplay(root);
      });
    });
  }

  function renderSummary(vendorStats) {
    const issues = state.records.filter(r => analyzeRisk(r).severity === 'danger').length;
    const cautions = state.records.filter(r => analyzeRisk(r).severity === 'warning').length;
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
    const dateHeader = dates.map(d => {
      const dow = d.getDay();
      const cls = dow === 0 ? 'eh-sun' : (dow === 6 ? 'eh-sat' : '');
      return `<th class="eh-date ${cls}" title="${formatDate(d)}">${d.getMonth()+1}/${d.getDate()}</th>`;
    }).join('');

    const rows = vendorStats.map(v => {
      const selected = v.vendor === state.selectedVendor ? ' eh-selected' : '';
      const cells = v.counts.map((count, i) => {
        const level = heatLevel(count);
        const title = `${v.vendor} / ${formatDate(dates[i])} / 同時案件 ${count}件`;
        const dow = dates[i].getDay();
        const weekendClass = dow === 0 ? ' eh-col-sun' : (dow === 6 ? ' eh-col-sat' : '');
        return `<td class="eh-heat eh-${level}${weekendClass}" title="${escapeHtml(title)}">${count || ''}</td>`;
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
          <td><a class="eh-record-link" href="${recordUrl(r.id)}">${escapeHtml(r.displayKey)}</a></td>
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
                <th>判定</th><th>発注番号</th><th>外注先</th><th>担当</th><th>理由</th>
                <th>注文日</th><th>加工着手</th><th>完成検査</th><th>納期</th>
              </tr>
            </thead>
            <tbody>${rows || '<tr><td colspan="9" class="eh-ok-message">現在、日付矛盾・要注意案件はありません。</td></tr>'}</tbody>
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
              <a href="${recordUrl(r.id)}" class="eh-record-link" title="${escapeAttr(displayKey)}">${escapeHtml(displayKey)}</a>
            </div>
            <div class="eh-dates" title="手配日 → 仕掛開始 → 検収日 → 納期">${fmt(r.orderDate)} → ${fmt(r.startDate)} → ${fmt(r.inspectionDate)} → ${fmt(r.dueDate)}</div>
            ${risk.severity !== 'ok'
              ? `<div class="eh-inline-risk eh-${risk.severity}-text" title="${escapeAttr(risk.messages.join(' / '))}">⚠ ${escapeHtml(risk.messages.join(' / '))}</div>`
              : ''}
            ${renderReceiptControl(r)}
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
          左側に手配書番号・指示先名・手配担当者を固定表示し、右側を1日1マスの予定表として表示します。M表示・太枠・背景色変更は手動調整（K日程種別=Manual）を示します。
        </div>
        <div class="eh-schedule-scroll eh-size-${state.detailSize}" style="--eh-total-days:${totalDays};">
          <div class="eh-schedule-head-row">
            <div class="eh-schedule-col-head eh-head-order">手配書番号</div>
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

      const heatmap = root.querySelector('.eh-heatmap');
      const upper = heatmap ? heatmap.closest('.eh-table-scroll') : null;
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
    const upper = root.querySelector('.eh-table-scroll');
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

      const upper = root.querySelector('.eh-table-scroll');
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

  function bindEvents(root) {
    bindScheduleRuleInputs(root);
    bindReceiptEvents(root);

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

  /* ------------------------------------------------------------------
   * Ver0.6.1.2: 外注入荷実績日 / 下段ガント完了表示
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
   * そのため、エディタは document.body 直下に置き、クリックされた
   * ボタンの実際の画面座標（getBoundingClientRect）をJavaScriptで
   * 計算してposition:fixedで直接指定する方式へ変更する。
   * 同時に開けるエディタは1つだけとし、外側クリックでも閉じられるようにする。
   */
  let activeReceipt = null; // { editor, row, recordId }

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
    // 同時に開けるエディタは1つだけ。同じ案件なら既存を再利用する。
    if (activeReceipt) {
      if (activeReceipt.recordId === String(record.id)) {
        const input = activeReceipt.editor.querySelector('.eh-receipt-date-input');
        if (input) input.focus();
        return;
      }
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
        <button type="button" class="eh-receipt-cancel">キャンセル</button>
      </div>
    `;

    document.body.appendChild(editor);
    positionReceiptEditor(editor, sourceButton);
    activeReceipt = { editor: editor, row: row, recordId: String(record.id) };

    const input = editor.querySelector('.eh-receipt-date-input');
    const saveButton = editor.querySelector('.eh-receipt-save');
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
        closeActiveReceiptEditor();
        if (sourceButton && document.body.contains(sourceButton)) sourceButton.focus();
      });
    }

    if (saveButton) {
      saveButton.addEventListener('click', async function (event) {
        event.preventDefault();
        event.stopPropagation();

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

        // 行単位の二重送信防止。他行のボタンには触れない。
        setReceiptRowSaving(row, true);
        saveButton.textContent = '保存中...';

        try {
          const response = await updateReceiptDate(record, value);
          record.receiptDate = parsed;
          if (response && response.revision != null) {
            record.revision = String(response.revision);
          } else if (/^\d+$/.test(record.revision)) {
            record.revision = String(Number(record.revision) + 1);
          }

          applyCompletedStateToRow(row, record);
          closeActiveReceiptEditor();
        } catch (error) {
          console.error('[Heatmap][Receipt] update failed', error);
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
    payload.record[CONFIG.fields.receiptDate] = { value: ymd };

    return kintone.api(
      kintone.api.url('/k/v1/record.json', true),
      'PUT',
      payload
    );
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

    const button = control.querySelector('.eh-receipt-button');
    if (button) {
      button.addEventListener('click', function (event) {
        event.preventDefault();
        event.stopPropagation();
        openReceiptEditor(row, record, this);
      });
    }

    setReceiptRowSaving(row, false);
  }

  function setReceiptRowSaving(row, saving) {
    // クリックされた案件行の入荷関連コントロールだけをdisableする。
    row.querySelectorAll('.eh-receipt-button, .eh-receipt-save, .eh-receipt-cancel, .eh-receipt-date-input')
      .forEach(el => {
        el.disabled = !!saving;
      });
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