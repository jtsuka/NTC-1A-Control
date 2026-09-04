/*
 * 外注課 外注先負荷ヒートマップ / 日程リスク可視化 Ver0.4.4
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
 */
(function () {
  'use strict';

  const CONFIG = {
    targetViewName: '外注負荷ヒートマップ',
    rootId: 'vendor-heatmap-root',

    fields: {
      recordId: '$id',
      key: 'ルックアップ',
      vendor: 'VENDOR_DD',
      vendorFallback: 'VENDOR_TEXT',
      staff: 'STAFF_DD',
      staffFallback: 'STAFF_TEXT',
      product: '商品名',
      orderDate: '日付',
      startDate: 'K加工着手日',
      inspectionDate: 'K完成検査日',
      dueDate: '日付_1'
    },

    initialDays: 60,
    rangeOptions: [30, 60, 90],
    minInspectionToDueDays: 3,

    // 暫定日程設定ボタンで現在使用している標準日程ルール
    // 将来はこの値をアプリ設定/共通設定から読める形に変更予定。
    scheduleRule: {
      inspectionDaysBeforeDue: 3,
      startDaysBeforeInspection: 5,
      adjustWeekendHoliday: true
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
    selectedVendor: null
  };

  kintone.events.on('app.record.index.show', async function (event) {
    if (event.viewName !== CONFIG.targetViewName) return event;

    const root = document.getElementById(CONFIG.rootId);
    if (!root) {
      console.error('[Heatmap] root element not found:', CONFIG.rootId);
      return event;
    }

    if (root.dataset.initialized === '1') return event;
    root.dataset.initialized = '1';

    root.innerHTML = '<div class="eh-loading">外注先負荷データを取得しています...</div>';

    try {
      state.records = await fetchActiveRecords();
      state.selectedVendor = chooseInitialVendor(state.records);
      render(root);
    } catch (err) {
      console.error('[Heatmap] error', err);
      root.innerHTML =
        '<div class="eh-error"><strong>データ取得に失敗しました。</strong><br>' +
        escapeHtml(err && err.message ? err.message : String(err)) +
        '</div>';
    }

    return event;
  });

  async function fetchActiveRecords() {
    const f = CONFIG.fields;
    const today = formatDate(new Date());
    const appId = kintone.app.getId();

    // 既存の update_master_button.js と同じ、
    // $id を使った 500件ずつの取得方式に統一する。
    // 納期が今日以降の案件を対象とする。
    // 完了条件が確定したら、ここへ完了除外条件を追加する。
    let allRecords = [];
    let lastId = 0;

    while (true) {
      const query =
        `$id > ${lastId} and ${f.dueDate} >= "${today}" ` +
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
      vendor: fieldValue(r, f.vendor) || fieldValue(r, f.vendorFallback) || '(外注先未設定)',
      staff: fieldValue(r, f.staff) || fieldValue(r, f.staffFallback) || '',
      product: fieldValue(r, f.product) || '',
      orderDate: parseYmd(fieldValue(r, f.orderDate)),
      startDate: parseYmd(fieldValue(r, f.startDate)),
      inspectionDate: parseYmd(fieldValue(r, f.inspectionDate)),
      dueDate: parseYmd(fieldValue(r, f.dueDate))
    };
  }

  function render(root) {
    const today = startOfDay(new Date());
    const end = addDays(today, state.rangeDays - 1);
    const dates = makeDateRange(today, end);

    const vendorStats = buildVendorStats(state.records, dates);
    if (!state.selectedVendor && vendorStats.length) {
      state.selectedVendor = vendorStats[0].vendor;
    }

    root.innerHTML = `
      <div class="eh-wrap">
        ${renderHeader(vendorStats)}
        ${renderSummary(vendorStats)}
        ${renderHeatmap(vendorStats, dates)}
        ${renderRiskSummary(state.records)}
        ${renderVendorDetail(state.records, state.selectedVendor, today, end)}
      </div>
    `;

    bindEvents(root);
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
          <div class="eh-top-alert"><span class="eh-legend-alert">赤＝日付矛盾・要注意</span></div>
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
          <span class="eh-count">対象 ${state.records.length}件 / 外注先 ${vendorStats.length}社</span>
        </div>
      </div>
    `;
  }

  function renderScheduleRule() {
    const rule = CONFIG.scheduleRule;
    return `
      <div class="eh-rule-box">
        <span class="eh-rule-title">現在の暫定日程ルール</span>
        <span class="eh-rule-item">検収日：納期の<strong>${rule.inspectionDaysBeforeDue}日前</strong></span>
        <span class="eh-rule-item">仕掛日：検収日の<strong>${rule.startDaysBeforeInspection}日前</strong></span>
        <span class="eh-rule-note">${rule.adjustWeekendHoliday ? '※ 土日・祝日は調整' : '※ 暦日で計算'}</span>
      </div>
    `;
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
          <td><a class="eh-record-link" href="${recordUrl(r.id)}">${escapeHtml(r.key || ('#' + r.id))}</a></td>
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
      const leftWhisker = clippedSpan(r.orderDate, r.startDate, viewStart, viewEnd, totalDays);
      const workbar = clippedSpan(r.startDate, r.inspectionDate, viewStart, viewEnd, totalDays);
      const rightWhisker = clippedSpan(r.inspectionDate, r.dueDate, viewStart, viewEnd, totalDays);
      const risk = analyzeRisk(r);

      const leftBad = !!(r.orderDate && r.startDate && r.startDate < r.orderDate);
      const workBad = !!(r.startDate && r.inspectionDate && r.inspectionDate < r.startDate);
      const rightBad = !!(r.inspectionDate && r.dueDate && r.inspectionDate > r.dueDate);

      return `
        <div class="eh-gantt-row">
          <div class="eh-gantt-info">
            <a href="${recordUrl(r.id)}" class="eh-record-link">${escapeHtml(r.key || ('#' + r.id))}</a>
            <div class="eh-product" title="${escapeAttr(r.product)}">${escapeHtml(r.product)}</div>
            <div class="eh-dates">${fmt(r.orderDate)} → ${fmt(r.startDate)} → ${fmt(r.inspectionDate)} → ${fmt(r.dueDate)}</div>
            ${risk.severity !== 'ok'
              ? `<div class="eh-inline-risk eh-${risk.severity}-text">⚠ ${escapeHtml(risk.messages.join(' / '))}</div>`
              : ''}
          </div>
          <div class="eh-track">
            ${renderTrackBands(viewStart, viewEnd, totalDays)}
            ${leftWhisker ? `<div class="eh-whisker eh-whisker-left${leftBad ? ' eh-alert-segment' : ''}" style="left:${leftWhisker.left}%;width:${leftWhisker.width}%"></div>` : ''}
            ${workbar ? `<div class="eh-workbar eh-work-${risk.severity}${workBad ? ' eh-alert-segment' : ''}" style="left:${workbar.left}%;width:${workbar.width}%"><span class="eh-workbar-label">${escapeHtml(r.product || '')}</span></div>` : ''}
            ${rightWhisker ? `<div class="eh-whisker eh-whisker-right${rightBad ? ' eh-alert-segment' : ''}" style="left:${rightWhisker.left}%;width:${rightWhisker.width}%"></div>` : ''}
            ${makeSymbolMarker(r.orderDate, viewStart, viewEnd, totalDays, '○', `eh-order-symbol${leftBad ? ' eh-alert-symbol' : ''}`, '手配日')}
            ${makeSymbolMarker(r.startDate, viewStart, viewEnd, totalDays, '▶', `eh-start-symbol${leftBad || workBad ? ' eh-alert-symbol' : ''}`, '仕掛開始')}
            ${makeSymbolMarker(r.inspectionDate, viewStart, viewEnd, totalDays, '●', `eh-inspection-symbol${workBad || rightBad ? ' eh-alert-symbol' : ''}`, '検収日')}
            ${makeSymbolMarker(r.dueDate, viewStart, viewEnd, totalDays, '◆', `eh-due-symbol${rightBad ? ' eh-alert-symbol' : ''}`, '納期')}
          </div>
        </div>
      `;
    }).join('');

    return `
      <section class="eh-section">
        <div class="eh-section-title">
          <strong>${escapeHtml(vendor)}：案件詳細</strong>
          <span>外注先行をクリックするとここが切り替わります。下段は1日1マスの予定表表示です。</span>
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
          <span class="eh-legend-alert">赤＝日付矛盾・要注意</span>
        </div>
        <div class="eh-schedule-note">
          1日1マスの予定表表示です。横スクロールして日程を確認できます。
        </div>
        <div class="eh-schedule-scroll" style="--eh-total-days:${totalDays};">
          <div class="eh-schedule-head-row">
            <div class="eh-schedule-left-head">案件 / 品名 / 日程</div>
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

  function bindEvents(root) {
    const range = root.querySelector('#eh-range');
    if (range) {
      range.addEventListener('change', function () {
        state.rangeDays = Number(this.value) || CONFIG.initialDays;
        render(root);
      });
    }

    root.querySelectorAll('.eh-vendor-row').forEach(row => {
      row.addEventListener('click', function () {
        state.selectedVendor = this.dataset.vendor;
        render(root);
        const detail = root.querySelector('.eh-gantt-list');
        if (detail) detail.scrollIntoView({ behavior: 'smooth', block: 'start' });
      });
    });
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
