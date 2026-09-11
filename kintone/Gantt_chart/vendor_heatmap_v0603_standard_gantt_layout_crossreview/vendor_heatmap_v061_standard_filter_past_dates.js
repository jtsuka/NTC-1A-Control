/*
 * 外注課 外注先負荷ヒートマップ / 日程リスク可視化 Ver0.6.1
 * 対象: kintone App 272
 *
 * 2026-09-11 修正版
 *  - kintone標準一覧の絞り込み条件を業務母集団の唯一の条件として使用
 *  - JS内部の「納期 >= 今日」固定条件を廃止
 *  - 横軸は今日より最大90日前から、未来30/60/90日まで保持
 *  - 初期表示は今日位置へ自動スクロール
 *  - 左へスクロールすると最大90日前まで確認可能
 *  - 「今日へ戻る」ボタン
 *  - 注文日→加工着手日＝左ヒゲ、加工期間＝太線、完成検査日→納期＝右ヒゲを表示
 *  - 閲覧専用。レコード更新は行わない
 */
(function () {
  'use strict';

  const CONFIG = {
    targetViewName: '外注負荷ヒートマップ',
    rootId: 'vendor-heatmap-root',
    fields: {
      recordId: '$id', key: 'K手配書番号', keyFallback: 'ルックアップ',
      vendor: 'VENDOR_DD', vendorFallback: 'VENDOR_TEXT',
      staff: 'STAFF_DD', staffFallback: 'STAFF_TEXT', product: '商品名',
      orderDate: '日付', startDate: 'K加工着手日', inspectionDate: 'K完成検査日',
      dueDate: '日付_1', scheduleType: 'K日程種別'
    },
    initialDays: 60,
    rangeOptions: [30, 60, 90],
    pastDays: 90,
    dayWidth: 34
  };

  const state = {
    records: [], rangeDays: CONFIG.initialDays, selectedVendor: null,
    listQueryCondition: '', listFilterApplied: false
  };

  kintone.events.on('app.record.index.show', async function (event) {
    if (event.viewName !== CONFIG.targetViewName) return event;
    const root = document.getElementById(CONFIG.rootId);
    if (!root) return event;

    const condition = String(kintone.app.getQueryCondition() || '').trim();
    const previous = String(root.dataset.listQueryCondition || '');
    if (root.dataset.initialized === '1' && previous === condition) return event;

    root.dataset.initialized = '1';
    root.dataset.listQueryCondition = condition;
    root.innerHTML = '<div class="eh-loading">外注先負荷データを取得しています...</div>';

    try {
      state.records = await fetchRecords(condition);
      state.selectedVendor = chooseInitialVendor(state.records);
      render(root, true);
    } catch (e) {
      root.dataset.initialized = '0';
      console.error('[Heatmap v0.6.1]', e);
      root.innerHTML = '<div class="eh-error">データ取得に失敗しました。 ' + esc(e.message || String(e)) + '</div>';
    }
    return event;
  });

  async function fetchRecords(listCondition) {
    state.listQueryCondition = listCondition;
    state.listFilterApplied = !!listCondition;
    const app = kintone.app.getId();
    let all = [], lastId = 0;

    while (true) {
      const parts = [];
      if (listCondition) parts.push('(' + listCondition + ')');
      parts.push('$id > ' + lastId);
      const query = parts.join(' and ') + ' order by $id asc limit 500';
      const resp = await kintone.api(
        kintone.api.url('/k/v1/records.json', true),
        'GET',
        { app: app, query: query }
      );
      const rows = resp.records || [];
      all = all.concat(rows);
      if (rows.length < 500) break;
      lastId = Number(rows[rows.length - 1].$id.value);
    }
    return all.map(normalize);
  }

  function normalize(r) {
    const f = CONFIG.fields;
    return {
      id: val(r, f.recordId),
      displayKey: val(r, f.key) || val(r, f.keyFallback) || ('#' + val(r, f.recordId)),
      vendor: val(r, f.vendor) || val(r, f.vendorFallback) || '(外注先未設定)',
      staff: val(r, f.staff) || val(r, f.staffFallback) || '',
      product: val(r, f.product) || '',
      orderDate: ymd(val(r, f.orderDate)),
      startDate: ymd(val(r, f.startDate)),
      inspectionDate: ymd(val(r, f.inspectionDate)),
      dueDate: ymd(val(r, f.dueDate)),
      scheduleType: String(val(r, f.scheduleType) || '').trim()
    };
  }

  function chooseInitialVendor(records) {
    const counts = {};
    records.forEach(r => { counts[r.vendor] = (counts[r.vendor] || 0) + 1; });
    return Object.keys(counts).sort((a, b) => counts[b] - counts[a] || a.localeCompare(b, 'ja'))[0] || null;
  }

  function viewRange() {
    const today = day(new Date());
    return {
      start: add(today, -CONFIG.pastDays),
      today: today,
      end: add(today, state.rangeDays - 1)
    };
  }

  function render(root, jumpToday) {
    const vr = viewRange();
    const dates = range(vr.start, vr.end);
    const stats = vendorStats(state.records, dates);
    if (!state.selectedVendor && stats.length) state.selectedVendor = stats[0].vendor;
    const canvasWidth = dates.length * CONFIG.dayWidth;

    root.innerHTML = '<div class="eh-wrap">' +
      '<style>' + styles(canvasWidth) + '</style>' +
      '<div class="eh-header"><div><h2>外注先 負荷ヒートマップ / 日程リスク</h2>' +
      '<div class="eh-subtitle">注文日→加工着手日、完成検査日→納期を細いヒゲ、加工期間を太線で表示します。左へスクロールすると最大90日前まで確認できます。</div></div>' +
      '<div class="eh-controls"><label>将来表示期間 <select id="eh-range">' +
      CONFIG.rangeOptions.map(n => '<option value="' + n + '" ' + (n === state.rangeDays ? 'selected' : '') + '>' + n + '日</option>').join('') +
      '</select></label><button type="button" id="eh-today">今日へ戻る</button>' +
      '<span class="eh-count">対象 ' + state.records.length + '件 / 外注先 ' + stats.length + '社 / 保持範囲 ' + fmt(vr.start) + ' ～ ' + fmt(vr.end) + ' ' + (state.listFilterApplied ? '一覧絞込：適用中' : '一覧絞込：なし') + '</span></div></div>' +
      heatmap(stats, dates, vr.today, canvasWidth) +
      detail(state.records, state.selectedVendor, vr, canvasWidth) +
      '</div>';

    const sel = root.querySelector('#eh-range');
    if (sel) sel.addEventListener('change', function () {
      state.rangeDays = Number(this.value);
      render(root, true);
    });

    root.querySelectorAll('[data-vendor]').forEach(el => el.addEventListener('click', function () {
      state.selectedVendor = this.dataset.vendor;
      render(root, false);
      scheduleTodayScroll(root);
    }));

    const todayBtn = root.querySelector('#eh-today');
    if (todayBtn) todayBtn.addEventListener('click', function () { scrollToToday(root); });

    bindScrollSync(root);
    if (jumpToday !== false) scheduleTodayScroll(root);
  }

  function heatmap(stats, dates, today, canvasWidth) {
    const h = dates.map(d => '<th class="' + (sameDay(d, today) ? 'eh-today-col' : '') + '">' + (d.getMonth() + 1) + '/' + d.getDate() + '</th>').join('');
    const rows = stats.map(s => '<tr data-vendor="' + attr(s.vendor) + '"><th class="eh-vendor-name">' + esc(s.vendor) + '</th>' +
      s.counts.map((c, i) => '<td class="eh-heat ' + (sameDay(dates[i], today) ? 'eh-today-col' : '') + '">' + (c || '') + '</td>').join('') + '</tr>').join('');

    return '<section class="eh-section"><h3>外注先別 日次同時案件数</h3>' +
      '<div class="eh-timeline-scroll" data-sync-scroll="1"><div class="eh-timeline-canvas" style="width:' + canvasWidth + 'px">' +
      '<table class="eh-heatmap"><thead><tr><th class="eh-vendor-name">外注先</th>' + h + '</tr></thead><tbody>' +
      (rows || '<tr><td>対象データなし</td></tr>') + '</tbody></table></div></div></section>';
  }

  function vendorStats(records, dates) {
    const m = {};
    records.forEach(r => { if (!m[r.vendor]) m[r.vendor] = []; m[r.vendor].push(r); });
    return Object.keys(m).sort().map(v => ({
      vendor: v,
      records: m[v],
      counts: dates.map(d => m[v].filter(r => r.startDate && r.inspectionDate && r.startDate <= d && r.inspectionDate >= d).length)
    }));
  }

  function detail(records, vendor, vr, canvasWidth) {
    if (!vendor) return '';
    const list = records.filter(r => r.vendor === vendor).sort((a, b) => compareDate(a.dueDate, b.dueDate));
    const total = Math.max(1, diffDays(vr.start, vr.end) + 1);
    const dateHeader = range(vr.start, vr.end).map(d => '<div class="eh-day ' + (sameDay(d, vr.today) ? 'eh-today-col' : '') + '">' + (d.getMonth() + 1) + '/' + d.getDate() + '</div>').join('');

    const rows = list.map(r => {
      const left = span(r.orderDate, r.startDate, vr.start, vr.end, total);
      const work = span(r.startDate, r.inspectionDate, vr.start, vr.end, total);
      const right = span(r.inspectionDate, r.dueDate, vr.start, vr.end, total);
      const pOrder = point(r.orderDate, vr.start, vr.end, total);
      const pStart = point(r.startDate, vr.start, vr.end, total);
      const pInspect = point(r.inspectionDate, vr.start, vr.end, total);
      const pDue = point(r.dueDate, vr.start, vr.end, total);
      const manual = r.scheduleType === 'Manual';

      return '<div class="eh-detail-row">' +
        '<div class="eh-meta"><div>' + (manual ? '<span class="eh-manual">M</span> ' : '') + '<a href="' + recordUrl(r.id) + '">' + esc(r.displayKey) + '</a></div>' +
        '<div>' + esc(r.vendor) + '</div><div>' + esc(r.staff) + '</div><div class="eh-dates">' + fmt(r.orderDate) + ' → ' + fmt(r.startDate) + ' → ' + fmt(r.inspectionDate) + ' → ' + fmt(r.dueDate) + '</div></div>' +
        '<div class="eh-track" style="width:' + canvasWidth + 'px">' +
        (left ? '<div class="eh-whisker" style="left:' + left.left + '%;width:' + left.width + '%"></div>' : '') +
        (work ? '<div class="eh-work" style="left:' + work.left + '%;width:' + work.width + '%"></div>' : '') +
        (right ? '<div class="eh-whisker" style="left:' + right.left + '%;width:' + right.width + '%"></div>' : '') +
        marker('○', pOrder, 'eh-order', '手配日 ' + fmt(r.orderDate)) +
        marker('▶', pStart, 'eh-start', '仕掛開始 ' + fmt(r.startDate)) +
        marker('●', pInspect, 'eh-inspect', '検収日 ' + fmt(r.inspectionDate)) +
        marker('◆', pDue, 'eh-due', '納期 ' + fmt(r.dueDate)) +
        '</div></div>';
    }).join('');

    return '<section class="eh-section"><h3>' + esc(vendor) + '：案件詳細</h3>' +
      '<div class="eh-guide">○ 手配日 ── ▶ 仕掛開始 ━━ ● 検収日 ── ◆ 納期</div>' +
      '<div class="eh-detail-layout"><div class="eh-meta-head">手配書番号 / 外注先 / 担当 / 日付</div>' +
      '<div class="eh-timeline-scroll eh-detail-scroll" data-sync-scroll="1"><div class="eh-detail-canvas" style="width:' + canvasWidth + 'px">' +
      '<div class="eh-date-header">' + dateHeader + '</div>' + rows + '</div></div></div></section>';
  }

  function marker(symbol, pos, cls, title) {
    return pos === null ? '' : '<span class="eh-marker ' + cls + '" style="left:' + pos + '%" title="' + attr(title) + '">' + symbol + '</span>';
  }

  function bindScrollSync(root) {
    const boxes = Array.from(root.querySelectorAll('[data-sync-scroll="1"]'));
    let syncing = false;
    boxes.forEach(box => box.addEventListener('scroll', function () {
      if (syncing) return;
      syncing = true;
      const x = box.scrollLeft;
      boxes.forEach(b => { if (b !== box) b.scrollLeft = x; });
      syncing = false;
    }, { passive: true }));
  }

  function scheduleTodayScroll(root) {
    const run = function () { scrollToToday(root); };
    requestAnimationFrame(function () {
      requestAnimationFrame(run);
      setTimeout(run, 120);
      setTimeout(run, 400);
      setTimeout(run, 800);
    });
  }

  function scrollToToday(root) {
    const vr = viewRange();
    const base = Math.max(0, diffDays(vr.start, vr.today) * CONFIG.dayWidth);
    root.querySelectorAll('[data-sync-scroll="1"]').forEach(function (box) {
      const todayCell = box.querySelector('.eh-today-col');
      let x = base;
      if (todayCell) x = Math.max(0, todayCell.offsetLeft - 8);
      const max = Math.max(0, box.scrollWidth - box.clientWidth);
      box.scrollLeft = Math.min(x, max);
    });
  }

  function span(a, b, s, e, total) {
    if (!a || !b || b < s || a > e) return null;
    const x = a < s ? s : a;
    const y = b > e ? e : b;
    const l = Math.max(0, diffDays(s, x));
    const w = Math.max(1, diffDays(x, y) + 1);
    return { left: l / total * 100, width: w / total * 100 };
  }

  function point(d, s, e, total) {
    if (!d || d < s || d > e) return null;
    return ((diffDays(s, d) + 0.5) / total) * 100;
  }

  function compareDate(a, b) {
    if (!a && !b) return 0;
    if (!a) return 1;
    if (!b) return -1;
    return a - b;
  }

  function recordUrl(id) {
    return kintone.api.url('/k/' + kintone.app.getId() + '/show#record=' + id, false);
  }

  function styles(canvasWidth) {
    return '#' + CONFIG.rootId + ' .eh-wrap{padding:14px;background:#f7f8fa;color:#222}' +
      '#' + CONFIG.rootId + ' .eh-header{display:flex;justify-content:space-between;gap:18px;flex-wrap:wrap;align-items:flex-start}' +
      '#' + CONFIG.rootId + ' h2,#' + CONFIG.rootId + ' h3{margin:0 0 8px}' +
      '#' + CONFIG.rootId + ' .eh-subtitle{color:#666}' +
      '#' + CONFIG.rootId + ' .eh-controls{display:flex;gap:10px;align-items:center;flex-wrap:wrap}' +
      '#' + CONFIG.rootId + ' select,#' + CONFIG.rootId + ' button{padding:6px 9px}' +
      '#' + CONFIG.rootId + ' .eh-count{font-size:12px;color:#666}' +
      '#' + CONFIG.rootId + ' .eh-section{background:#fff;border:1px solid #dfe3e8;border-radius:8px;margin:12px 0;padding:12px;overflow:hidden}' +
      '#' + CONFIG.rootId + ' .eh-timeline-scroll{overflow-x:auto;overflow-y:hidden;max-width:100%;scroll-behavior:smooth;border:1px solid #e2e5e9}' +
      '#' + CONFIG.rootId + ' .eh-timeline-canvas{min-width:' + canvasWidth + 'px;width:' + canvasWidth + 'px}' +
      '#' + CONFIG.rootId + ' .eh-heatmap{border-collapse:collapse;table-layout:fixed;width:max-content;font-size:12px}' +
      '#' + CONFIG.rootId + ' .eh-heatmap th,#' + CONFIG.rootId + ' .eh-heatmap td{border:1px solid #e6e9ed;height:28px;padding:0;text-align:center}' +
      '#' + CONFIG.rootId + ' .eh-heatmap th:not(.eh-vendor-name),#' + CONFIG.rootId + ' .eh-heatmap td{width:' + CONFIG.dayWidth + 'px;min-width:' + CONFIG.dayWidth + 'px;max-width:' + CONFIG.dayWidth + 'px}' +
      '#' + CONFIG.rootId + ' .eh-vendor-name{min-width:180px;text-align:left!important;padding:0 6px!important;background:#fff;position:sticky;left:0;z-index:3}' +
      '#' + CONFIG.rootId + ' .eh-today-col{box-shadow:inset 2px 0 0 #d9534f;background:rgba(217,83,79,.04)}' +
      '#' + CONFIG.rootId + ' .eh-detail-layout{display:grid;grid-template-columns:320px minmax(0,1fr);border:1px solid #dfe3e8}' +
      '#' + CONFIG.rootId + ' .eh-meta-head{background:#f2f4f6;padding:8px;font-weight:700;border-right:1px solid #dfe3e8}' +
      '#' + CONFIG.rootId + ' .eh-detail-canvas{position:relative;min-width:' + canvasWidth + 'px}' +
      '#' + CONFIG.rootId + ' .eh-date-header{display:grid;grid-template-columns:repeat(' + Math.round(canvasWidth / CONFIG.dayWidth) + ',' + CONFIG.dayWidth + 'px);height:30px;background:#f2f4f6}' +
      '#' + CONFIG.rootId + ' .eh-day{width:' + CONFIG.dayWidth + 'px;height:30px;line-height:30px;text-align:center;border-right:1px solid #e3e6ea;box-sizing:border-box;font-size:11px}' +
      '#' + CONFIG.rootId + ' .eh-detail-row{height:58px;position:relative;border-top:1px solid #e6e9ed}' +
      '#' + CONFIG.rootId + ' .eh-meta{position:absolute;left:-320px;top:0;width:320px;height:58px;background:#fff;border-right:1px solid #dfe3e8;padding:5px 7px;box-sizing:border-box;z-index:5;font-size:11px;overflow:hidden}' +
      '#' + CONFIG.rootId + ' .eh-dates{color:#777;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}' +
      '#' + CONFIG.rootId + ' .eh-track{height:58px;position:relative;background:repeating-linear-gradient(to right,transparent 0,transparent ' + (CONFIG.dayWidth - 1) + 'px,#edf0f2 ' + (CONFIG.dayWidth - 1) + 'px,#edf0f2 ' + CONFIG.dayWidth + 'px)}' +
      '#' + CONFIG.rootId + ' .eh-whisker{position:absolute;top:29px;height:2px;background:#6e7f8d;z-index:2}' +
      '#' + CONFIG.rootId + ' .eh-work{position:absolute;top:24px;height:12px;background:#347aa3;border-radius:5px;z-index:3;min-width:3px}' +
      '#' + CONFIG.rootId + ' .eh-marker{position:absolute;top:19px;transform:translateX(-50%);background:#fff;z-index:4;font-size:14px;line-height:20px}' +
      '#' + CONFIG.rootId + ' .eh-manual{display:inline-block;background:#74518f;color:#fff;border-radius:9px;padding:1px 5px;font-size:10px;font-weight:700}' +
      '#' + CONFIG.rootId + ' .eh-guide{font-size:12px;color:#555;margin:6px 0 10px}';
  }

  function val(r, c) { return r && r[c] ? r[c].value : null; }
  function ymd(v) { if (!v) return null; const m = /^(\d{4})-(\d{2})-(\d{2})$/.exec(v); return m ? new Date(+m[1], +m[2] - 1, +m[3]) : null; }
  function day(d) { return new Date(d.getFullYear(), d.getMonth(), d.getDate()); }
  function add(d, n) { const x = new Date(d); x.setDate(x.getDate() + n); return x; }
  function range(a, b) { const x = []; for (let d = new Date(a); d <= b; d = add(d, 1)) x.push(new Date(d)); return x; }
  function diffDays(a, b) { return Math.round((day(b) - day(a)) / 86400000); }
  function sameDay(a, b) { return a && b && a.getFullYear() === b.getFullYear() && a.getMonth() === b.getMonth() && a.getDate() === b.getDate(); }
  function fmt(d) { return d ? (d.getFullYear() + '-' + String(d.getMonth() + 1).padStart(2, '0') + '-' + String(d.getDate()).padStart(2, '0')) : '-'; }
  function esc(v) { return String(v == null ? '' : v).replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c])); }
  function attr(v) { return esc(v); }
})();
