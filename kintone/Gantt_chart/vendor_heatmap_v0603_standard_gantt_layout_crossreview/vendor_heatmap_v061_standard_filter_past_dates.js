/*
 * 外注課 外注先負荷ヒートマップ / 日程リスク可視化 Ver0.6.1
 * 対象: kintone App 272
 *
 * Ver0.6.0.3 からの変更:
 *  - kintone標準一覧の絞り込み条件を業務母集団の唯一の条件として使用
 *  - JS内部の「納期 >= 今日」固定条件を廃止
 *  - 横軸は今日より最大90日前から、未来30/60/90日まで保持
 *  - 初期表示は今日位置へ自動スクロールし、左へスクロールすると過去を確認可能
 *  - 「今日へ戻る」ボタンで今日位置へ復帰
 *  - 遠い将来納期や過去着手日1件だけでは表示軸を無制限に拡張しない
 *
 * 注意: 本ファイルはVer0.6.1の先行実装。既存Ver0.6.0.3は置換しない。
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
    dayWidth: 34,
    minInspectionToDueDays: 3
  };

  const state = { records: [], rangeDays: CONFIG.initialDays, selectedVendor: null,
    listQueryCondition: '', listFilterApplied: false };

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
      state.selectedVendor = state.records.length ? state.records[0].vendor : null;
      render(root, true);
    } catch (e) {
      root.dataset.initialized = '0';
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
      const resp = await kintone.api(kintone.api.url('/k/v1/records.json', true), 'GET', { app: app, query: query });
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
      id: val(r,f.recordId), displayKey: val(r,f.key)||val(r,f.keyFallback)||('#'+val(r,f.recordId)),
      vendor: val(r,f.vendor)||val(r,f.vendorFallback)||'(外注先未設定)',
      staff: val(r,f.staff)||val(r,f.staffFallback)||'', product: val(r,f.product)||'',
      orderDate: ymd(val(r,f.orderDate)), startDate: ymd(val(r,f.startDate)),
      inspectionDate: ymd(val(r,f.inspectionDate)), dueDate: ymd(val(r,f.dueDate)),
      scheduleType: String(val(r,f.scheduleType)||'').trim()
    };
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
    const vr = viewRange(), dates = range(vr.start, vr.end);
    const stats = vendorStats(state.records, dates);
    if (!state.selectedVendor && stats.length) state.selectedVendor = stats[0].vendor;
    const todayIndex = diffDays(vr.start, vr.today);
    const canvasWidth = dates.length * CONFIG.dayWidth;

    root.innerHTML = '<div class="eh-wrap">' +
      '<div class="eh-header"><h2>外注先 負荷ヒートマップ / 日程リスク</h2>' +
      '<div class="eh-subtitle">kintone標準一覧の絞り込み結果をそのまま表示します。左へスクロールすると最大90日前まで確認できます。</div>' +
      '<div class="eh-controls"><label>将来表示期間 <select id="eh-range">' + CONFIG.rangeOptions.map(n=>'<option value="'+n+'" '+(n===state.rangeDays?'selected':'')+'>'+n+'日</option>').join('') + '</select></label>' +
      '<button type="button" id="eh-today">今日へ戻る</button>' +
      '<span class="eh-count">対象 '+state.records.length+'件 / 外注先 '+stats.length+'社 / 保持範囲 '+fmt(vr.start)+' ～ '+fmt(vr.end)+' '+(state.listFilterApplied?'一覧絞込：適用中':'一覧絞込：なし')+'</span></div></div>' +
      '<style>#'+CONFIG.rootId+' .eh-timeline-scroll{overflow-x:auto;overflow-y:hidden;max-width:100%;scroll-behavior:smooth}#'+CONFIG.rootId+' .eh-timeline-canvas{min-width:'+canvasWidth+'px;width:'+canvasWidth+'px}#'+CONFIG.rootId+' .eh-heatmap{table-layout:fixed}#'+CONFIG.rootId+' .eh-heatmap th:not(:first-child),#'+CONFIG.rootId+' .eh-heatmap td{width:'+CONFIG.dayWidth+'px;min-width:'+CONFIG.dayWidth+'px}#'+CONFIG.rootId+' .eh-today-col{box-shadow:inset 2px 0 0 #d9534f}#'+CONFIG.rootId+' #eh-today{margin-left:8px;cursor:pointer}</style>' +
      heatmap(stats, dates, vr.today, canvasWidth) + detail(state.records,state.selectedVendor,vr.start,vr.end,canvasWidth) + '</div>';

    const sel=root.querySelector('#eh-range');
    if(sel) sel.addEventListener('change',function(){state.rangeDays=Number(this.value);render(root,true);});
    root.querySelectorAll('[data-vendor]').forEach(el=>el.addEventListener('click',function(){state.selectedVendor=this.dataset.vendor;render(root,false);syncToToday(root,todayIndex);}));
    const todayBtn=root.querySelector('#eh-today');
    if(todayBtn) todayBtn.addEventListener('click',function(){syncToToday(root,todayIndex);});
    bindScrollSync(root);
    if (jumpToday !== false) requestAnimationFrame(function(){syncToToday(root,todayIndex);});
  }

  function heatmap(stats,dates,today,canvasWidth){
    const h=dates.map(d=>'<th class="'+(sameDay(d,today)?'eh-today-col':'')+'">'+ (d.getMonth()+1)+'/'+d.getDate()+'</th>').join('');
    const rows=stats.map(s=>'<tr data-vendor="'+attr(s.vendor)+'"><th>'+esc(s.vendor)+'</th>'+s.counts.map((c,i)=>'<td class="eh-heat '+(sameDay(dates[i],today)?'eh-today-col':'')+'">'+(c||'')+'</td>').join('')+'</tr>').join('');
    return '<section class="eh-section"><div class="eh-timeline-scroll" data-sync-scroll="1"><div class="eh-timeline-canvas" style="width:'+canvasWidth+'px"><table class="eh-heatmap"><thead><tr><th>外注先</th>'+h+'</tr></thead><tbody>'+ (rows||'<tr><td>対象データなし</td></tr>') +'</tbody></table></div></div></section>';
  }

  function vendorStats(records, dates) {
    const m={}; records.forEach(r=>{if(!m[r.vendor])m[r.vendor]=[];m[r.vendor].push(r);});
    return Object.keys(m).sort().map(v=>({vendor:v,records:m[v],counts:dates.map(d=>m[v].filter(r=>r.startDate&&r.inspectionDate&&r.startDate<=d&&r.inspectionDate>=d).length)}));
  }

  function detail(records,vendor,start,end,canvasWidth){
    if(!vendor)return '';
    const list=records.filter(r=>r.vendor===vendor).sort((a,b)=>(a.dueDate?+a.dueDate:0)-(b.dueDate?+b.dueDate:0));
    const total=Math.max(1,diffDays(start,end)+1);
    const rows=list.map(r=>{
      const a=span(r.startDate,r.inspectionDate,start,end,total);
      return '<tr><td><a href="'+kintone.api.url('/k/'+kintone.app.getId()+'/show#record='+r.id,false)+'">'+esc(r.displayKey)+'</a></td><td>'+esc(r.vendor)+'</td><td>'+esc(r.staff)+'</td><td class="eh-schedule-cell"><div class="eh-schedule-track">'+(a?'<span class="eh-schedule-work" style="left:'+a.left+'%;width:'+a.width+'%"></span>':'')+'</div><div class="eh-date-text">'+fmt(r.orderDate)+' → '+fmt(r.startDate)+' → '+fmt(r.inspectionDate)+' → 納期 '+fmt(r.dueDate)+'</div></td></tr>';
    }).join('');
    return '<section class="eh-section"><h3>'+esc(vendor)+' 案件詳細</h3><div class="eh-timeline-scroll" data-sync-scroll="1"><div class="eh-timeline-canvas" style="width:'+canvasWidth+'px"><table class="eh-risk-table"><thead><tr><th>手配書番号</th><th>外注先</th><th>担当</th><th>日程</th></tr></thead><tbody>'+rows+'</tbody></table></div></div></section>';
  }

  function bindScrollSync(root){
    const boxes=Array.from(root.querySelectorAll('[data-sync-scroll="1"]'));
    let syncing=false;
    boxes.forEach(function(box){box.addEventListener('scroll',function(){if(syncing)return;syncing=true;const x=box.scrollLeft;boxes.forEach(b=>{if(b!==box)b.scrollLeft=x;});syncing=false;},{passive:true});});
  }

  function syncToToday(root,todayIndex){
    const x=Math.max(0,todayIndex*CONFIG.dayWidth);
    root.querySelectorAll('[data-sync-scroll="1"]').forEach(function(box){box.scrollLeft=x;});
  }

  function span(a,b,s,e,total){if(!a||!b||b<s||a>e)return null;const x=a<s?s:a,y=b>e?e:b;const l=Math.max(0,diffDays(s,x));const w=Math.max(1,diffDays(x,y)+1);return{left:l/total*100,width:w/total*100};}
  function val(r,c){return r&&r[c]?r[c].value:null;}
  function ymd(v){if(!v)return null;const m=/^(\d{4})-(\d{2})-(\d{2})$/.exec(v);return m?new Date(+m[1],+m[2]-1,+m[3]):null;}
  function day(d){return new Date(d.getFullYear(),d.getMonth(),d.getDate());}
  function add(d,n){const x=new Date(d);x.setDate(x.getDate()+n);return x;}
  function range(a,b){const x=[];for(let d=new Date(a);d<=b;d=add(d,1))x.push(d);return x;}
  function diffDays(a,b){return Math.round((day(b)-day(a))/86400000);}
  function sameDay(a,b){return a&&b&&a.getFullYear()===b.getFullYear()&&a.getMonth()===b.getMonth()&&a.getDate()===b.getDate();}
  function fmt(d){return d?(d.getFullYear()+'-'+String(d.getMonth()+1).padStart(2,'0')+'-'+String(d.getDate()).padStart(2,'0')):'-';}
  function esc(v){return String(v==null?'':v).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));}
  function attr(v){return esc(v);}
})();
