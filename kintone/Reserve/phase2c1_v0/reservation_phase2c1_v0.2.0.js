/*
 * 会議室・設備予約 kintone
 * Phase 2C-1 v0.2.0
 * 日次タイムライン表示（実データ版）
 *
 * App 290: 施設予約リソースマスタ
 * App 291: 会議室・設備予約
 *
 * 前提:
 * - カスタマイズ一覧HTMLに <div id="reservation-timeline-root"></div>
 * - vis-timeline 7.7.3 を本JSより先に読み込む
 * - vis-timeline CSS と phase2a CSS を読み込む
 *
 * Phase 2A表示を維持しつつ、Phase 2C-1として
 * 15分単位のドラッグ選択と選択内容の確認表示まで実装する。
 *
 * 未実装:
 * - フローティング入力ダイアログ
 * - Validator
 * - REST新規登録
 * - 保存後再描画
 */
(() => {
  'use strict';

  const CONFIG = Object.freeze({
    RESOURCE_APP_ID: 290,
    RESERVATION_APP_ID: 291,
    ROOT_ID: 'reservation-timeline-root',
    OPEN_TIME: '07:00',
    CLOSE_TIME: '19:00',
    SLOT_MINUTES: 15,
    PAGE_SIZE: 500,

    COLOR_PALETTE: Object.freeze([
      { background: '#D7E9FF', border: '#6B9FD6', text: '#1F2D3D' },
      { background: '#DFF4E4', border: '#72B47E', text: '#1F2D3D' },
      { background: '#FFF0CC', border: '#D9A640', text: '#1F2D3D' },
      { background: '#F6DDEE', border: '#C67BA9', text: '#1F2D3D' },
      { background: '#E9DFFF', border: '#9274C8', text: '#1F2D3D' },
      { background: '#DDF3F2', border: '#5FA9A6', text: '#1F2D3D' },
      { background: '#FFE1D6', border: '#D8866A', text: '#1F2D3D' },
      { background: '#E7E7E7', border: '#8A8A8A', text: '#1F2D3D' },
      { background: '#E5F0D2', border: '#89A95B', text: '#1F2D3D' },
      { background: '#FDE0E0', border: '#C86F6F', text: '#1F2D3D' }
    ]),

    RESOURCE_FIELDS: Object.freeze({
      CODE: 'RESOURCE_CODE',
      NAME: 'RESOURCE_NAME',
      TYPE: 'RESOURCE_TYPE',
      ORDER: 'DISPLAY_ORDER',
      ACTIVE: 'ACTIVE'
    }),

    RESERVATION_FIELDS: Object.freeze({
      DATE: 'RESERVE_DATE',
      RESOURCE_CODE: 'RESOURCE_CODE',
      RESOURCE_NAME: 'RESOURCE_NAME',
      RESOURCE_ORDER: 'RESOURCE_ORDER',
      START: 'START_TIME',
      END: 'END_TIME',
      RESERVED_BY: 'RESERVED_BY',
      PURPOSE: 'PURPOSE',
      NOTE: 'NOTE'
    })
  });

  const state = {
    selectedDate: null,
    timeline: null,
    itemsDataSet: null,
    resources: [],
    reservations: [],
    isLoading: false,

    // Phase 2C-1 drag selection
    drag: null,
    dragCleanup: null,
    suppressNextSelect: false
  };

  const pad2 = (n) => String(n).padStart(2, '0');

  const formatDateLocal = (date) =>
    `${date.getFullYear()}-${pad2(date.getMonth() + 1)}-${pad2(date.getDate())}`;

  const parseLocalDate = (yyyyMmDd) => {
    const [y, m, d] = yyyyMmDd.split('-').map(Number);
    return new Date(y, m - 1, d);
  };

  const addDays = (yyyyMmDd, days) => {
    const d = parseLocalDate(yyyyMmDd);
    d.setDate(d.getDate() + days);
    return formatDateLocal(d);
  };

  const toTimelineDate = (yyyyMmDd, hhMm) =>
    new Date(`${yyyyMmDd}T${hhMm}:00`);

  const minutesFromMidnight = (date) =>
    (date.getHours() * 60) + date.getMinutes();

  const timeFromMinutes = (minutes) =>
    `${pad2(Math.floor(minutes / 60))}:${pad2(minutes % 60)}`;

  const clamp = (value, min, max) =>
    Math.min(max, Math.max(min, value));

  const parseTimeToMinutes = (hhMm) => {
    const [h, m] = hhMm.split(':').map(Number);
    return (h * 60) + m;
  };

  const snapFloor = (minutes) =>
    Math.floor(minutes / CONFIG.SLOT_MINUTES) * CONFIG.SLOT_MINUTES;

  const snapCeil = (minutes) =>
    Math.ceil(minutes / CONFIG.SLOT_MINUTES) * CONFIG.SLOT_MINUTES;

  const resourceNameByCode = (resourceCode) => {
    const F = CONFIG.RESOURCE_FIELDS;
    const resource = state.resources.find(
      (r) => r[F.CODE]?.value === resourceCode
    );
    return resource?.[F.NAME]?.value || resourceCode || '';
  };

  const escapeHtml = (value) =>
    String(value ?? '')
      .replace(/&/g, '&amp;')
      .replace(/</g, '&lt;')
      .replace(/>/g, '&gt;')
      .replace(/"/g, '&quot;')
      .replace(/'/g, '&#039;');

  const hashString = (text) => {
    let hash = 0;
    const s = String(text || '');
    for (let i = 0; i < s.length; i += 1) {
      hash = ((hash << 5) - hash) + s.charCodeAt(i);
      hash |= 0;
    }
    return Math.abs(hash);
  };

  const reservationStyle = (reservedBy) => {
    const palette = CONFIG.COLOR_PALETTE;
    const color = palette[hashString(reservedBy) % palette.length];

    return [
      `background-color:${color.background}`,
      `border-color:${color.border}`,
      `color:${color.text}`
    ].join(';');
  };

  const apiGetAllRecords = async ({ app, queryBase, fields }) => {
    const all = [];
    let offset = 0;

    while (true) {
      const query = `${queryBase} limit ${CONFIG.PAGE_SIZE} offset ${offset}`.trim();

      const response = await kintone.api(
        kintone.api.url('/k/v1/records.json', true),
        'GET',
        { app, query, fields }
      );

      const records = response.records || [];
      all.push(...records);

      if (records.length < CONFIG.PAGE_SIZE) break;
      offset += CONFIG.PAGE_SIZE;
    }

    return all;
  };

  const ResourceService = {
    async getActiveResources() {
      const F = CONFIG.RESOURCE_FIELDS;
      return apiGetAllRecords({
        app: CONFIG.RESOURCE_APP_ID,
        queryBase: `${F.ACTIVE} in ("使用中") order by ${F.ORDER} asc, ${F.NAME} asc`,
        fields: [F.CODE, F.NAME, F.TYPE, F.ORDER, F.ACTIVE]
      });
    }
  };

  const ReservationService = {
    async getByDate(date) {
      const F = CONFIG.RESERVATION_FIELDS;
      return apiGetAllRecords({
        app: CONFIG.RESERVATION_APP_ID,
        queryBase:
          `${F.DATE} = "${date}" ` +
          `order by ${F.RESOURCE_ORDER} asc, ${F.START} asc`,
        fields: [
          '$id',
          F.DATE,
          F.RESOURCE_CODE,
          F.RESOURCE_NAME,
          F.RESOURCE_ORDER,
          F.START,
          F.END,
          F.RESERVED_BY,
          F.PURPOSE,
          F.NOTE
        ]
      });
    }
  };

  const createShell = (root) => {
    root.innerHTML = `
      <div class="reservation-phase2a">
        <div class="reservation-toolbar">
          <button type="button" data-action="prev">＜ 前日</button>
          <button type="button" data-action="today">今日</button>
          <input type="date" data-role="date">
          <button type="button" data-action="next">翌日 ＞</button>
          <span class="reservation-status" data-role="status"></span>
        </div>

        <div class="reservation-message" data-role="message" hidden></div>

        <div class="reservation-drag-result" data-role="drag-result" hidden>
          <span data-role="drag-result-text"></span>
          <button type="button" data-action="clear-drag">選択解除</button>
        </div>

        <div class="reservation-timeline" data-role="timeline"></div>

        <div class="reservation-legend">
          予約バー：予約者｜件名　／　クリックで詳細表示　／　空き時間をドラッグすると15分単位で選択
        </div>
      </div>
    `;

    return {
      dateInput: root.querySelector('[data-role="date"]'),
      status: root.querySelector('[data-role="status"]'),
      message: root.querySelector('[data-role="message"]'),
      dragResult: root.querySelector('[data-role="drag-result"]'),
      dragResultText: root.querySelector('[data-role="drag-result-text"]'),
      clearDrag: root.querySelector('[data-action="clear-drag"]'),
      timeline: root.querySelector('[data-role="timeline"]'),
      prev: root.querySelector('[data-action="prev"]'),
      today: root.querySelector('[data-action="today"]'),
      next: root.querySelector('[data-action="next"]')
    };
  };

  const showMessage = (ui, text, kind = 'error') => {
    ui.message.hidden = false;
    ui.message.className = `reservation-message ${kind}`;
    ui.message.textContent = text;
  };

  const clearMessage = (ui) => {
    ui.message.hidden = true;
    ui.message.textContent = '';
  };

  const makeGroups = (resources) => {
    const F = CONFIG.RESOURCE_FIELDS;

    return resources.map((r) => ({
      id: r[F.CODE].value,
      content: `<div class="reservation-resource-label">${escapeHtml(r[F.NAME].value)}</div>`,
      order: Number(r[F.ORDER].value || 0)
    }));
  };


  const makeItems = (reservations) => {
    const F = CONFIG.RESERVATION_FIELDS;

    return reservations
      .filter((r) => r[F.START]?.value && r[F.END]?.value)
      .map((r) => {
        const reservedBy = (r[F.RESERVED_BY]?.value || '').trim();
        const purpose = (r[F.PURPOSE]?.value || '').trim();
        const resourceName = r[F.RESOURCE_NAME]?.value || '';
        const note = r[F.NOTE]?.value || '';

        const label = [reservedBy, purpose].filter(Boolean).join('｜') || '予約';

        return {
          id: String(r.$id.value),
          group: r[F.RESOURCE_CODE].value,
          start: toTimelineDate(r[F.DATE].value, r[F.START].value),
          end: toTimelineDate(r[F.DATE].value, r[F.END].value),
          content: escapeHtml(label),
          title: [
            resourceName,
            `${r[F.START].value}〜${r[F.END].value}`,
            label,
            note
          ].filter(Boolean).join('\n'),
          style: reservationStyle(reservedBy)
        };
      });
  };


  const DRAG_PREVIEW_ID = '__phase2c_drag_preview__';
  const DRAG_MOVE_THRESHOLD_PX = 4;

  const clearDragPreview = (ui) => {
    state.drag = null;

    if (state.itemsDataSet && state.itemsDataSet.get(DRAG_PREVIEW_ID)) {
      state.itemsDataSet.remove(DRAG_PREVIEW_ID);
    }

    if (ui?.dragResult) {
      ui.dragResult.hidden = true;
      ui.dragResultText.textContent = '';
    }
  };

  const makeDragRange = (anchorDate, currentDate) => {
    const openMinutes = parseTimeToMinutes(CONFIG.OPEN_TIME);
    const closeMinutes = parseTimeToMinutes(CONFIG.CLOSE_TIME);

    const anchorMinutes = clamp(
      minutesFromMidnight(anchorDate),
      openMinutes,
      closeMinutes
    );
    const currentMinutes = clamp(
      minutesFromMidnight(currentDate),
      openMinutes,
      closeMinutes
    );

    const low = Math.min(anchorMinutes, currentMinutes);
    const high = Math.max(anchorMinutes, currentMinutes);

    let startMinutes = snapFloor(low);
    let endMinutes = snapCeil(high);

    startMinutes = clamp(startMinutes, openMinutes, closeMinutes);
    endMinutes = clamp(endMinutes, openMinutes, closeMinutes);

    // 最低15分を保証する
    if (endMinutes <= startMinutes) {
      if (startMinutes + CONFIG.SLOT_MINUTES <= closeMinutes) {
        endMinutes = startMinutes + CONFIG.SLOT_MINUTES;
      } else {
        startMinutes = closeMinutes - CONFIG.SLOT_MINUTES;
        endMinutes = closeMinutes;
      }
    }

    return {
      startMinutes,
      endMinutes,
      startTime: timeFromMinutes(startMinutes),
      endTime: timeFromMinutes(endMinutes)
    };
  };

  const updateDragPreview = (ui, groupId, range) => {
    if (!state.itemsDataSet) return;

    const item = {
      id: DRAG_PREVIEW_ID,
      group: groupId,
      start: toTimelineDate(state.selectedDate, range.startTime),
      end: toTimelineDate(state.selectedDate, range.endTime),
      content: `${range.startTime}〜${range.endTime}`,
      className: 'reservation-drag-preview',
      title: 'Phase 2C-1 選択範囲'
    };

    if (state.itemsDataSet.get(DRAG_PREVIEW_ID)) {
      state.itemsDataSet.update(item);
    } else {
      state.itemsDataSet.add(item);
    }
  };

  const showDragResult = (ui, groupId, range) => {
    const resourceName = resourceNameByCode(groupId);

    ui.dragResultText.textContent =
      `選択: ${resourceName} / ${state.selectedDate} / ` +
      `${range.startTime}〜${range.endTime}`;
    ui.dragResult.hidden = false;
  };

  const bindDragSelection = (ui) => {
    const container = ui.timeline;

    const onMouseDown = (event) => {
      if (event.button !== 0 || !state.timeline) return;

      const props = state.timeline.getEventProperties(event);
      if (!props?.time || !props?.group) return;

      // 左側ラベルや時刻軸からの開始は無視
      if (props.what === 'axis' || props.what === 'group-label') return;

      clearDragPreview(ui);

      state.drag = {
        groupId: String(props.group),
        anchorDate: props.time,
        currentDate: props.time,
        startClientX: event.clientX,
        startClientY: event.clientY,
        moved: false
      };

      document.body.classList.add('reservation-dragging');
    };

    const onMouseMove = (event) => {
      if (!state.drag || !state.timeline) return;

      const dx = event.clientX - state.drag.startClientX;
      const dy = event.clientY - state.drag.startClientY;

      if (!state.drag.moved &&
          Math.hypot(dx, dy) < DRAG_MOVE_THRESHOLD_PX) {
        return;
      }

      state.drag.moved = true;

      const props = state.timeline.getEventProperties(event);
      if (!props?.time) return;

      // 施設はmousedown開始時のgroupを固定
      state.drag.currentDate = props.time;

      const range = makeDragRange(
        state.drag.anchorDate,
        state.drag.currentDate
      );

      updateDragPreview(ui, state.drag.groupId, range);
      event.preventDefault();
    };

    const onMouseUp = (event) => {
      if (!state.drag) return;

      const drag = state.drag;
      state.drag = null;
      document.body.classList.remove('reservation-dragging');

      // 単なるクリックはPhase 2Bへ任せる
      if (!drag.moved) {
        return;
      }

      const props = state.timeline?.getEventProperties(event);
      const currentDate = props?.time || drag.currentDate || drag.anchorDate;

      const range = makeDragRange(drag.anchorDate, currentDate);

      updateDragPreview(ui, drag.groupId, range);
      showDragResult(ui, drag.groupId, range);

      // ドラッグ終了直後にvisのselectが発火しても詳細画面へ飛ばさない
      state.suppressNextSelect = true;
      window.setTimeout(() => {
        state.suppressNextSelect = false;
      }, 0);

      event.preventDefault();
    };

    const onKeyDown = (event) => {
      if (event.key !== 'Escape') return;

      if (state.drag) {
        state.drag = null;
        document.body.classList.remove('reservation-dragging');
      }

      clearDragPreview(ui);
    };

    container.addEventListener('mousedown', onMouseDown);
    window.addEventListener('mousemove', onMouseMove);
    window.addEventListener('mouseup', onMouseUp);
    window.addEventListener('keydown', onKeyDown);

    state.dragCleanup = () => {
      container.removeEventListener('mousedown', onMouseDown);
      window.removeEventListener('mousemove', onMouseMove);
      window.removeEventListener('mouseup', onMouseUp);
      window.removeEventListener('keydown', onKeyDown);
      document.body.classList.remove('reservation-dragging');
      state.drag = null;
    };
  };

  const destroyTimeline = () => {
    if (state.dragCleanup) {
      state.dragCleanup();
      state.dragCleanup = null;
    }

    state.itemsDataSet = null;
    state.suppressNextSelect = false;

    if (state.timeline) {
      state.timeline.destroy();
      state.timeline = null;
    }
  };

  const renderTimeline = (ui) => {
    destroyTimeline();

    const groups = new vis.DataSet(makeGroups(state.resources));
    const items = new vis.DataSet(makeItems(state.reservations));
    state.itemsDataSet = items;

    const options = {
      start: toTimelineDate(state.selectedDate, CONFIG.OPEN_TIME),
      end: toTimelineDate(state.selectedDate, CONFIG.CLOSE_TIME),
      min: toTimelineDate(state.selectedDate, CONFIG.OPEN_TIME),
      max: toTimelineDate(state.selectedDate, CONFIG.CLOSE_TIME),

      // 各グループはラベルから決まった高さを、予約0件でも維持する。
      groupHeightMode: 'fixed',
      stack: false,
      selectable: true,
      editable: false,
      moveable: false,
      zoomable: false,
      showCurrentTime: false,

      groupOrder: 'order',

      orientation: {
        axis: 'top',
        item: 'bottom'
      },

      // 表示ラベルは30分間隔。予約単位自体は15分のまま。
      timeAxis: {
        scale: 'minute',
        step: 30
      },

      format: {
        minorLabels: {
          minute: 'HH:mm'
        },
        majorLabels: {
          minute: 'M月D日'
        }
      },

      margin: {
        item: 6,
        axis: 8
      }
    };

    state.timeline = new vis.Timeline(
      ui.timeline,
      items,
      groups,
      options
    );

    state.timeline.on('select', (properties) => {
      if (state.suppressNextSelect) return;
      if (!properties.items || properties.items.length !== 1) return;

      const recordId = String(properties.items[0]);
      if (recordId === DRAG_PREVIEW_ID) return;

      const url =
        `${location.origin}/k/${CONFIG.RESERVATION_APP_ID}/show#record=${encodeURIComponent(recordId)}`;
      location.href = url;
    });

    bindDragSelection(ui);
  };

  const loadDate = async (ui, date) => {
    if (state.isLoading) return;
    state.isLoading = true;

    clearDragPreview(ui);
    state.selectedDate = date;
    ui.dateInput.value = date;
    ui.status.textContent = '読み込み中…';
    clearMessage(ui);

    try {
      const [resources, reservations] = await Promise.all([
        ResourceService.getActiveResources(),
        ReservationService.getByDate(date)
      ]);

      state.resources = resources;
      state.reservations = reservations;

      if (resources.length === 0) {
        destroyTimeline();
        showMessage(ui, '使用中の予約施設がありません。', 'info');
        ui.status.textContent = '';
        return;
      }

      renderTimeline(ui);

      ui.status.textContent =
        `${resources.length}施設 / ${reservations.length}件`;
    } catch (error) {
      console.error('[Phase2A] 読み込みエラー:', error);
      destroyTimeline();
      showMessage(
        ui,
        '予約施設または予約情報を取得できませんでした。権限・通信状態を確認してください。'
      );
      ui.status.textContent = '';
    } finally {
      state.isLoading = false;
    }
  };

  const bindToolbar = (ui) => {
    ui.prev.addEventListener('click', () => {
      loadDate(ui, addDays(state.selectedDate, -1));
    });

    ui.today.addEventListener('click', () => {
      loadDate(ui, formatDateLocal(new Date()));
    });

    ui.next.addEventListener('click', () => {
      loadDate(ui, addDays(state.selectedDate, 1));
    });

    ui.dateInput.addEventListener('change', () => {
      if (ui.dateInput.value) {
        loadDate(ui, ui.dateInput.value);
      }
    });

    ui.clearDrag.addEventListener('click', () => {
      clearDragPreview(ui);
    });
  };

  kintone.events.on('app.record.index.show', async () => {
    const root = document.getElementById(CONFIG.ROOT_ID);

    // カスタマイズ一覧以外では何もしない
    if (!root) return;

    if (typeof vis === 'undefined' || !vis.Timeline || !vis.DataSet) {
      root.innerHTML =
        '<div class="reservation-message error">' +
        'vis-timelineを読み込めませんでした。JavaScriptの読み込み順を確認してください。' +
        '</div>';
      return;
    }

    destroyTimeline();

    const ui = createShell(root);
    bindToolbar(ui);

    const initialDate = formatDateLocal(new Date());
    await loadDate(ui, initialDate);
  });
})();
