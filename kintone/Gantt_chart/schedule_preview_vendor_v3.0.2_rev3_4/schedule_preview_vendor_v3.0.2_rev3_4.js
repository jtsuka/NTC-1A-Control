(function () {
  'use strict';

  /*
   * 外注先別 暫定日程設定 v3.0.2
   * 対応仕様：コードレビュー仕様書 Rev.3.4 (2026-09-08)
   * 対象：kintone「外注課発注データ-TEST」App ID 272
   *
   * 重要方針：
   * - 外注先ごとに「検／着」の基準日数を保持する。
   * - 暫定ON：選択外注先 かつ K暫定設定=ON かつ K日程種別=Auto のみ更新。
   * - 暫定OFF：選択外注先 かつ K日程種別=Auto のK日付を基準日程（加工着手日=手配日／完成検査日=納期）へ戻す。
   * - Manual はON/OFFの双方から保護する。
   * - 標準編集画面でK日付を人が変更して保存した場合、自動で K日程種別=Manual にする。
   * - 旧v2.1の全件復元は使用せず、選択外注先＋Autoに限定して基準日程へ戻す。
   */

  const CONFIG = {
    vendorField: 'VENDOR_DD',
    provisionalEnabledField: 'K暫定設定',
    scheduleTypeField: 'K日程種別',

    dueField: '日付_1',
    orderDateField: '日付',
    startField: 'K加工着手日',
    inspectionField: 'K完成検査日',

    provisionalOnValue: 'ON',
    provisionalOffValue: 'OFF',
    autoValue: 'Auto',
    manualValue: 'Manual',

    defaultInspectionDaysBeforeDue: 3,
    defaultStartDaysBeforeInspection: 5,

    controlsId: 'schedule_preview_vendor_controls_v3',
    vendorSelectId: 'schedule_preview_vendor_select_v3',
    inspectionInputId: 'schedule_preview_inspection_days_v3',
    startInputId: 'schedule_preview_start_days_v3',
    onButtonId: 'schedule_preview_on_button_v3',
    offButtonId: 'schedule_preview_off_button_v3',

    vendorRulesStorageKey: 'schedulePreview.vendorRules'
  };

  // 内閣府公表の2026年「国民の祝日・休日」。会社カレンダーは未反映。
  const HOLIDAYS = new Set([
    '2026-01-01', '2026-01-12', '2026-02-11', '2026-02-23',
    '2026-03-20', '2026-04-29', '2026-05-03', '2026-05-04',
    '2026-05-05', '2026-05-06', '2026-07-20', '2026-08-11',
    '2026-09-21', '2026-09-22', '2026-09-23', '2026-10-12',
    '2026-11-03', '2026-11-23'
  ]);

  // 標準編集画面での手動変更検出用。
  // edit.show 時点の値と edit.submit 時点の値を比較する。
  let originalEditStart = '';
  let originalEditInspection = '';

  /* ------------------------------------------------------------------
   * 1. 標準レコード編集画面：Manual 自動設定
   * ------------------------------------------------------------------ */

  kintone.events.on('app.record.edit.show', function (event) {
    originalEditStart = getRecordFieldValue(event.record, CONFIG.startField);
    originalEditInspection = getRecordFieldValue(event.record, CONFIG.inspectionField);
    return event;
  });

  kintone.events.on('app.record.edit.submit', function (event) {
    const currentStart = getRecordFieldValue(event.record, CONFIG.startField);
    const currentInspection = getRecordFieldValue(event.record, CONFIG.inspectionField);

    if (
      currentStart !== originalEditStart ||
      currentInspection !== originalEditInspection
    ) {
      if (event.record[CONFIG.scheduleTypeField]) {
        event.record[CONFIG.scheduleTypeField].value = CONFIG.manualValue;
      }
    }

    return event;
  });

  /* ------------------------------------------------------------------
   * 2. 一覧画面：外注先別 暫定ON/OFF UI
   * ------------------------------------------------------------------ */

  kintone.events.on('app.record.index.show', function (event) {
    if (document.getElementById(CONFIG.controlsId)) return event;

    const space = kintone.app.getHeaderMenuSpaceElement();
    if (!space) return event;

    const controls = document.createElement('div');
    controls.id = CONFIG.controlsId;
    controls.style.cssText = [
      'display:inline-flex',
      'align-items:center',
      'gap:4px',
      'flex-wrap:nowrap',
      'margin:4px 4px',
      'vertical-align:middle'
    ].join(';');

    const vendorLabel = createLabel('外注先');
    vendorLabel.title = '暫定日程を反映／解除する外注先';

    const vendorSelect = createVendorSelect();

    const inspectionLabel = createLabel('検');
    inspectionLabel.title = '完成検査日：納期の何日前';

    const inspectionInput = createNumberInput(
      CONFIG.inspectionInputId,
      CONFIG.defaultInspectionDaysBeforeDue
    );
    inspectionInput.title = '完成検査日：納期の何日前';

    const startLabel = createLabel('着');
    startLabel.title = '加工着手日：完成検査日の何日前';

    const startInput = createNumberInput(
      CONFIG.startInputId,
      CONFIG.defaultStartDaysBeforeInspection
    );
    startInput.title = '加工着手日：完成検査日の何日前';

    const onButton = createActionButton(
      CONFIG.onButtonId,
      '暫定ON',
      '#d64545',
      '選択外注先の K暫定設定=ON / K日程種別=Auto の案件へ暫定日程を反映'
    );

    const offButton = createActionButton(
      CONFIG.offButtonId,
      '暫定OFF',
      '#2f6fdd',
      '選択外注先の K日程種別=Auto を基準日程（着手=手配日／検査=納期）へ戻す（Manualは保護）'
    );

    controls.appendChild(vendorLabel);
    controls.appendChild(vendorSelect);
    controls.appendChild(inspectionLabel);
    controls.appendChild(inspectionInput);
    controls.appendChild(startLabel);
    controls.appendChild(startInput);
    controls.appendChild(onButton);
    controls.appendChild(offButton);
    space.appendChild(controls);

    setActionAvailability(vendorSelect, onButton, offButton, false);

    vendorSelect.addEventListener('change', function () {
      const vendor = vendorSelect.value;
      if (!vendor) {
        inspectionInput.value = String(CONFIG.defaultInspectionDaysBeforeDue);
        startInput.value = String(CONFIG.defaultStartDaysBeforeInspection);
        setActionAvailability(vendorSelect, onButton, offButton, false);
        return;
      }

      const offsets = loadVendorOffsets(vendor);
      inspectionInput.value = String(offsets.inspectionDaysBeforeDue);
      startInput.value = String(offsets.startDaysBeforeInspection);
      setActionAvailability(vendorSelect, onButton, offButton, true);
    });

    [inspectionInput, startInput].forEach(function (input) {
      input.addEventListener('change', function () {
        const vendor = vendorSelect.value;
        if (!vendor) return;

        const offsets = readOffsets(inspectionInput, startInput);
        if (offsets.ok) {
          saveVendorOffsets(vendor, offsets);
        }
      });
    });

    onButton.onclick = async function () {
      const vendor = vendorSelect.value;
      if (!vendor) {
        window.alert('外注先を選択してください。');
        return;
      }

      const offsets = readOffsets(inspectionInput, startInput);
      if (!offsets.ok) {
        window.alert(offsets.message);
        return;
      }
      saveVendorOffsets(vendor, offsets);

      setProcessingState(onButton, offButton, vendorSelect, inspectionInput, startInput, 'on');

      try {
        const appId = kintone.app.getId();
        const records = await fetchAllRecords(appId);
        const result = buildApplyUpdates(records, vendor, offsets);

        const message = [
          `外注先：${vendor}`,
          '',
          `完成検査日：納期の${offsets.inspectionDaysBeforeDue}日前`,
          `加工着手日：完成検査日の${offsets.startDaysBeforeInspection}日前`,
          '',
          '対象条件：',
          `・${CONFIG.provisionalEnabledField} = ${CONFIG.provisionalOnValue}`,
          `・${CONFIG.scheduleTypeField} = ${CONFIG.autoValue}`,
          '',
          `反映予定：${result.updates.length}件`,
          `Manual保護：${result.manualProtected}件`,
          `暫定OFF：${result.provisionalOff}件`,
          `納期未設定：${result.noDueDate}件`,
          `短納期・要確認：${result.shortLeadTime}件`,
          `日付関係異常：${result.invalidDate}件`,
          '',
          '開始側の休日：翌稼働日へ補正',
          '終了側の休日：前稼働日へ補正',
          '',
          '実行しますか？'
        ].join('\n');

        if (!window.confirm(message)) {
          restoreIdleState(onButton, offButton, vendorSelect, inspectionInput, startInput);
          return;
        }

        const updated = await updateRecords(appId, result.updates);

        window.alert([
          '暫定日程 ON 結果',
          '',
          `外注先：${vendor}`,
          `更新成功：${updated}件`,
          `Manual保護：${result.manualProtected}件`,
          `暫定OFF：${result.provisionalOff}件`,
          `納期未設定：${result.noDueDate}件`,
          `短納期・要確認：${result.shortLeadTime}件`,
          `日付関係異常：${result.invalidDate}件`
        ].join('\n'));

        window.location.reload();
      } catch (error) {
        handleError('暫定日程ONの処理に失敗しました。', error);
        restoreIdleState(onButton, offButton, vendorSelect, inspectionInput, startInput);
      }
    };

    offButton.onclick = async function () {
      const vendor = vendorSelect.value;
      if (!vendor) {
        window.alert('外注先を選択してください。');
        return;
      }

      setProcessingState(onButton, offButton, vendorSelect, inspectionInput, startInput, 'off');

      try {
        const appId = kintone.app.getId();
        const records = await fetchAllRecords(appId);
        const result = buildRestoreUpdates(records, vendor);

        const message = [
          `外注先：${vendor}`,
          '',
          'Auto日程を基準日程へ戻します。',
          '',
          `復元予定：${result.updates.length}件`,
          `Manual保護：${result.manualProtected}件`,
          `基準日程済み：${result.alreadyBaseline}件`,
          `手配日／納期不足：${result.missingBaseDate}件`,
          '',
          `${CONFIG.startField} = ${CONFIG.orderDateField}（手配日）`,
          `${CONFIG.inspectionField} = ${CONFIG.dueField}（納期）`,
          'Manual案件は変更しません。',
          '',
          '実行しますか？'
        ].join('\n');

        if (!window.confirm(message)) {
          restoreIdleState(onButton, offButton, vendorSelect, inspectionInput, startInput);
          return;
        }

        const updated = await updateRecords(appId, result.updates);

        window.alert([
          '暫定日程 OFF 結果',
          '',
          `外注先：${vendor}`,
          `復元成功：${updated}件`,
          `Manual保護：${result.manualProtected}件`,
          `基準日程済み：${result.alreadyBaseline}件`,
          `手配日／納期不足：${result.missingBaseDate}件`
        ].join('\n'));

        window.location.reload();
      } catch (error) {
        handleError('暫定日程OFFの処理に失敗しました。', error);
        restoreIdleState(onButton, offButton, vendorSelect, inspectionInput, startInput);
      }
    };

    // 外注先一覧はUI表示後に読み込む。失敗しても一覧画面自体は壊さない。
    loadVendorOptions(kintone.app.getId(), vendorSelect, onButton, offButton)
      .catch(function (error) {
        console.error(error);
        vendorSelect.innerHTML = '';
        addOption(vendorSelect, '', '外注先取得エラー');
        vendorSelect.disabled = true;
        onButton.disabled = true;
        offButton.disabled = true;
      });

    return event;
  });

  /* ------------------------------------------------------------------
   * 3. UI helper
   * ------------------------------------------------------------------ */

  function createLabel(text) {
    const span = document.createElement('span');
    span.textContent = text;
    span.style.cssText = [
      'display:inline-block',
      'margin-left:4px',
      'margin-right:2px',
      'font-size:12px',
      'font-weight:bold',
      'color:#444',
      'vertical-align:middle'
    ].join(';');
    return span;
  }

  function createVendorSelect() {
    const select = document.createElement('select');
    select.id = CONFIG.vendorSelectId;
    select.style.cssText = [
      'display:inline-block',
      'width:190px',
      'height:30px',
      'padding:2px 6px',
      'box-sizing:border-box',
      'border:1px solid #b8c0cc',
      'border-radius:3px',
      'font-size:13px',
      'background:#fff',
      'vertical-align:middle'
    ].join(';');
    addOption(select, '', '読込中...');
    select.disabled = true;
    return select;
  }

  function addOption(select, value, text) {
    const option = document.createElement('option');
    option.value = value;
    option.textContent = text;
    select.appendChild(option);
  }

  function createNumberInput(id, value) {
    const input = document.createElement('input');
    input.id = id;
    input.type = 'number';
    input.min = '0';
    input.max = '365';
    input.step = '1';
    input.value = String(value);
    input.style.cssText = [
      'display:inline-block',
      'width:42px',
      'height:30px',
      'margin-right:2px',
      'padding:2px 3px',
      'box-sizing:border-box',
      'border:1px solid #b8c0cc',
      'border-radius:3px',
      'font-size:13px',
      'text-align:center',
      'vertical-align:middle',
      'background:#fff'
    ].join(';');
    return input;
  }

  function createActionButton(id, text, background, title) {
    const button = document.createElement('button');
    button.id = id;
    button.textContent = text;
    button.title = title;
    button.style.cssText = [
      'height:30px',
      'margin:0 2px',
      'padding:4px 10px',
      'border:0',
      'border-radius:4px',
      `background:${background}`,
      'color:#fff',
      'font-weight:bold',
      'cursor:pointer',
      'vertical-align:middle'
    ].join(';');
    return button;
  }

  function setActionAvailability(vendorSelect, onButton, offButton, enabled) {
    const allowed = Boolean(enabled && vendorSelect.value);
    onButton.disabled = !allowed;
    offButton.disabled = !allowed;
    onButton.style.opacity = allowed ? '1' : '.55';
    offButton.style.opacity = allowed ? '1' : '.55';
  }

  function setProcessingState(onButton, offButton, vendorSelect, inspectionInput, startInput, mode) {
    onButton.disabled = true;
    offButton.disabled = true;
    vendorSelect.disabled = true;
    inspectionInput.disabled = true;
    startInput.disabled = true;
    onButton.style.opacity = '.65';
    offButton.style.opacity = '.65';

    if (mode === 'on') {
      onButton.dataset.originalText = onButton.textContent;
      onButton.textContent = '処理中...';
    } else {
      offButton.dataset.originalText = offButton.textContent;
      offButton.textContent = '処理中...';
    }
  }

  function restoreIdleState(onButton, offButton, vendorSelect, inspectionInput, startInput) {
    onButton.textContent = onButton.dataset.originalText || '暫定ON';
    offButton.textContent = offButton.dataset.originalText || '暫定OFF';
    delete onButton.dataset.originalText;
    delete offButton.dataset.originalText;

    vendorSelect.disabled = false;
    inspectionInput.disabled = false;
    startInput.disabled = false;
    setActionAvailability(vendorSelect, onButton, offButton, true);
  }

  /* ------------------------------------------------------------------
   * 4. 外注先／localStorage
   * ------------------------------------------------------------------ */

  async function loadVendorOptions(appId, vendorSelect, onButton, offButton) {
    const records = await fetchVendorRecords(appId);
    const vendors = Array.from(new Set(
      records
        .map(function (record) {
          return getValue(record, CONFIG.vendorField);
        })
        .filter(function (value) {
          return Boolean(value);
        })
    )).sort(function (a, b) {
      return a.localeCompare(b, 'ja');
    });

    vendorSelect.innerHTML = '';
    addOption(vendorSelect, '', '外注先を選択');
    vendors.forEach(function (vendor) {
      addOption(vendorSelect, vendor, vendor);
    });
    vendorSelect.disabled = false;
    setActionAvailability(vendorSelect, onButton, offButton, false);
  }

  async function fetchVendorRecords(appId) {
    const all = [];
    let lastId = 0;

    while (true) {
      const response = await kintone.api(
        kintone.api.url('/k/v1/records.json', true),
        'GET',
        {
          app: appId,
          query: `$id > ${lastId} order by $id asc limit 500`,
          fields: ['$id', CONFIG.vendorField]
        }
      );

      all.push(...response.records);
      if (response.records.length < 500) break;
      lastId = Number(response.records[response.records.length - 1].$id.value);
    }

    return all;
  }

  function readOffsets(inspectionInput, startInput) {
    const inspection = Number(inspectionInput.value);
    const start = Number(startInput.value);

    if (!Number.isInteger(inspection) || inspection < 0 || inspection > 365) {
      return {
        ok: false,
        message: '検査の日数は0～365の整数で入力してください。'
      };
    }

    if (!Number.isInteger(start) || start < 0 || start > 365) {
      return {
        ok: false,
        message: '着手の日数は0～365の整数で入力してください。'
      };
    }

    return {
      ok: true,
      inspectionDaysBeforeDue: inspection,
      startDaysBeforeInspection: start
    };
  }

  function loadVendorRules() {
    try {
      const raw = localStorage.getItem(CONFIG.vendorRulesStorageKey);
      if (!raw) return {};

      const parsed = JSON.parse(raw);
      if (!parsed || typeof parsed !== 'object' || Array.isArray(parsed)) {
        return {};
      }
      return parsed;
    } catch (e) {
      return {};
    }
  }

  function saveVendorRules(rules) {
    try {
      localStorage.setItem(CONFIG.vendorRulesStorageKey, JSON.stringify(rules));
    } catch (e) {
      // localStorage保存失敗時でも、現在の操作は継続する。
    }
  }

  function loadVendorOffsets(vendor) {
    const rules = loadVendorRules();
    const saved = rules[vendor];

    if (saved && isValidOffsetPair(saved)) {
      return {
        inspectionDaysBeforeDue: Number(saved.inspectionDaysBeforeDue),
        startDaysBeforeInspection: Number(saved.startDaysBeforeInspection)
      };
    }

    return {
      inspectionDaysBeforeDue: CONFIG.defaultInspectionDaysBeforeDue,
      startDaysBeforeInspection: CONFIG.defaultStartDaysBeforeInspection
    };
  }

  function saveVendorOffsets(vendor, offsets) {
    if (!vendor || !isValidOffsetPair(offsets)) return;

    const rules = loadVendorRules();
    rules[vendor] = {
      inspectionDaysBeforeDue: Number(offsets.inspectionDaysBeforeDue),
      startDaysBeforeInspection: Number(offsets.startDaysBeforeInspection)
    };
    saveVendorRules(rules);
  }

  function isValidOffsetPair(value) {
    if (!value || typeof value !== 'object') return false;

    const inspection = Number(value.inspectionDaysBeforeDue);
    const start = Number(value.startDaysBeforeInspection);

    return (
      Number.isInteger(inspection) &&
      inspection >= 0 &&
      inspection <= 365 &&
      Number.isInteger(start) &&
      start >= 0 &&
      start <= 365
    );
  }

  /* ------------------------------------------------------------------
   * 5. レコード取得／判定／更新
   * ------------------------------------------------------------------ */

  async function fetchAllRecords(appId) {
    const all = [];
    let lastId = 0;

    while (true) {
      const response = await kintone.api(
        kintone.api.url('/k/v1/records.json', true),
        'GET',
        {
          app: appId,
          query: `$id > ${lastId} order by $id asc limit 500`,
          fields: [
            '$id',
            '$revision',
            CONFIG.vendorField,
            CONFIG.provisionalEnabledField,
            CONFIG.scheduleTypeField,
            CONFIG.dueField,
            CONFIG.orderDateField,
            CONFIG.startField,
            CONFIG.inspectionField
          ]
        }
      );

      all.push(...response.records);
      if (response.records.length < 500) break;
      lastId = Number(response.records[response.records.length - 1].$id.value);
    }

    return all;
  }

  function buildApplyUpdates(records, selectedVendor, offsets) {
    const result = {
      updates: [],
      vendorMismatch: 0,
      provisionalOff: 0,
      manualProtected: 0,
      noDueDate: 0,
      shortLeadTime: 0,
      invalidDate: 0
    };

    records.forEach(function (record) {
      const vendor = getValue(record, CONFIG.vendorField);
      if (vendor !== selectedVendor) {
        result.vendorMismatch++;
        return;
      }

      const provisionalEnabled = getValue(record, CONFIG.provisionalEnabledField);
      if (provisionalEnabled !== CONFIG.provisionalOnValue) {
        result.provisionalOff++;
        return;
      }

      const scheduleType = getValue(record, CONFIG.scheduleTypeField);
      if (scheduleType !== CONFIG.autoValue) {
        result.manualProtected++;
        return;
      }

      const calc = calculateProvisionalDates(record, offsets);
      if (!calc.ok) {
        if (calc.reason === 'noDueDate') {
          result.noDueDate++;
        } else if (calc.reason === 'shortLeadTime') {
          result.shortLeadTime++;
        } else {
          result.invalidDate++;
        }
        return;
      }

      result.updates.push({
        id: record.$id.value,
        revision: record.$revision.value,
        record: {
          [CONFIG.startField]: { value: calc.start },
          [CONFIG.inspectionField]: { value: calc.inspection },
          [CONFIG.scheduleTypeField]: { value: CONFIG.autoValue }
        }
      });
    });

    return result;
  }

  function buildRestoreUpdates(records, selectedVendor) {
    const result = {
      updates: [],
      vendorMismatch: 0,
      manualProtected: 0,
      alreadyBaseline: 0,
      missingBaseDate: 0
    };

    records.forEach(function (record) {
      const vendor = getValue(record, CONFIG.vendorField);
      if (vendor !== selectedVendor) {
        result.vendorMismatch++;
        return;
      }

      const scheduleType = getValue(record, CONFIG.scheduleTypeField);
      if (scheduleType !== CONFIG.autoValue) {
        result.manualProtected++;
        return;
      }

      const orderDateValue = getValue(record, CONFIG.orderDateField);
      const dueValue = getValue(record, CONFIG.dueField);

      if (!orderDateValue || !dueValue) {
        result.missingBaseDate++;
        return;
      }

      const currentStart = getValue(record, CONFIG.startField);
      const currentInspection = getValue(record, CONFIG.inspectionField);

      if (
        currentStart === orderDateValue &&
        currentInspection === dueValue
      ) {
        result.alreadyBaseline++;
        return;
      }

      result.updates.push({
        id: record.$id.value,
        revision: record.$revision.value,
        record: {
          [CONFIG.startField]: { value: orderDateValue },
          [CONFIG.inspectionField]: { value: dueValue },
          [CONFIG.scheduleTypeField]: { value: CONFIG.autoValue }
        }
      });
    });

    return result;
  }

  function calculateProvisionalDates(record, offsets) {
    const dueValue = getValue(record, CONFIG.dueField);
    const orderDateValue = getValue(record, CONFIG.orderDateField);

    if (!dueValue) {
      return { ok: false, reason: 'noDueDate' };
    }

    const dueDate = parseDate(dueValue);
    const orderDate = orderDateValue ? parseDate(orderDateValue) : null;

    if (!dueDate || (orderDateValue && !orderDate)) {
      return { ok: false, reason: 'invalidDate' };
    }

    let inspectionDate = addDays(dueDate, -offsets.inspectionDaysBeforeDue);
    inspectionDate = moveToPreviousWorkingDay(inspectionDate);

    let startDate = addDays(inspectionDate, -offsets.startDaysBeforeInspection);
    startDate = moveToNextWorkingDay(startDate);

    if (startDate > inspectionDate || inspectionDate > dueDate) {
      return { ok: false, reason: 'invalidDate' };
    }

    if (orderDate && startDate < orderDate) {
      return { ok: false, reason: 'shortLeadTime' };
    }

    return {
      ok: true,
      start: formatDate(startDate),
      inspection: formatDate(inspectionDate)
    };
  }

  async function updateRecords(appId, updates) {
    let count = 0;

    for (let i = 0; i < updates.length; i += 100) {
      const chunk = updates.slice(i, i + 100);
      if (chunk.length === 0) continue;

      await kintone.api(
        kintone.api.url('/k/v1/records.json', true),
        'PUT',
        {
          app: appId,
          records: chunk
        }
      );

      count += chunk.length;
    }

    return count;
  }

  /* ------------------------------------------------------------------
   * 6. 共通 helper
   * ------------------------------------------------------------------ */

  function getRecordFieldValue(record, fieldCode) {
    if (!record || !record[fieldCode]) return '';
    const value = record[fieldCode].value;
    return value === null || value === undefined ? '' : String(value).trim();
  }

  function getValue(record, fieldCode) {
    return getRecordFieldValue(record, fieldCode);
  }

  function parseDate(value) {
    if (!/^\d{4}-\d{2}-\d{2}$/.test(value)) return null;

    const parts = value.split('-').map(Number);
    const date = new Date(Date.UTC(parts[0], parts[1] - 1, parts[2]));
    return formatDate(date) === value ? date : null;
  }

  function addDays(date, days) {
    const copy = new Date(date.getTime());
    copy.setUTCDate(copy.getUTCDate() + days);
    return copy;
  }

  function isNonWorkingDay(date) {
    const day = date.getUTCDay();
    return day === 0 || day === 6 || HOLIDAYS.has(formatDate(date));
  }

  function moveToNextWorkingDay(date) {
    let result = new Date(date.getTime());
    while (isNonWorkingDay(result)) {
      result = addDays(result, 1);
    }
    return result;
  }

  function moveToPreviousWorkingDay(date) {
    let result = new Date(date.getTime());
    while (isNonWorkingDay(result)) {
      result = addDays(result, -1);
    }
    return result;
  }

  function formatDate(date) {
    const year = date.getUTCFullYear();
    const month = String(date.getUTCMonth() + 1).padStart(2, '0');
    const day = String(date.getUTCDate()).padStart(2, '0');
    return `${year}-${month}-${day}`;
  }

  function handleError(prefix, error) {
    console.error(error);
    window.alert(`${prefix}\n${error && error.message ? error.message : error}`);
  }
})();
