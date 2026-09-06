(function () {
  'use strict';

  const CONFIG = {
    dueField: '日付_1',
    orderDateField: '日付',
    startField: 'K加工着手日',
    inspectionField: 'K完成検査日',

    defaultInspectionDaysBeforeDue: 3,
    defaultStartDaysBeforeInspection: 5,

    buttonId: 'schedule_preview_button',
    inspectionInputId: 'schedule_preview_inspection_days',
    startInputId: 'schedule_preview_start_days',

    stateStorageKey: 'schedulePreview.toggleState',
    offsetStorageKey: 'schedulePreview.offsetDays',
    appliedOffsetStorageKey: 'schedulePreview.appliedOffsetDays'
  };

  // 内閣府公表の2026年「国民の祝日・休日」。会社カレンダーは未反映。
  const HOLIDAYS = new Set([
    '2026-01-01', '2026-01-12', '2026-02-11', '2026-02-23',
    '2026-03-20', '2026-04-29', '2026-05-03', '2026-05-04',
    '2026-05-05', '2026-05-06', '2026-07-20', '2026-08-11',
    '2026-09-21', '2026-09-22', '2026-09-23', '2026-10-12',
    '2026-11-03', '2026-11-23'
  ]);

  /*
   * 元の正常動作版と同じ構造。
   * index.show は同期。
   * APIはクリックされるまで一切呼ばない。
   */
  kintone.events.on('app.record.index.show', function (event) {
    if (document.getElementById(CONFIG.buttonId)) return event;

    const space = kintone.app.getHeaderMenuSpaceElement();
    if (!space) return event;

    const savedOffsets = loadOffsets(
      CONFIG.offsetStorageKey,
      CONFIG.defaultInspectionDaysBeforeDue,
      CONFIG.defaultStartDaysBeforeInspection
    );

    /*
     * まずボタンを元JSとほぼ同じ方法で作り、即appendする。
     * これより前に余計な処理をしない。
     */
    const button = document.createElement('button');
    button.id = CONFIG.buttonId;
    button.style.cssText = [
      'margin:8px 4px',
      'padding:8px 12px',
      'border:0',
      'border-radius:4px',
      'color:#fff',
      'font-weight:bold',
      'cursor:pointer'
    ].join(';');

    // 保存状態だけで色を決める。表示時API取得はしない。
    const storedState = loadToggleState();
    setButtonState(button, storedState);

    space.appendChild(button);

    /*
     * ボタンが表示された後で、3/5入力欄を作る。
     * ラベルとinputはボタンの直前にinsertBeforeする。
     */
    const inspectionLabel = createLabel('検');
    inspectionLabel.title = '完成検査日：納期の何日前';

    const inspectionInput = createNumberInput(
      CONFIG.inspectionInputId,
      savedOffsets.inspectionDaysBeforeDue
    );
    inspectionInput.title = '完成検査日：納期の何日前';

    const startLabel = createLabel('着');
    startLabel.title = '加工着手日：完成検査日の何日前';

    const startInput = createNumberInput(
      CONFIG.startInputId,
      savedOffsets.startDaysBeforeInspection
    );
    startInput.title = '加工着手日：完成検査日の何日前';

    space.insertBefore(inspectionLabel, button);
    space.insertBefore(inspectionInput, button);
    space.insertBefore(startLabel, button);
    space.insertBefore(startInput, button);

    if (storedState === 'on') {
      const applied = loadOffsets(
        CONFIG.appliedOffsetStorageKey,
        savedOffsets.inspectionDaysBeforeDue,
        savedOffsets.startDaysBeforeInspection
      );
      inspectionInput.value = String(applied.inspectionDaysBeforeDue);
      startInput.value = String(applied.startDaysBeforeInspection);
      setInputsDisabled(inspectionInput, startInput, true);
    }

    inspectionInput.addEventListener('change', function () {
      saveCurrentOffsetsIfValid(inspectionInput, startInput);
    });

    startInput.addEventListener('change', function () {
      saveCurrentOffsetsIfValid(inspectionInput, startInput);
    });

    button.onclick = async function () {
      const currentState = loadToggleState();

      let offsets;
      if (currentState === 'on') {
        offsets = loadOffsets(
          CONFIG.appliedOffsetStorageKey,
          CONFIG.defaultInspectionDaysBeforeDue,
          CONFIG.defaultStartDaysBeforeInspection
        );
      } else {
        const checked = readOffsets(inspectionInput, startInput);
        if (!checked.ok) {
          window.alert(checked.message);
          return;
        }
        offsets = checked;
        saveOffsets(CONFIG.offsetStorageKey, offsets);
      }

      button.disabled = true;
      button.textContent = '処理中...';
      button.style.background = '#777';

      try {
        const appId = kintone.app.getId();
        const records = await fetchAllRecords(appId);

        if (currentState === 'on') {
          const result = buildRestoreUpdates(records, offsets);

          const message = [
            '暫定日程を解除します。',
            '',
            `検査：納期の${offsets.inspectionDaysBeforeDue}日前`,
            `着手：検査の${offsets.startDaysBeforeInspection}日前`,
            '',
            `解除予定：${result.updates.length}件`,
            `手入力・変更済みとして保護：${result.protected}件`,
            '',
            'この条件で自動設定されたと判定できる案件だけ、元の手配日・納期へ戻します。',
            '実行しますか？'
          ].join('\n');

          if (!window.confirm(message)) {
            button.disabled = false;
            setButtonState(button, 'on');
            return;
          }

          const updated = await updateRecords(appId, result.updates);

          saveToggleState('off');
          clearStorage(CONFIG.appliedOffsetStorageKey);

          window.alert([
            '暫定日程 解除結果（元の日付へ復元）',
            '',
            `解除成功：${updated}件`,
            `手入力・変更済みとして保護：${result.protected}件`,
            `対象外：${result.skipped}件`
          ].join('\n'));

        } else {
          const result = buildApplyUpdates(records, offsets);

          const message = [
            '空欄のレコードだけに暫定日程を反映します。',
            '',
            `完成検査日：納期の${offsets.inspectionDaysBeforeDue}日前`,
            `加工着手日：完成検査日の${offsets.startDaysBeforeInspection}日前`,
            '開始側の休日：翌稼働日へ補正',
            '終了側の休日：前稼働日へ補正',
            '',
            `反映予定：${result.updates.length}件`,
            `入力済みのため対象外：${result.alreadySet}件`,
            `納期未設定：${result.noDueDate}件`,
            `短納期・要確認：${result.shortLeadTime}件`,
            `日付関係異常：${result.invalidDate}件`,
            '',
            '実行しますか？'
          ].join('\n');

          if (!window.confirm(message)) {
            button.disabled = false;
            setButtonState(button, 'off');
            return;
          }

          const updated = await updateRecords(appId, result.updates);

          saveOffsets(CONFIG.appliedOffsetStorageKey, offsets);
          saveToggleState('on');

          window.alert([
            '暫定日程反映結果',
            '',
            `更新成功：${updated}件`,
            `入力済みのため対象外：${result.alreadySet}件`,
            `納期未設定：${result.noDueDate}件`,
            `短納期・要確認：${result.shortLeadTime}件`,
            `日付関係異常：${result.invalidDate}件`
          ].join('\n'));
        }

        window.location.reload();

      } catch (error) {
        console.error(error);
        window.alert(
          `暫定日程の切替に失敗しました。\n${error.message || error}`
        );

        button.disabled = false;
        setButtonState(button, currentState);
      }
    };

    return event;
  });

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
      'width:40px',
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

  function setButtonState(button, state) {
    button.disabled = false;
    button.style.opacity = '1';

    if (state === 'on') {
      button.textContent = '暫定 ON';
      button.style.background = '#d64545';
      button.title = '赤＝暫定日程ON。クリックすると解除します。';
    } else {
      button.textContent = '暫定 OFF';
      button.style.background = '#2f6fdd';
      button.title = '青＝暫定日程OFF。クリックすると反映します。';
    }
  }

  function setInputsDisabled(inspectionInput, startInput, disabled) {
    inspectionInput.disabled = disabled;
    startInput.disabled = disabled;
    inspectionInput.style.opacity = disabled ? '.65' : '1';
    startInput.style.opacity = disabled ? '.65' : '1';
  }

  function saveCurrentOffsetsIfValid(inspectionInput, startInput) {
    const offsets = readOffsets(inspectionInput, startInput);
    if (offsets.ok) {
      saveOffsets(CONFIG.offsetStorageKey, offsets);
    }
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

  function loadToggleState() {
    try {
      return localStorage.getItem(CONFIG.stateStorageKey) === 'on'
        ? 'on'
        : 'off';
    } catch (e) {
      return 'off';
    }
  }

  function saveToggleState(state) {
    try {
      localStorage.setItem(
        CONFIG.stateStorageKey,
        state === 'on' ? 'on' : 'off'
      );
    } catch (e) {
      // no-op
    }
  }

  function loadOffsets(key, fallbackInspection, fallbackStart) {
    try {
      const raw = localStorage.getItem(key);
      if (raw) {
        const parsed = JSON.parse(raw);
        const inspection = Number(parsed.inspectionDaysBeforeDue);
        const start = Number(parsed.startDaysBeforeInspection);

        if (
          Number.isInteger(inspection) &&
          inspection >= 0 &&
          inspection <= 365 &&
          Number.isInteger(start) &&
          start >= 0 &&
          start <= 365
        ) {
          return {
            inspectionDaysBeforeDue: inspection,
            startDaysBeforeInspection: start
          };
        }
      }
    } catch (e) {
      // fallback
    }

    return {
      inspectionDaysBeforeDue: Number(fallbackInspection),
      startDaysBeforeInspection: Number(fallbackStart)
    };
  }

  function saveOffsets(key, offsets) {
    try {
      localStorage.setItem(
        key,
        JSON.stringify({
          inspectionDaysBeforeDue: offsets.inspectionDaysBeforeDue,
          startDaysBeforeInspection: offsets.startDaysBeforeInspection
        })
      );
    } catch (e) {
      // no-op
    }
  }

  function clearStorage(key) {
    try {
      localStorage.removeItem(key);
    } catch (e) {
      // no-op
    }
  }

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
            CONFIG.dueField,
            CONFIG.orderDateField,
            CONFIG.startField,
            CONFIG.inspectionField
          ]
        }
      );

      all.push(...response.records);

      if (response.records.length < 500) break;

      lastId = Number(
        response.records[response.records.length - 1].$id.value
      );
    }

    return all;
  }

  function buildApplyUpdates(records, offsets) {
    const result = {
      updates: [],
      alreadySet: 0,
      noDueDate: 0,
      shortLeadTime: 0,
      invalidDate: 0
    };

    records.forEach(function (record) {
      const startExisting = getValue(record, CONFIG.startField);
      const inspectionExisting = getValue(record, CONFIG.inspectionField);

      if (startExisting || inspectionExisting) {
        result.alreadySet++;
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
          [CONFIG.inspectionField]: { value: calc.inspection }
        }
      });
    });

    return result;
  }

  function buildRestoreUpdates(records, offsets) {
    const result = {
      updates: [],
      protected: 0,
      skipped: 0
    };

    records.forEach(function (record) {
      const startExisting = getValue(record, CONFIG.startField);
      const inspectionExisting = getValue(record, CONFIG.inspectionField);

      if (!startExisting && !inspectionExisting) {
        result.skipped++;
        return;
      }

      if (!isAutoProvisionalRecord(record, offsets)) {
        result.protected++;
        return;
      }

      const orderDateValue = getValue(record, CONFIG.orderDateField);
      const dueValue = getValue(record, CONFIG.dueField);

      // 暫定解除時は空欄にせず、
      // K加工着手日 = 元の手配日、K完成検査日 = 元の納期へ戻す。
      if (!orderDateValue || !dueValue) {
        result.protected++;
        return;
      }

      result.updates.push({
        id: record.$id.value,
        revision: record.$revision.value,
        record: {
          [CONFIG.startField]: { value: orderDateValue },
          [CONFIG.inspectionField]: { value: dueValue }
        }
      });
    });

    return result;
  }

  function isAutoProvisionalRecord(record, offsets) {
    const startExisting = getValue(record, CONFIG.startField);
    const inspectionExisting = getValue(record, CONFIG.inspectionField);

    if (!startExisting || !inspectionExisting) return false;

    const calc = calculateProvisionalDates(record, offsets);
    if (!calc.ok) return false;

    return (
      startExisting === calc.start &&
      inspectionExisting === calc.inspection
    );
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

    let inspectionDate = addDays(
      dueDate,
      -offsets.inspectionDaysBeforeDue
    );
    inspectionDate = moveToPreviousWorkingDay(inspectionDate);

    let startDate = addDays(
      inspectionDate,
      -offsets.startDaysBeforeInspection
    );
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

  function getValue(record, fieldCode) {
    return record[fieldCode] && record[fieldCode].value
      ? String(record[fieldCode].value).trim()
      : '';
  }

  function parseDate(value) {
    if (!/^\d{4}-\d{2}-\d{2}$/.test(value)) return null;

    const parts = value.split('-').map(Number);
    const date = new Date(
      Date.UTC(parts[0], parts[1] - 1, parts[2])
    );

    return formatDate(date) === value ? date : null;
  }

  function addDays(date, days) {
    const copy = new Date(date.getTime());
    copy.setUTCDate(copy.getUTCDate() + days);
    return copy;
  }

  function isNonWorkingDay(date) {
    const day = date.getUTCDay();

    return (
      day === 0 ||
      day === 6 ||
      HOLIDAYS.has(formatDate(date))
    );
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
})();
