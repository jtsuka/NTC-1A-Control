(function () {
  'use strict';

  const CONFIG = {
    dueField: '日付_1',
    orderDateField: '日付',
    startField: 'K加工着手日',
    inspectionField: 'K完成検査日',
    inspectionDaysBeforeDue: 3,
    startDaysBeforeInspection: 5,
    buttonId: 'schedule_preview_button'
  };

  // 内閣府公表の2026年「国民の祝日・休日」。会社カレンダーは未反映。
  const HOLIDAYS = new Set([
    '2026-01-01', '2026-01-12', '2026-02-11', '2026-02-23',
    '2026-03-20', '2026-04-29', '2026-05-03', '2026-05-04',
    '2026-05-05', '2026-05-06', '2026-07-20', '2026-08-11',
    '2026-09-21', '2026-09-22', '2026-09-23', '2026-10-12',
    '2026-11-03', '2026-11-23'
  ]);

  kintone.events.on('app.record.index.show', function (event) {
    if (document.getElementById(CONFIG.buttonId)) return event;

    const space = kintone.app.getHeaderMenuSpaceElement();
    if (!space) return event;

    const button = document.createElement('button');
    button.id = CONFIG.buttonId;
    button.textContent = '暫定日程を反映（TEST）';
    button.style.cssText = [
      'margin:8px', 'padding:8px 14px', 'border:0', 'border-radius:4px',
      'background:#2f6fdd', 'color:#fff', 'font-weight:bold', 'cursor:pointer'
    ].join(';');

    button.onclick = async function () {
      const message = [
        '空欄のレコードだけに暫定日程を反映します。',
        '',
        '完成検査日：納期の3日前',
        '加工着手日：完成検査日の5日前',
        '開始側の休日：翌稼働日へ補正',
        '終了側の休日：前稼働日へ補正',
        '',
        '入力済み日付と短納期候補は更新しません。実行しますか？'
      ].join('\n');

      if (!window.confirm(message)) return;

      button.disabled = true;
      button.textContent = '暫定日程を処理中...';

      try {
        const appId = kintone.app.getId();
        const records = await fetchAllRecords(appId);
        const result = buildUpdates(records);
        const updated = await updateRecords(appId, result.updates);

        window.alert([
          '暫定日程反映結果',
          '',
          `更新成功：${updated}件`,
          `入力済みのため対象外：${result.alreadySet}件`,
          `納期未設定：${result.noDueDate}件`,
          `短納期・要確認：${result.shortLeadTime}件`,
          `日付関係異常：${result.invalidDate}件`,
          '',
          '短納期・イレギュラー案件の正式ルールは未確定です。'
        ].join('\n'));

        window.location.reload();
      } catch (error) {
        console.error(error);
        window.alert(`暫定日程の反映に失敗しました。\n${error.message || error}`);
        button.disabled = false;
        button.textContent = '暫定日程を反映（TEST）';
      }
    };

    space.appendChild(button);
    return event;
  });

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
            '$id', '$revision', CONFIG.dueField, CONFIG.orderDateField,
            CONFIG.startField, CONFIG.inspectionField
          ]
        }
      );

      all.push(...response.records);
      if (response.records.length < 500) break;
      lastId = Number(response.records[response.records.length - 1].$id.value);
    }

    return all;
  }

  function buildUpdates(records) {
    const result = {
      updates: [], alreadySet: 0, noDueDate: 0,
      shortLeadTime: 0, invalidDate: 0
    };

    records.forEach(function (record) {
      const startExisting = getValue(record, CONFIG.startField);
      const inspectionExisting = getValue(record, CONFIG.inspectionField);
      const dueValue = getValue(record, CONFIG.dueField);
      const orderDateValue = getValue(record, CONFIG.orderDateField);

      if (startExisting || inspectionExisting) {
        result.alreadySet++;
        return;
      }

      if (!dueValue) {
        result.noDueDate++;
        return;
      }

      const dueDate = parseDate(dueValue);
      const orderDate = orderDateValue ? parseDate(orderDateValue) : null;
      if (!dueDate || (orderDateValue && !orderDate)) {
        result.invalidDate++;
        return;
      }

      let inspectionDate = addDays(dueDate, -CONFIG.inspectionDaysBeforeDue);
      inspectionDate = moveToPreviousWorkingDay(inspectionDate);

      let startDate = addDays(inspectionDate, -CONFIG.startDaysBeforeInspection);
      startDate = moveToNextWorkingDay(startDate);

      if (startDate > inspectionDate || inspectionDate > dueDate) {
        result.invalidDate++;
        return;
      }

      // 手配日より前になる案件は短納期候補として更新しない。
      if (orderDate && startDate < orderDate) {
        result.shortLeadTime++;
        return;
      }

      result.updates.push({
        id: record.$id.value,
        revision: record.$revision.value,
        record: {
          [CONFIG.startField]: { value: formatDate(startDate) },
          [CONFIG.inspectionField]: { value: formatDate(inspectionDate) }
        }
      });
    });

    return result;
  }

  async function updateRecords(appId, updates) {
    let count = 0;
    for (let i = 0; i < updates.length; i += 100) {
      const chunk = updates.slice(i, i + 100);
      await kintone.api(
        kintone.api.url('/k/v1/records.json', true),
        'PUT',
        { app: appId, records: chunk }
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
    while (isNonWorkingDay(result)) result = addDays(result, 1);
    return result;
  }

  function moveToPreviousWorkingDay(date) {
    let result = new Date(date.getTime());
    while (isNonWorkingDay(result)) result = addDays(result, -1);
    return result;
  }

  function formatDate(date) {
    const year = date.getUTCFullYear();
    const month = String(date.getUTCMonth() + 1).padStart(2, '0');
    const day = String(date.getUTCDate()).padStart(2, '0');
    return `${year}-${month}-${day}`;
  }
})();
