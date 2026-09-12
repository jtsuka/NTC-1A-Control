/*
 * 会議室・設備予約 kintone
 * Phase 1: 入力検証・重複予約チェック
 *
 * 対象アプリ: App 291
 * 作成日: 2026-09-12
 * Version: 1.0.0
 *
 * 前提フィールドコード:
 *   RESERVE_DATE    予約日
 *   RESOURCE_CODE   資源コード（ルックアップ）
 *   RESOURCE_NAME   資源名
 *   RESOURCE_TYPE   資源種別
 *   RESOURCE_ORDER  資源表示順
 *   START_TIME      開始時刻
 *   END_TIME        終了時刻
 *   RESERVED_BY     予約者（文字列1行）
 *   PURPOSE         件名／用件
 *   NOTE            メモ
 *
 * 仕様:
 *   - 予約可能時間: 07:00〜19:00
 *   - 15分単位
 *   - 開始時刻 < 終了時刻
 *   - 同一日・同一資源の時間重複を禁止
 *   - 隣接予約（例 09:00-10:00 と 10:00-11:00）は許可
 *   - API取得失敗時は保存禁止
 *   - 編集時は自身のレコードを重複判定から除外
 *
 * 注意:
 *   JavaScriptによる事前確認のため、完全な排他制御ではありません。
 *   ほぼ同時に複数人が登録した場合、競合が発生する可能性があります。
 */

(() => {
  'use strict';

  const CONFIG = Object.freeze({
    OPEN_TIME: '07:00',
    CLOSE_TIME: '19:00',
    SLOT_MINUTES: 15,

    FIELDS: Object.freeze({
      DATE: 'RESERVE_DATE',
      RESOURCE_CODE: 'RESOURCE_CODE',
      RESOURCE_NAME: 'RESOURCE_NAME',
      START: 'START_TIME',
      END: 'END_TIME',
      RESERVED_BY: 'RESERVED_BY',
      PURPOSE: 'PURPOSE'
    })
  });

  const escapeQueryValue = (value) =>
    String(value)
      .replace(/\\/g, '\\\\')
      .replace(/"/g, '\\"');

  const timeToMinutes = (hhmm) => {
    if (!/^\d{2}:\d{2}$/.test(hhmm || '')) {
      return NaN;
    }
    const [h, m] = hhmm.split(':').map(Number);
    return h * 60 + m;
  };

  const isValidSlot = (hhmm) => {
    const minutes = timeToMinutes(hhmm);
    if (Number.isNaN(minutes)) return false;
    return (minutes % CONFIG.SLOT_MINUTES) === 0;
  };

  const validateBasic = (record) => {
    const F = CONFIG.FIELDS;

    const date = record[F.DATE]?.value || '';
    const resourceCode = record[F.RESOURCE_CODE]?.value || '';
    const start = record[F.START]?.value || '';
    const end = record[F.END]?.value || '';
    const reservedBy = (record[F.RESERVED_BY]?.value || '').trim();
    const purpose = (record[F.PURPOSE]?.value || '').trim();

    if (!date) return '予約日を入力してください。';
    if (!resourceCode) return '資源を選択してください。';
    if (!start) return '開始時刻を入力してください。';
    if (!end) return '終了時刻を入力してください。';
    if (!reservedBy) return '予約者を入力してください。';
    if (!purpose) return '件名／用件を入力してください。';

    if (!isValidSlot(start) || !isValidSlot(end)) {
      return '開始時刻・終了時刻は15分単位（00・15・30・45分）で入力してください。';
    }

    const startMin = timeToMinutes(start);
    const endMin = timeToMinutes(end);
    const openMin = timeToMinutes(CONFIG.OPEN_TIME);
    const closeMin = timeToMinutes(CONFIG.CLOSE_TIME);

    if (startMin < openMin || endMin > closeMin) {
      return `予約可能時間は ${CONFIG.OPEN_TIME}〜${CONFIG.CLOSE_TIME} です。`;
    }

    if (startMin >= endMin) {
      return '終了時刻は開始時刻より後の時刻を指定してください。';
    }

    return null;
  };

  const fetchSameDayReservations = async (record) => {
    const F = CONFIG.FIELDS;
    const resourceCode = record[F.RESOURCE_CODE].value;
    const date = record[F.DATE].value;

    const query =
      `${F.RESOURCE_CODE} = "${escapeQueryValue(resourceCode)}" ` +
      `and ${F.DATE} = "${escapeQueryValue(date)}"`;

    const params = {
      app: kintone.app.getId(),
      query,
      fields: [
        '$id',
        F.RESOURCE_CODE,
        F.RESOURCE_NAME,
        F.START,
        F.END,
        F.RESERVED_BY,
        F.PURPOSE
      ]
    };

    const response = await kintone.api(
      kintone.api.url('/k/v1/records.json', true),
      'GET',
      params
    );

    return response.records || [];
  };

  const getCurrentRecordId = (event) => {
    if (event.recordId !== undefined && event.recordId !== null) {
      return String(event.recordId);
    }

    const id = event.record?.$id?.value;
    return id ? String(id) : null;
  };

  const findConflict = (event, existingRecords) => {
    const F = CONFIG.FIELDS;
    const record = event.record;
    const newStart = record[F.START].value;
    const newEnd = record[F.END].value;
    const currentId = getCurrentRecordId(event);

    for (const existing of existingRecords) {
      const existingId = existing.$id?.value ? String(existing.$id.value) : null;

      if (currentId && existingId === currentId) {
        continue;
      }

      const existingStart = existing[F.START]?.value || '';
      const existingEnd = existing[F.END]?.value || '';

      if (!existingStart || !existingEnd) {
        continue;
      }

      if (newStart < existingEnd && newEnd > existingStart) {
        return existing;
      }
    }

    return null;
  };

  const buildConflictMessage = (record, conflict) => {
    const F = CONFIG.FIELDS;

    const resourceName =
      record[F.RESOURCE_NAME]?.value ||
      record[F.RESOURCE_CODE]?.value ||
      '選択した資源';

    const start = conflict[F.START]?.value || '';
    const end = conflict[F.END]?.value || '';
    const reservedBy = (conflict[F.RESERVED_BY]?.value || '').trim();
    const purpose = (conflict[F.PURPOSE]?.value || '').trim();

    let detail = '';
    if (reservedBy || purpose) {
      const parts = [];
      if (reservedBy) parts.push(reservedBy);
      if (purpose) parts.push(purpose);
      detail = `（${parts.join('｜')}）`;
    }

    return (
      `${resourceName} は ${start}〜${end} に予約済みです${detail}。` +
      '本人または管理者に調整を依頼してください。'
    );
  };

  const handleSubmit = async (event) => {
    const record = event.record;

    const basicError = validateBasic(record);
    if (basicError) {
      event.error = basicError;
      return event;
    }

    try {
      const existingRecords = await fetchSameDayReservations(record);
      const conflict = findConflict(event, existingRecords);

      if (conflict) {
        event.error = buildConflictMessage(record, conflict);
        return event;
      }
    } catch (error) {
      console.error('[施設予約] 重複予約チェックAPIエラー:', error);
      event.error =
        '予約状況を確認できなかったため保存できませんでした。' +
        '通信状態を確認して、もう一度実行してください。';
      return event;
    }

    return event;
  };

  kintone.events.on(
    [
      'app.record.create.submit',
      'app.record.edit.submit'
    ],
    handleSubmit
  );
})();
