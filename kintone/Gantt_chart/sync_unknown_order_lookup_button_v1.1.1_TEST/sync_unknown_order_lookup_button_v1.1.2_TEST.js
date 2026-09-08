(function () {
  'use strict';

  /*
   * App272 未知手配書番号 Lookup同期ボタン
   * Ver 1.1.2 TEST / 2026-09-08
   *
   * 目的:
   *   - CSVは App272.ODERNO（表示名：K手配書番号／通常文字列）へ取り込む。
   *   - App270に存在しない手配書番号だけを参照先へ追加する。
   *   - 追加確認後、App272の旧Lookup「ルックアップ」へ同じ番号を反映する。
   *
   * TEST安全策:
   *   - 2026-09-08 TEST CSVで想定する未知番号は TEST260908001 の1件。
   *   - 想定外の未知番号が見つかった場合は書込み前に停止する。
   *
   * 注意:
   *   - APIトークンは埋め込まない。ログイン中ユーザーのkintone権限で実行する。
   *   - App270既存レコードの納期等は更新しない。
   *   - K日程管理フィールドには触れない。
   */

  const CONFIG = {
    app272: 272,
    app270: 270,

    fields272: {
      id: '$id',
      key: 'ODERNO',
      oldLookup: 'ルックアップ',
      due: '日付_1'
    },

    fields270: {
      id: '$id',
      key: '文字列__1行_',
      due: '納期'
    },

    buttonId: 'k-unknown-order-lookup-sync-button',
    buttonText: '未知手配書番号をLookup同期',

    batchSize: 100,

    // TEST完了後の本番化時は [] にする。
    expectedMissingKeys: ['TEST260908001'],

    // TESTではLookup更新対象もダミー1件だけに固定する。
    expectedLookupUpdateKeys: ['TEST260908001'],
    expectedLookupUpdateCount: 1
  };

  function fieldValue(record, code) {
    return record && record[code] ? record[code].value : '';
  }

  function normalize(value) {
    return String(value == null ? '' : value).trim();
  }

  function formatDueFor270(value) {
    // App272 DATE: YYYY-MM-DD -> App270既存形式: YYYY/M/D
    const s = normalize(value);
    const m = s.match(/^(\d{4})-(\d{2})-(\d{2})$/);
    if (!m) return s;
    return `${Number(m[1])}/${Number(m[2])}/${Number(m[3])}`;
  }

  function chunks(items, size) {
    const out = [];
    for (let i = 0; i < items.length; i += size) {
      out.push(items.slice(i, i + size));
    }
    return out;
  }

  async function getAllRecords(app, fields) {
    const cursor = await kintone.api(
      kintone.api.url('/k/v1/records/cursor.json', true),
      'POST',
      {
        app,
        fields,
        size: 500
      }
    );

    const records = [];
    let completed = false;

    try {
      let next = true;
      while (next) {
        const page = await kintone.api(
          kintone.api.url('/k/v1/records/cursor.json', true),
          'GET',
          { id: cursor.id }
        );

        records.push(...(page.records || []));
        next = page.next;
      }
      completed = true;
    } finally {
      // 最終ページ取得後はカーソルが消費済みの場合がある。
      // 途中失敗時のみ削除を試み、削除エラー自体は無視する。
      if (!completed) {
        try {
          await kintone.api(
            kintone.api.url('/k/v1/records/cursor.json', true),
            'DELETE',
            { id: cursor.id }
          );
        } catch (_) {
          // no-op
        }
      }
    }

    return records;
  }

  function build270Index(records270) {
    const keyToIds = new Map();

    for (const r of records270) {
      const key = normalize(fieldValue(r, CONFIG.fields270.key));
      if (!key) continue;

      if (!keyToIds.has(key)) {
        keyToIds.set(key, []);
      }
      keyToIds.get(key).push(normalize(fieldValue(r, CONFIG.fields270.id)));
    }

    return keyToIds;
  }

  function findDuplicateKeys(index270) {
    const duplicates = [];
    for (const [key, ids] of index270.entries()) {
      if (ids.length > 1) {
        duplicates.push({ key, ids });
      }
    }
    return duplicates;
  }

  function analyze272(records272, index270) {
    const uniqueByKey = new Map();
    const lookupUpdates = [];
    let blankKeyCount = 0;
    let alreadySyncedCount = 0;

    for (const r of records272) {
      const key = normalize(fieldValue(r, CONFIG.fields272.key));
      if (!key) {
        blankKeyCount++;
        continue;
      }

      // App270追加は同じ手配書番号につき1回だけ。
      if (!uniqueByKey.has(key)) {
        uniqueByKey.set(key, {
          key,
          due: fieldValue(r, CONFIG.fields272.due)
        });
      }

      const oldLookup = normalize(fieldValue(r, CONFIG.fields272.oldLookup));
      if (oldLookup === key) {
        alreadySyncedCount++;
      } else {
        lookupUpdates.push({
          id: normalize(fieldValue(r, CONFIG.fields272.id)),
          key
        });
      }
    }

    const missing270 = [];
    for (const item of uniqueByKey.values()) {
      if (!index270.has(item.key)) {
        missing270.push(item);
      }
    }

    missing270.sort((a, b) => a.key.localeCompare(b.key, 'ja'));

    return {
      uniqueKeyCount: uniqueByKey.size,
      blankKeyCount,
      alreadySyncedCount,
      missing270,
      lookupUpdates
    };
  }

  function validateExpectedMissingKeys(missing270) {
    const expected = (CONFIG.expectedMissingKeys || [])
      .map(normalize)
      .filter(Boolean)
      .sort();

    if (expected.length === 0) {
      return { ok: true, message: '' };
    }

    const actual = missing270.map((x) => x.key).sort();

    const expectedText = JSON.stringify(expected);
    const actualText = JSON.stringify(actual);

    if (expectedText !== actualText) {
      return {
        ok: false,
        message:
          'TEST安全停止：想定していた未知手配書番号と実際の差分が一致しません。\n\n' +
          `想定: ${expected.join(', ') || '(なし)'}\n` +
          `実際: ${actual.join(', ') || '(なし)'}\n\n` +
          'CSV取込内容・App270の状態を確認してから再実行してください。'
      };
    }

    return { ok: true, message: '' };
  }


  function validateExpectedLookupUpdates(lookupUpdates) {
    const expectedKeys = (CONFIG.expectedLookupUpdateKeys || [])
      .map(normalize)
      .filter(Boolean)
      .sort();

    const expectedCount = Number(CONFIG.expectedLookupUpdateCount || 0);

    if (expectedKeys.length === 0 && expectedCount === 0) {
      return { ok: true, message: '' };
    }

    const actualKeys = [...new Set(
      lookupUpdates.map((x) => normalize(x.key)).filter(Boolean)
    )].sort();

    if (
      JSON.stringify(actualKeys) !== JSON.stringify(expectedKeys) ||
      lookupUpdates.length !== expectedCount
    ) {
      return {
        ok: false,
        message:
          'TEST安全停止：旧Lookup更新対象が想定と一致しません。\n\n' +
          `想定キー: ${expectedKeys.join(', ') || '(なし)'}\n` +
          `実際キー: ${actualKeys.join(', ') || '(なし)'}\n` +
          `想定件数: ${expectedCount}\n` +
          `実際件数: ${lookupUpdates.length}\n\n` +
          '今回のTESTではダミー1件以外の旧Lookupを更新しません。'
      };
    }

    return { ok: true, message: '' };
  }

  async function addMissingTo270(items, button) {
    let added = 0;

    for (const batch of chunks(items, CONFIG.batchSize)) {
      const records = batch.map((item) => {
        const record = {
          [CONFIG.fields270.key]: { value: item.key }
        };

        if (normalize(item.due)) {
          record[CONFIG.fields270.due] = {
            value: formatDueFor270(item.due)
          };
        }

        return record;
      });

      await kintone.api(
        kintone.api.url('/k/v1/records.json', true),
        'POST',
        {
          app: CONFIG.app270,
          records
        }
      );

      added += batch.length;
      button.textContent = `App270追加中 ${added}/${items.length}`;
    }

    return added;
  }

  async function verify270Contains(keys) {
    if (keys.length === 0) return [];

    // 件数が小さいTESTでも、将来件数が増えた時も同じ確認ロジックにする。
    const records270 = await getAllRecords(
      CONFIG.app270,
      [CONFIG.fields270.id, CONFIG.fields270.key]
    );

    const index270 = build270Index(records270);

    return keys.filter((key) => !index270.has(key));
  }

  async function updateOldLookup272(items, button) {
    let updated = 0;

    for (const batch of chunks(items, CONFIG.batchSize)) {
      await kintone.api(
        kintone.api.url('/k/v1/records.json', true),
        'PUT',
        {
          app: CONFIG.app272,
          records: batch.map((item) => ({
            id: item.id,
            record: {
              [CONFIG.fields272.oldLookup]: {
                value: item.key
              }
            }
          }))
        }
      );

      updated += batch.length;
      button.textContent = `Lookup反映中 ${updated}/${items.length}`;
    }

    return updated;
  }

  function buildPreviewMessage(records272, records270, analysis) {
    const missingLines = analysis.missing270.length
      ? analysis.missing270
          .slice(0, 20)
          .map((x) => `  - ${x.key}${x.due ? ` / 納期 ${x.due}` : ''}`)
          .join('\n')
      : '  (なし)';

    const suffix =
      analysis.missing270.length > 20
        ? `\n  ...他 ${analysis.missing270.length - 20}件`
        : '';

    return (
      '未知手配書番号Lookup同期の事前確認\n\n' +
      `App272取得: ${records272.length}件\n` +
      `App270取得: ${records270.length}件\n` +
      `App272 K手配書番号(ODERNO)ユニーク: ${analysis.uniqueKeyCount}件\n` +
      `K手配書番号空欄: ${analysis.blankKeyCount}件\n` +
      `App270未登録: ${analysis.missing270.length}件\n` +
      `旧Lookup更新対象: ${analysis.lookupUpdates.length}件\n` +
      `旧Lookup同期済み: ${analysis.alreadySyncedCount}件\n\n` +
      'App270へ追加予定:\n' +
      missingLines +
      suffix +
      '\n\n' +
      '処理順:\n' +
      '1. App270へ未登録番号だけ追加\n' +
      '2. App270への登録を再取得して確認\n' +
      '3. 確認できた場合だけApp272旧Lookupへ反映\n\n' +
      '続行しますか？'
    );
  }

  async function runSync(button) {
    const originalText = button.textContent;
    button.disabled = true;
    button.textContent = '事前確認中...';

    try {
      const [records272, records270] = await Promise.all([
        getAllRecords(
          CONFIG.app272,
          [
            CONFIG.fields272.id,
            CONFIG.fields272.key,
            CONFIG.fields272.oldLookup,
            CONFIG.fields272.due
          ]
        ),
        getAllRecords(
          CONFIG.app270,
          [
            CONFIG.fields270.id,
            CONFIG.fields270.key
          ]
        )
      ]);

      const index270 = build270Index(records270);
      const duplicate270 = findDuplicateKeys(index270);

      if (duplicate270.length > 0) {
        const sample = duplicate270
          .slice(0, 10)
          .map((x) => `${x.key} (records: ${x.ids.join(', ')})`)
          .join('\n');

        throw new Error(
          'App270のLookup参照キーに重複が見つかりました。安全のため停止します。\n\n' +
          sample
        );
      }

      const analysis = analyze272(records272, index270);

      // TEST CSV専用の想定差分ガード
      const testGuard = validateExpectedMissingKeys(analysis.missing270);
      if (!testGuard.ok) {
        alert(testGuard.message);
        return;
      }

      const lookupGuard = validateExpectedLookupUpdates(analysis.lookupUpdates);
      if (!lookupGuard.ok) {
        alert(lookupGuard.message);
        return;
      }

      if (!window.confirm(buildPreviewMessage(records272, records270, analysis))) {
        return;
      }

      const lookupConfigConfirmed = window.confirm(
        '重要確認：App272の旧Lookup「ルックアップ」の設定で、\n' +
        '参照先から他フィールドへ自動コピーする設定が残っていないか、\n' +
        '管理画面で確認済みですか？\n\n' +
        '確認済みの場合のみ「OK」を押してください。\n' +
        '未確認なら「キャンセル」で停止してください。'
      );

      if (!lookupConfigConfirmed) {
        alert(
          '処理を中止しました。\n\n' +
          'App272管理画面で旧Lookup「ルックアップ」の設定を確認してから再実行してください。'
        );
        return;
      }

      let added270 = 0;
      let updated272 = 0;

      if (analysis.missing270.length > 0) {
        added270 = await addMissingTo270(analysis.missing270, button);

        button.textContent = 'App270登録確認中...';

        const notFoundAfterAdd = await verify270Contains(
          analysis.missing270.map((x) => x.key)
        );

        if (notFoundAfterAdd.length > 0) {
          throw new Error(
            'App270追加後の確認で、まだ存在しない手配書番号があります。\n\n' +
            notFoundAfterAdd.join('\n') +
            '\n\nApp272の旧Lookup更新は実行していません。'
          );
        }
      }

      if (analysis.lookupUpdates.length > 0) {
        // TEST版では、上の完全一致ガードを通過した対象だけを更新する。
        // 想定外対象を暗黙に除外して続行せず、ガード不一致なら事前に停止する。
        updated272 = await updateOldLookup272(
          analysis.lookupUpdates,
          button
        );
      }

      alert(
        '未知手配書番号Lookup同期が完了しました。\n\n' +
        `App270 新規追加: ${added270}件\n` +
        `App272 旧Lookup反映: ${updated272}件\n` +
        `K手配書番号空欄: ${analysis.blankKeyCount}件\n\n` +
        'TESTでは App270 / App272 と標準ガントの表示を確認してください。'
      );

      location.reload();

    } catch (err) {
      console.error('[未知手配書番号Lookup同期] ERROR', err);

      const code = err && err.code ? `\ncode: ${err.code}` : '';
      const message = err && err.message ? err.message : String(err);

      alert(
        '未知手配書番号Lookup同期でエラーが発生しました。\n\n' +
        message +
        code +
        '\n\n開発者コンソールにも詳細を出力しています。'
      );
    } finally {
      button.disabled = false;
      button.textContent = originalText;
    }
  }

  kintone.events.on('app.record.index.show', function (event) {
    // App272以外では何もしない。
    if (kintone.app.getId() !== CONFIG.app272) {
      return event;
    }

    const space = kintone.app.getHeaderMenuSpaceElement();
    if (!space) return event;

    if (document.getElementById(CONFIG.buttonId)) {
      return event;
    }

    const button = document.createElement('button');
    button.id = CONFIG.buttonId;
    button.type = 'button';
    button.textContent = CONFIG.buttonText;

    button.style.marginLeft = '8px';
    button.style.padding = '6px 14px';
    button.style.border = '1px solid #3498db';
    button.style.borderRadius = '4px';
    button.style.background = '#ffffff';
    button.style.color = '#1f5f8b';
    button.style.fontWeight = '600';
    button.style.cursor = 'pointer';

    button.addEventListener('click', function () {
      runSync(button);
    });

    space.appendChild(button);
    return event;
  });

})();
