(function () {
  'use strict';

  // ★ここを自分のアプリのフィールドコードに合わせてください
  const FIELD_PAIRS = [
    { text: 'VENDOR_TEXT', dropdown: 'VENDOR_DD', label: '外注先' },
    { text: 'STAFF_TEXT',  dropdown: 'STAFF_DD',  label: '担当者' }
  ];

  // 一覧画面表示時にボタンを設置
  kintone.events.on('app.record.index.show', function (event) {
    if (document.getElementById('update_options_button')) return;

    const btn = document.createElement('button');
    btn.id = 'update_options_button';
    btn.innerText = '外注先・担当者マスタを更新';
    btn.style = 'margin: 8px; padding: 8px 14px; background-color: #f39c12; color: white; border: none; border-radius: 4px; cursor: pointer; font-weight: bold;';

    btn.onclick = async function () {
      if (!confirm('CSVで取り込んだ外注先・担当者をもとに、プルダウンの選択肢と各レコードの値を一括更新しますか？')) {
        return;
      }

      try {
        const appId = kintone.app.getId();

        // 1) 全レコード取得
        const records = await fetchAllRecords(appId);
        if (records.length === 0) {
          alert('対象レコードがありません。');
          return;
        }

        // 2) フォーム設定を取得（プレビュー）
        const formInfo = await kintone.api(
          kintone.api.url('/k/v1/preview/app/form/fields.json', true),
          'GET',
          { app: appId, lang: 'ja' }
        );
        const props = formInfo.properties;
        let changed = false;

        // 3) プルダウンの選択肢を追加
        FIELD_PAIRS.forEach(pair => {
          const ddField = props[pair.dropdown];
          if (!ddField || ddField.type !== 'DROP_DOWN') return;

          const currentOptions = ddField.options;

          const uniqueValues = [...new Set(
            records
              .map(r => (r[pair.text] && r[pair.text].value || '').trim())
              .filter(v => v) // 空除外
          )];

          uniqueValues.forEach(val => {
            if (!currentOptions[val]) {
              const nextIndex = Object.keys(currentOptions).length;
              currentOptions[val] = {
                label: val,
                index: String(nextIndex),
                disabled: false
              };
              changed = true;
            }
          });

          ddField.options = currentOptions;
          props[pair.dropdown] = ddField;
        });

        if (changed) {
          // 4) フォーム設定を更新 → デプロイ
          await kintone.api(
            kintone.api.url('/k/v1/preview/app/form/fields.json', true),
            'PUT',
            { app: appId, properties: props }
          );

          await kintone.api(
            kintone.api.url('/k/v1/preview/app/deploy.json', true),
            'POST',
            { apps: [{ app: appId }], revert: false }
          );

          // デプロイ完了待ち（最大10回 × 3秒）
          await waitForDeploy(appId);
        }

        // 5) 各レコードのプルダウン値を、テキストからコピー
        await updateDropdownValues(appId, records);

        alert('外注先・担当者のプルダウンと値を更新しました。画面を再読み込みします。');
        location.reload();

      } catch (e) {
        console.error(e);
        alert('更新中にエラーが発生しました。詳細はブラウザのコンソールを確認してください。');
      }
    };

    kintone.app.getHeaderMenuSpaceElement().appendChild(btn);
  });

  // 全レコードを取得（500件ずつ）
  async function fetchAllRecords(appId) {
    let allRecords = [];
    let lastId = 0;
    while (true) {
      const query = `$id > ${lastId} order by $id asc limit 500`;
      const resp = await kintone.api(
        kintone.api.url('/k/v1/records.json', true),
        'GET',
        { app: appId, query: query }
      );
      allRecords = allRecords.concat(resp.records);
      if (resp.records.length < 500) break;
      lastId = resp.records[resp.records.length - 1].$id.value;
    }
    return allRecords;
  }

  // デプロイ完了待ち
  async function waitForDeploy(appId) {
    for (let i = 0; i < 10; i++) {
      const statusResp = await kintone.api(
        kintone.api.url('/k/v1/preview/app/deploy/status.json', true),
        'GET',
        { apps: [appId] }
      );
      const status = statusResp.apps[0].status;
      if (status === 'SUCCESS') return;
      await new Promise(resolve => setTimeout(resolve, 3000));
    }
  }

  // 各レコードのプルダウン値をテキストからコピーする
  async function updateDropdownValues(appId, records) {
    const updates = [];

    records.forEach(r => {
      const recUpdate = { id: r.$id.value, record: {} };

      FIELD_PAIRS.forEach(pair => {
        const textVal = (r[pair.text] && r[pair.text].value || '').trim();
        if (textVal) {
          recUpdate.record[pair.dropdown] = { value: textVal };
        }
      });

      // 何か更新があるレコードだけ対象
      if (Object.keys(recUpdate.record).length > 0) {
        updates.push(recUpdate);
      }
    });

    // 100件ずつバッチ更新
    const chunkSize = 100;
    for (let i = 0; i < updates.length; i += chunkSize) {
      const chunk = updates.slice(i, i + chunkSize);
      await kintone.api(
        kintone.api.url('/k/v1/records.json', true),
        'PUT',
        { app: appId, records: chunk }
      );
    }
  }

})();
