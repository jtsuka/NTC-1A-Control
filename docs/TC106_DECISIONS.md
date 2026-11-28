# TC106 Decisions and Corrections

**Purpose:** Preserve important conclusions and, especially, corrections so an AI or developer does not reintroduce superseded interpretations.

## 2026-11-28 — Wi-Fi: "second 2.4 GHz access point" was the same router

### Previous interpretation

The WIFI SETUP list showed `IODATA-1a4714-2G` stronger than the connected SSID, so another
2.4 GHz access point in the house (possible interference, possible better connection target)
was suspected during the session.

### New evidence

`nmcli` BSSID listing: all entries share the address prefix and differ only in the last byte;
the operator states the router (Buffalo WSR-5400XE6) was given the old I-O DATA SSID for
compatibility. The router's wireless settings page confirmed a migrated-settings SSID.

### Corrected conclusion

All visible SSIDs (including the hidden one) come from one router; there is no second access
point and no sign of a crowded channel. The interference hypothesis is withdrawn. The old
SSID was disabled by the operator on 2026-11-28; the router then re-selected its 2.4 GHz
channel (9 → 5, channel setting is "auto"), so channel fixing is not a priority. The list's dBm
in WIFI SETUP is an approximation from percent; WIFI SCAN's value is the one to compare.

### Remaining unresolved

Day-long stability (`UNRESOLVED.md` D8); whether height or channel contributed more to the
6-7 dB gain.

---

## 2026-11-28 — Pi GUI documentation gap, and what the GUI line has decided

### Previous incomplete state

`TC106-Doc/CURRENT_SYSTEM_STATUS.md` §1 listed Pi GUI v2.4 / Pi communication v1.3.4 as the
only Pi software, and nothing under `docs/` mentioned the GUI beta line, although the tree
already holds v2.6 and v3.6-v3.9.

### Evidence

- Git tree (`main` @ `9085370`, branch `claude/gui-v3-9-wifi-setup`): GUI v2.6, v3.6 (two
  files), v3.7, v3.8, v3.9; serial/log modules v3.6-beta.
- Bench Pi launcher `/home/ntc/start_ntc_gui.sh` starts `ntc_pygame_ui_v3_7_beta.py`
  (operator-supplied shell output, 2026-11-25 / 26).

### Recorded decisions (none changes the Direction-A/B protocol path)

1. **Label:** the GUI shows **TENS ADC**, not "ACTUAL TENSION" (decided 2026-11-23, v3.6
   design document §5). It is a 0..1023 ADC count; do not add a gf unit before calibration.
   Do not reintroduce the old label.
2. **Operator-support features** requested by the operator, all outside the control path:
   REBOOT (Pi only, behind a confirmation dialog), WIFI SCAN (read-only), WIFI SETUP (writes
   only a NetworkManager profile; password never logged). Keep them that way.
3. **Wi-Fi conclusion for the bench Pi:** the earlier drops are attributed mainly to weak
   signal (-77 dBm, `tx failed` 5099 → -41 dBm, 0 after moving) — DERIVED, not final.

### Deviation noticed, not decided

The v3.6 plan said the Pi serial / log / GUI modules bump together. v3.7-v3.9 bumped the GUI
only. Two different `ntc_pygame_ui_v3_6_beta.py` files exist. Both are listed as open items
(`TC106_UNRESOLVED.md` D1, D2); do not assume either was settled.

### Affected documents

`TC106-Doc/CURRENT_SYSTEM_STATUS.md` (§1 Pi rows, new §7), `docs/TC106_UNRESOLVED.md`
(Priority D), `docs/AI_PROJECT_CONTEXT.md` (pointer).

---

## 2026-09-09 — Direction-B topology correction

### Previous incorrect/insufficient interpretation

“Direction-B is one shared return line” was interpreted too broadly, leading to the possibility that Main CH1 and CH2 might not have independent physical receive paths.

### Evidence

- Main Controller PC-1348 circuit diagram shows separate `ch1糸長入力` and `ch2糸長入力` receive circuits/PIC paths.
- Main ASM has independent `rx_ch1` / `Data_110bps_1` and `rx_ch2` / `Data_110bps_2` receive processing.
- Interface-board material has one `TC_TX_REP` return net for the board/group and note `RX CN4のみ結線`.

### Corrected decision

The two statements describe different hierarchy levels:

- CH1 and CH2 are physically independent at the Main Controller.
- Multiple TC106 units **within each CH group** share that group's Direction-B return bus.

Therefore:

```text
CH1 group shared return -> Main CH1 -> Data_110bps_1 -> windlen1_*
CH2 group shared return -> Main CH2 -> Data_110bps_2 -> windlen2_*
```

Do not say “Direction-B is globally one line across CH1 and CH2.”

### Still unresolved

Whether the actual machine physically contains two identical interface-board instances (CH1 and CH2) has not yet been directly verified. Treat this as a strong topology hypothesis pending physical/documentary confirmation.

---

## 2026-09-09 — Received-value display correction

### Superseded interpretation

“The old Main receives and stores actual values but does not use them in normal display.”

### New source finding

The dedicated `disp_ten` / `disp_len` mode setters are unreachable, so those dedicated modes remain dead. However, received wound-length change handling can set `ten_set` and call `Disp_set`; a normal executable display path exists and selects received wound-length data including `windlen2_0` under the relevant state.

### Corrected decision

Use this wording:

> Dedicated actual-tension/wound-length display modes are dead code, but a separate live path exists that can reflect received wound-length data on the 7-segment display.

Do not return to the blanket statement that the old Main never displayed received values.

---

## 2026-09-03 — `tx_enable` classification

Historical `tx_enable` gating was added during communication/noise investigation. It is not currently supported as original TC106 protocol behavior by the primary source hierarchy.

Decision: formal target is **Original Continuous Direction-B**. Keep `tx_enable`-gated behavior classified as experimental/test history unless primary evidence changes this conclusion.

---

## Decision-recording rule

For future corrections include:

1. old interpretation,
2. new evidence,
3. corrected decision,
4. remaining uncertainty,
5. affected implementation/test documents.

## 2026-11-28 — REBOOT: close the evidence log before the reboot command (v3.9.2)

旧解釈: REBOOT後はSIGTERMでGUIが正常終了し、CSVに `STOP` が残る。
新しい証拠: 実機v3.9.1で CSV は `NOTE,REBOOT` で終わり `STOP` なし。journalでは 18:13:22 に
logind が再起動を開始し同時に GUI のセッション(session-1.scope)を停止。`pkill -TERM` 単体では
`PORT close`+`STOP` が書かれる。
訂正後の結論: ハンドラは正常。再起動時はセッション停止が早く終了処理が間に合わない(DERIVED)。
→ 再起動コマンドの前に NOTE→logger.stop→os.sync→0.5 s 待つ(v3.9.2)。拒否時は新ログを開き REBOOT FAILED。
実機結果: v3.9.2 で旧CSVが `NOTE,REBOOT`→`STOP` で終わることを確認(PASS)。
残る未確定: OS がリブートを拒否した場合の実機挙動(擬似環境のみ確認)。

## 2026-11-28 — D7 Pi side: single GUI file name + build id (v3.9.3)

決定: GUI のファイル名は `Python/ntc_pygame_ui.py` の1本にし、動作中のリビジョンは `git describe --tags --always --dirty`
(画面左下・ウィンドウタイトル・ログ START 行の `build=`)で識別する。旧版 v3.6–v3.9.1 は `Python/OLD/gui/` へ移動(履歴は Git に残る)。
タグ規則: `gui-vX.Y.Z-beta`。`-dirty` が付けば未コミット変更あり。
制約: このセッションからはタグを push できない(プロキシが拒否)。タグは Pi か手元の Git から作る。
ESP32 側のバージョン報告は未着手(プロトコル変更を伴うため、仕様書→回帰試験の順)。
実機結果(2026-11-28): 画面左下・ログ START の `build=`・Pi 上の `git describe` が一致(`gui-v3.9.2-beta-2-g87265d4`、-dirty なし)。
Pi の `start_ntc_gui.sh` は `APP="ntc_pygame_ui.py"`。残る未確定: ESP32 側バージョン報告、他の版名ファイルの整理、起動スクリプトの Git 管理。


## 2026-11-28 — tailscale "rebind" at 19:39:00 は Wi-Fi 切替ではなく時計の飛び

旧解釈: 電源断の直前にネットワークが切り替わった可能性。
新しい証拠: 同じ行に `time jump detected (slept 10m28s)`、`rebind-reason=[time-jumped(10m28s)]`。wlan0 の IP は不変、wpa 切断の記録なし。
訂正後の結論: 原因は RTC 無しの Pi が NTP で時刻を進められたこと(DERIVED)。CSV 内の時間差は `mono_s` を使う。
残る未確定: 電源断2回の正確な時刻と原因(バッテリー操作か)、`vcgencmd get_throttled`。

## 2026-11-28 — ESP32 version report: draft spec only (no implementation)

決定: ESP32 ファームのバージョン報告は、Boot Magic の直後に1回だけ送る 12 バイトの VERSION フレーム案(A案)を
`docs/ESP32_VERSION_REPORT_SPEC.md` にドラフトとして記録。旧 Pi では `rx_legacy` として扱われ、テレメトリに影響しない(コードからの導出、DERIVED)。
実装はレビューと回帰計画(同ドキュメント §6)の合意後。残る未確定: バイト配置、ファームのタグ名、Arduino IDE でのビルド時埋め込み方法。

## 2026-11-28 — Pi 起動スクリプトを Git に記録

決定: `/home/ntc/start_ntc_gui.sh` を `Pi/start_ntc_gui.sh` として記録(Git は記録用、実際に動くのは Pi 上のファイル)。秘密情報なしを確認済み。
残る未確定: 起動時にこのスクリプトを呼ぶ仕組み(autostart / .bash_profile / systemd のどれか)。
