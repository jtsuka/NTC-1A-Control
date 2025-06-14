#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NTC-1A MAIN – GUI Terminal
Raspberry Pi ⇆ Seeeduino Nano (UART) ⇆ Arduino Nano Every (TC Emulator)
8-bit binary packet, checksum verified.
※インターバル制御および送信成功後の受信待ちを追加
"""

import tkinter as tk
from tkinter import ttk
from functools import partial
import serial, serial.tools.list_ports
import threading, time

# ---------- 設定 ----------
BAUDRATE = 9600
PORT = "/dev/serial0"  # 必要に応じて変更
INTERVAL = 1.0          # パケット送信インターバル (秒)
LOG_FILE = "serial_log.txt"
font_label = ("Noto Sans CJK JP", 14)
font_btn = ("Noto Sans CJK JP", 18)

# ---------- グローバル ----------
selected_entry = None
entries = {}
running = True
ser = None
selected_channel = 1

# ---------- GUI 初期化 ----------
root = tk.Tk()
root.title("NTC‑1A GUI")
root.geometry("1024x600")
root.configure(bg="black")
# （以下略：元コードと同じ GUI 部分）

# ---------- ログ出力 ----------
def out(msg: str):
    try:
        log.insert(tk.END, msg + "\n")
        log.see(tk.END)
    except tk.TclError:
        pass
    with open(LOG_FILE, "a", encoding="utf-8") as f:
        f.write(msg + "\n")

# ---------- シリアル初期化 ----------
def open_port():
    global ser
    if ser and ser.is_open:
        return ser
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=0.5)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        out(f"[INFO] Port open: {PORT}")
        return ser
    except Exception as e:
        out(f"[エラー] ポート開放失敗: {e}")
        return None

# ---------- パケット生成・送信 ----------
def make_packet(ch, cmd, val):
    data = [ch, cmd, val, 0, 0]
    data.append(sum(data) & 0xFF)
    return bytes(data)

def tx_packet(pkt):
    s = open_port()
    if not s:
        out("[WARN] ポート未接続")
        return False
    try:
        s.write(pkt)
        s.flush()
        out("[TX] " + " ".join(f"{b:02X}" for b in pkt))
        return True
    except Exception as e:
        out(f"[送信エラー] {e}")
        return False

# ---------- 固定長受信 ----------
def read_exact(s, n, timeout=0.5):
    buf = bytearray(n)
    view = memoryview(buf)
    idx = 0
    t0 = time.time()
    while idx < n and time.time() - t0 < timeout:
        got = s.readinto(view[idx:])
        if not got:
            continue
        idx += got
    return buf if idx == n else None

# ---------- 受信スレッド ----------
def rx_worker():
    global running, ser
    while running:
        s = ser
        if s and s.is_open and s.in_waiting >= 1:
            try:
                b = s.read(1)
                if not b:
                    continue
                v = b[0]
                if 0x20 <= v < 0x7F:
                    out(f"[ASCII] {chr(v)}")
                    continue
                # バイナリ応答なら5バイト追加して検証
                rem = read_exact(s, 5, timeout=0.5)
                if not rem:
                    continue
                data = bytes([v]) + rem
                chk = sum(data[:5]) & 0xFF
                ok = (chk == data[5])
                out(f"[RX] CH{data[0]}: " + " ".join(f"{x:02X}" for x in data) + f" CHK:{'OK' if ok else 'NG'}")
            except Exception as e:
                out(f"[受信エラー] {e}")
        else:
            time.sleep(0.05)

threading.Thread(target=rx_worker, daemon=True).start()

# ---------- 送信コマンド処理 送信後にインターバル待機を追加 ----------
def do_cmd(key: str):
    global selected_channel
    out(f"[COMMAND] {key} → CH{selected_channel}")
    cmds = []
    if key == "SEND":
        try:
            t = int(entries[f"ch{selected_channel}_tension"].get())
            cmds.append( (0x06, t) )
        except:
            out("[入力エラー] テンション")
        try:
            l = int(float(entries[f"ch{selected_channel}_length"].get()) * 10)
            cmds.append( (0x04, l) )
        except:
            out("[入力エラー] 線長")
        try:
            c = int(entries[f"ch{selected_channel}_count"].get())
            cmds.append( (0x05, c) )
        except:
            out("[入力エラー] カウント")
    elif key == "RESET":
        cmds.append( (0x03, 0) )
    elif key == "STOP":
        cmds.append( (0x07, 0) )

    # 送信と応答読み取り
    for cmd, val in cmds:
        pkt = make_packet(selected_channel, cmd, val)
        if tx_packet(pkt):
            time.sleep(0.3)  # 各パケット後の応答待ち
    # 最後に全体インターバル
    time.sleep(INTERVAL)

# ---------- GUI キー登録部分は元のまま ----------
# tk.Button(..., command=lambda: do_cmd("SEND")) など、send に紐付け済み

# ---------- 終了・クリーンアップ ----------
def on_close():
    global running
    running = False
    try:
        if ser and ser.is_open:
            ser.close()
    except:
        pass
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)

# ---------- メイン起動 ----------
open_port()
root.mainloop()
