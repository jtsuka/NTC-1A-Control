#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# NTC-1A GUI 完全版 - チェックサム/インターバル制御付き
# USBシリアル → Seeeduino Nano → BitBang → TC（エミュレータ）

import tkinter as tk
from tkinter import ttk
from functools import partial
import serial, time, threading

PORT = "/dev/ttyUSB0"  # 適宜変更
BAUD = 9600
SEND_INTERVAL = 0.2    # 200msインターバル
LOG_FILE = "serial_log.txt"

selected_entry = None
selected_channel = 1
entries = {}
ser = None
running = True

root = tk.Tk()
root.title("NTC-1A GUI")
root.geometry("1024x600")
root.configure(bg="black")

font_label = ("Noto Sans CJK JP", 14)
font_button = ("Noto Sans CJK JP", 18)

# ==================== ログ出力 ====================
log = tk.Text(root, height=8, width=60, bg="black", fg="lime", font=("Courier", 12))
log.grid(row=1, column=3, columnspan=4, padx=5, pady=5, sticky="nsew")
for i in range(7): root.grid_columnconfigure(i, weight=1)
for i in range(12): root.grid_rowconfigure(i, weight=1)

def out(msg):
    log.insert(tk.END, msg + "\n")
    log.see(tk.END)
    with open(LOG_FILE, "a") as f:
        f.write(msg + "\n")

# ==================== ポートオープン ====================
def open_port():
    global ser
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        out(f"[INFO] Port open: {PORT}")
    except Exception as e:
        out(f"[エラー] ポート開放失敗: {e}")

# ==================== チャンネル切替 ====================
def toggle_channel():
    global selected_channel
    selected_channel = 1 if selected_channel == 2 else 2
    ch_btn.config(text=f"[CH{selected_channel} 選択中]")
    refresh_entry_colors()

ch_btn = tk.Button(root, text="[CH1 選択中]", font=font_label,
                   bg="darkblue", fg="white", command=toggle_channel)
ch_btn.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky="ew")

# ==================== 入力欄作成 ====================
def create_entry(label, row, key):
    bg = "#003300" if f"ch{selected_channel}" in key else "black"
    tk.Label(root, text=label, font=font_label, bg=bg, fg="white").grid(row=row, column=0, sticky="e", padx=5, pady=5)
    ent = tk.Entry(root, font=font_label, width=10, justify="right", bg=bg, fg="white")
    ent.grid(row=row, column=1, padx=5, pady=5)
    ent.bind("<Button-1>", lambda e, w=ent: set_selected(w))
    entries[key] = ent

def refresh_entry_colors():
    for key, ent in entries.items():
        ch = 1 if "ch1" in key else 2
        color = "#003300" if ch == selected_channel else "black"
        if ent == selected_entry: color = "#00FF00"
        ent.configure(bg=color)

def set_selected(ent):
    global selected_entry
    selected_entry = ent
    refresh_entry_colors()

row = 2
for ch in (1,2):
    create_entry(f"CH{ch} テンション (gf)", row, f"ch{ch}_tension"); row += 1
    create_entry(f"CH{ch} 線長 (m)",       row, f"ch{ch}_length");  row += 1
    create_entry(f"CH{ch} カウント",        row, f"ch{ch}_count");   row += 1

# ==================== テンキー ====================
def handle_key(k):
    global selected_entry
    if k in ("SEND", "RESET", "STOP"): send_command(k); return
    if not selected_entry: return
    if k == "CLR": selected_entry.delete(0, tk.END)
    elif k == "ENTER": selected_entry = None; refresh_entry_colors()
    else: selected_entry.insert(tk.END, k); refresh_entry_colors()

pad = tk.Frame(root, bg="black")
pad.grid(row=2, column=3, columnspan=4, rowspan=6, sticky="nsew")
keys = [["7","8","9","CLR"],["4","5","6","ENTER"],["1","2","3","SEND"],["0","STOP","","RESET"]]
for r,rowv in enumerate(keys):
    for c,k in enumerate(rowv):
        if k:
            b = tk.Button(pad, text=k, font=font_button, width=4, height=2, command=partial(handle_key, k))
            b.grid(row=r, column=c, padx=5, pady=5, sticky="nsew")
for i in range(4): pad.grid_columnconfigure(i, weight=1); pad.grid_rowconfigure(i, weight=1)

# ==================== パケット送受 ====================
def make_packet(ch, cmd, val):
    pkt = [ch, cmd, val, 0x00, 0x00]
    pkt.append(sum(pkt) & 0xFF)
    return pkt

def send_packet(pkt):
    try:
        ser.write(bytes(pkt))
        ser.flush()
        hexs = ' '.join(f"0x{x:02X}" for x in pkt)
        out(f"[送信 CH{pkt[0]}] {hexs}")
        time.sleep(SEND_INTERVAL)
    except Exception as e:
        out(f"[送信失敗] {e}")

# ==================== コマンド実行 ====================
def send_command(cmd):
    ch = selected_channel
    out(f"[COMMAND] {cmd} → CH{ch}")
    try:
        if cmd == "SEND":
            t = int(entries[f"ch{ch}_tension"].get())
            l = int(float(entries[f"ch{ch}_length"].get()) * 10)
            c = int(entries[f"ch{ch}_count"].get())
            send_packet(make_packet(ch, 0x06, t))
            send_packet(make_packet(ch, 0x04, l))
            send_packet(make_packet(ch, 0x05, c))
        elif cmd == "RESET":
            send_packet(make_packet(ch, 0x03, 0))
        elif cmd == "STOP":
            send_packet(make_packet(ch, 0x07, 0))
    except Exception as e:
        out(f"[エラー] 入力値不正: {e}")

# ==================== 受信ループ ====================
def receive_loop():
    buf = bytearray()
    global running
    while running:
        if ser.in_waiting:
            b = ser.read(1)
            if b: buf += b
        if len(buf) >= 6:
            pkt = list(buf[:6])
            buf = buf[6:]
            chk = sum(pkt[:5]) & 0xFF
            valid = chk == pkt[5]
            hexs = ' '.join(f"0x{x:02X}" for x in pkt)
            out(f"[TC応答 CH{pkt[0]}] {hexs} CHK:{'OK' if valid else 'NG'}")
        time.sleep(0.01)

# ==================== アプリ終了 ====================
def on_close():
    global running
    running = False
    if ser: ser.close()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)

# ==================== 実行 ====================
open_port()
threading.Thread(target=receive_loop, daemon=True).start()
root.mainloop()