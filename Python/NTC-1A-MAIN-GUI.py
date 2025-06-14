#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#**********************************************************************
#  NTC-1A MAIN – Python3 + Tkinter GUI (Raspberry Pi)
#  UART: /dev/serial0 (9600bps) ⇄ Seeeduino Nano ⇄ Bit-Banging TC
#  ▸ チャンネル選択, テンション／線長／カウント設定
#  ▸ 6バイト固定長パケット (CH, CMD, VAL, 0x00, 0x00, CHK)
#  ▸ CHK = 下位5バイトの合計 (8bit)
#  ▸ OLEDログ表示 (Seeeduino経由) 対応済
#**********************************************************************

import tkinter as tk
from tkinter import ttk
from functools import partial
import serial, threading, time

# ==================== 設定 ====================
PORT = "/dev/serial0"
BAUD = 9600
LOG_FILE = "serial_log.txt"

# ==================== グローバル変数 ====================
selected_entry = None
selected_channel = 1
entries = {}
ser = None
running = True

# ==================== GUI初期化 ====================
root = tk.Tk()
root.title("NTC-1A タッチパネル操作")
root.geometry("1024x600")
root.configure(bg="black")
font_label = ("Noto Sans CJK JP", 14)
font_button = ("Noto Sans CJK JP", 18)

# ==================== チャンネル切替 ====================
def toggle_channel():
    global selected_channel
    selected_channel = 2 if selected_channel == 1 else 1
    ch_button.config(text=f"[CH{selected_channel} 設定中]")
    refresh_entry_colors()

ch_button = tk.Button(root, text="[CH1 設定中]", font=font_label,
                      bg="darkblue", fg="white", command=toggle_channel)
ch_button.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky="ew")

# ==================== ログ表示エリア ====================
log_text = tk.Text(root, height=8, width=60, bg="black", fg="lime", font=("Courier", 12))
log_text.grid(row=1, column=3, columnspan=4, padx=5, pady=5, sticky="nsew")

def append_log(msg):
    log_text.insert(tk.END, msg + "\n")
    log_text.see(tk.END)
    with open(LOG_FILE, "a") as f:
        f.write(msg + "\n")

# ==================== エントリー入力欄 ====================
def create_entry(label_text, row, key):
    label = tk.Label(root, text=label_text, font=font_label, bg="black", fg="white")
    label.grid(row=row, column=0, sticky="e", padx=5, pady=5)
    entry = tk.Entry(root, font=font_label, width=10, justify="right", bg="black", fg="white")
    entry.grid(row=row, column=1, padx=5, pady=5)
    entry.bind("<Button-1>", lambda e: set_selected(entry))
    entries[key] = entry

def set_selected(entry):
    global selected_entry
    selected_entry = entry
    refresh_entry_colors()

def refresh_entry_colors():
    for key, entry in entries.items():
        ch = 1 if "ch1" in key else 2
        color = "#003300" if ch == selected_channel else "black"
        if entry == selected_entry:
            entry.configure(bg="#00FF00")
        else:
            entry.configure(bg=color)

row = 2
for ch in (1, 2):
    create_entry(f"CH{ch} テンション(gf)", row, f"ch{ch}_tension"); row += 1
    create_entry(f"CH{ch} 線長(m)",       row, f"ch{ch}_length");  row += 1
    create_entry(f"CH{ch} カウント",      row, f"ch{ch}_count");   row += 1

# ==================== テンキー ====================
def handle_key(k):
    global selected_entry
    if k in ["SEND", "STOP", "RESET"]:
        handle_command(k)
        return
    if not selected_entry:
        return
    if k == "CLR":
        selected_entry.delete(0, tk.END)
    elif k == "ENTER":
        selected_entry = None
        refresh_entry_colors()
    else:
        selected_entry.insert(tk.END, k)

keys = [
    ["7","8","9","CLR"],
    ["4","5","6","ENTER"],
    ["1","2","3","SEND"],
    ["0","STOP","","RESET"]
]

pad_frame = tk.Frame(root, bg="black")
pad_frame.grid(row=2, column=3, columnspan=4, rowspan=6, sticky="nsew")
for r, row_keys in enumerate(keys):
    for c, key in enumerate(row_keys):
        if key:
            b = tk.Button(pad_frame, text=key, font=font_button, width=4, height=2,
                          command=partial(handle_key, key))
            b.grid(row=r, column=c, padx=5, pady=5, sticky="nsew")

# ==================== パケット送信 ====================
def create_packet(ch, cmd, val):
    packet = [ch, cmd, val, 0x00, 0x00]
    chk = sum(packet) & 0xFF
    packet.append(chk)
    return packet

def send_packet(packet):
    global ser
    try:
        if not ser or not ser.is_open:
            ser = serial.Serial(PORT, BAUD, timeout=1)
        ser.write(bytes(packet))
        ser.flush()
        append_log("[送信 CH{}] {}".format(packet[0], " ".join(f"{b:02X}" for b in packet)))
    except Exception as e:
        append_log(f"[送信エラー] {e}")

# ==================== 受信スレッド ====================
def read_serial_loop():
    global ser, running
    buffer = bytearray()
    while running:
        try:
            if ser and ser.is_open:
                byte = ser.read(1)
                if byte:
                    buffer.append(byte[0])
                while len(buffer) >= 6:
                    pkt = buffer[:6]
                    buffer = buffer[6:]
                    chk = sum(pkt[:5]) & 0xFF
                    valid = chk == pkt[5]
                    append_log("[TC応答 CH{}] {} CHK:{}".format(
                        pkt[0], " ".join(f"{b:02X}" for b in pkt), "OK" if valid else "NG"))
        except Exception as e:
            append_log(f"[受信エラー] {e}")
        time.sleep(0.01)

threading.Thread(target=read_serial_loop, daemon=True).start()

# ==================== コマンド処理 ====================
def handle_command(cmd):
    ch = selected_channel
    append_log(f"[COMMAND] {cmd} → CH{ch}")
    try:
        if cmd == "SEND":
            t = int(entries[f"ch{ch}_tension"].get())
            l = int(float(entries[f"ch{ch}_length"].get()) * 10)
            c = int(entries[f"ch{ch}_count"].get())
            send_packet(create_packet(ch, 0x06, t))
            send_packet(create_packet(ch, 0x04, l))
            send_packet(create_packet(ch, 0x05, c))
        elif cmd == "RESET":
            send_packet(create_packet(ch, 0x03, 0))
        elif cmd == "STOP":
            send_packet(create_packet(ch, 0x07, 0))
    except ValueError:
        append_log("[入力エラー] 数値が不正です")

# ==================== 終了処理 ====================
def on_close():
    global running, ser
    running = False
    if ser and ser.is_open:
        ser.close()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()