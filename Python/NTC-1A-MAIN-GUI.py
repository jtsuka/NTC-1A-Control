#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#  NTC-1A GUI  – 8-bit CHK  / 6-byte fixed packet

import tkinter as tk
from tkinter import ttk
from functools import partial
import serial, serial.tools.list_ports
import threading, time, sys

# ---------- 定数 ----------
BAUDRATE   = 9600
LOG_FILE   = "serial_log.txt"
font_lbl   = ("Noto Sans CJK JP", 14)
font_btn   = ("Noto Sans CJK JP", 18)

# ---------- GUI 基本 ----------
root = tk.Tk()
root.title("NTC-1A タッチパネル操作")
root.geometry("1024x600")
root.configure(bg="black")
root.bind("<Escape>", lambda e: root.destroy())

# ---------- ポート選択 ----------
port_var = tk.StringVar()
cmb = ttk.Combobox(root, textvariable=port_var, state="readonly")
cmb.grid(row=0, column=0, padx=5, pady=5, sticky="w")
def update_ports():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    cmb["values"] = ports
    if ports: port_var.set(ports[0])
root.after(0, update_ports)
ttk.Button(root,text="更新",command=update_ports).grid(row=0,column=1)

# ---------- ログ ----------
log = tk.Text(root, height=10, width=80, bg="black", fg="lime",
              font=("Courier",12))
log.grid(row=1, column=0, columnspan=6, padx=5, pady=5, sticky="nsew")
def out(msg):
    log.insert(tk.END, msg+"\n"); log.see(tk.END)
    open(LOG_FILE,"a",encoding="utf-8").write(msg+"\n")

# ---------- パケット生成 (8-bit CHK) ----------
def create_packet(ch, cmd, val):
    pkt = [ch, cmd, val, 0x00, 0x00]
    checksum = sum(pkt) & 0xFF          # ★ 8-bit に統一
    pkt.append(checksum)
    return pkt

# ---------- シリアル送信 ----------
ser = None
def open_port():
    global ser
    if ser and ser.is_open: return ser
    try:
        ser = serial.Serial(port_var.get(), BAUDRATE, timeout=1)
        ser.reset_input_buffer(); ser.reset_output_buffer()
        return ser
    except Exception as e:
        out(f"[ERR] Open port: {e}")
        return None

def send_packet(pkt):
    s = open_port()
    if not s: return
    try:
        s.write(bytes(pkt)); s.flush()        # ← flush 追加
        out("[SEND] " + ' '.join(f"{b:02X}" for b in pkt))
    except Exception as e:
        out(f"[ERR] Send: {e}")

# ---------- 受信スレッド ----------
def rx_thread():
    buf = bytearray()
    while True:
        if ser and ser.is_open:
            b = ser.read(1)
            if b: buf.append(b[0])
            while len(buf) >= 6:
                pkt = list(buf[:6]); del buf[:6]
                chk = sum(pkt[:5]) & 0xFF
                ok  = (chk == pkt[5])
                out("[RECV] " + ' '.join(f"{x:02X}" for x in pkt) +
                    f" CHK:{'OK' if ok else 'NG'}")
        time.sleep(0.01)
threading.Thread(target=rx_thread, daemon=True).start()

# ---------- 簡易送信ボタン（テスト用） ----------
def send_test():
    pkt = create_packet(0x01, 0x06, 0x05)
    send_packet(pkt)
tk.Button(root,text="TEST SEND",font=font_btn,
          command=send_test).grid(row=2,column=0,padx=10,pady=10)

root.mainloop()
