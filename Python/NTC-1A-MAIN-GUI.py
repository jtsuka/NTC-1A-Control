#!/usr/bin/env python3 + tkinter
# -*- coding: utf-8 -*-
#  NTC-1A GUI  (ASCII + 6-byte Binary / 固定長 read_exact 方式)

import tkinter as tk
from tkinter import ttk
from functools import partial
import serial, serial.tools.list_ports
import threading, time

# ---------- 設定 ----------
BAUDRATE   = 9600
LOG_FILE   = "serial_log.txt"
font_label = ("Noto Sans CJK JP", 14)
font_btn   = ("Noto Sans CJK JP", 18)

# ---------- グローバル ----------
selected_entry   = None
entries          = {}
running          = True
ser              = None
selected_channel = 1   # CH1 デフォルト

# ---------- GUI ----------
root = tk.Tk()
root.title("NTC-1A タッチパネル操作")
root.geometry("1024x600")
root.configure(bg="black")

# フルスクリーン
root.bind("<F11>", lambda e: root.attributes("-fullscreen",
                not root.attributes("-fullscreen")))
root.bind("<Escape>", lambda e: root.attributes("-fullscreen", False))
tk.Button(root, text="[] 全画面", font=("Arial", 10),
          command=lambda: root.event_generate("<F11>"))          .grid(row=0, column=6, sticky="ne", padx=5, pady=5)
tk.Button(root, text="X 通常", font=("Arial", 10),
          command=lambda: root.event_generate("<Escape>"))          .grid(row=0, column=5, sticky="ne", padx=5, pady=5)

# CH 切替
def tog_ch():
    global selected_channel
    selected_channel = 1 if selected_channel==2 else 2
    ch_btn.config(text=f"[CH{selected_channel} 選択中]")
    refresh_colors()
ch_btn = tk.Button(root, text="[CH1 選択中]", font=font_label,
                   bg="darkblue", fg="white", command=tog_ch)
ch_btn.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky="ew")

# ログ表示
log = tk.Text(root, height=8, width=60, bg="black",
              fg="lime", font=("Courier", 12))
log.grid(row=1, column=3, columnspan=4, padx=5, pady=5, sticky="nsew")
for i in range(7):  root.grid_columnconfigure(i, weight=1)
for i in range(12): root.grid_rowconfigure(i, weight=1)
def out(msg:str):
    try: log.insert(tk.END, msg+"\n"); log.see(tk.END)
    except tk.TclError: pass
    with open(LOG_FILE, "a", encoding="utf-8") as f: f.write(msg+"\n")

# 入力欄
def sel(e):  # 選択ハイライト
    global selected_entry
    selected_entry=e; refresh_colors()
def refresh_colors():
    for k,e in entries.items():
        ch=1 if "ch1" in k else 2
        e.configure(bg="#00FF00" if e is selected_entry
                    else "#003300" if ch==selected_channel else "black")
def make_entry(label,row,key):
    bg="#003300" if f"ch{selected_channel}" in key else "black"
    tk.Label(root,text=label,font=font_label,bg=bg,fg="white")      .grid(row=row,column=0,sticky="e",padx=5,pady=5)
    ent=tk.Entry(root,font=font_label,width=10,justify="right",bg=bg,fg="white")
    ent.grid(row=row,column=1,padx=5,pady=5)
    ent.bind("<Button-1>", lambda _,e=ent: sel(e))
    entries[key]=ent
row=2
for ch in (1,2):
    make_entry(f"CH{ch} テンション(gf)",row,f"ch{ch}_tension"); row+=1
    make_entry(f"CH{ch} 線長(m)",      row,f"ch{ch}_length");   row+=1
    make_entry(f"CH{ch} カウント",     row,f"ch{ch}_count");    row+=1

# テンキー
def key_press(k):
    global selected_entry
    if k in ("SEND","RESET","STOP"): do_cmd(k); return
    if not selected_entry: return
    if k=="CLR":   selected_entry.delete(0,tk.END)
    elif k=="ENTER": selected_entry=None; refresh_colors()
    else:          selected_entry.insert(tk.END,k); refresh_colors()

keys=[["7","8","9","CLR"],["4","5","6","ENTER"],
      ["1","2","3","SEND"],["0","STOP","","RESET"]]
pad=tk.Frame(root,bg="black")
pad.grid(row=2,column=3,columnspan=4,rowspan=6,sticky="nsew")
for r,row_k in enumerate(keys):
    for c,k in enumerate(row_k):
        if k:
            tk.Button(pad,text=k,font=font_btn,width=4,height=2,
                      command=partial(key_press,k))                      .grid(row=r,column=c,padx=5,pady=5,sticky="nsew")
for i in range(4):
    pad.grid_columnconfigure(i,weight=1)
    pad.grid_rowconfigure(i,weight=1)

# パケット送受
def pkt(ch,cmd,val):
    data=[ch,cmd,val,0,0]; data.append(sum(data)&0xFF); return data

def open_port():
    global ser
    if ser and ser.is_open:
        return ser
    try:
        ser = serial.Serial('/dev/serial0', BAUDRATE, timeout=1)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        return ser
    except Exception as e:
        out(f"[エラー] ポート失敗: {e}")
        return None

def tx(data):
    s = open_port()
    if not s:
        out("[送信スキップ] ポート未接続")
        return False
    try:
        s.write(bytes(data))
        s.flush()
        out("[送信 CH{}] {}".format(data[0], ' '.join(f"{b:02X}" for b in data)))
        return True
    except Exception as e:
        out(f"[送信エラー] {e}")
        return False

def read_exact(s,n,timeout=0.5):
    buf=bytearray(n); mv=memoryview(buf); idx=0; t0=time.time()
    while idx<n and time.time()-t0<timeout:
        idx+=s.readinto(mv[idx:])
    return buf if idx==n else None

def rx_worker():
    global running,ser
    while running:
        if ser and ser.is_open:
            try:
                b=ser.read(1)
                if not b: continue
                v=b[0]
                if 0x20<=v<0x7F:
                    out(f"[ASCII] {chr(v)}"); continue
                data=bytearray([v])
                rem=read_exact(ser,5,0.5)
                if not rem: continue
                data.extend(rem)
                chk=sum(data[:5])&0xFF
                ok = chk==data[5]
                out("[TC応答 CH{}] {} CHK:{}".format(
                    data[0],' '.join(f"{x:02X}" for x in data),
                    "OK" if ok else "NG"))
            except Exception as e:
                out(f"[受信エラー] {e}")
        else:
            time.sleep(0.05)
threading.Thread(target=rx_worker,daemon=True).start()

def do_cmd(cmd):
    ch = selected_channel
    out(f"[COMMAND受信] {cmd} が押されました → CH{ch}")
    try:
        if cmd == "SEND":
            t = int(entries[f"ch{ch}_tension"].get())
            l = int(float(entries[f"ch{ch}_length"].get()) * 10)
            c = int(entries[f"ch{ch}_count"].get())
            if not tx(pkt(ch, 0x06, t)):
                out("[送信失敗] テンション値")
            if not tx(pkt(ch, 0x04, l)):
                out("[送信失敗] 線長値")
            if not tx(pkt(ch, 0x05, c)):
                out("[送信失敗] カウント値")
        elif cmd == "RESET":
            if not tx(pkt(ch, 0x03, 0)):
                out("[送信失敗] リセット")
        elif cmd == "STOP":
            if not tx(pkt(ch, 0x07, 0)):
                out("[送信失敗] ストップ")
    except ValueError:
        out("[入力エラー] 数値が正しくありません")

def cleanup():
    global running
    running=False
    try:
        if ser and ser.is_open: ser.close()
    except: pass
    root.destroy()
root.protocol("WM_DELETE_WINDOW", lambda _: cleanup())
root.mainloop()

