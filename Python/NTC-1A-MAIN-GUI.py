#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#  NTC-1A GUI  (8-bit checksum / 6-byte fixed packet)

import tkinter as tk
from tkinter import ttk
from functools import partial
import serial, serial.tools.list_ports
import threading, time, sys, os

# ------------------ 設定 ------------------
BAUDRATE   = 9600
LOG_FILE   = "serial_log.txt"
font_lbl   = ("Noto Sans CJK JP", 14)
font_btn   = ("Noto Sans CJK JP", 18)

# ------------------ グローバル ------------------
entries, selected_entry = {}, None
selected_channel = 1
ser, running = None, True

# ------------------ GUI 基本 ------------------
root = tk.Tk()
root.title("NTC-1A タッチパネル操作")
root.geometry("1024x600")
root.configure(bg="black")
root.bind("<Escape>", lambda e: root.destroy())

# ------------------ ポート選択 ------------------
port_var = tk.StringVar()
cb_port = ttk.Combobox(root, textvariable=port_var, state="readonly", width=15)
cb_port.grid(row=0, column=0, padx=5, pady=5, sticky="w")
def update_ports():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    cb_port["values"] = ports
    if ports: port_var.set(ports[0])
ttk.Button(root,text="更新",command=update_ports).grid(row=0,column=1,padx=5)
update_ports()

# ------------------ ログ ------------------
log = tk.Text(root, height=10, width=80, bg="black", fg="lime",
              font=("Courier",12))
log.grid(row=1, column=0, columnspan=6, padx=5, pady=5, sticky="nsew")
def out(msg:str):
    log.insert(tk.END, msg+"\n"); log.see(tk.END)
    with open(LOG_FILE,"a",encoding="utf-8") as f: f.write(msg+"\n")

# ～～～ log テキストウィジェットを配置した直後に追記 ～～
for i in range(6):          # 列 0-5 を可変に
    root.grid_columnconfigure(i, weight=1)
for i in range(10):         # 行 0-9 を可変に
    root.grid_rowconfigure(i, weight=1)


# ------------------ CHトグル ------------------
def toggle_ch():
    global selected_channel
    selected_channel = 1 if selected_channel==2 else 2
    btn_ch.configure(text=f"[CH{selected_channel} 設定中]")
    refresh_entry_colors()
btn_ch = tk.Button(root,text="[CH1 設定中]",font=font_lbl,
                   bg="darkblue", fg="white", command=toggle_ch)
btn_ch.grid(row=2,column=0,columnspan=2,sticky="ew", padx=5, pady=5)

# ------------------ 入力欄 ------------------
def set_selected(ent):  # クリック選択
    global selected_entry
    selected_entry = ent
    refresh_entry_colors()

def refresh_entry_colors():
    for key,ent in entries.items():
        ch = 1 if "ch1" in key else 2
        ent.configure(bg = "#00FF00" if ent is selected_entry
                      else "#003300" if ch==selected_channel else "black")

def make_entry(lbl,row,key):
    bg = "#003300" if f"ch{selected_channel}" in key else "black"
    tk.Label(root,text=lbl,font=font_lbl,bg=bg,fg="white")\
      .grid(row=row,column=0,sticky="e",padx=5,pady=3)
    ent = tk.Entry(root,font=font_lbl,width=10,justify="right",
                   bg=bg,fg="white")
    ent.grid(row=row,column=1,padx=5,pady=3)
    ent.bind("<Button-1>", lambda e,ent=ent: set_selected(ent))
    entries[key]=ent

row=3
for ch in (1,2):
    make_entry(f"CH{ch} テンション(gf)",row,f"ch{ch}_tension"); row+=1
    make_entry(f"CH{ch} 線長(m)",      row,f"ch{ch}_length");   row+=1
    make_entry(f"CH{ch} カウント(m)",  row,f"ch{ch}_count");    row+=1

# ------------------ テンキー ------------------
def key_press(k):
    global selected_entry
    if k in ("SEND","RESET","STOP"):
        do_command(k); return
    if not selected_entry: return
    if k=="CLR": selected_entry.delete(0,tk.END)
    elif k=="ENTER": selected_entry=None; refresh_entry_colors()
    else: selected_entry.insert(tk.END,k); refresh_entry_colors()

keys=[["7","8","9","CLR"],["4","5","6","ENTER"],
      ["1","2","3","SEND"],["0","STOP","","RESET"]]
pad=tk.Frame(root,bg="black")
pad.grid(row=3,column=3,columnspan=3,rowspan=6,sticky="nsew")
for r,rowk in enumerate(keys):
    for c,k in enumerate(rowk):
        if not k: continue
        tk.Button(pad,text=k,font=font_btn,width=5,height=2,
                  command=partial(key_press,k))\
                  .grid(row=r,column=c,padx=5,pady=5,sticky="nsew")
for i in range(4): pad.grid_columnconfigure(i,weight=1)
for i in range(4): pad.grid_rowconfigure(i,weight=1)

# ------------------ パケット (8-bit CHK) ------------------
def create_packet(ch, cmd, val):
    pkt=[ch,cmd,val,0,0]; pkt.append(sum(pkt)&0xFF); return pkt

def open_port():
    global ser
    if ser and ser.is_open: return ser
    try:
        ser = serial.Serial(port_var.get(), BAUDRATE, timeout=1)
        ser.reset_input_buffer(); ser.reset_output_buffer()
        return ser
    except Exception as e:
        out(f"[ERR] Port open: {e}")
        return None

def send_packet(pkt):
    s=open_port()
    if not s: return
    try:
        s.write(bytes(pkt)); s.flush()
        out("[TX] "+" ".join(f"{b:02X}" for b in pkt))
    except Exception as e:
        out(f"[ERR] Send: {e}")

# ------------------ コマンド ------------------
def do_command(name):
    ch = selected_channel
    out(f"[CMD] {name} → CH{ch}")
    try:
        if name=="SEND":
            t = int(entries[f"ch{ch}_tension"].get())
            l = int(float(entries[f"ch{ch}_length"].get())*10)
            c = int(entries[f"ch{ch}_count"].get())
            send_packet(create_packet(ch,0x06,t))
            send_packet(create_packet(ch,0x04,l))
            send_packet(create_packet(ch,0x05,c))
        elif name=="RESET":
            send_packet(create_packet(ch,0x03,0))
        elif name=="STOP":
            send_packet(create_packet(ch,0x07,0))
    except ValueError:
        out("[ERR] 数値入力")

# ------------------ 受信スレッド ------------------
def read_exact(s,n,timeout=0.5):
    buf = bytearray(n); mv=memoryview(buf); idx=0; t0=time.time()
    while idx<n and time.time()-t0<timeout:
        idx += s.readinto(mv[idx:])
    return buf if idx==n else None

def rx_worker():
    global running,ser
    while running:
        if ser and ser.is_open:
            try:
                b = ser.read(1)
                if not b: continue
                data = bytearray([b[0]]) + read_exact(ser,5,0.3)
                if len(data)!=6: continue
                chk = sum(data[:5]) & 0xFF
                ok  = (chk == data[5])
                out("[RX] "+" ".join(f\"{x:02X}\" for x in data)+
                    f\" CHK:{'OK' if ok else 'NG'}\")
            except Exception as e:
                out(f\"[ERR] RX: {e}\")
        else:
            time.sleep(0.05)
threading.Thread(target=rx_worker,daemon=True).start()

# ------------------ 終了処理 ------------------
def on_close():
    global running
    running=False
    if ser and ser.is_open: ser.close()
    root.destroy()
root.protocol(\"WM_DELETE_WINDOW\", lambda _: on_close())

root.mainloop()
