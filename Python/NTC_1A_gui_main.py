# gui_main.py
# NTC-1A GUI – 完全版
import tkinter as tk
from tkinter import ttk, messagebox
from NTC_1A_serial_comm import start_serial_thread, send_packet, stop_serial
from NTC_1A_utils import out, setup_log_display, make_labeled_entry, make_keypad

selected_channel = 1
selected_entry = None
entries = {}

def toggle_channel():
    global selected_channel
    selected_channel = 1 if selected_channel == 2 else 2
    ch_btn.config(text=f"[CH{selected_channel} 選択中]")
    refresh_colors()

def refresh_colors():
    for k, e in entries.items():
        ch = 1 if "ch1" in k else 2
        e.configure(bg="#00FF00" if e is selected_entry
                    else "#003300" if ch == selected_channel else "black")

def entry_selected(e):
    global selected_entry
    selected_entry = e
    refresh_colors()

def on_keypad_press(k):
    global selected_entry
    if k in ("SEND", "RESET", "STOP"):
        handle_command(k)
        return
    if not selected_entry:
        return
    if k == "CLR":
        selected_entry.delete(0, tk.END)
    elif k == "ENTER":
        selected_entry = None
    else:
        selected_entry.insert(tk.END, k)
    refresh_colors()

def handle_command(cmd):
    ch = selected_channel
    out(f"[COMMAND] {cmd} sent to CH{ch}")
    try:
        if cmd == "SEND":
            t = int(entries[f"ch{ch}_tension"].get())
            l = int(float(entries[f"ch{ch}_length"].get()) * 10)
            c = int(entries[f"ch{ch}_count"].get())
            send_packet(ch, 0x06, t)
            send_packet(ch, 0x04, l)
            send_packet(ch, 0x05, c)
        elif cmd == "RESET":
            send_packet(ch, 0x03, 0)
        elif cmd == "STOP":
            send_packet(ch, 0x07, 0)
    except ValueError:
        out("[ERROR] 数値入力エラー")

# GUI 初期化
root = tk.Tk()
root.title("NTC-1A タッチパネル操作")
root.geometry("1024x600")
root.configure(bg="black")

# ログ
log = setup_log_display(root)

# CHボタン
ch_btn = tk.Button(root, text="[CH1 選択中]", font=("Arial", 14),
                   bg="darkblue", fg="white", command=toggle_channel)
ch_btn.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky="ew")

# 入力フィールド
row = 2
for ch in (1, 2):
    make_labeled_entry(root, f"CH{ch} テンション(gf)", row, f"ch{ch}_tension", entries, entry_selected)
    row += 1
    make_labeled_entry(root, f"CH{ch} 線長(m)", row, f"ch{ch}_length", entries, entry_selected)
    row += 1
    make_labeled_entry(root, f"CH{ch} カウント", row, f"ch{ch}_count", entries, entry_selected)
    row += 1

# テンキー
make_keypad(root, on_keypad_press)

# シリアル受信スレッド起動
start_serial_thread()

def on_close():
    stop_serial()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()
