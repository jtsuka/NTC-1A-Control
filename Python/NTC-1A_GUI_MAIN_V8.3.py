# NTC-1A_GUI_MAIN_V8.3.py
import tkinter as tk
from tkinter import ttk, messagebox
from serial.tools import list_ports
from NTC_1A_serial_comm import start_serial_thread, send_packet, stop_serial, set_timeout
from NTC_1A_utils import out, setup_log_display

root = tk.Tk()
root.title("NTC-1A タッチパネル操作")
root.geometry("1024x600")
root.configure(bg="black")

font_label = ("Arial", 12)
font_entry = ("Arial", 14)
font_button = ("Arial", 12)

selected_channel = 1
selected_entry = None
entries = {}

# ------------------- トップバー -------------------
top_frame = tk.Frame(root, bg="black")
top_frame.pack(fill="x")

tk.Label(top_frame, text="Port", bg="black", fg="white").pack(side="left", padx=(5, 0))
port_combobox = ttk.Combobox(top_frame, width=10)
port_combobox.pack(side="left")

ports = [port.device for port in list_ports.comports()]
if ports:
    port_combobox['values'] = ports
    port_combobox.set(ports[0])
    start_serial_thread(ports[0])
port_combobox.bind("<<ComboboxSelected>>", lambda e: start_serial_thread(port_combobox.get()))

tk.Label(top_frame, text=" Timeout(sec)", bg="black", fg="white").pack(side="left")
timeout_entry = tk.Entry(top_frame, width=5, font=font_entry, justify="center")
timeout_entry.insert(0, "2.0")
timeout_entry.pack(side="left")
tk.Button(top_frame, text="Set", command=lambda: set_timeout(float(timeout_entry.get()))).pack(side="left", padx=5)

ch_btn = tk.Button(top_frame, text="[CH1 選択中]", bg="#003366", fg="white", command=lambda: toggle_channel())
ch_btn.pack(side="left")

# ------------------- ログ表示 -------------------
log = setup_log_display(root)
out("> GUI 起動完了 - V8.3")

# ------------------- 入力・テンキー -------------------
middle_frame = tk.Frame(root, bg="black")
middle_frame.pack(fill="both", expand=True)
middle_frame.grid_columnconfigure(0, weight=2)
middle_frame.grid_columnconfigure(1, weight=2)
middle_frame.grid_rowconfigure(0, weight=1)

left_frame = tk.Frame(middle_frame, bg="black")
left_frame.grid(row=0, column=0, sticky="ns")

keypad_frame = tk.Frame(middle_frame, bg="black")
keypad_frame.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)

def make_labeled_entry(label, varname, row):
    tk.Label(left_frame, text=label, fg="green", bg="black", font=font_label).grid(row=row, column=0, sticky="w")
    ent = tk.Entry(left_frame, font=font_entry, width=10, bg="white",
                   highlightthickness=2, highlightbackground="black", highlightcolor="black",
                   justify="right")  # 数値右寄せ
    ent.grid(row=row, column=1, pady=2)
    entries[varname] = ent
    ent.bind("<FocusIn>", lambda e: entry_selected(ent))

row = 0
for ch in (1, 2):
    make_labeled_entry(f"CH{ch} 張力(gf)", f"ch{ch}_tension", row); row += 1
    make_labeled_entry(f"CH{ch} 長さ(m)", f"ch{ch}_length", row); row += 1
    make_labeled_entry(f"CH{ch} カウント(m)", f"ch{ch}_count", row); row += 1

# ------------------- テンキー -------------------
key_labels = [
    ("7", 0, 0), ("8", 0, 1), ("9", 0, 2), ("CLR", 0, 3),
    ("4", 1, 0), ("5", 1, 1), ("6", 1, 2), ("ENTER", 1, 3),
    ("1", 2, 0), ("2", 2, 1), ("3", 2, 2), ("SEND", 2, 3),
    ("0", 3, 0), ("STOP", 3, 1), ("RESET", 3, 2)
]

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
        refresh_colors()
    else:
        selected_entry.insert(tk.END, k)

def entry_selected(e):
    global selected_entry
    selected_entry = e
    refresh_colors()

def refresh_colors():
    group_prefix = f"ch{selected_channel}_"
    for key, ent in entries.items():
        if key.startswith(group_prefix):
            if ent == selected_entry:
                ent.configure(bg="#00FF00", highlightbackground="yellow", highlightcolor="yellow")
            else:
                ent.configure(bg="#cceeff", highlightbackground="black", highlightcolor="black")
        else:
            if ent == selected_entry:
                ent.configure(bg="#00FF00", highlightbackground="gray", highlightcolor="gray")
            else:
                ent.configure(bg="white", highlightbackground="black", highlightcolor="black")

def toggle_channel():
    global selected_channel
    selected_channel = 1 if selected_channel == 2 else 2
    ch_btn.config(
        text=f"[CH{selected_channel} 選択中]",
        bg="#003366" if selected_channel == 1 else "#006600"
    )
    refresh_colors()

def handle_command(cmd):
    ch = selected_channel
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
    except Exception as e:
        out(f"[ERROR] コマンド送信失敗: {e}")

# テンキーグリッド設定
for i in range(4):
    keypad_frame.columnconfigure(i, weight=1, uniform="equal")
    keypad_frame.rowconfigure(i, weight=1, uniform="equal")

for label, r, c in key_labels:
    b = tk.Button(keypad_frame, text=label, font=font_button, command=lambda k=label: on_keypad_press(k))
    b.grid(row=r, column=c, sticky="nsew", padx=2, pady=2)

# 終了時処理
def on_close():
    stop_serial()
    root.quit()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()
