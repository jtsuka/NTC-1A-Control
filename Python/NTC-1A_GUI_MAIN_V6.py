# Next_Main_GUI_v6.py
# NTC-1A GUI – 改良統合版（UI美化＋機能統合）
import tkinter as tk
from tkinter import ttk, messagebox
from serial.tools import list_ports
from NTC_1A_serial_comm import start_serial_thread, send_packet, stop_serial, set_timeout
from NTC_1A_utils import out, setup_log_display, make_keypad

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

def get_serial_ports():
    return [port.device for port in list_ports.comports()]

def on_port_selected(event):
    selected = port_combobox.get()
    start_serial_thread(selected)

def update_timeout():
    try:
        t = float(timeout_entry.get())
        if t < 0.1:
            raise ValueError("Timeout too short (< 0.1)")
        set_timeout(t)
        messagebox.showinfo("Timeout", f"Timeout set to {t:.1f} sec")
    except ValueError:
        messagebox.showerror("Error", "0.1以上の数値を入力してください")

# GUI 初期化
root = tk.Tk()
root.title("NTC-1A タッチパネル操作")
root.geometry("1024x600")
root.configure(bg="black")

font_label = ("Arial", 14)
font_entry = ("Arial", 16)
font_button = ("Arial", 18)

# レイアウトフレーム
control_frame = tk.Frame(root, bg="black")
control_frame.pack(side="top", fill="x")
main_frame = tk.Frame(root, bg="black")
main_frame.pack(fill="both", expand=True)

# ポート・タイムアウト
tk.Label(control_frame, text="Port", font=font_label, bg="black", fg="white") \
    .grid(row=0, column=0, padx=5, pady=5, sticky="e")
port_combobox = ttk.Combobox(control_frame, font=font_label, width=20)
ports = get_serial_ports()
if ports:
    port_combobox['values'] = ports
    port_combobox.current(0)
    start_serial_thread(ports[0])
port_combobox.grid(row=0, column=1, padx=5, pady=5)
port_combobox.bind("<<ComboboxSelected>>", on_port_selected)

tk.Label(control_frame, text="Timeout(sec)", font=font_label, bg="black", fg="white") \
    .grid(row=0, column=2, padx=5, pady=5, sticky="e")
timeout_entry = tk.Entry(control_frame, font=font_label, width=10, justify="right")
timeout_entry.insert(0, "2.0")
timeout_entry.grid(row=0, column=3, padx=5, pady=5)
tk.Button(control_frame, text="Set", font=font_button, command=update_timeout) \
    .grid(row=0, column=4, padx=5, pady=5)

# チャンネル選択
ch_btn = tk.Button(control_frame, text="[CH1 選択中]", font=font_label,
                   bg="darkblue", fg="white", command=toggle_channel)
ch_btn.grid(row=0, column=5, padx=10)

# CH1 / CH2 横並び
for i, ch in enumerate((1, 2)):
    col = i * 3
    tk.Label(main_frame, text=f"CH{ch} テンション(gf)", font=font_label, bg="black", fg="white") \
        .grid(row=0, column=col, sticky="e", padx=5, pady=5)
    entries[f"ch{ch}_tension"] = tk.Entry(main_frame, font=font_entry, width=8, justify="right")
    entries[f"ch{ch}_tension"].grid(row=0, column=col+1, padx=5)
    entries[f"ch{ch}_tension"].bind("<FocusIn>", lambda e, k=f"ch{ch}_tension": entry_selected(entries[k]))

    tk.Label(main_frame, text=f"CH{ch} 線長(m)", font=font_label, bg="black", fg="white") \
        .grid(row=1, column=col, sticky="e", padx=5, pady=5)
    entries[f"ch{ch}_length"] = tk.Entry(main_frame, font=font_entry, width=8, justify="right")
    entries[f"ch{ch}_length"].grid(row=1, column=col+1, padx=5)
    entries[f"ch{ch}_length"].bind("<FocusIn>", lambda e, k=f"ch{ch}_length": entry_selected(entries[k]))

    tk.Label(main_frame, text=f"CH{ch} カウント", font=font_label, bg="black", fg="white") \
        .grid(row=2, column=col, sticky="e", padx=5, pady=5)
    entries[f"ch{ch}_count"] = tk.Entry(main_frame, font=font_entry, width=8, justify="right")
    entries[f"ch{ch}_count"].grid(row=2, column=col+1, padx=5)
    entries[f"ch{ch}_count"].bind("<FocusIn>", lambda e, k=f"ch{ch}_count": entry_selected(entries[k]))

# ボタン縦並び
btn_frame = tk.Frame(main_frame, bg="black")
btn_frame.grid(row=0, column=6, rowspan=3, padx=10)
for cmd in ["SEND", "RESET", "STOP"]:
    tk.Button(btn_frame, text=cmd, font=font_button, width=8, height=2,
              command=lambda c=cmd: handle_command(c)).pack(pady=5)

# ログとテンキー（下部）
bottom_frame = tk.Frame(root, bg="black")
bottom_frame.pack(fill="both", expand=True)
log_frame = tk.Frame(bottom_frame, bg="black")
log_frame.pack(side="left", fill="both", expand=True)
keypad_frame = tk.Frame(bottom_frame, bg="black")
keypad_frame.pack(side="right", fill="both")

log = setup_log_display(log_frame)
make_keypad(keypad_frame, on_keypad_press)

# 終了処理
def on_close():
    stop_serial()
    root.quit()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()
