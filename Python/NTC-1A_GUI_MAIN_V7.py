# Next_Main_GUI_v7.py – OK-PANEL 構成準拠版（縦並びUI + テンキー一体化）
import tkinter as tk
from tkinter import ttk, messagebox
from serial.tools import list_ports
from NTC_1A_serial_comm import start_serial_thread, send_packet, stop_serial, set_timeout
from NTC_1A_utils import out, setup_log_display, make_keypad

selected_channel = 1
selected_entry = None
entries = {}

# ==== 基本操作関数 ====
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

# ==== GUI構築 ====
root = tk.Tk()
root.title("NTC-1A タッチパネル – OKパネル構成")
root.geometry("1024x600")
root.configure(bg="black")

font_label = ("Arial", 14)
font_entry = ("Arial", 16)
font_button = ("Arial", 18)

# ==== 上部バー：Port選択 & Timeout ====
top_frame = tk.Frame(root, bg="black")
top_frame.pack(side="top", fill="x")

tk.Label(top_frame, text="Port", font=font_label, bg="black", fg="white") \
    .pack(side="left", padx=5)
port_combobox = ttk.Combobox(top_frame, font=font_label, width=20)
ports = get_serial_ports()
if ports:
    port_combobox['values'] = ports
    port_combobox.current(0)
    start_serial_thread(ports[0])
port_combobox.pack(side="left")
port_combobox.bind("<<ComboboxSelected>>", on_port_selected)

tk.Label(top_frame, text=" Timeout(sec) ", font=font_label, bg="black", fg="white") \
    .pack(side="left")
timeout_entry = tk.Entry(top_frame, font=font_label, width=6, justify="right")
timeout_entry.insert(0, "2.0")
timeout_entry.pack(side="left")
tk.Button(top_frame, text="Set", font=font_button, command=update_timeout) \
    .pack(side="left", padx=5)

ch_btn = tk.Button(top_frame, text="[CH1 選択中]", font=font_label, bg="darkblue", fg="white", command=toggle_channel)
ch_btn.pack(side="left", padx=10)

# ==== 中央部：左=入力欄 / 右=テンキー&ログ ====
main_frame = tk.Frame(root, bg="black")
main_frame.pack(fill="both", expand=True)

left_frame = tk.Frame(main_frame, bg="black")
left_frame.pack(side="left", fill="y", padx=10)
right_frame = tk.Frame(main_frame, bg="black")
right_frame.pack(side="right", fill="both", expand=True)

# ==== CH1 & CH2フィールド（縦並び） ====
for ch in (1, 2):
    for key in ["tension", "length", "count"]:
        label = tk.Label(left_frame, text=f"CH{ch} {key}({ 'gf' if key=='tension' else 'm' if key=='length' else ''})",
                         font=font_label, bg="black", fg="green")
        label.pack(anchor="w")
        entry = tk.Entry(left_frame, font=font_entry, width=10, justify="right")
        entry.pack(fill="x", pady=2)
        entry.bind("<FocusIn>", lambda e, k=f"ch{ch}_{key}": entry_selected(entries[k]))
        entries[f"ch{ch}_{key}"] = entry

# ==== ログ + テンキー ====
log_frame = tk.Frame(right_frame, bg="black")
log_frame.pack(side="top", fill="x")
log = setup_log_display(log_frame)

keypad_frame = tk.Frame(right_frame, bg="black")
keypad_frame.pack(side="bottom", fill="both", expand=True)
make_keypad(keypad_frame, on_keypad_press)

# ==== 終了処理 ====
def on_close():
    stop_serial()
    root.quit()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()
