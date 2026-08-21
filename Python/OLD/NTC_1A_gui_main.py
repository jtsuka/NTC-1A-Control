# NTC-1A GUI – 修正済完全コード（1024x600対応、ログ幅調整済）
import tkinter as tk
from tkinter import ttk, messagebox
from serial.tools import list_ports
from NTC_1A_serial_comm import start_serial_thread, send_packet, stop_serial, set_timeout
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
root.minsize(1024, 600)
root.configure(bg="black")

font_label = ("Arial", 14)
font_button = ("Arial", 18)

left_frame = tk.Frame(root, bg="black")
left_frame.grid(row=0, column=0, sticky="nsew")

right_frame = tk.Frame(root, bg="black")
right_frame.grid(row=0, column=1, sticky="nsew")

root.grid_columnconfigure(0, weight=3)  # 左フレーム
root.grid_columnconfigure(1, weight=5)  # 右フレーム（ログ＋テンキー）

#上下の調整
root.grid_rowconfigure(0, weight=1)
left_frame.grid_rowconfigure(99, weight=1)  # 任意の空行に伸縮余地
right_frame.grid_rowconfigure(99, weight=1)


# 左側
tk.Label(left_frame, text="Port", font=font_label, bg="black", fg="white") \
    .grid(row=0, column=0, padx=5, pady=5, sticky="e")
# ① コンボボックス定義
port_combobox = ttk.Combobox(left_frame, font=font_label, width=20)
# ② ポートリストの取得と初期化（values, current, start）
ports = get_serial_ports()
if ports:
    port_combobox['values'] = ports
    port_combobox.current(0)
    start_serial_thread(ports[0])

port_combobox.grid(row=0, column=1, padx=5, pady=5, sticky="w")
port_combobox.bind("<<ComboboxSelected>>", on_port_selected)

tk.Label(left_frame, text="Timeout(sec)", font=font_label, bg="black", fg="white") \
    .grid(row=1, column=0, padx=5, pady=5, sticky="e")
timeout_entry = tk.Entry(left_frame, font=font_label, width=10, justify="right")
timeout_entry.insert(0, "2.0")
timeout_entry.grid(row=1, column=1, padx=5, pady=5, sticky="w")
tk.Button(left_frame, text="Set", font=font_button, command=update_timeout) \
    .grid(row=1, column=2, padx=5, pady=5, sticky="w")

ch_btn = tk.Button(left_frame, text="[CH1 選択中]", font=("Arial", 14),
                   bg="darkblue", fg="white", command=toggle_channel)
ch_btn.grid(row=2, column=0, columnspan=3, padx=5, pady=5, sticky="ew")

row = 3
for ch in (1, 2):
    make_labeled_entry(left_frame, f"CH{ch} テンション(gf)", row, f"ch{ch}_tension", entries, entry_selected)
    row += 1
    make_labeled_entry(left_frame, f"CH{ch} 線長(m)", row, f"ch{ch}_length", entries, entry_selected)
    row += 1
    make_labeled_entry(left_frame, f"CH{ch} カウント", row, f"ch{ch}_count", entries, entry_selected)
    row += 1

# 右側（ログ + テンキー 縦並び）
log_frame = tk.Frame(right_frame, bg="black")
log_frame.pack(side="top", fill="x")
keypad_frame = tk.Frame(right_frame, bg="black")
keypad_frame.pack(side="top", fill="both", expand=True)

log = setup_log_display(log_frame)
make_keypad(keypad_frame, on_keypad_press)

# start_serial_thread() ← この行は削除またはコメントアウト

def on_close():
    stop_serial()
    root.quit()      # ← 明示的にイベントループ停止
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()