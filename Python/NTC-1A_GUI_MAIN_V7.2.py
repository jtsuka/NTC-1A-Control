# Next_Main_GUI_v7_2.py – 完全レスポンシブ対応版（OK-PANEL仕様）
import tkinter as tk
from tkinter import ttk, messagebox
from serial.tools import list_ports
from NTC_1A_serial_comm import start_serial_thread, send_packet, stop_serial, set_timeout
from NTC_1A_utils import out

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

root = tk.Tk()
root.title("NTC-1A Touch Panel – Responsive Edition")
root.geometry("1024x600")
root.minsize(800, 480)

font_label = ("Arial", 14)
font_entry = ("Arial", 16)
font_button = ("Arial", 18)

root.columnconfigure(0, weight=1)
root.rowconfigure(1, weight=1)

# ==== Top Bar ====
top_frame = tk.Frame(root, bg="black")
top_frame.grid(row=0, column=0, sticky="ew")
top_frame.columnconfigure(6, weight=1)

port_label = tk.Label(top_frame, text="Port", font=font_label, bg="black", fg="white")
port_label.grid(row=0, column=0, padx=5)
port_combobox = ttk.Combobox(top_frame, font=font_label, width=12)
ports = get_serial_ports()
if ports:
    port_combobox['values'] = ports
    port_combobox.current(0)
    start_serial_thread(ports[0])
port_combobox.grid(row=0, column=1)
port_combobox.bind("<<ComboboxSelected>>", on_port_selected)

timeout_label = tk.Label(top_frame, text=" Timeout(sec) ", font=font_label, bg="black", fg="white")
timeout_label.grid(row=0, column=2)
timeout_entry = tk.Entry(top_frame, font=font_label, width=6, justify="right")
timeout_entry.insert(0, "2.0")
timeout_entry.grid(row=0, column=3)
tk.Button(top_frame, text="Set", font=font_button, command=update_timeout).grid(row=0, column=4, padx=5)

ch_btn = tk.Button(top_frame, text="[CH1 選択中]", font=font_label, bg="darkblue", fg="white", command=toggle_channel)
ch_btn.grid(row=0, column=5, padx=10)

# ==== Middle Frame (Log + Main UI) ====
center_frame = tk.Frame(root, bg="black")
center_frame.grid(row=1, column=0, sticky="nsew")
center_frame.columnconfigure(0, weight=1)
center_frame.columnconfigure(1, weight=2)
center_frame.rowconfigure(1, weight=1)

# Log Area
log_text = tk.Text(center_frame, height=5, bg="black", fg="lime", font=("Consolas", 12))
log_text.grid(row=0, column=0, columnspan=2, sticky="nsew", padx=5, pady=5)
def out(msg):
    log_text.insert(tk.END, msg + "\n")
    log_text.see(tk.END)

# ==== Left Input Area ====
input_frame = tk.Frame(center_frame, bg="black")
input_frame.grid(row=1, column=0, sticky="nsew", padx=20)
input_frame.columnconfigure(1, weight=1)

row_idx = 0
for ch in (1, 2):
    for key in ["tension", "length", "count"]:
        unit = "gf" if key == "tension" else "m" if key == "length" else ""
        label = tk.Label(input_frame, text=f"CH{ch} {key}({unit})", font=font_label, bg="black", fg="green")
        label.grid(row=row_idx, column=0, sticky="w", pady=3)
        entry = tk.Entry(input_frame, font=font_entry, justify="right")
        entry.grid(row=row_idx, column=1, sticky="ew", padx=5)
        entry.bind("<FocusIn>", lambda e, k=f"ch{ch}_{key}": entry_selected(entries[k]))
        entries[f"ch{ch}_{key}"] = entry
        row_idx += 1

# ==== Right Keypad Area ====
keypad_frame = tk.Frame(center_frame, bg="black")
keypad_frame.grid(row=1, column=1, sticky="nsew", padx=20, pady=10)
for i in range(4): keypad_frame.columnconfigure(i, weight=1)
for i in range(4): keypad_frame.rowconfigure(i, weight=1)

keys = [
    ["7", "8", "9", "CLR"],
    ["4", "5", "6", "ENTER"],
    ["1", "2", "3", "SEND"],
    ["0", "STOP", "RESET", ""]
]
for r, row in enumerate(keys):
    for c, key in enumerate(row):
        if key:
            b = tk.Button(keypad_frame, text=key, font=font_button, command=lambda k=key: on_keypad_press(k))
            b.grid(row=r, column=c, sticky="nsew", padx=2, pady=2)

def on_close():
    stop_serial()
    root.quit()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()
