# NTC_1A_utils.py – ログ幅調整・UI両対応版
import sys

# ログオブジェクトの初期化
log = None

def setup_log_display(parent):
    """Tkinter UI用：ログ表示エリアの構築"""
    try:
        import tkinter as tk
        global log
        log = tk.Text(parent, width=48, height=6, bg="black", fg="lime",
                      font=("Courier", 10), wrap="none")
        log.pack(fill="x", padx=5, pady=5)
        return log
    except ImportError:
        print("[WARN] Tkinter not found. GUI logging disabled.")
        return None

def out(msg):
    """標準ログ出力：Pygame UI側から log_callback に差し替えられる"""
    print(msg)
    # log が Tkinter オブジェクトの場合のみ書き込み
    if log and hasattr(log, 'insert'):
        try:
            import tkinter as tk
            log.insert(tk.END, str(msg) + "\n")
            log.see(tk.END)
        except:
            pass

# --- 以下、Tkinter UIを使用する場合のヘルパー関数 ---
def make_labeled_entry(parent, label_text, row, key, entry_dict, command=None):
    import tkinter as tk
    label = tk.Label(parent, text=label_text, font=("Arial", 14), bg="darkgreen", fg="white")
    label.grid(row=row, column=0, sticky="w", padx=5, pady=2)
    entry = tk.Entry(parent, font=("Arial", 14), width=10, justify="right")
    entry.grid(row=row, column=1, columnspan=2, sticky="w", padx=5, pady=2)
    if command:
        entry.bind("<Button-1>", lambda e: command(entry))
    entry_dict[key] = entry

def make_keypad(parent, press_callback):
    import tkinter as tk
    buttons = [
        ["7", "8", "9", "CLR"],
        ["4", "5", "6", "ENTER"],
        ["1", "2", "3", "SEND"],
        ["0", "STOP", "RESET"]
    ]
    for r, row in enumerate(buttons):
        for c, key in enumerate(row):
            btn = tk.Button(parent, text=key, font=("Arial", 16), width=6, height=2,
                            command=lambda k=key: press_callback(k))
            btn.grid(row=r, column=c, padx=3, pady=3)