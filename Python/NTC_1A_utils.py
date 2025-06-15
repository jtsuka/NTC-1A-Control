# utils.py
import tkinter as tk
from tkinter import scrolledtext

log = None

def out(msg):
    global log
    print(msg)
    if log:
        try:
            log.insert(tk.END, msg + "
")
            log.see(tk.END)
        except:
            pass

def setup_log_display(root):
    global log
    log = scrolledtext.ScrolledText(root, height=8, bg="black", fg="lime", font=("Courier", 12))
    log.grid(row=1, column=3, columnspan=4, rowspan=1, padx=5, pady=5, sticky="nsew")
    return log

def make_labeled_entry(root, label_text, row, key, entries, callback):
    bg = "#003300" if "ch1" in key else "black"
    label = tk.Label(root, text=label_text, font=("Arial", 14), bg=bg, fg="white")
    label.grid(row=row, column=0, sticky="e", padx=5, pady=5)
    entry = tk.Entry(root, font=("Arial", 14), width=10, justify="right", bg=bg, fg="white")
    entry.grid(row=row, column=1, padx=5, pady=5)
    entry.bind("<Button-1>", lambda e: callback(entry))
    entries[key] = entry

def make_keypad(root, handler):
    keys = [["7", "8", "9", "CLR"],
            ["4", "5", "6", "ENTER"],
            ["1", "2", "3", "SEND"],
            ["0", "STOP", "", "RESET"]]
    pad = tk.Frame(root, bg="black")
    pad.grid(row=2, column=3, columnspan=4, rowspan=6, sticky="nsew")
    for r, row_k in enumerate(keys):
        for c, k in enumerate(row_k):
            if k:
                btn = tk.Button(pad, text=k, font=("Arial", 18), width=4, height=2,
                                command=lambda k=k: handler(k))
                btn.grid(row=r, column=c, padx=5, pady=5, sticky="nsew")
    for i in range(4): pad.grid_columnconfigure(i, weight=1)
    for i in range(4): pad.grid_rowconfigure(i, weight=1)
