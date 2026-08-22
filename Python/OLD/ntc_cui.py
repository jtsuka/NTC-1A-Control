#!/usr/bin/env python3
import curses
from curses import textpad
from serial.tools import list_ports
from NTC_1A_serial_comm import start_serial_thread, send_packet, build_packet
from NTC_1A_utils import out, setup_log_display

# 初期設定
ports = [p.device for p in list_ports.comports()]
if "/dev/serial0" not in ports:
    ports.insert(0, "/dev/serial0")
current_port = ports[0]
timeout = 2.0
selected_channel = 1
values = {
    "ch1_tension": "0",
    "ch1_length": "0",
    "ch1_count":  "0",
    "ch2_tension": "0",
    "ch2_length": "0",
    "ch2_count":  "0",
}

def draw_menu(stdscr, idx):
    stdscr.clear()
    h,w = stdscr.getmaxyx()
    menu = [
        f"Port: {current_port}",
        f"Timeout: {timeout:.1f}s",
        f"Channel: CH{selected_channel}",
        f"CH1 張力: {values['ch1_tension']}",
        f"CH1 長さ: {values['ch1_length']}",
        f"CH1 カウント: {values['ch1_count']}",
        f"CH2 張力: {values['ch2_tension']}",
        f"CH2 長さ: {values['ch2_length']}",
        f"CH2 カウント: {values['ch2_count']}",
        "SAFE MODE ON",
        "SAFE MODE OFF",
        "Test CMD1…CMD5",
        "SEND Command",
        "RESET Command",
        "STOP Command",
        "Exit",
    ]
    for i, item in enumerate(menu):
        x = 2
        y = 2 + i
        if i == idx:
            stdscr.attron(curses.A_REVERSE)
            stdscr.addstr(y, x, item)
            stdscr.attroff(curses.A_REVERSE)
        else:
            stdscr.addstr(y, x, item)
    stdscr.refresh()

def prompt_string(stdscr, prompt, initial=""):
    curses.echo()
    stdscr.addstr(curses.LINES-2,2, prompt)
    stdscr.clrtoeol()
    stdscr.refresh()
    win = curses.newwin(1,40,curses.LINES-2, 2+len(prompt))
    textpad.rectangle(stdscr, curses.LINES-3,1, curses.LINES-1, 2+len(prompt)+40)
    stdscr.refresh()
    tb = textpad.Textbox(win)
    win.addstr(0,0, initial)
    s = tb.edit().strip()
    curses.noecho()
    return s

def main(stdscr):
    global current_port, timeout, selected_channel
    # シリアルスレッド起動
    start_serial_thread(current_port)
    idx = 0
    n = 16
    while True:
        draw_menu(stdscr, idx)
        key = stdscr.getch()
        if key in (curses.KEY_UP, ord('k')) and idx>0:
            idx-=1
        elif key in (curses.KEY_DOWN, ord('j')) and idx< n-1:
            idx+=1
        elif key in (curses.KEY_ENTER, ord('\n')):
            # 各メニュー選択時の処理
            if idx == 0:
                # Port 選択
                sel = prompt_string(stdscr, "Select Port: "+",".join(ports)+" > ", current_port)
                if sel in ports:
                    current_port = sel
                    start_serial_thread(current_port)
            elif idx == 1:
                # Timeout
                t = prompt_string(stdscr, "Timeout(sec)> ", str(timeout))
                try: timeout = float(t); from NTC_1A_serial_comm import set_timeout; set_timeout(timeout)
                except: pass
            elif idx == 2:
                # Channel 切替
                selected_channel = 1 if selected_channel==2 else 2
            elif 3 <= idx <= 5 or 6 <= idx <= 8:
                # CHx 入力
                key = ["ch1_tension","ch1_length","ch1_count","ch2_tension","ch2_length","ch2_count"][idx-3]
                values[key] = prompt_string(stdscr, f"{key}> ", values[key])
            elif idx == 9:
                # SAFE ON
                from NTC_1A_serial_comm import send_packet_raw
                send_packet_raw([0xF0,1])
            elif idx == 10:
                from NTC_1A_serial_comm import send_packet_raw
                send_packet_raw([0xF0,0])
            elif idx == 11:
                # Test CMD1-5 -> サブメニュー
                sel = prompt_string(stdscr, "CMD index(1-5)> ", "1")
                try:
                    i = int(sel)-1
                    from NTC_1A_serial_comm import send_packet_raw
                    test_packets = [
                        [0x01,0x06,0x05,0,0,0x0C],
                        build_packet([0x01,100,0,0,0]),
                        build_packet([0x02,0x10,0x27,0,0]),
                    ]
                    if 0<=i<len(test_packets): send_packet_raw(test_packets[i])
                except: pass
            elif idx == 12:
                # SEND
                t = int(values[f"ch{selected_channel}_tension"])
                l = int(float(values[f"ch{selected_channel}_length"])*10)
                c = int(values[f"ch{selected_channel}_count"])
                send_packet([0x01,t&0xFF,0,0,0])
                send_packet([0x02,l&0xFF,(l>>8)&0xFF,(l>>16)&0xFF,0])
                send_packet([0x03,c&0xFF,(c>>8)&0xFF,(c>>16)&0xFF,0])
            elif idx == 13:
                send_packet([0x04,0,0,0,0])
            elif idx == 14:
                send_packet([0x05,0,0,0,0])
            elif idx == 15:
                break
        elif key in (ord('q'),27):
            break

curses.wrapper(main)
