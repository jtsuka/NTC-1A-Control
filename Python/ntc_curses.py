#!/usr/bin/env python3
"""
NTC-1A メンテナンス用 Curses UI サンプル
Tkinter GUI を Python curses に置き換えた例です。
- 上部ステータス（Port, Timeout, Channel）
- 左部入力フィールド（CH1/CH2 張力・長さ・カウント）
- 右部ログウィンドウ
- 下部キー操作（SAFE ON/OFF, TEST CMD, SEND/RESET/STOP）
キー操作:
  Tab: フォーカス移動
  Enter: 操作確定
  'q': 終了
"""
import curses
import threading
from serial.tools import list_ports
from NTC_1A_serial_comm import start_serial_thread, send_packet, build_packet, send_packet_raw, set_timeout, stop_serial
from NTC_1A_utils import out

# 初期設定
ports = [p.device for p in list_ports.comports()]
if '/dev/serial0' not in ports:
    ports.insert(0, '/dev/serial0')
current_port = ports[0]
timeout = 2.0
channel = 1
fields = {
    'ch1_tension': '0', 'ch1_length': '0', 'ch1_count': '0',
    'ch2_tension': '0', 'ch2_length': '0', 'ch2_count': '0',
}
log_lines = []
lock = threading.Lock()

# シリアル受信ログを Curses に追加
def log_listener():
    def callback(msg):
        with lock:
            log_lines.append(msg)
            if len(log_lines) > 100:
                log_lines.pop(0)
    out.register(callback)

# Curses 画面構成
def draw_ui(stdscr, focus):
    stdscr.clear()
    h, w = stdscr.getmaxyx()
    # 上部ステータス
    status = f"Port:{current_port}  Timeout:{timeout:.1f}s  Channel:CH{channel}"
    stdscr.addstr(0, 1, status, curses.A_BOLD)
    # 左部入力フィールド
    for i, key in enumerate(['tension','length','count']):
        y = 2 + i*2
        # CH1
        stdscr.addstr(y, 2, f"CH1 {key}: ")
        val = fields[f'ch1_{key}']
        attr = curses.A_REVERSE if focus==('field', f'ch1_{key}') else curses.A_NORMAL
        stdscr.addstr(y, 15, val.ljust(6), attr)
        # CH2
        stdscr.addstr(y, 30, f"CH2 {key}: ")
        val2 = fields[f'ch2_{key}']
        attr2 = curses.A_REVERSE if focus==('field', f'ch2_{key}') else curses.A_NORMAL
        stdscr.addstr(y, 45, val2.ljust(6), attr2)
    # 下部操作ボタン
    ops = ['SAFE ON','SAFE OFF','TEST1','TEST2','TEST3','SEND','RESET','STOP']
    for i, op in enumerate(ops):
        x = 2 + i*12
        y = h-3
        attr = curses.A_REVERSE if focus==('op', op) else curses.A_NORMAL
        stdscr.addstr(y, x, f"[{op}]", attr)
    # 右部ログ
    log_h = h - 6
    for i, line in enumerate(log_lines[-log_h:]):
        stdscr.addstr(2+i, w-40, line[:38])
    stdscr.refresh()

# メインループ
def main(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(False)
    focus_list = []
    # フォーカス順のリスト作成
    for key in ['ch1_tension','ch1_length','ch1_count','ch2_tension','ch2_length','ch2_count']:
        focus_list.append(('field', key))
    for op in ['SAFE ON','SAFE OFF','TEST1','TEST2','TEST3','SEND','RESET','STOP']:
        focus_list.append(('op', op))
    idx = 0
    focus = focus_list[idx]

    # シリアルスレッド起動
    start_serial_thread(current_port)
    threading.Thread(target=log_listener, daemon=True).start()

    while True:
        draw_ui(stdscr, focus)
        key = stdscr.getch()
        if key in (curses.KEY_TAB, 9):
            idx = (idx + 1) % len(focus_list)
            focus = focus_list[idx]
        elif key in (curses.KEY_BTAB,):
            idx = (idx - 1) % len(focus_list)
            focus = focus_list[idx]
        elif key in (curses.KEY_ENTER, ord('\n')):
            typ, name = focus
            if typ=='field':
                # 入力モード
                curses.echo()
                stdscr.addstr(2+focus_list.index(focus)//2*2, 15 if name.startswith('ch1') else 45, ' '*6)
                stdscr.move(2+(focus_list.index(focus)//2)*2, 15 if name.startswith('ch1') else 45)
                val = stdscr.getstr(6).decode()
                fields[name] = val
                curses.noecho()
            else:
                # 操作実行
                if name=='SAFE ON': send_packet_raw([0xF0,1])
                elif name=='SAFE OFF': send_packet_raw([0xF0,0])
                elif name.startswith('TEST'):
                    idx=int(name[-1])-1
                    send_packet_raw([0x01,0x06,0x05,0,0,0] if idx==0 else build_packet([0x01,100,0,0,0]))
                elif name=='SEND':
                    ch=channel; t=int(fields[f'ch{ch}_tension']); l=int(float(fields[f'ch{ch}_length'])*10);
                    c=int(fields[f'ch{ch}_count']);
                    send_packet([0x01,t&0xFF,0,0,0])
                    send_packet([0x02,l&0xFF,(l>>8)&0xFF,(l>>16)&0xFF,0])
                    send_packet([0x03,c&0xFF,(c>>8)&0xFF,(c>>16)&0xFF,0])
                elif name=='RESET': send_packet([0x04,0,0,0,0])
                elif name=='STOP': send_packet([0x05,0,0,0,0])
        elif key in (ord('q'), 27):
            break
    stop_serial()

if __name__=='__main__':
    curses.wrapper(main)
