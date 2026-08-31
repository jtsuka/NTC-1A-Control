#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# phase3_uart_menu.py
#
# TCコントローラー代替機 Phase3 通信/parser試験用 CUIツール
#
# 初版制作:
#   2026-08-23
# RXモニタ追加:
#   2026-08-25
# TC106 Original 250201 Direction-B frame monitor追加:
#   2026-08-31
#
# 使用用途:
#   Raspberry Pi → FTDI TTL-232R-3V3 → ESP32-S3 間の
#   UART 9600bps / 8N1 による Phase3 意味層packet通信を、
#   本体Pygame GUIとは独立して手動試験するための保守・回帰試験ツール。
#
#   正常系:
#     - RESET (0x01 / 12byte)
#     - SEND (0x02 / 6byte)
#     - SENS.ADJ (0x03 / 8byte)
#     - SAFE ON (0x10 / 6byte)
#     - SAFE OFF (0x11 / 6byte)
#
#   異常系 / parser回帰試験:
#     - checksum NG packetの破棄確認
#     - 無効command 0x99の破棄確認
#     - RESET early-detect prevention
#       （前半6byteだけでchecksumが成立し得ても、12byte到達前に
#        RESET packetとして早期確定しないことを確認）
#     - Stage1 false-packet reproduction
#       （Stage1で観測した 00 00 00 00 10 10 が、
#        command起点parser導入後に6byte packetとして成立しないことを確認）
#     - 任意RAW HEX送信
#
#   RXモニタ:
#     ESP32-S3 → Pi方向の受信raw byteをHEX表示する。
#     menuの r でON/OFF切替可能（起動時ON）。
#
#   TC106 Direction-Bモニタ:
#     ESP32-S3がTC106 Direction-Bの6byte frameをUARTへそのまま転送する
#     試験構成を前提として、0x7F footerでframe同期し、
#       [lastlen0 lastlen1 lastlen2 txtens0 txtens1 7F]
#     を表示する。
#     menuの t でON/OFF切替可能（起動時ON）。
#
#     注意:
#       現ESP32-S3 FWがDirection-B frameをUARTへ転送していない場合は
#       Pi側だけではTC106 frameは見えない。Step4C側で転送経路が必要。
#
# 位置付け:
#   本番GUIとは分離した開発・保守用ツール。
#   Step3C Stage2 command起点parserの実機確認に使用し、
#   Stage2 RUNTIME PASS / CLOSED の根拠となった。
#   将来Pi→ESP UART parserを変更した際の回帰試験にも使用する。
#
# 接続:
#   Default port : /dev/ttyUSB0
#   Baud         : 9600
#
# 注意:
#   本体GUIなど、同じシリアルポートを使用する別プロセスとは
#   同時に起動しないこと。
#   Menu 9 実行後はparserがCOLLECTING状態に残る可能性があるため、
#   次のparser試験前にESP32-S3を再起動すること。
#

import sys
import time
import threading
import serial

DEFAULT_PORT = "/dev/ttyUSB0"
BAUD = 9600


TC_FRAME_LEN = 6
TC_FOOTER = 0x7F

class TcDirectionBMonitor:
    """TC106 Original 250201 Direction-B 6-byte frame monitor.

    Synchronization rule:
      - Bytes are collected until footer 0x7F is seen.
      - Exactly 6 bytes ending in 0x7F => valid frame.
      - Any other length => sync miss; discard and restart after footer.

    This makes startup in the middle of a continuous TC frame self-resynchronizing
    at the next footer.
    """

    def __init__(self):
        self.buf = bytearray()
        self.frame_count = 0
        self.sync_miss_count = 0

    def reset(self):
        self.buf.clear()
        self.frame_count = 0
        self.sync_miss_count = 0

    def feed(self, data):
        frames = []
        for b in data:
            self.buf.append(b)

            # Guard against unrelated UART streams with no 0x7F.
            if len(self.buf) > 64:
                self.buf.clear()
                self.sync_miss_count += 1
                continue

            if b != TC_FOOTER:
                continue

            if len(self.buf) == TC_FRAME_LEN:
                frame = bytes(self.buf)
                self.frame_count += 1
                frames.append(frame)
            else:
                self.sync_miss_count += 1

            self.buf.clear()

        return frames

def format_tc_frame(frame, frame_count, sync_miss_count):
    """Format one Original 250201 payload without assuming engineering units."""
    b0, b1, b2, t0, t1, footer = frame

    # Original 250201 sends lastlen[] as three base-100 digits.
    lastlen_base100 = (b2 * 10000) + (b1 * 100) + b0

    # Original C:
    #   txtens[0] = tens_actual % 10
    #   txtens[1] = tens_actual / 10
    # Therefore this reconstructs the integer tens_actual code.
    tension_code = (t1 * 10) + t0

    raw = " ".join(f"{b:02X}" for b in frame)
    return (
        f"[TC250201 #{frame_count:06d}] RX {raw}  "
        f"lastlen={lastlen_base100} "
        f"(digits={b2:02d}:{b1:02d}:{b0:02d})  "
        f"tension_code={tension_code} "
        f"(digits={t1:02d}:{t0:02d})  "
        f"sync_miss={sync_miss_count}"
    )

def checksum7(data):
    return sum(data) & 0x7F

def send_raw_hex(ser, text):
    data = bytes.fromhex(text)
    ser.write(data)
    ser.flush()
    print("TX RAW :", " ".join(f"{b:02X}" for b in data))

def send_payload(ser, payload):
    pkt = bytes(payload + [checksum7(payload)])
    ser.write(pkt)
    ser.flush()
    print("TX     :", " ".join(f"{b:02X}" for b in pkt))

def ask_int(prompt, default=0, minimum=0, maximum=None):
    s = input(f"{prompt} [{default}]: ").strip()
    value = default if not s else int(s)
    if value < minimum or (maximum is not None and value > maximum):
        raise ValueError(f"range {minimum}..{maximum}")
    return value

def send_reset(ser):
    ch1_len = ask_int("CH1 LENGTH", 5, 0, 0xFFFFFF)
    ch1_ten = ask_int("CH1 TENSION", 10, 0, 255)
    ch2_len = ask_int("CH2 LENGTH", 0, 0, 0xFFFFFF)
    ch2_ten = ask_int("CH2 TENSION", 0, 0, 255)
    send_payload(ser, [
        0x01,
        (ch1_len >> 16) & 0xFF,
        (ch1_len >> 8) & 0xFF,
        ch1_len & 0xFF,
        ch1_ten,
        (ch2_len >> 16) & 0xFF,
        (ch2_len >> 8) & 0xFF,
        ch2_len & 0xFF,
        ch2_ten,
        0, 0
    ])

def send_send(ser):
    ch1 = ask_int("CH1 TENSION", 10, 0, 255)
    ch2 = ask_int("CH2 TENSION", 0, 0, 255)
    send_payload(ser, [0x02, ch1, ch2, 0, 0])


def rx_worker(ser, stop_event, raw_enabled_event, tc_enabled_event, tc_monitor):
    """ESP32-S3 -> Pi RX monitor.

    raw_enabled_event:
        Show every UART receive chunk as HEX.

    tc_enabled_event:
        Feed the same received bytes into the TC106 Original 250201
        Direction-B 6-byte/footer monitor.
    """
    while not stop_event.is_set():
        try:
            waiting = ser.in_waiting
            if waiting:
                data = ser.read(waiting)
                if not data:
                    continue

                if raw_enabled_event.is_set():
                    print("\nRX RAW :", " ".join(f"{b:02X}" for b in data))

                if tc_enabled_event.is_set():
                    for frame in tc_monitor.feed(data):
                        print("\n" + format_tc_frame(
                            frame,
                            tc_monitor.frame_count,
                            tc_monitor.sync_miss_count
                        ))
            else:
                time.sleep(0.01)
        except (serial.SerialException, OSError):
            if not stop_event.is_set():
                time.sleep(0.05)

def menu(port, rx_enabled, tc_enabled):
    print(f"""
========================================
 Phase3 UART Test Menu
 Port: {port}  Baud: {BAUD}
 RAW RX: {'ON' if rx_enabled else 'OFF'}  TC250201: {'ON' if tc_enabled else 'OFF'}
========================================
 1 : RESET
 2 : SEND
 3 : SENS.ADJ
 4 : SAFE ON
 5 : SAFE OFF
 6 : Checksum NG test
 7 : Invalid command 0x99
 8 : RESET early-detect prevention test
 9 : Stage1 false-packet reproduction
10 : RAW HEX send
 r : Toggle RAW RX monitor
 t : Toggle TC106 250201 frame monitor
 c : Clear TC frame counters / resync
 p : Change serial port
 q : Quit
========================================""")

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    ser = None
    rx_thread = None
    rx_stop = None
    rx_enabled = threading.Event()
    rx_enabled.set()
    tc_enabled = threading.Event()
    tc_enabled.set()
    tc_monitor = TcDirectionBMonitor()

    def stop_rx_thread():
        nonlocal rx_thread, rx_stop
        if rx_stop is not None:
            rx_stop.set()
        if rx_thread is not None and rx_thread.is_alive():
            rx_thread.join(timeout=1.0)
        rx_thread = None
        rx_stop = None

    def start_rx_thread():
        nonlocal rx_thread, rx_stop
        rx_stop = threading.Event()
        rx_thread = threading.Thread(
            target=rx_worker,
            args=(ser, rx_stop, rx_enabled, tc_enabled, tc_monitor),
            daemon=True
        )
        rx_thread.start()

    def reopen():
        nonlocal ser
        stop_rx_thread()
        if ser and ser.is_open:
            ser.close()
        ser = serial.Serial(port, BAUD, timeout=0.1, write_timeout=1)
        print(f"[OPEN] {port} @ {BAUD}")
        start_rx_thread()

    try:
        reopen()
        while True:
            menu(port, rx_enabled.is_set(), tc_enabled.is_set())
            cmd = input("Select > ").strip().lower()
            try:
                if cmd == "1":
                    send_reset(ser)
                elif cmd == "2":
                    send_send(ser)
                elif cmd == "3":
                    send_payload(ser, [0x03,0,0,0,0,0,0])
                elif cmd == "4":
                    send_payload(ser, [0x10,0,0,0,0])
                elif cmd == "5":
                    send_payload(ser, [0x11,0,0,0,0])
                elif cmd == "6":
                    send_raw_hex(ser, "10 00 00 00 00 11")
                    print("EXPECT : no [PiTask RX]")
                elif cmd == "7":
                    send_raw_hex(ser, "99")
                    print("EXPECT : no [PiTask RX]")
                elif cmd == "8":
                    send_raw_hex(ser, "01 00 00 00 00 01")
                    print("Wait 3 sec. EXPECT: no len=6 RX")
                    time.sleep(3)
                    send_raw_hex(ser, "00 00 00 00 00 02")
                    print("EXPECT : one len=12 RX")
                elif cmd == "9":
                    send_raw_hex(ser, "00 00 00 00 10 10")
                    print("EXPECT : no RX for this 6-byte sequence")
                    print("NOTE   : reboot ESP before next parser test")
                elif cmd == "10":
                    raw = input("HEX > ").strip()
                    if raw:
                        send_raw_hex(ser, raw)
                elif cmd == "r":
                    if rx_enabled.is_set():
                        rx_enabled.clear()
                        print("[RAW RX MONITOR] OFF")
                    else:
                        rx_enabled.set()
                        print("[RAW RX MONITOR] ON")
                elif cmd == "t":
                    if tc_enabled.is_set():
                        tc_enabled.clear()
                        print("[TC250201 MONITOR] OFF")
                    else:
                        tc_enabled.set()
                        tc_monitor.buf.clear()
                        print("[TC250201 MONITOR] ON (resync from next footer)")
                elif cmd == "c":
                    tc_monitor.reset()
                    print("[TC250201 MONITOR] counters cleared / resync")
                elif cmd == "p":
                    new_port = input(f"Port [{port}] > ").strip()
                    if new_port:
                        port = new_port
                        reopen()
                elif cmd == "q":
                    break
                else:
                    print("Unknown selection")
            except Exception as e:
                print("[ERROR]", e)
    finally:
        stop_rx_thread()
        if ser and ser.is_open:
            ser.close()
        print("[CLOSE]")

if __name__ == "__main__":
    main()
