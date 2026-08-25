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
#     packet解析は行わず、menuの r でON/OFF切替可能（起動時ON）。
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


def rx_worker(ser, stop_event, enabled_event):
    """ESP32-S3 -> Pi raw RX monitor. No packet parsing."""
    while not stop_event.is_set():
        if not enabled_event.is_set():
            time.sleep(0.05)
            continue
        try:
            waiting = ser.in_waiting
            if waiting:
                data = ser.read(waiting)
                if data:
                    print("\nRX     :", " ".join(f"{b:02X}" for b in data))
            else:
                time.sleep(0.01)
        except (serial.SerialException, OSError):
            if not stop_event.is_set():
                time.sleep(0.05)

def menu(port, rx_enabled):
    print(f"""
========================================
 Phase3 UART Test Menu
 Port: {port}  Baud: {BAUD}  RX: {'ON' if rx_enabled else 'OFF'}
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
            args=(ser, rx_stop, rx_enabled),
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
            menu(port, rx_enabled.is_set())
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
                        print("[RX MONITOR] OFF")
                    else:
                        rx_enabled.set()
                        print("[RX MONITOR] ON")
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
