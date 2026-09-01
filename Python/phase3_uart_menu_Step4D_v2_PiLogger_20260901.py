#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# phase3_uart_menu_Step4D_v2_PiLogger_20260901.py
#
# TCコントローラー代替機 Phase3 通信/parser試験用 CUIツール
#
# 初版制作:
#   2026-08-23
# RXモニタ追加:
#   2026-08-25
# TC106 Original 250201 Direction-B frame monitor追加:
#   2026-08-31
# Pi RX自動ログ保存追加:
#   2026-09-01
#
# 使用用途:
#   Raspberry Pi → FTDI TTL-232R-3V3 → ESP32-S3 間の
#   UART 9600bps / 8N1 による Phase3 意味層packet通信を、
#   本体GUIとは独立して手動試験するための保守・回帰試験ツール。
#
#   RXログ:
#     起動時に自動でログファイルを作成する。
#     スクリプトと同じフォルダ配下の tc_logs/ に保存する。
#     RX RAW と TC250201 decode結果を時刻付きで記録する。
#     1行ごとにflushするため、正常終了前でもログが残りやすい。
#
# 接続:
#   Default port : /dev/ttyUSB0
#   Baud         : 9600
#

import sys
import time
import threading
from datetime import datetime
from pathlib import Path
import serial

DEFAULT_PORT = "/dev/ttyUSB0"
BAUD = 9600

LOG_DIR_NAME = "tc_logs"

TC_FRAME_LEN = 6
TC_FOOTER = 0x7F


def now_text():
    """Local timestamp with millisecond resolution."""
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def create_log_file():
    """Create automatic RX log beside this script."""
    base_dir = Path(__file__).resolve().parent
    log_dir = base_dir / LOG_DIR_NAME
    log_dir.mkdir(parents=True, exist_ok=True)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = log_dir / f"TC250201_RX_{stamp}.log"
    fp = path.open("a", encoding="utf-8", buffering=1)
    return path, fp


def log_line(log_fp, text):
    """Write one timestamped line and flush immediately."""
    if log_fp is None:
        return
    log_fp.write(f"{now_text()} {text}\n")
    log_fp.flush()


class TcDirectionBMonitor:
    """TC106 Original 250201 Direction-B / Step4D telemetry monitor.

    Step4D v1.3 FIX:
      - Footer is 0x7F.
      - Keep only the latest five data bytes in a fixed-size rolling buffer.
      - Track the total number of non-footer bytes independently.
      - count == 5 : valid frame
      - count <  5 : short sync miss
      - count >  5 : overlong sync miss
    """

    DATA_LEN = TC_FRAME_LEN - 1

    def __init__(self):
        self.data_buf = bytearray()
        self.data_byte_count = 0
        self.frame_count = 0
        self.sync_miss_short = 0
        self.sync_miss_overlong = 0

    @property
    def sync_miss_count(self):
        return self.sync_miss_short + self.sync_miss_overlong

    def clear_partial(self):
        self.data_buf.clear()
        self.data_byte_count = 0

    def reset(self):
        self.clear_partial()
        self.frame_count = 0
        self.sync_miss_short = 0
        self.sync_miss_overlong = 0

    def feed(self, data):
        frames = []

        for b in data:
            if b != TC_FOOTER:
                self.data_byte_count += 1

                if len(self.data_buf) < self.DATA_LEN:
                    self.data_buf.append(b)
                else:
                    del self.data_buf[0]
                    self.data_buf.append(b)
                continue

            if self.data_byte_count == self.DATA_LEN:
                frame = bytes(self.data_buf) + bytes([TC_FOOTER])
                self.frame_count += 1
                frames.append(frame)
            elif self.data_byte_count < self.DATA_LEN:
                self.sync_miss_short += 1
            else:
                self.sync_miss_overlong += 1

            self.clear_partial()

        return frames


def format_tc_frame(frame, frame_count, sync_miss_short, sync_miss_overlong):
    """Format one Original 250201 telemetry frame using Step4D semantics."""
    b0, b1, b2, t0, t1, footer = frame

    remaining_length = (b2 * 10000) + (b1 * 100) + b0
    tension_actual = (t1 * 10) + t0
    sync_miss_total = sync_miss_short + sync_miss_overlong

    raw = " ".join(f"{b:02X}" for b in frame)
    return (
        f"[TC250201 #{frame_count:06d}] RX {raw}  "
        f"remaining_length={remaining_length} "
        f"(digits={b2:02d}:{b1:02d}:{b0:02d})  "
        f"tension_actual={tension_actual} "
        f"(digits={t1:02d}:{t0:02d})  "
        f"sync_miss={sync_miss_total} "
        f"(short={sync_miss_short} overlong={sync_miss_overlong})"
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


def rx_worker(ser, stop_event, raw_enabled_event, tc_enabled_event, tc_monitor, log_fp):
    """ESP32-S3 -> Pi RX monitor.

    Screen RAW display can be toggled, but RX RAW logging always continues.
    TC decode logging occurs while TC250201 monitor is enabled.
    """
    while not stop_event.is_set():
        try:
            waiting = ser.in_waiting
            if waiting:
                data = ser.read(waiting)
                if not data:
                    continue

                raw_text = " ".join(f"{b:02X}" for b in data)
                log_line(log_fp, f"RX RAW : {raw_text}")

                if raw_enabled_event.is_set():
                    print("\nRX RAW :", raw_text)

                if tc_enabled_event.is_set():
                    for frame in tc_monitor.feed(data):
                        frame_text = format_tc_frame(
                            frame,
                            tc_monitor.frame_count,
                            tc_monitor.sync_miss_short,
                            tc_monitor.sync_miss_overlong
                        )
                        log_line(log_fp, frame_text)
                        print("\n" + frame_text)
            else:
                time.sleep(0.01)

        except (serial.SerialException, OSError) as e:
            log_line(log_fp, f"[RX ERROR] {type(e).__name__}: {e}")
            if not stop_event.is_set():
                time.sleep(0.05)


def menu(port, rx_enabled, tc_enabled, log_path):
    print(f"""
========================================
 Phase3 UART Test Menu / Step4D Pi Logger
 Port: {port}  Baud: {BAUD}
 RAW RX: {'ON' if rx_enabled else 'OFF'}  TC250201: {'ON' if tc_enabled else 'OFF'}
 Log: {log_path}
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
    log_path = None
    log_fp = None

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
            args=(ser, rx_stop, rx_enabled, tc_enabled, tc_monitor, log_fp),
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
        log_line(log_fp, f"[OPEN] {port} @ {BAUD}")

        start_rx_thread()

    try:
        log_path, log_fp = create_log_file()

        log_line(log_fp, "=== TC250201 Pi RX LOGGER START ===")
        log_line(log_fp, f"port={port} baud={BAUD}")

        print(f"[LOG ] {log_path}")

        reopen()

        while True:
            menu(port, rx_enabled.is_set(), tc_enabled.is_set(), log_path)
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
                        print("[RAW RX MONITOR] OFF (file logging continues)")
                    else:
                        rx_enabled.set()
                        print("[RAW RX MONITOR] ON")

                elif cmd == "t":
                    if tc_enabled.is_set():
                        tc_enabled.clear()
                        print("[TC250201 MONITOR] OFF")
                    else:
                        tc_enabled.set()
                        tc_monitor.clear_partial()
                        print("[TC250201 MONITOR] ON (resync from next footer)")

                elif cmd == "c":
                    tc_monitor.reset()
                    log_line(log_fp, "[TC250201 MONITOR] counters cleared / resync")
                    print("[TC250201 MONITOR] counters cleared / resync")

                elif cmd == "p":
                    new_port = input(f"Port [{port}] > ").strip()
                    if new_port:
                        port = new_port
                        log_line(log_fp, f"[PORT CHANGE] {port}")
                        reopen()

                elif cmd == "q":
                    break

                else:
                    print("Unknown selection")

            except Exception as e:
                log_line(log_fp, f"[ERROR] {type(e).__name__}: {e}")
                print("[ERROR]", e)

    finally:
        stop_rx_thread()

        if ser and ser.is_open:
            ser.close()

        if log_fp is not None:
            log_line(log_fp, "=== TC250201 Pi RX LOGGER END ===")
            log_fp.close()

        print("[CLOSE]")

        if log_path is not None:
            print(f"[LOG ] saved: {log_path}")


if __name__ == "__main__":
    main()
