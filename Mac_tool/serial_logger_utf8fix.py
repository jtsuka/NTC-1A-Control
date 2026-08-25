#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
serial_logger.py
TCコントローラー代替機 / Mac用シリアルログCUI

用途:
  - ESP32-S3 / Nano Every のUSBシリアルをCUIで表示しながらログ保存
  - CoolTerm代替の軽量ログ取得
  - SSH経由でWindows側から確認可能

既定ポート:
  ESP32-S3 : /dev/cu.usbmodem1201
  Nano     : /dev/cu.usbmodem1301

2026-08-25:
  UTF-8日本語がSerial.read()境界で分割された場合の文字化けを防ぐため、
  incremental UTF-8 decoderを使用。

使い方:
  python3 serial_logger.py ESP
  python3 serial_logger.py NANO

任意指定:
  python3 serial_logger.py ESP --port /dev/cu.usbmodem1201 --baud 115200
  python3 serial_logger.py NANO --port /dev/cu.usbmodem1301 --baud 115200

終了:
  Ctrl+C

注意:
  CoolTerm等、同じシリアルポートを使用するアプリは閉じてから実行すること。
"""

import argparse
import codecs
import os
import sys
from datetime import datetime
from pathlib import Path

try:
    import serial
except ImportError:
    print("pyserial が必要です。")
    print("  python3 -m pip install pyserial")
    sys.exit(1)

DEFAULTS = {
    "ESP": {
        "port": "/dev/cu.usbmodem1201",
        "baud": 115200,
        "label": "ESP32-S3",
    },
    "NANO": {
        "port": "/dev/cu.usbmodem1301",
        "baud": 115200,
        "label": "NanoEvery",
    },
}


def parse_args():
    p = argparse.ArgumentParser(description="Mac serial logger for ESP32-S3 / Nano Every")
    p.add_argument("target", choices=["ESP", "NANO"], help="対象機器")
    p.add_argument("--port", help="シリアルポートを上書き")
    p.add_argument("--baud", type=int, help="baudrateを上書き")
    p.add_argument(
        "--log-dir",
        default=str(Path.home() / "tc_serial_logs"),
        help="ログ保存先 (default: ~/tc_serial_logs)",
    )
    return p.parse_args()


def main():
    args = parse_args()
    cfg = DEFAULTS[args.target]

    port = args.port or cfg["port"]
    baud = args.baud or cfg["baud"]
    label = cfg["label"]

    log_dir = Path(os.path.expanduser(args.log_dir))
    log_dir.mkdir(parents=True, exist_ok=True)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    logfile = log_dir / f"{label}_{stamp}.txt"

    print("=" * 64)
    print(f" Target : {label}")
    print(f" Port   : {port}")
    print(f" Baud   : {baud}")
    print(f" Log    : {logfile}")
    print(" Stop   : Ctrl+C")
    print("=" * 64)

    try:
        ser = serial.Serial(port, baudrate=baud, timeout=0.2)
    except Exception as e:
        print(f"[OPEN ERROR] {e}")
        return 1

    try:
        with logfile.open("a", encoding="utf-8", buffering=1) as f:
            f.write(f"# START {datetime.now().isoformat(timespec='seconds')}\n")
            f.write(f"# TARGET={label} PORT={port} BAUD={baud}\n")

            # Serial.read() can split a UTF-8 Japanese character between reads.
            # An incremental decoder keeps incomplete multibyte sequences until
            # the following read instead of replacing them with "�".
            decoder = codecs.getincrementaldecoder("utf-8")(errors="replace")

            while True:
                data = ser.read(ser.in_waiting or 1)
                if not data:
                    continue

                decoded = decoder.decode(data, final=False)
                if decoded:
                    print(decoded, end="", flush=True)
                    f.write(decoded)

    except KeyboardInterrupt:
        print("\n[STOP]")
    finally:
        try:
            ser.close()
        except Exception:
            pass

        with logfile.open("a", encoding="utf-8") as f:
            f.write(f"\n# STOP {datetime.now().isoformat(timespec='seconds')}\n")

        print(f"[SAVED] {logfile}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
