#!/usr/bin/env python3
"""
ESP32-S3 boot marker logger for Raspberry Pi.

Default wiring:
  XIAO ESP32-S3 D2 / GPIO3  --->  Raspberry Pi GPIO17 (BCM17)
  ESP GND                    --->  Raspberry Pi GND

The Pi input uses an internal pull-down.
When ESP is reset / in bootloader, the marker line is expected LOW.
When firmware reaches setup(), it drives HIGH and this logger records the rising edge.

Run:
  python3 esp_boot_marker_logger.py

Optional:
  python3 esp_boot_marker_logger.py --pin 17 --log ./tc_logs/ESP_BOOT_MARKER.log
"""

from __future__ import annotations
import argparse
from datetime import datetime
from pathlib import Path
import signal
import sys
import time

try:
    from gpiozero import DigitalInputDevice
except ImportError:
    print("gpiozero がありません。Raspberry Pi OS では通常入っています。")
    print("必要なら: sudo apt install python3-gpiozero")
    sys.exit(2)


def now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--pin", type=int, default=17, help="BCM GPIO number (default: 17)")
    p.add_argument(
        "--log",
        default=None,
        help="Log path. Default: ./tc_logs/ESP_BOOT_MARKER_YYYYMMDD_HHMMSS.log",
    )
    args = p.parse_args()

    if args.log:
        log_path = Path(args.log).expanduser()
    else:
        log_dir = Path(__file__).resolve().parent / "tc_logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        log_path = log_dir / f"ESP_BOOT_MARKER_{datetime.now():%Y%m%d_%H%M%S}.log"

    log_path.parent.mkdir(parents=True, exist_ok=True)

    running = True

    def stop_handler(signum, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    # pull_up=False => internal pull-down.
    marker = DigitalInputDevice(args.pin, pull_up=False, bounce_time=0.002)

    with log_path.open("a", encoding="utf-8", buffering=1) as f:
        def log(msg: str) -> None:
            line = f"{now_iso()} {msg}"
            print(line, flush=True)
            f.write(line + "\n")
            f.flush()

        def on_rise() -> None:
            log(f"[ESP BOOT] rising edge BCM{args.pin}")

        marker.when_activated = on_rise

        log("=== ESP BOOT MARKER LOGGER START ===")
        log(f"pin=BCM{args.pin} initial_level={int(marker.value)}")
        log(f"log={log_path}")

        while running:
            time.sleep(0.1)

        log("=== ESP BOOT MARKER LOGGER END ===")

    marker.close()
    print(f"saved: {log_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
