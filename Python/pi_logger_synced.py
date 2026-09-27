#!/usr/bin/env python3
"""Direction-B evidence logger for TC replacement project.

Reads ESP SerialPi (9600 bps) one byte at a time and frames telemetry by the
0x7F footer used by the current ESP Direction-B receiver.  No sequence-number
assumption is made.

IMPORTANT: do not run the normal Pi GUI/serial process at the same time; only
one process should own the UART during this evidence test.
"""

import argparse
import csv
import time
from datetime import datetime
from pathlib import Path

import serial

BAUD_RATE = 9600
DEFAULT_DURATION_SEC = 120.0
BOOT_MAGIC = bytes((0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x7F))
FOOTER = 0x7F
FRAME_LEN = 6
PAYLOAD_LEN = 5


def iso_timestamp_now() -> str:
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def run_logger(port: str, output: Path, duration_sec: float) -> int:
    total_frames = 0
    sync_misses = 0
    boot_magic_seen = 0
    last_frame_ns = None
    bytes_since_footer = bytearray()

    with serial.Serial(port, BAUD_RATE, timeout=0.05) as ser, \
            output.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.writer(fp)
        writer.writerow([
            "Timestamp",
            "Raw_Hex",
            "Interval_ms",
            "Footer_OK",
        ])

        # Discard bytes that pre-date this formal capture. Starting the logger
        # before resetting/powering the ESP is the preferred procedure.
        ser.reset_input_buffer()

        start_ns = time.monotonic_ns()
        end_ns = start_ns + int(duration_sec * 1_000_000_000)

        print(f"--- Pi Direction-B Logger Started ({duration_sec:.1f}s) ---")
        print(f"Port : {port} @ {BAUD_RATE} bps")
        print(f"File : {output}")

        while time.monotonic_ns() < end_ns:
            b = ser.read(1)
            if not b:
                continue

            value = b[0]
            if value != FOOTER:
                bytes_since_footer.append(value)
                # Bound memory if framing is lost for an unexpectedly long time.
                if len(bytes_since_footer) > 4096:
                    bytes_since_footer = bytes_since_footer[-4096:]
                continue

            # 0x7F establishes a boundary. The current ESP receiver treats
            # 0x7F as the unique Direction-B footer; mirror that behavior here.
            if len(bytes_since_footer) >= PAYLOAD_LEN:
                candidate = bytes(bytes_since_footer[-PAYLOAD_LEN:]) + bytes((FOOTER,))
            else:
                candidate = b""

            # ESP Boot Magic is also six bytes ending in 0x7F. It is not TC
            # telemetry and must never be counted as a Direction-B frame.
            if candidate == BOOT_MAGIC:
                boot_magic_seen += 1
                bytes_since_footer.clear()
                continue

            if len(bytes_since_footer) != PAYLOAD_LEN:
                # Cold-start/misalignment evidence. This footer re-anchors the
                # next frame, just as the ESP receiver resynchronizes on footer.
                sync_misses += 1
                bytes_since_footer.clear()
                continue

            now_ns = time.monotonic_ns()
            interval_ms = "" if last_frame_ns is None else f"{(now_ns - last_frame_ns) / 1_000_000.0:.3f}"
            last_frame_ns = now_ns
            total_frames += 1

            writer.writerow([
                iso_timestamp_now(),
                candidate.hex().upper(),
                interval_ms,
                1,  # frame is committed only when the observed footer is 0x7F
            ])
            bytes_since_footer.clear()

        fp.flush()

    print("\n--- Test Completed ---")
    print(f"Total Frames Received : {total_frames}")
    print(f"Pi Sync Misses         : {sync_misses}")
    print(f"Boot Magic Ignored     : {boot_magic_seen}")
    print(f"Log saved to           : {output}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="TC Direction-B 0x7F-synchronized evidence logger")
    parser.add_argument("--port", default="/dev/ttyAMA0", help="Pi UART device (default: /dev/ttyAMA0)")
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION_SEC, help="capture seconds (default: 120)")
    parser.add_argument("--output", default="pi_rx_evidence.csv", help="CSV output path")
    args = parser.parse_args()

    if args.duration <= 0:
        parser.error("--duration must be > 0")

    return run_logger(args.port, Path(args.output), args.duration)


if __name__ == "__main__":
    raise SystemExit(main())
