#!/usr/bin/env python3
"""TCBridge Evidence V2 synchronized 120 s logger.

- Sends EVIDENCE_START (E5 1D 53 54) three times to ESP over Pi UART.
- Logs normal 6-byte Direction-B telemetry to CSV.
- Recognizes the 124-byte Evidence RESULT frame before telemetry parsing.
- Saves validated Evidence RESULT to JSON and exits.
- Times out safely if no valid Evidence RESULT arrives.
"""
import argparse
import csv
import json
import struct
import time
from datetime import datetime
from pathlib import Path

import serial

BOOT_MAGIC = bytes((0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x7F))
TELEMETRY_LEN = 6
TELEMETRY_FOOTER = 0x7F
TELEMETRY_VALUE_MAX = 99

EVIDENCE_START_MAGIC = bytes((0xE5, 0x1D, 0x53, 0x54))
EVIDENCE_RESULT_MAGIC = bytes((0xE5, 0x1D, 0xE7, 0xC5))
EVIDENCE_VERSION = 0x01
EVIDENCE_PAYLOAD_LEN = 116
EVIDENCE_RESULT_LEN = 124
EVIDENCE_DURATION_MS = 120000

COUNTER_NAMES = [
    "poll_max_interval_us",
    "poll_over1000_count",
    "c1_isr_edges",
    "c1_edge_overflow",
    "c2_decoded_bytes",
    "c2_decode_errors",
    "c2_timing_errors",
    "c2_local_edge_overflow",
    "c3_valid_frames",
    "c3_frame_errors",
    "c3_sync_misses",
    "c3_resyncs",
    "c4_queued_frames",
    "c4_queue_drops",
]


def is_telemetry(pkt: bytes) -> bool:
    return (len(pkt) == TELEMETRY_LEN
            and pkt[5] == TELEMETRY_FOOTER
            and all(0 <= b <= TELEMETRY_VALUE_MAX for b in pkt[:5]))


def checksum16(data: bytes) -> int:
    return sum(data) & 0xFFFF


def parse_evidence_frame(frame: bytes):
    if len(frame) != EVIDENCE_RESULT_LEN:
        return None, "frame_length"
    if frame[:4] != EVIDENCE_RESULT_MAGIC:
        return None, "magic"

    version = frame[4]
    length = frame[5]
    if length != EVIDENCE_PAYLOAD_LEN:
        return None, "payload_length"

    expected = int.from_bytes(frame[-2:], "little")
    actual = checksum16(frame[:-2])
    if actual != expected:
        return None, "checksum"

    payload = frame[6:-2]
    duration_ms = struct.unpack_from("<I", payload, 0)[0]
    values = struct.unpack_from("<" + "I" * 28, payload, 4)

    counters = {}
    for i, name in enumerate(COUNTER_NAMES):
        start = values[i * 2]
        end = values[i * 2 + 1]
        counters[name] = {
            "start": start,
            "end": end,
            "delta": (end - start) & 0xFFFFFFFF,
        }

    return {
        "magic_ok": True,
        "version": version,
        "version_ok": version == EVIDENCE_VERSION,
        "length": length,
        "length_ok": length == EVIDENCE_PAYLOAD_LEN,
        "checksum_type": "uint16_additive_sum_magic_through_payload",
        "checksum_received": expected,
        "checksum_calculated": actual,
        "checksum_ok": True,
        "duration_ms": duration_ms,
        "duration_ok": duration_ms == EVIDENCE_DURATION_MS,
        "counters": counters,
        "raw_hex": frame.hex(" ").upper(),
    }, None


def default_json_name():
    return datetime.now().strftime("evidence_report_%Y%m%d_%H%M%S.json")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default="/dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=9600)
    p.add_argument("--timeout", type=float, default=150.0,
                   help="seconds from first START transmission")
    p.add_argument("--csv", default="pi_rx_evidence_v2.csv")
    p.add_argument("--json", dest="json_path", default=None)
    p.add_argument("--start-repeat", type=int, default=3)
    p.add_argument("--start-gap-ms", type=float, default=30.0)
    a = p.parse_args()

    json_path = a.json_path or default_json_name()
    ser = serial.Serial(a.port, a.baud, timeout=0.05, write_timeout=1)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    frames = 0
    boot = 0
    resync = 0
    footer_ok = 0
    last_frame_mono = None
    max_interval_ms = 0.0
    invalid_evidence = []
    result = None

    # Pi-side statistics start immediately before the first START request.
    measure_start = time.monotonic()
    wall_start = datetime.now().astimezone().isoformat(timespec="milliseconds")

    for i in range(max(1, a.start_repeat)):
        ser.write(EVIDENCE_START_MAGIC)
        ser.flush()
        if i + 1 < max(1, a.start_repeat):
            time.sleep(max(0.0, a.start_gap_ms) / 1000.0)

    deadline = measure_start + a.timeout
    rxbuf = bytearray()

    print(f"[START] port={a.port} baud={a.baud} timeout={a.timeout:.1f}s")
    print(f"[START] EVIDENCE_START sent x{max(1, a.start_repeat)}")

    with open(a.csv, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["FrameNo", "Timestamp", "Monotonic_s", "Raw_Hex",
                    "Interval_ms", "Footer_OK", "Wind_Length", "Actual_Tension"])

        try:
            while time.monotonic() < deadline and result is None:
                n = ser.in_waiting
                chunk = ser.read(n if n else 1)
                if not chunk:
                    continue
                rxbuf.extend(chunk)

                while rxbuf and result is None:
                    # Priority 1: Evidence RESULT. Never feed its payload to
                    # the 6-byte telemetry parser.
                    if rxbuf.startswith(EVIDENCE_RESULT_MAGIC):
                        if len(rxbuf) < 6:
                            break
                        length = rxbuf[5]
                        total = 4 + 1 + 1 + length + 2
                        if total > 512:
                            invalid_evidence.append({"reason": "unreasonable_length",
                                                     "length": length})
                            del rxbuf[0]
                            resync += 1
                            continue
                        if len(rxbuf) < total:
                            break
                        candidate = bytes(rxbuf[:total])
                        del rxbuf[:total]
                        parsed, err = parse_evidence_frame(candidate)
                        if parsed is None:
                            invalid_evidence.append({
                                "reason": err,
                                "raw_hex": candidate.hex(" ").upper(),
                            })
                            continue
                        # A checksum-valid frame with unsupported VERSION or
                        # wrong configured duration is preserved as an invalid
                        # attempt, but does not terminate as a successful V2 run.
                        if not parsed["version_ok"]:
                            invalid_evidence.append({
                                "reason": "version",
                                "version": parsed["version"],
                                "raw_hex": candidate.hex(" ").upper(),
                            })
                            continue
                        if not parsed["duration_ok"]:
                            invalid_evidence.append({
                                "reason": "duration",
                                "duration_ms": parsed["duration_ms"],
                                "raw_hex": candidate.hex(" ").upper(),
                            })
                            continue

                        now = time.monotonic()
                        parsed["evidence_received_at"] = datetime.now().astimezone().isoformat(
                            timespec="milliseconds")
                        parsed["evidence_monotonic_s"] = now - measure_start
                        result = parsed
                        break

                    # If current bytes are a prefix of the 4-byte RESULT magic,
                    # wait for more rather than discarding them as telemetry junk.
                    if len(rxbuf) < 4 and EVIDENCE_RESULT_MAGIC.startswith(bytes(rxbuf)):
                        break

                    # Priority 2: Boot Magic / normal telemetry, both fixed 6 bytes.
                    if len(rxbuf) < TELEMETRY_LEN:
                        break
                    first6 = bytes(rxbuf[:TELEMETRY_LEN])
                    if first6 == BOOT_MAGIC:
                        boot += 1
                        del rxbuf[:TELEMETRY_LEN]
                        continue
                    if is_telemetry(first6):
                        now = time.monotonic()
                        wall = datetime.now().astimezone().isoformat(timespec="milliseconds")
                        interval = ""
                        if last_frame_mono is not None:
                            iv = (now - last_frame_mono) * 1000.0
                            interval = f"{iv:.3f}"
                            max_interval_ms = max(max_interval_ms, iv)
                        last_frame_mono = now
                        b0, b1, b2, b3, b4, _ = first6
                        frames += 1
                        footer_ok += 1
                        w.writerow([
                            frames, wall, f"{now - measure_start:.6f}",
                            " ".join(f"{x:02X}" for x in first6),
                            interval, 1, b2 * 10000 + b1 * 100 + b0,
                            b4 * 10 + b3,
                        ])
                        del rxbuf[:TELEMETRY_LEN]
                        continue

                    # Resynchronize one byte at a time. RESULT magic remains
                    # protected by the prefix check above.
                    del rxbuf[0]
                    resync += 1
        finally:
            ser.close()

    wall_end = datetime.now().astimezone().isoformat(timespec="milliseconds")
    report = result or {
        "magic_ok": False,
        "version_ok": False,
        "length_ok": False,
        "checksum_ok": False,
        "duration_ok": False,
        "evidence_received": False,
        "timeout": True,
    }
    report["evidence_received"] = result is not None
    report["logger"] = {
        "port": a.port,
        "baud": a.baud,
        "started_at": wall_start,
        "ended_at": wall_end,
        "timeout_s": a.timeout,
        "start_magic_hex": EVIDENCE_START_MAGIC.hex(" ").upper(),
        "start_repeat": max(1, a.start_repeat),
        "start_gap_ms": a.start_gap_ms,
        "invalid_evidence_attempts": invalid_evidence,
    }
    report["normal_telemetry"] = {
        "frames_received": frames,
        "footer_ok_count": footer_ok,
        "max_interval_ms": max_interval_ms,
        "boot_magic_frames": boot,
        "resync_drop_bytes": resync,
        "buffered_bytes_end": len(rxbuf),
    }

    Path(json_path).write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                               encoding="utf-8")

    print("[DONE]")
    print(f"Evidence_received  : {result is not None}")
    print(f"Telemetry_frames   : {frames}")
    print(f"Footer_OK          : {footer_ok}")
    print(f"Boot_magic_frames  : {boot}")
    print(f"Resync_drop_bytes  : {resync}")
    print(f"Max_interval_ms    : {max_interval_ms:.3f}")
    print(f"CSV                 : {a.csv}")
    print(f"JSON                : {json_path}")

    raise SystemExit(0 if result is not None else 2)


if __name__ == "__main__":
    main()
