# NTC_1A_serial_comm.py - v1.3.4
# Phase3 GUI/CUI common serial communication layer
#
# v1.3.4:
# - Distinguish 6-byte ESP Boot Magic from TC106 Direction-B telemetry.
# - Direction-B uses fixed footer 0x7F, not checksum7.
# - Validate telemetry bytes 0..4 are decimal groups in range 0..99.
# - Publish telemetry snapshots by whole-dict replacement.
# - Keep existing 12/8-byte checksum7 parsing unchanged.
#
# Historical note:
# The original Main ASM contains real-value display routines, but the inspected
# zhukongban.asm has unreachable paths that set disp_ten/disp_len. Do not state
# that the original hardware necessarily entered those display modes at runtime.

import threading
import time
import serial
import NTC_1A_utils

PORT = "/dev/serial0"
BAUD = 9600

ser = None
_running = False
_thread = None

USE_LSB = False  # Fusion仕様に合わせ通常False

BOOT_MAGIC = bytes((0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x7F))
TELEMETRY_LEN = 6
TELEMETRY_FOOTER = 0x7F
TELEMETRY_VALUE_MAX = 99
TELEMETRY_LOST_TIMEOUT_SEC = 2.0

# RX thread replaces this object as one unit after each valid telemetry frame.
# GUI readers obtain a copied snapshot via get_telemetry().
_telemetry = {
    "wind_length": None,
    "actual_tension": None,
    "last_rx_time": None,
}


def reverse_bits(byte: int) -> int:
    return int(f"{byte:08b}"[::-1], 2)


def checksum7(data) -> int:
    return sum(data) & 0x7F


def build_packet(payload):
    return list(payload) + [checksum7(payload)]


def get_telemetry():
    """Return the latest TC106 telemetry snapshot plus derived RX status."""
    snap = dict(_telemetry)
    last_rx_time = snap.get("last_rx_time")
    if last_rx_time is None:
        snap["rx_status"] = "WAIT"
    elif (time.monotonic() - last_rx_time) <= TELEMETRY_LOST_TIMEOUT_SEC:
        snap["rx_status"] = "OK"
    else:
        snap["rx_status"] = "LOST"
    return snap


def _publish_telemetry(pkt):
    """Decode and publish one already-validated 6-byte Direction-B frame."""
    global _telemetry
    b0, b1, b2, b3, b4, _footer = pkt
    _telemetry = {
        "wind_length": b2 * 10000 + b1 * 100 + b0,
        "actual_tension": b4 * 10 + b3,
        "last_rx_time": time.monotonic(),
    }


def _is_direction_b_telemetry(pkt):
    if len(pkt) != TELEMETRY_LEN:
        return False
    if pkt[5] != TELEMETRY_FOOTER:
        return False
    return all(0 <= b <= TELEMETRY_VALUE_MAX for b in pkt[:5])


def open_port():
    global ser
    ser = serial.Serial(PORT, BAUD, timeout=0.1, write_timeout=1)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    NTC_1A_utils.out(f"[INFO] Port open: {PORT}")


def _write_bytes(data, label="TX"):
    """Write bytes exactly as supplied. No checksum is appended."""
    if not ser or not ser.is_open:
        NTC_1A_utils.out(f"[{label} ERROR] Serial port is not open")
        return False
    try:
        raw = bytes(data)
        ser.write(raw)
        ser.flush()
        NTC_1A_utils.out(f"[{label}] {' '.join(f'{x:02X}' for x in raw)}")
        return True
    except Exception as e:
        NTC_1A_utils.out(f"[{label} ERROR] {e}")
        return False


def send_packet(payload):
    """Send a normal Phase3 payload with 7-bit checksum appended."""
    pkt = build_packet(payload)
    tx = [reverse_bits(b) for b in pkt] if USE_LSB else pkt
    return _write_bytes(tx, "TX")


# ==========================================
# Phase3 Pi -> ESP meaning-level commands
# ==========================================
def _check_u8(name, value):
    try:
        value = int(value)
    except (TypeError, ValueError):
        NTC_1A_utils.out(f"[TX ERROR] {name}: invalid value")
        return None
    if not 0 <= value <= 0xFF:
        NTC_1A_utils.out(f"[TX ERROR] {name}: out of range 0..255 ({value})")
        return None
    return value


def _check_u24(name, value):
    try:
        value = int(value)
    except (TypeError, ValueError):
        NTC_1A_utils.out(f"[TX ERROR] {name}: invalid value")
        return None
    if not 0 <= value <= 0xFFFFFF:
        NTC_1A_utils.out(f"[TX ERROR] {name}: out of range 0..16777215 ({value})")
        return None
    return value


def send_phase3_reset(ch1_length, ch1_tension, ch2_length, ch2_tension):
    ch1_length = _check_u24("CH1 LENGTH", ch1_length)
    ch1_tension = _check_u8("CH1 TENSION", ch1_tension)
    ch2_length = _check_u24("CH2 LENGTH", ch2_length)
    ch2_tension = _check_u8("CH2 TENSION", ch2_tension)
    if None in (ch1_length, ch1_tension, ch2_length, ch2_tension):
        return False

    payload = [
        0x01,
        (ch1_length >> 16) & 0xFF,
        (ch1_length >> 8) & 0xFF,
        ch1_length & 0xFF,
        ch1_tension,
        (ch2_length >> 16) & 0xFF,
        (ch2_length >> 8) & 0xFF,
        ch2_length & 0xFF,
        ch2_tension,
        0x00,
        0x00,
    ]
    return send_packet(payload)


def send_phase3_tension(ch1_tension, ch2_tension):
    ch1_tension = _check_u8("CH1 TENSION", ch1_tension)
    ch2_tension = _check_u8("CH2 TENSION", ch2_tension)
    if None in (ch1_tension, ch2_tension):
        return False
    return send_packet([0x02, ch1_tension, ch2_tension, 0x00, 0x00])


def send_phase3_safe(enabled):
    cmd = 0x10 if enabled else 0x11
    return send_packet([cmd, 0x00, 0x00, 0x00, 0x00])


def send_phase3_sensadj():
    return send_packet([0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])


# ==========================================
# Phase3 parser regression-test helpers
# ==========================================
def send_raw_hex(text):
    """Send a whitespace-separated HEX string exactly as entered."""
    try:
        raw = bytes.fromhex(text)
    except (TypeError, ValueError) as e:
        NTC_1A_utils.out(f"[RAW ERROR] Invalid HEX: {e}")
        return False
    if not raw:
        NTC_1A_utils.out("[RAW ERROR] Empty HEX")
        return False
    return _write_bytes(raw, "TX RAW")


def send_test_checksum_ng():
    ok = send_raw_hex("10 00 00 00 00 11")
    if ok:
        NTC_1A_utils.out("[EXPECT] no [PiTask RX]")
    return ok


def send_test_invalid_command():
    ok = send_raw_hex("99")
    if ok:
        NTC_1A_utils.out("[EXPECT] no [PiTask RX]")
    return ok


def send_test_stage1_false_packet():
    ok = send_raw_hex("00 00 00 00 10 10")
    if ok:
        NTC_1A_utils.out("[EXPECT] no RX for Stage1 false packet")
        NTC_1A_utils.out("[NOTE] reboot ESP before next parser test")
    return ok


def send_test_reset_early_detect(delay_sec=3.0):
    """Run CUI menu 8 without blocking the Pygame main thread."""
    if not send_raw_hex("01 00 00 00 00 01"):
        return False
    NTC_1A_utils.out(f"[TEST] RESET early-detect: wait {delay_sec:.0f}s")
    NTC_1A_utils.out("[EXPECT] no len=6 RX before second half")

    def _second_half():
        time.sleep(delay_sec)
        if send_raw_hex("00 00 00 00 00 02"):
            NTC_1A_utils.out("[EXPECT] one len=12 RX")

    threading.Thread(target=_second_half, daemon=True).start()
    return True


def _rx_loop():
    global _running
    buffer = bytearray()

    while _running:
        try:
            if not ser or not ser.in_waiting:
                time.sleep(0.02)
                continue

            buffer += ser.read(ser.in_waiting)

            while len(buffer) >= TELEMETRY_LEN:
                # Boot Magic has the same 6-byte/footer shape as telemetry.
                first6 = bytes(buffer[:TELEMETRY_LEN])
                if first6 == BOOT_MAGIC:
                    NTC_1A_utils.out(
                        f"[BOOT MAGIC] {' '.join(f'{x:02X}' for x in first6)}"
                    )
                    del buffer[:TELEMETRY_LEN]
                    continue

                # Direction-B: fixed footer 0x7F, not checksum7.
                if _is_direction_b_telemetry(first6):
                    _publish_telemetry(first6)
                    NTC_1A_utils.out(
                        f"[RX TELEMETRY] {' '.join(f'{x:02X}' for x in first6)}"
                    )
                    del buffer[:TELEMETRY_LEN]
                    continue

                # Preserve legacy 12/8-byte checksum7 frames.
                matched = False
                for length in (12, 8):
                    if len(buffer) >= length:
                        pkt = bytes(buffer[:length])
                        if pkt[-1] == checksum7(pkt[:-1]):
                            NTC_1A_utils.out(
                                f"[RX OK] {' '.join(f'{x:02X}' for x in pkt)}"
                            )
                            del buffer[:length]
                            matched = True
                            break

                if matched:
                    continue

                # 6/7 bytes may still become a valid 8-byte checksum frame.
                # 8..11 bytes may still become a valid 12-byte checksum frame.
                if len(buffer) < 12:
                    break

                # At 12+ bytes all supported checks failed at this offset.
                # Drop one byte and resynchronize.
                buffer.pop(0)

        except Exception as e:
            if _running:
                NTC_1A_utils.out(f"[RX ERR] {e}")
            time.sleep(0.1)


def start_serial_thread(port=None):
    global PORT, _running, _thread
    if port:
        PORT = port
    if _running:
        return
    try:
        open_port()
        _running = True
        _thread = threading.Thread(target=_rx_loop, daemon=True)
        _thread.start()
    except Exception as e:
        _running = False
        NTC_1A_utils.out(f"[ERROR] Port open failed: {e}")


def stop_serial():
    global _running, _thread, ser
    _running = False
    try:
        if _thread and _thread.is_alive():
            _thread.join(timeout=0.3)
    except Exception:
        pass
    try:
        if ser and ser.is_open:
            ser.close()
            NTC_1A_utils.out("[INFO] Serial port closed.")
    except Exception as e:
        NTC_1A_utils.out(f"[ERR] Close failed: {e}")
    finally:
        ser = None
        _thread = None
