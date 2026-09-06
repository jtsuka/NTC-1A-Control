# NTC_1A_serial_comm.py - v1.3.3
# Phase3 GUI/CUI common serial communication layer
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


def reverse_bits(byte: int) -> int:
    return int(f"{byte:08b}"[::-1], 2)


def checksum7(data) -> int:
    return sum(data) & 0x7F


def build_packet(payload):
    return list(payload) + [checksum7(payload)]


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
# GUI layout/port-selection code is intentionally independent of these helpers.
# send_packet() appends the 7-bit checksum.
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
# These reproduce CUI menu items 6..10.
# They intentionally send malformed/raw sequences and therefore do NOT append checksum.
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
            if ser and ser.in_waiting:
                buffer += ser.read(ser.in_waiting)
                while len(buffer) >= 6:
                    matched = False
                    for length in (12, 8, 6):
                        if len(buffer) >= length:
                            pkt = buffer[:length]
                            if pkt[length - 1] == (sum(pkt[:length - 1]) & 0x7F):
                                NTC_1A_utils.out(f"[RX OK] {' '.join(f'{x:02X}' for x in pkt)}")
                                del buffer[:length]
                                matched = True
                                break
                    if not matched:
                        buffer.pop(0)
            else:
                time.sleep(0.02)
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
