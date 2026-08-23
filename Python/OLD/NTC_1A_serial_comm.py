# NTC_1A_serial_comm.py - v1.3.2 clean
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

def send_packet(payload):
    if not ser or not ser.is_open:
        return False
    pkt = build_packet(payload)
    tx = [reverse_bits(b) for b in pkt] if USE_LSB else pkt
    try:
        ser.write(bytes(tx))
        ser.flush()
        NTC_1A_utils.out(f"[TX] {' '.join(f'{x:02X}' for x in tx)}")
        return True
    except Exception as e:
        NTC_1A_utils.out(f"[TX ERROR] {e}")
        return False

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