# NTC_1A_serial_comm.py – スプール対応・応答待ち機能付き
import serial, threading, time
from NTC_1A_utils import out

PORT = "/dev/ttyUSB0"
BAUD = 9600
ser = None
running = True
current_timeout_value = 2.0  # 初期値（秒）

def set_timeout(val):
    global current_timeout_value
    current_timeout_value = val
    out(f"[INFO] Timeout set: {val:.1f} sec")

def open_port():
    global ser
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        out(f"[INFO] Port open: {PORT}")
    except Exception as e:
        out(f"[ERROR] Port open failed: {e}")

def send_packet(ch, cmd, val):
    if not ser or not ser.is_open:
        out("[SKIP] Port not open")
        return False

    pkt = [ch, cmd, val, 0, 0]
    pkt.append(sum(pkt) & 0xFF)
    try:
        ser.write(bytes(pkt))
        ser.flush()
        out(f"[TX] {' '.join(f'{x:02X}' for x in pkt)}")
    except Exception as e:
        out(f"[TX ERROR] {e}")
        return False

    try:
        resp = read_exact(6, timeout=current_timeout_value)
        if resp is None:
            out("[TIMEOUT] No response.")
            return False
        elif len(resp) < 6:
            out(f"[RX ERROR] Incomplete response: {len(resp)} bytes → {' '.join(f'{x:02X}' for x in resp)}")
            return False
        else:
            # チェックサム検証
            calc_chk = sum(resp[:5]) & 0xFF
            if resp[5] != calc_chk:
                out(f"[RX WARN] Checksum mismatch: Expected {calc_chk:02X}, Got {resp[5]:02X}")
                out(f"[RX] {' '.join(f'{x:02X}' for x in resp)}")
                return False
            else:
                out(f"[RX] {' '.join(f'{x:02X}' for x in resp)}")
                return True
    except Exception as e:
        out(f"[RX ERROR] {e}")
        return False

def read_exact(n, timeout=0.5):
    buf = bytearray(n)
    idx = 0
    t0 = time.time()
    while idx < n and time.time() - t0 < timeout:
        idx += ser.readinto(memoryview(buf)[idx:])
    return buf if idx == n else None

def rx_loop():
    global running
    while running:
        time.sleep(0.05)  # スプール送信に対し不要な受信処理を抑制

def start_serial_thread(port=None):
    global PORT
    if port:
        PORT = port
    open_port()
    threading.Thread(target=rx_loop, daemon=True).start()

def stop_serial():
    global running
    running = False
    if ser and ser.is_open:
        ser.close()
