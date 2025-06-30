# NTC_1A_serial_comm_bitbang.py

import pigpio, threading, time
from NTC_1A_utils import out

# GPIO設定
BAUD = 300
TX_PIN = 14
RX_PIN = 15
BIT_TIME = 1.0 / BAUD
PACKET_SIZE = 6

pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("pigpiodを起動してください: sudo pigpiod")
pi.set_mode(TX_PIN, pigpio.OUTPUT)
pi.set_mode(RX_PIN, pigpio.INPUT)
pi.write(TX_PIN, 1)

rx_buffer = bytearray()
current_timeout_value = 2.0
running = False
_lock = threading.Lock()

def open_port():
    out("[INFO] BitBangポート初期化済み")

def stop_serial():
    global running
    running = False
    time.sleep(BIT_TIME)
    pi.write(TX_PIN, 1)
    pi.stop()
    out("[INFO] BitBang通信停止")

def set_timeout(val):
    global current_timeout_value
    current_timeout_value = val
    out(f"[INFO] タイムアウト設定: {val:.1f}秒")

def send_byte(b):
    pi.write(TX_PIN, 0); time.sleep(BIT_TIME)
    for i in range(8):
        pi.write(TX_PIN, (b >> i) & 1)
        time.sleep(BIT_TIME)
    pi.write(TX_PIN, 1); time.sleep(BIT_TIME)

def send_packet_raw(byte_list):
    out(f"[TX] {' '.join(f'{x:02X}' for x in byte_list)}")
    for b in byte_list:
        send_byte(b)
        time.sleep(BIT_TIME * 0.5)

def read_byte(timeout=1.0):
    t0 = time.time()
    while pi.read(RX_PIN) == 1:
        if time.time() - t0 > timeout:
            return None
    time.sleep(BIT_TIME * 1.5)
    val = 0
    for i in range(8):
        val |= (pi.read(RX_PIN) << i)
        time.sleep(BIT_TIME)
    time.sleep(BIT_TIME)
    return val

def read_exact(n, timeout=1.0):
    pkt = []
    t_end = time.time() + timeout
    while len(pkt) < n and time.time() < t_end:
        b = read_byte(timeout)
        if b is None: break
        pkt.append(b)
    return pkt if len(pkt) == n else None

def send_packet(ch, cmd, val):
    pkt = [ch, cmd, val, 0, 0]
    pkt.append(sum(pkt) & 0xFF)
    send_packet_raw(pkt)
    try:
        resp = read_exact(PACKET_SIZE, timeout=current_timeout_value)
        if resp is None:
            out("[TIMEOUT] 応答なし")
            return False
        out(f"[RAW] {' '.join(f'{x:02X}' for x in resp)}")
        if len(resp) != PACKET_SIZE:
            out("[RX ERROR] 長さ不正")
            return False
        if resp[5] != sum(resp[:5]) & 0xFF:
            out("[RX WARN] チェックサム不一致")
            return False
        out(f"[RX] {' '.join(f'{x:02X}' for x in resp)}")
        return True
    except Exception as e:
        out(f"[RX ERROR] {e}")
        return False

def rx_loop():
    global running
    while running:
        b = read_byte(timeout=0.1)
        if b is not None:
            with _lock:
                rx_buffer.append(b)
        time.sleep(0.005)

def start_serial_thread(port=None):
    global running
    open_port()
    running = True
    t = threading.Thread(target=rx_loop, daemon=True)
    t.start()
    out("[INFO] RXスレッド起動")

# GUI側から rx_buffer を直接参照してください。
