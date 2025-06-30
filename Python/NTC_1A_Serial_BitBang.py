
# NTC_1A_serial_comm_bitbang.py – BitBang版 送信＋受信対応 (300bps)
import pigpio
import time
from NTC_1A_utils import out

PORT = "GPIO"
BAUD = 300
TX_PIN = 14
RX_PIN = 15
PACKET_SIZE = 6
BIT_TIME = 1.0 / BAUD

pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("pigpioデーモンが起動していません。sudo pigpiod を実行してください。")

pi.set_mode(TX_PIN, pigpio.OUTPUT)
pi.set_mode(RX_PIN, pigpio.INPUT)
pi.write(TX_PIN, 1)  # Idle HIGH

current_timeout_value = 2.0
running = True

def open_port():
    out("[INFO] BitBang GPIOポート初期化完了")

def stop_serial():
    global running
    running = False
    pi.write(TX_PIN, 1)
    pi.stop()

def set_timeout(val):
    global current_timeout_value
    current_timeout_value = val
    out(f"[INFO] Timeout set: {val:.1f} sec")

def send_byte(b):
    pi.write(TX_PIN, 0)
    time.sleep(BIT_TIME)
    for i in range(8):
        pi.write(TX_PIN, (b >> i) & 1)
        time.sleep(BIT_TIME)
    pi.write(TX_PIN, 1)
    time.sleep(BIT_TIME)

def send_packet_raw(byte_list):
    out(f"[TX] {' '.join(f'{x:02X}' for x in byte_list)}")
    for b in byte_list:
        send_byte(b)
        time.sleep(BIT_TIME * 0.5)

def send_packet(ch, cmd, val):
    pkt = [ch, cmd, val, 0, 0]
    pkt.append(sum(pkt) & 0xFF)
    send_packet_raw(pkt)
    try:
        resp = read_exact(PACKET_SIZE, timeout=current_timeout_value)
        if resp is None:
            out("[TIMEOUT] No response.")
            return False
        out(f"[RAW] {' '.join(f'{x:02X}' for x in resp)}")
        if len(resp) < PACKET_SIZE:
            out(f"[RX ERROR] Incomplete response: {len(resp)} bytes → {' '.join(f'{x:02X}' for x in resp)}")
            return False
        calc_chk = sum(resp[:5]) & 0xFF
        if resp[5] != calc_chk:
            out(f"[RX WARN] Checksum mismatch: Expected {calc_chk:02X}, Got {resp[5]:02X}")
            return False
        out(f"[RX] {' '.join(f'{x:02X}' for x in resp)}")
        return True
    except Exception as e:
        out(f"[RX ERROR] {e}")
        return False

def read_byte(timeout=1.0):
    start = time.time()
    while pi.read(RX_PIN) == 1:
        if time.time() - start > timeout:
            return None
    time.sleep(BIT_TIME * 1.5)
    byte = 0
    for i in range(8):
        bit = pi.read(RX_PIN)
        byte |= (bit << i)
        time.sleep(BIT_TIME)
    time.sleep(BIT_TIME)  # Stop bit
    return byte

def read_exact(n, timeout=1.0):
    pkt = []
    t0 = time.time()
    while len(pkt) < n and time.time() - t0 < timeout:
        b = read_byte(timeout)
        if b is None:
            break
        pkt.append(b)
    return pkt if len(pkt) == n else None

def rx_loop():
    pass  # GUI側が使わないのでダミー

def start_serial_thread(port=None):
    open_port()
    # RXループ不要だがAPI互換で起動メッセージだけ
    out("[INFO] BitBang Serial thread started.")
