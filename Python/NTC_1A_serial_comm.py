# NTC_1A_serial_comm.py – スプール対応・応答待ち機能付き
import serial, threading, time
from NTC_1A_utils import out

PORT = "/dev/serial0"
BAUD = 9600
ser = None
running = True
current_timeout_value = 2.0  # 初期値（秒）
# === LSB送信モード設定 ===
USE_LSB = False  # ← TrueにするとMSBファースト送信

# === ビット反転ユーティリティ ===
def reverse_bits(byte):
    return int('{:08b}'.format(byte)[::-1], 2)

def send_packet(ch, cmd, val):
    if not ser or not ser.is_open:
        out("[SKIP] Port not open")
        return False

    pkt = [ch, cmd, val, 0, 0]
    pkt.append(sum(pkt) & 0xFF)

    # === LSB変換（オプション）===
    tx_data = [reverse_bits(b) for b in pkt] if USE_LSB else pkt

    try:
        ser.write(bytes(tx_data))
        ser.flush()
        time.sleep(0.1)
        out(f"[TX{'-LSB' if USE_LSB else ''}] {' '.join(f'{x:02X}' for x in tx_data)}")
    except Exception as e:
        out(f"[TX ERROR] {e}")
        return False

    try:
        resp = ser.read(6)
        if resp is not None:
            out(f"[RAW] {' '.join(f'{x:02X}' for x in resp)}")

        if resp is None:
            out("[TIMEOUT] No response.")
            return False
        elif len(resp) < 6:
            out(f"[RX ERROR] Incomplete response: {len(resp)} bytes → {' '.join(f'{x:02X}' for x in resp)}")
            return False
        else:
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

def send_packet_raw(packet_bytes):
    """Rawバイナリパケットを送信する（可変長）"""
    global ser
    if ser and ser.is_open:
        tx_data = [reverse_bits(b) for b in packet_bytes] if USE_LSB else packet_bytes
        ser.write(bytes(tx_data))
        print(f"[TX raw{'-LSB' if USE_LSB else ''}] {' '.join(f'{b:02X}' for b in tx_data)}")


#def send_packet_raw(byte_list):
#    if ser and ser.is_open:
#        ser.write(bytes(byte_list))

def set_timeout(val):
    global current_timeout_value
    current_timeout_value = val
    out(f"[INFO] Timeout set: {val:.1f} sec")

def open_port():
    global ser
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1, write_timeout=2) # 明示的な書き込み完了待ち時間
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        out(f"[INFO] Port open: {PORT}")
    except Exception as e:
        out(f"[ERROR] Port open failed: {e}")

#def send_packet(ch, cmd, val):
#    if not ser or not ser.is_open:
#        out("[SKIP] Port not open")
#        return False
#
#    pkt = [ch, cmd, val, 0, 0]
#    pkt.append(sum(pkt) & 0xFF)
#    try:
#        ser.write(bytes(pkt))
#        ser.flush()
#        time.sleep(0.1)  # 100msの送信待機（2400bpsで6バイト＝25ms以上必要）
#        out(f"[TX] {' '.join(f'{x:02X}' for x in pkt)}")
#    except Exception as e:
#        out(f"[TX ERROR] {e}")
#        return False
#
#    try:
        # === ここだけ簡易方式 ===
#        resp = ser.read(6)  # 応答バイトを直接読む（6バイト固定）
        # resp = read_exact(6, timeout=current_timeout_value)
        # RAWダンプをここに追加
#        if resp is not None:
#            out(f"[RAW] {' '.join(f'{x:02X}' for x in resp)}")
            
#        if resp is None:
#            out("[TIMEOUT] No response.")
#            return False
#        elif len(resp) < 6:
#            out(f"[RX ERROR] Incomplete response: {len(resp)} bytes → {' '.join(f'{x:02X}' for x in resp)}")
#            return False
#        else:
            # チェックサム検証
#            calc_chk = sum(resp[:5]) & 0xFF
#            if resp[5] != calc_chk:
#                out(f"[RX WARN] Checksum mismatch: Expected {calc_chk:02X}, Got {resp[5]:02X}")
#                out(f"[RX] {' '.join(f'{x:02X}' for x in resp)}")
#                return False
#            else:
#                out(f"[RX] {' '.join(f'{x:02X}' for x in resp)}")
#                return True
#    except Exception as e:
#        out(f"[RX ERROR] {e}")
#        return False

def read_exact(n, timeout=0.5):
    buf = bytearray(n)
    idx = 0
    t0 = time.time()
    while idx < n and time.time() - t0 < timeout:
        idx += ser.readinto(memoryview(buf)[idx:])
    return buf if idx == n else None

def rx_loop():
    global running
    buffer = bytearray()
    while running:
        if ser and ser.in_waiting:
            byte = ser.read(1)
            buffer += byte
            if len(buffer) >= 6:
                pkt = buffer[:6]
                buffer = buffer[6:]  # 余剰バイトは残す
                calc_chk = sum(pkt[:5]) & 0xFF
                if pkt[5] == calc_chk:
                    out(f"[ASYNC RX] {' '.join(f'{x:02X}' for x in pkt)}")
                else:
                    out(f"[ASYNC RX WARN] Checksum mismatch: {' '.join(f'{x:02X}' for x in pkt)}")
        else:
            time.sleep(0.01)

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
        