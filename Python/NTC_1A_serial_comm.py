# NTC_1A_serial_comm.py – ESP32 Fusion v1.2.1 完全対応版
import serial, threading, time
from NTC_1A_utils import out

PORT = "/dev/serial0"
BAUD = 9600
ser = None
running = True

# ---------- 共通チェックサム & ビルダー ----------
def checksum7(data):
    """7-bit 加算チェックサム (ESP32/TC実機互換)"""
    return sum(data) & 0x7F

def build_packet(payload):
    """
    payload : list[int] (CMD + Data部)
    return  : list[int] (末尾に7bit CSを付与)
    """
    chk = checksum7(payload)
    return payload + [chk]

def open_port():
    global ser
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1, write_timeout=1)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        out(f"[INFO] Port open: {PORT}")
    except Exception as e:
        out(f"[ERROR] Port open failed: {e}")

def send_packet(payload):
    """
    payload : list[int] (CSを含まないデータ)
    例) [0x01, 100, 0, 0, 0] -> 6バイトパケットとして送信
    """
    if not ser or not ser.is_open: return False
    pkt = build_packet(payload)
    try:
        ser.write(bytes(pkt))
        ser.flush()
        out(f"[TX] {' '.join(f'{x:02X}' for x in pkt)}")
        return True
    except Exception as e:
        out(f"[TX ERROR] {e}")
        return False

def send_packet_raw(raw_bytes):
    """チェックサムが含まれていない場合は自動付与して送信"""
    if len(raw_bytes) in [5, 7, 11]:
        raw_bytes = build_packet(raw_bytes)
    try:
        ser.write(bytes(raw_bytes))
        ser.flush()
        out(f"[TX-RAW] {' '.join(f'{x:02X}' for x in raw_bytes)}")
    except Exception as e:
        out(f"[TX ERROR] {e}")

def rx_loop():
    """バックグラウンド受信スレッド：6, 8, 12バイトのパケットを自動検知"""
    global running
    buffer = bytearray()
    while running:
        if ser and ser.in_waiting:
            try:
                new_data = ser.read(ser.in_waiting)
                buffer += new_data
                
                # パケットスキャン
                processed = True
                while processed and len(buffer) >= 6:
                    processed = False
                    # 可能なパケット長を長い方から試行
                    for length in [12, 8, 6]:
                        if len(buffer) >= length:
                            pkt = buffer[:length]
                            # 7bitチェックサム検証
                            if pkt[length-1] == (sum(pkt[:length-1]) & 0x7F):
                                out(f"[RX OK] {' '.join(f'{x:02X}' for x in pkt)}")
                                buffer = buffer[length:]
                                processed = True
                                break
                    if not processed and len(buffer) >= 12:
                        # どの長さも合致せずバッファが溜まったら1バイト捨てる
                        buffer.pop(0)
                        processed = True
            except Exception as e:
                out(f"[RX ERR] {e}")
        else:
            time.sleep(0.02)

def start_serial_thread(port=None):
    global PORT
    if port: PORT = port
    open_port()
    t = threading.Thread(target=rx_loop, daemon=True)
    t.start()

def stop_serial():
    global running
    running = False
    if ser and ser.is_open: ser.close()