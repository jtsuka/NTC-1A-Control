# NTC_1A_serial_comm.py – ESP32 Fusion v1.2.1 完全対応版 v1.3
import serial, threading, time
import NTC_1A_utils  # 'from ... import out' ではなくモジュールごとインポート

PORT = "/dev/serial0"
BAUD = 9600
ser = None
running = True

# ---------- 共通チェックサム & ビルダー ----------
def checksum7(data):
    """7-bit 加算チェックサム (ESP32/TC実機互換)"""
    return sum(data) & 0x7F

def build_packet(payload):
    """末尾に7bit CSを付与したパケットを生成"""
    chk = checksum7(payload)
    return payload + [chk]

def open_port():
    global ser
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1, write_timeout=1)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        NTC_1A_utils.out(f"[INFO] Port open: {PORT}")
    except Exception as e:
        NTC_1A_utils.out(f"[ERROR] Port open failed: {e}")

def send_packet(payload):
    """CSを含まないデータ(5, 7, 11B)を送り、ESP32が認識できる6, 8, 12Bにする"""
    if not ser or not ser.is_open: return False
    pkt = build_packet(payload)
    try:
        ser.write(bytes(pkt))
        ser.flush()
        NTC_1A_utils.out(f"[TX] {' '.join(f'{x:02X}' for x in pkt)}")
        return True
    except Exception as e:
        NTC_1A_utils.out(f"[TX ERROR] {e}")
        return False

def rx_loop():
    """堅牢なパケット受信：常にスライドして6/8/12Bを探す"""
    global running
    buffer = bytearray()
    while running:
        if ser and ser.in_waiting:
            try:
                buffer += ser.read(ser.in_waiting)
                
                while len(buffer) >= 6:
                    found = False
                    # 長いパケットから順にチェックサムを確認
                    for length in [12, 8, 6]:
                        if len(buffer) >= length:
                            pkt = buffer[:length]
                            if pkt[length-1] == (sum(pkt[:length-1]) & 0x7F):
                                NTC_1A_utils.out(f"[RX OK] {' '.join(f'{x:02X}' for x in pkt)}")
                                buffer = buffer[length:]
                                found = True
                                break
                    
                    if found: continue # 次のパケットへ
                    
                    # どの長さでもCSが合わないなら1バイト捨ててスライド
                    buffer.pop(0)
            except Exception as e:
                NTC_1A_utils.out(f"[RX ERR] {e}")
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