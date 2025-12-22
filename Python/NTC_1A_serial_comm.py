# NTC_1A_serial_comm.py – v1.3.1 完全対応版
import serial, threading, time
import NTC_1A_utils 

PORT = "/dev/serial0"
BAUD = 9600
ser = None
running = True

def checksum7(data):
    """7-bit 加算チェックサム [cite: 15, 16]"""
    return sum(data) & 0x7F

def build_packet(payload):
    return payload + [checksum7(payload)]

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
    """CSを含まない5, 7, 11Bのリストをパケット化して送信"""
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
    """堅牢なパケット検知ロジック (0x01ヘッダー縛りからの脱却)"""
    global running
    buffer = bytearray()
    while running:
        if ser and ser.in_waiting:
            try:
                buffer += ser.read(ser.in_waiting)
                while len(buffer) >= 6:
                    found = False
                    for length in [12, 8, 6]:
                        if len(buffer) >= length:
                            pkt = buffer[:length]
                            if pkt[length-1] == (sum(pkt[:length-1]) & 0x7F):
                                NTC_1A_utils.out(f"[RX OK] {' '.join(f'{x:02X}' for x in pkt)}")
                                buffer = buffer[length:]
                                found = True
                                break
                    if found: continue
                    buffer.pop(0) # 同期が合わない場合は1バイトスライド
            except Exception as e:
                NTC_1A_utils.out(f"[RX ERR] {e}")
        else:
            time.sleep(0.02)

def start_serial_thread(port=None):
    global PORT
    if port: PORT = port
    open_port()
    threading.Thread(target=rx_loop, daemon=True).start()

def stop_serial():
    global running
    running = False
    if ser and ser.is_open: ser.close()