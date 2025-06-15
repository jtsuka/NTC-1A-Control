# serial_comm.py
import serial, threading, time
from NTC_1A_utils import out

PORT = "/dev/ttyS0"  # 適宜変更
BAUD = 9600
ser = None
running = True

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
        return
    pkt = [ch, cmd, val, 0, 0]
    pkt.append(sum(pkt) & 0xFF)
    try:
        ser.write(bytes(pkt))
        ser.flush()
        out(f"[TX] {' '.join(f'{x:02X}' for x in pkt)}")
        time.sleep(0.1)
    except Exception as e:
        out(f"[TX ERROR] {e}")

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
        if ser and ser.is_open:
            try:
                b = ser.read(1)
                if not b: continue
                data = bytearray([b[0]])
                rem = read_exact(5, 0.5)
                if not rem: continue
                data.extend(rem)
                chk = sum(data[:5]) & 0xFF
                ok = chk == data[5]
                out("[RX] " + " ".join(f"{x:02X}" for x in data) + f" CHK:{'OK' if ok else 'NG'}")
            except Exception as e:
                out(f"[RX ERROR] {e}")
        else:
            time.sleep(0.1)

def start_serial_thread():
    open_port()
    threading.Thread(target=rx_loop, daemon=True).start()

def stop_serial():
    global running
    running = False
    if ser and ser.is_open:
        ser.close()
