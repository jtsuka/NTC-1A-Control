# uart_rx_test.py
import serial

ser = serial.Serial('/dev/serial0', 9600, timeout=1)
print("Listening on /dev/serial0...")

while True:
    data = ser.read(6)  # 6�o�C�g�Œ蒷�p�P�b�g
    if data:
        print("Received:", ' '.join(f"{b:02X}" for b in data))
    else:
        print("[TIMEOUT] No data")
