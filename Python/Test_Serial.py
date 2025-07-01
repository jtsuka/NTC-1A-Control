# uart_rx_test.py
import serial

ser = serial.Serial('/dev/serial0', 9600, timeout=1)
print("Listening on /dev/serial0...")

while True:
    data = ser.read(6)  # 6バイト固定長パケット
    if data:
        print("Received:", ' '.join(f"{b:02X}" for b in data))
    else:
        print("[TIMEOUT] No data")
