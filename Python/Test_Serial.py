import serial

ser = serial.Serial('/dev/serial0', 9600, timeout=2)
print("Listening on /dev/serial0 (9600bps)...")

while True:
    data = ser.read(6)
    if data:
        print("RX:", ' '.join(f"{b:02X}" for b in data))
    else:
        print("[TIMEOUT] No data")
