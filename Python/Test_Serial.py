import serial

ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)
print("Listening on /dev/ttyAMA0...")

while True:
    data = ser.read(6)
    if data:
        print("Received:", ' '.join(f"{b:02X}" for b in data))
