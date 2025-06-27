import serial

ser = serial.Serial("/dev/ttyAMA0", 9600)
while True:
    data = ser.read(6)
    print("Received:", " ".join(f"{b:02X}" for b in data))
