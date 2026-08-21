import serial

ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)
ser.reset_input_buffer()

while True:
    if ser.in_waiting >= 6:
        data = ser.read(6)
        print("Received:", ' '.join(f'{b:02X}' for b in data))
