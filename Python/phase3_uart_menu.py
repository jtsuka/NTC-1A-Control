#!/usr/bin/env python3
import sys
import time
import serial

DEFAULT_PORT = "/dev/ttyUSB0"
BAUD = 9600

def checksum7(data):
    return sum(data) & 0x7F

def send_raw_hex(ser, text):
    data = bytes.fromhex(text)
    ser.write(data)
    ser.flush()
    print("TX RAW :", " ".join(f"{b:02X}" for b in data))

def send_payload(ser, payload):
    pkt = bytes(payload + [checksum7(payload)])
    ser.write(pkt)
    ser.flush()
    print("TX     :", " ".join(f"{b:02X}" for b in pkt))

def ask_int(prompt, default=0, minimum=0, maximum=None):
    s = input(f"{prompt} [{default}]: ").strip()
    value = default if not s else int(s)
    if value < minimum or (maximum is not None and value > maximum):
        raise ValueError(f"range {minimum}..{maximum}")
    return value

def send_reset(ser):
    ch1_len = ask_int("CH1 LENGTH", 5, 0, 0xFFFFFF)
    ch1_ten = ask_int("CH1 TENSION", 10, 0, 255)
    ch2_len = ask_int("CH2 LENGTH", 0, 0, 0xFFFFFF)
    ch2_ten = ask_int("CH2 TENSION", 0, 0, 255)
    send_payload(ser, [
        0x01,
        (ch1_len >> 16) & 0xFF,
        (ch1_len >> 8) & 0xFF,
        ch1_len & 0xFF,
        ch1_ten,
        (ch2_len >> 16) & 0xFF,
        (ch2_len >> 8) & 0xFF,
        ch2_len & 0xFF,
        ch2_ten,
        0, 0
    ])

def send_send(ser):
    ch1 = ask_int("CH1 TENSION", 10, 0, 255)
    ch2 = ask_int("CH2 TENSION", 0, 0, 255)
    send_payload(ser, [0x02, ch1, ch2, 0, 0])

def menu(port):
    print(f"""
========================================
 Phase3 UART Test Menu
 Port: {port}  Baud: {BAUD}
========================================
 1 : RESET
 2 : SEND
 3 : SENS.ADJ
 4 : SAFE ON
 5 : SAFE OFF
 6 : Checksum NG test
 7 : Invalid command 0x99
 8 : RESET early-detect prevention test
 9 : Stage1 false-packet reproduction
10 : RAW HEX send
 p : Change serial port
 q : Quit
========================================""")

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    ser = None

    def reopen():
        nonlocal ser
        if ser and ser.is_open:
            ser.close()
        ser = serial.Serial(port, BAUD, timeout=0.1, write_timeout=1)
        print(f"[OPEN] {port} @ {BAUD}")

    try:
        reopen()
        while True:
            menu(port)
            cmd = input("Select > ").strip().lower()
            try:
                if cmd == "1":
                    send_reset(ser)
                elif cmd == "2":
                    send_send(ser)
                elif cmd == "3":
                    send_payload(ser, [0x03,0,0,0,0,0,0])
                elif cmd == "4":
                    send_payload(ser, [0x10,0,0,0,0])
                elif cmd == "5":
                    send_payload(ser, [0x11,0,0,0,0])
                elif cmd == "6":
                    send_raw_hex(ser, "10 00 00 00 00 11")
                    print("EXPECT : no [PiTask RX]")
                elif cmd == "7":
                    send_raw_hex(ser, "99")
                    print("EXPECT : no [PiTask RX]")
                elif cmd == "8":
                    send_raw_hex(ser, "01 00 00 00 00 01")
                    print("Wait 3 sec. EXPECT: no len=6 RX")
                    time.sleep(3)
                    send_raw_hex(ser, "00 00 00 00 00 02")
                    print("EXPECT : one len=12 RX")
                elif cmd == "9":
                    send_raw_hex(ser, "00 00 00 00 10 10")
                    print("EXPECT : no RX for this 6-byte sequence")
                    print("NOTE   : reboot ESP before next parser test")
                elif cmd == "10":
                    raw = input("HEX > ").strip()
                    if raw:
                        send_raw_hex(ser, raw)
                elif cmd == "p":
                    new_port = input(f"Port [{port}] > ").strip()
                    if new_port:
                        port = new_port
                        reopen()
                elif cmd == "q":
                    break
                else:
                    print("Unknown selection")
            except Exception as e:
                print("[ERROR]", e)
    finally:
        if ser and ser.is_open:
            ser.close()
        print("[CLOSE]")

if __name__ == "__main__":
    main()
