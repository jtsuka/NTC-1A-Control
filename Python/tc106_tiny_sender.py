#!/usr/bin/env python3
"""
tc106_tiny_sender.py

Phase3 v0.8正式PASS仕様に基づく最小対話式送信スクリプト。
Pi Zero W -> /dev/ttyUSB0 (FTDI) -> ESP32-S3 TinyTest_v2 -> TcMainTx -> Nano Every

スコープ: このTiny senderの作成のみ。旧Pi GUI/NTC_1A_serial_comm.py等の
本番コードは変更しない。ESP32/Nano側コードもここでは変更しない。
"""

import serial

PORT = "/dev/ttyUSB0"
BAUD = 9600

CMD_RESET    = 0x01
CMD_SEND     = 0x02
CMD_SENSADJ  = 0x03
CMD_SAFE_ON  = 0x10
CMD_SAFE_OFF = 0x11


def checksum7(data):
    return sum(data) & 0x7F


def build_reset(ch1_len, ch1_tens, ch2_len, ch2_tens):
    buf = [CMD_RESET]
    buf += [(ch1_len >> 16) & 0xFF, (ch1_len >> 8) & 0xFF, ch1_len & 0xFF]
    buf += [ch1_tens & 0xFF]
    buf += [(ch2_len >> 16) & 0xFF, (ch2_len >> 8) & 0xFF, ch2_len & 0xFF]
    buf += [ch2_tens & 0xFF]
    buf += [0x00, 0x00]  # reserved
    buf.append(checksum7(buf))
    return bytes(buf)


def build_send(ch1_tens, ch2_tens):
    buf = [CMD_SEND, ch1_tens & 0xFF, ch2_tens & 0xFF, 0x00, 0x00]
    buf.append(checksum7(buf))
    return bytes(buf)


def build_sensadj():
    buf = [CMD_SENSADJ] + [0x00] * 6
    buf.append(checksum7(buf))
    return bytes(buf)


def build_safe_on():
    buf = [CMD_SAFE_ON] + [0x00] * 4
    buf.append(checksum7(buf))
    return bytes(buf)


def build_safe_off():
    buf = [CMD_SAFE_OFF] + [0x00] * 4
    buf.append(checksum7(buf))
    return bytes(buf)


def hexdump(data):
    return " ".join(f"{b:02X}" for b in data)


def ask_int(prompt, default):
    s = input(f"{prompt} [{default}]: ").strip()
    return int(s) if s else default


def send(ser, data, label):
    ser.write(data)
    print(f"[SEND] {label}  len={len(data)}  raw={hexdump(data)}  checksum=0x{data[-1]:02X}")


def main():
    ser = serial.Serial(PORT, BAUD, bytesize=8, parity="N",
                         stopbits=1, rtscts=False, timeout=1)
    print(f"Opened {PORT} @ {BAUD}bps 8N1 (no flow control)")

    menu = """
1 : RESET
2 : SEND
3 : SENS.ADJ
4 : SAFE ON
5 : SAFE OFF
q : quit
"""
    while True:
        print(menu)
        choice = input("select> ").strip().lower()

        if choice == "1":
            ch1_len  = ask_int("CH1 length", 123456)
            ch1_tens = ask_int("CH1 tension", 30)
            ch2_len  = ask_int("CH2 length", 0)
            ch2_tens = ask_int("CH2 tension", 0)
            send(ser, build_reset(ch1_len, ch1_tens, ch2_len, ch2_tens), "RESET")

        elif choice == "2":
            ch1_tens = ask_int("CH1 tension", 50)
            ch2_tens = ask_int("CH2 tension", 0)
            send(ser, build_send(ch1_tens, ch2_tens), "SEND")

        elif choice == "3":
            send(ser, build_sensadj(), "SENS.ADJ")

        elif choice == "4":
            send(ser, build_safe_on(), "SAFE ON")

        elif choice == "5":
            send(ser, build_safe_off(), "SAFE OFF")

        elif choice == "q":
            break

        else:
            print("不明な選択です。1/2/3/4/5/q のいずれかを入力してください。")

    ser.close()
    print("closed.")


if __name__ == "__main__":
    main()
