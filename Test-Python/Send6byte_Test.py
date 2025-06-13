import serial
import time

# ======== シリアルポートの設定 ========
PORT = '/dev/ttyUSB0'     # ← 必要に応じて書き換えてください
BAUDRATE = 9600
INTERVAL = 2.0            # 送信間隔（秒）

# ======== 送信する6バイトの固定パケット ========
# 例：CH=0x01, CMD=0x06, VAL=0x05, 他は0 + CHK
packet = bytearray([0x01, 0x06, 0x05, 0x00, 0x00, 0x0C])  # チェックサム = 0x0C

# ======== 送信ループ処理 ========
try:
    with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
        print(f"Start sending to {PORT} every {INTERVAL} sec...")
        while True:
            ser.write(packet)
            print(f"[Sent] {packet.hex(' ').upper()}")
            time.sleep(INTERVAL)

except serial.SerialException as e:
    print(f"[Serial Error] {e}")
except KeyboardInterrupt:
    print("Stopped by user.")
