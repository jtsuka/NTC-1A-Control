#!/usr/bin/env python3
import argparse, csv, time
from datetime import datetime
import serial

BOOT_MAGIC = bytes((0xDE,0xAD,0xBE,0xEF,0x01,0x7F))
TELEMETRY_LEN = 6
TELEMETRY_FOOTER = 0x7F
TELEMETRY_VALUE_MAX = 99

def is_telemetry(pkt):
    return (len(pkt)==6 and pkt[5]==TELEMETRY_FOOTER
            and all(0 <= b <= TELEMETRY_VALUE_MAX for b in pkt[:5]))

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--port", default="/dev/serial0")
    p.add_argument("--baud", type=int, default=9600)
    p.add_argument("--duration", type=float, default=120.0)
    p.add_argument("--output", default="pi_rx_evidence.csv")
    a=p.parse_args()

    ser=serial.Serial(a.port,a.baud,timeout=0.1,write_timeout=1)
    ser.reset_input_buffer()
    buf=bytearray()
    frames=boot=resync=0
    last=None
    max_interval=0.0
    start=time.monotonic()
    end=start+a.duration
    print(f"[START] port={a.port} baud={a.baud} duration={a.duration:.1f}s")

    with open(a.output,"w",newline="",encoding="utf-8") as f:
        w=csv.writer(f)
        w.writerow(["FrameNo","Timestamp","Monotonic_s","Raw_Hex",
                    "Interval_ms","Footer_OK","Wind_Length","Actual_Tension"])
        try:
            while time.monotonic() < end:
                n=ser.in_waiting
                if not n:
                    time.sleep(0.005)
                    continue
                buf += ser.read(n)
                while len(buf) >= TELEMETRY_LEN:
                    first6=bytes(buf[:6])
                    if first6 == BOOT_MAGIC:
                        boot += 1
                        del buf[:6]
                        continue
                    if is_telemetry(first6):
                        now=time.monotonic()
                        wall=datetime.now().astimezone().isoformat(timespec="milliseconds")
                        interval=""
                        if last is not None:
                            iv=(now-last)*1000.0
                            interval=f"{iv:.3f}"
                            max_interval=max(max_interval,iv)
                        last=now
                        b0,b1,b2,b3,b4,_=first6
                        frames += 1
                        w.writerow([frames,wall,f"{now-start:.6f}",
                                    " ".join(f"{x:02X}" for x in first6),
                                    interval,1,b2*10000+b1*100+b0,b4*10+b3])
                        del buf[:6]
                        continue
                    # Same conservative resync policy as v1.3.4:
                    # preserve 6..11 bytes because a legacy 8/12-byte frame
                    # could still be incomplete; at 12+ drop one byte.
                    if len(buf) < 12:
                        break
                    buf.pop(0)
                    resync += 1
        finally:
            ser.close()

    print("[DONE]")
    print(f"Telemetry_frames   : {frames}")
    print(f"Boot_magic_frames  : {boot}")
    print(f"Resync_drop_bytes  : {resync}")
    print(f"Max_interval_ms    : {max_interval:.3f}")
    print(f"Buffered_bytes_end : {len(buf)}")
    print(f"CSV                 : {a.output}")

if __name__ == "__main__":
    main()
