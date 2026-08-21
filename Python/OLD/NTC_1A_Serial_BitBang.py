#!/usr/bin/env python3
"""
NTC-1A Serial Communication (BitBang UART over GPIO)
- Replaces pigpio with RPi.GPIO for basic GPIO access
- Supports 300bps bitbang UART Tx/Rx
- For use with NTC-1A_GUI_MAIN_V8.3.py
"""

import RPi.GPIO as GPIO
import time
import threading

TX_PIN = 14  # BCM GPIO14
RX_PIN = 15  # BCM GPIO15
BAUD_RATE = 300
BIT_DURATION = 1.0 / BAUD_RATE
PACKET_SIZE = 6

recv_callback = None
running = False
recv_thread = None

def gpio_setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TX_PIN, GPIO.OUT)
    GPIO.output(TX_PIN, GPIO.HIGH)
    GPIO.setup(RX_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def gpio_cleanup():
    GPIO.cleanup()

def send_bit(byte):
    GPIO.output(TX_PIN, GPIO.LOW)
    time.sleep(BIT_DURATION)
    for i in range(8):
        bit = (byte >> i) & 0x01
        GPIO.output(TX_PIN, bit)
        time.sleep(BIT_DURATION)
    GPIO.output(TX_PIN, GPIO.HIGH)
    time.sleep(BIT_DURATION)

def send_packet(ch, cmd, val):
    pkt = [ch, cmd, val, 0, 0]
    pkt.append(sum(pkt) & 0xFF)
    for b in pkt:
        send_bit(b)
        time.sleep(BIT_DURATION * 3)

def recv_byte():
    timeout = time.time() + 2
    while GPIO.input(RX_PIN) == GPIO.HIGH:
        if time.time() > timeout:
            return None
    time.sleep(BIT_DURATION / 2)
    byte = 0
    for i in range(8):
        time.sleep(BIT_DURATION)
        bit = GPIO.input(RX_PIN)
        byte |= (bit << i)
    time.sleep(BIT_DURATION)  # stop bit
    return byte

def receive_loop():
    global running
    while running:
        pkt = []
        for _ in range(PACKET_SIZE):
            b = recv_byte()
            if b is None:
                break
            pkt.append(b)
        if len(pkt) == PACKET_SIZE:
            if recv_callback:
                recv_callback(pkt)
        time.sleep(0.01)

def start_serial_thread(callback=None):
    global running, recv_thread, recv_callback
    recv_callback = callback
    gpio_setup()
    running = True
    recv_thread = threading.Thread(target=receive_loop, daemon=True)
    recv_thread.start()

def stop_serial():
    global running
    running = False
    time.sleep(0.2)
    gpio_cleanup()

def set_timeout(val):
    pass  # dummy for compatibility

# Debug run
if __name__ == "__main__":
    def show(pkt):
        print("[RECV]", ' '.join(f"{b:02X}" for b in pkt))
    start_serial_thread(show)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        stop_serial()
