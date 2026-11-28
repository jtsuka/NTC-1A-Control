#!/bin/bash
# NTC-1A kiosk launcher

APP_DIR="/home/ntc/NTC-1A-Control/Python"
PYTHON="/home/ntc/kivy-venv/bin/python3"
APP="ntc_pygame_ui.py"
LOG="/home/ntc/ntc_gui.log"

cd "$APP_DIR" || exit 1

export SDL_VIDEODRIVER=kmsdrm
export PYTHONUNBUFFERED=1

echo "=== NTC GUI START $(date) ===" >> "$LOG"

# Wait for display and USB devices to settle
sleep 5

exec "$PYTHON" "$APP" >> "$LOG" 2>&1
