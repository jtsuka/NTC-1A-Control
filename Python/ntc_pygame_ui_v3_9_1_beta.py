#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ntc_pygame_ui_v3_9_1_beta.py
# NTC-1A 1ch メインコントローラ GUI。
#
# v3.9.1 BETA (2026-11-28; BASE = v3.9 BETA, SHA256 abe1e9dd17a4bc2c):
# - Fix WIFI SETUP SAVE for a NEW network. The `connection add` call passed the
#   internal label 'psk' as wifi-sec.key-mgmt, which nmcli rejects ("failed to
#   modify 802-11-wireless-security.key-mgmt ... not valid"); NetworkManager's
#   value for WPA/WPA2-personal is 'wpa-psk' ('sae' for WPA3 was already right).
#   Found on the real Pi on 2026-11-28; the v3.9 headless test used an nmcli
#   stand-in that did not validate values (now it does). Nothing else changed.
#
# v3.9 BETA (2026-11-28; BASE = v3.8 BETA, SHA256 6fe255d2c88f9453):
# - Add WIFI SETUP: a full-screen page to join a Wi-Fi network from the touch
#   panel alone (no keyboard, no ssh). Step 1: pick an SSID from the list of
#   networks around the Pi (2.4 GHz only, because the Pi Zero 2 W has no 5 GHz;
#   strongest first; duplicates merged), or MANUAL to type one. Step 2: type the
#   password on an on-screen ANSI (US) keyboard (SHIFT, CAPS, symbols, SHOW
#   toggle). SAVE only stores the network as a NetworkManager profile and does
#   not touch the current connection; CONNECT NOW stores it and switches to it
#   (the current connection, and any ssh session, drops).
# - Implemented with `nmcli` (NetworkManager). If nmcli is refused for lack of
#   privilege it is retried once with `sudo -n`; any failure is shown on screen
#   and nothing else changes. The password is passed to nmcli as an argument
#   (visible to `ps` for a moment), is masked on screen, is never written to
#   the log / evidence log / stdout, and is cleared when the page closes. The
#   page closes by itself after 180 s without input.
# - Limits: ASCII only (ANSI keyboard), 2.4 GHz only, WPA/WPA2/WPA3-personal
#   and open networks only (802.1X / WEP rows are greyed out and refused).
#   Typing is accepted on touch release so a duplicate touch+mouse event pair
#   cannot type a key twice.
# - Layout only: EXIT / REBOOT / WIFI SCAN / WIFI SETUP are now evenly spaced
#   (pitch 0.13 of the screen height). REBOOT moved from y=0.18 to y=0.15 and
#   WIFI SCAN from y=0.36 to y=0.28; WIFI SETUP is at y=0.41. The gap between
#   EXIT and REBOOT is therefore 0.07 (was 0.10); REBOOT still needs a
#   confirmation tap.
# - Everything else is unchanged from v3.8.
#
# v3.8 BETA (2026-11-28; BASE = v3.7 BETA, SHA256 66a05427623de68d):
# - Add a WIFI SCAN button two steps below REBOOT, and a small readout panel
#   to its left. Tapping the button starts a live Wi-Fi link monitor (signal
#   strength in dBm with a GOOD / FAIR / WEAK / BAD label, SSID, frequency,
#   rx / tx bitrate, min..max signal since start, tx-failed count since
#   start); tapping it again (WIFI STOP) stops it and hides the panel.
#   Use it with the Pi on a battery to find where the signal is good enough.
# - The monitor only READS the link state of this Pi (`iw dev wlan0 link` and
#   `iw dev wlan0 station dump`, once per second, in a background thread). It
#   does not send anything to the ESP32 / TC106, does not change the Wi-Fi
#   settings, and does not touch the evidence log. Nothing runs until the
#   button is tapped.
# - The quality labels are a rough guide chosen for this project (GOOD >= -60,
#   FAIR >= -70, WEAK >= -80 dBm), not a standard.
# - Everything else (layout of existing parts, REBOOT, command path, RX
#   parsing, logging format, startup port selection) is unchanged from v3.7.
#
# v3.7 BETA (2026-11-25; BASE = v3.6 BETA, SHA256 cf23e596145e70bd):
# - 2026-11-26: the REBOOT button was moved one step further down (y 0.09 ->
#   0.18) so it is not easily hit by mistake next to EXIT. Layout only.
# - Add a REBOOT button under EXIT. It reboots this Raspberry Pi (the kiosk
#   host), nothing else: no command is sent to the ESP32 / TC106, and the
#   ESP32 and TC106 keep running while the Pi restarts.
# - Tapping REBOOT only opens a confirmation dialog (CANCEL / REBOOT). The
#   dialog cancels itself after 10 s and on ESC. Nothing is rebooted by a
#   single stray tap.
# - Reboot is requested with `systemctl reboot --no-ask-password`, falling
#   back to `sudo -n systemctl reboot`. If both are refused (e.g. missing
#   privilege) the GUI stays up, keeps logging and shows REBOOT FAILED.
# - SIGTERM / SIGHUP now end the GUI the same way EXIT does, so the
#   evidence log (CSV) is stopped, flushed and fsync'ed before the OS goes
#   down. (Before, a SIGTERM killed the process without closing the log.)
# - A NOTE line `REBOOT` is written to the evidence log when reboot is
#   accepted. Everything else (layout of existing parts, command path, RX
#   parsing, logging format, startup port selection) is unchanged from v3.6.
#
# v3.6 BETA (2026-11-23; successor of v2.6; BASE = v2.6):
# - Add receive/transmit evidence logging (NTC_1A_rxlog_v3_6_beta): raw bytes,
#   parsed Direction-B frames, RX status changes, dropped bytes, commands sent,
#   operator marks. CSV files under ~/ntc_logs (see --log-dir / --no-log).
# - Select NTC_1A_serial_comm_v3_6_beta (parsing identical to v1.3.5 BETA;
#   adds only an optional observer hook used by the logger).
# - Add three small mark buttons (RUN / STOP / MARK) that write NOTE lines.
# - Rename the on-screen label ACTUAL TENSION -> TENS ADC. The value is the
#   TC106's 16-sample average of the tension-sensor ADC input (tens_actual,
#   10-bit count 0..1023, not zero-corrected, not a force in gf). Only the
#   label text changes; the internal field key and all logic are unchanged.
# - Startup port selection (USB tty auto-detect, --port priority), command
#   path (SEND / RESET / SENS.ADJ / SAFE), layout of the existing fields,
#   hidden test panel and the 0..1023 tension display are unchanged from v2.6.
# - This file does not change the launcher, v2.4, v2.5 BETA or v2.6.
#
# v2.6 (2026-11-04):
# - 変更点は「起動時のシリアルポート選択」のみ。
#   --port を省略した場合、最初に見つかったUSB tty
#   (/dev/ttyACM* を優先、次に /dev/ttyUSB*)を起動時に開き、
#   ポート選択ドロップダウンの先頭にも表示する。
#   USB tty が無ければ v2.5 と同じく /dev/serial0 を使う。
#   --port を明示した場合は、常にその指定を優先する。
# - ドロップダウンの並び順は「起動ポート、USB tty、/dev/serial0、その他」。
# - 通信プロトコル、画面レイアウト、各コマンド、テレメトリ処理、
#   隠しTESTパネルは v2.5 から変更なし。
# - シリアル層は従来どおり NTC_1A_serial_comm_v1_3_5_beta を使用(未変更)。
#
# v2.5 BETA (2026-10-24):
# - NTC_1A_serial_comm_v1_3_5_beta を選択(方向B テンション 0..1023)。
# - 読み取り専用の ACTUAL_TENSION 表示を、既存のモニタ値ボックスに収める。
# - v2.4 のレイアウト、全コマンド、隠しTESTパネルの動作は維持。
# - 本ファイルは既定のGUIランチャーおよび v2.4 を変更しない。
#
# v2.4:
# - v2.3 の機能をベースに、プロトコルは変更しない。
# - 右側の ACTUAL/MONITOR 値ボックスだけを少し右へ移動し、
#   ラベルとの重なり(特に ACTUAL TENSION)を防ぐ。
# - 左側の SETTING 入力位置、テレメトリ処理、Boot Magic の扱い、
#   隠しPhase3回帰試験パネルは変更なし。
#
# 歴史的注記:
# 原典の Main ASM には実測値の表示ルーチンがあるが、調査した
# zhukongban.asm では disp_ten/disp_len を設定する経路に到達不能な
# 部分がある。本GUIは TC106 で確認済みのテレメトリ形式を直接使用しており、
# 原典のハードウェアが実行時に必ずそれらの表示モードに入っていたとは
# 主張しない。

import pygame, sys
import argparse
import os
import re
import signal
import subprocess
import threading
import time
import serial.tools.list_ports
from collections import deque
import NTC_1A_utils
import NTC_1A_serial_comm_v3_6_beta as comm
import NTC_1A_rxlog_v3_6_beta as rxlog
from NTC_1A_serial_comm_v3_6_beta import (
    start_serial_thread, stop_serial, get_telemetry,
    send_phase3_reset, send_phase3_tension, send_phase3_safe, send_phase3_sensadj,
    send_test_checksum_ng, send_test_invalid_command,
    send_test_reset_early_detect, send_test_stage1_false_packet,
    send_raw_hex,
)

GUI_REVISION = "v3.9.1-beta"
GUI_BASE = "v3.9-beta"

# ==========================================
# 0. 起動オプション
# ==========================================
DEFAULT_UART_PORT = "/dev/serial0"
USB_TTY_PREFIXES = ("/dev/ttyACM", "/dev/ttyUSB")


def _natural_key(name):
    return [int(t) if t.isdigit() else t for t in re.split(r"(\d+)", name)]


def order_ports(devices, startup=None):
    """Return a de-duplicated port list: startup, USB ttys, serial0, others."""
    devs = list(dict.fromkeys(devices))
    usb = sorted((d for d in devs if d.startswith(USB_TTY_PREFIXES)), key=_natural_key)
    ordered = []
    for p in [startup] + usb + [DEFAULT_UART_PORT] + devs:
        if p and p not in ordered:
            ordered.append(p)
    return ordered


def detect_usb_tty():
    """First USB tty (ttyACM before ttyUSB, natural order), or None."""
    try:
        devs = [p.device for p in serial.tools.list_ports.comports()]
    except Exception:
        return None
    usb = sorted((d for d in devs if d.startswith(USB_TTY_PREFIXES)), key=_natural_key)
    return usb[0] if usb else None


parser = argparse.ArgumentParser(description="NTC-1A Main Controller UI")
parser.add_argument(
    "--test-mode",
    action="store_true",
    help="Enable the hidden Phase3 parser regression-test UI."
)
parser.add_argument(
    "--port",
    default=None,
    help="Serial port to open at startup. Default: first USB tty "
         "(/dev/ttyACM*, /dev/ttyUSB*) if present, else /dev/serial0."
)
parser.add_argument(
    "--log-dir",
    default=None,
    help="Directory for receive/transmit evidence logs (default: ~/ntc_logs)."
)
parser.add_argument(
    "--no-log",
    action="store_true",
    help="Disable the receive/transmit evidence log."
)
args = parser.parse_args()

TEST_MODE_ENABLED = args.test_mode
if args.port:
    STARTUP_PORT, STARTUP_PORT_SRC = args.port, "--port"
else:
    _usb = detect_usb_tty()
    if _usb:
        STARTUP_PORT, STARTUP_PORT_SRC = _usb, "auto USB"
    else:
        STARTUP_PORT, STARTUP_PORT_SRC = DEFAULT_UART_PORT, "fallback"

# ==========================================
# 1. ログ・通信の初期設定
# ==========================================
log_lines = deque(maxlen=8)

def log_callback(msg):
    log_lines.append(msg)

NTC_1A_utils.out = log_callback

# Evidence logger. Registered BEFORE the port is opened so the first events
# (port open, ESP32 Boot Magic) are captured. A logger failure never stops the GUI.
logger = None
if not args.no_log:
    try:
        logger = rxlog.RxLogger(
            log_dir=args.log_dir,
            meta={"gui": GUI_REVISION, "base": GUI_BASE, "comm": comm.REVISION,
                  "port": STARTUP_PORT, "port_src": STARTUP_PORT_SRC,
                  "mode": 'TEST' if TEST_MODE_ENABLED else 'NORMAL'},
            get_status=lambda: get_telemetry()["rx_status"],
        )
        if logger.start():
            comm.set_log_hook(logger.hook)
            NTC_1A_utils.out(f"[LOG] {logger.path}")
        else:
            NTC_1A_utils.out(f"[LOG] disabled: {logger.disabled_reason}")
    except Exception as e:
        logger = None
        NTC_1A_utils.out(f"[LOG] disabled: {e}")

start_serial_thread(STARTUP_PORT)
NTC_1A_utils.out(f"[UI] mode={'TEST' if TEST_MODE_ENABLED else 'NORMAL'} port={STARTUP_PORT} ({STARTUP_PORT_SRC})")

# ==========================================
# 2. Pygame 基本設定
# ==========================================
pygame.init()
info = pygame.display.Info()
SCREEN_W, SCREEN_H = info.current_w, info.current_h
screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
pygame.display.set_caption("NTC-1A Main Controller UI v3.9.1 BETA")

FONT_SIZE = int(SCREEN_H * 0.05)
font = pygame.font.Font(None, FONT_SIZE)
small_font = pygame.font.Font(None, int(FONT_SIZE * 0.7))
tiny_font = pygame.font.Font(None, max(16, int(FONT_SIZE * 0.55)))
clock = pygame.time.Clock()

# Existing 2-column / 3-row layout is preserved.
# Left column: editable SETTING fields.
# Right column: read-only ACTUAL/MONITOR fields.
fields = {
    'SET_TENSION':'',
    'SET_LENGTH':'',
    'SET_COUNT':'',
    'ACTUAL_TENSION':'---',
    'WIND_LENGTH':'---',
    'RX_STATUS':'WAIT',
}
field_keys = list(fields.keys())
editable_field_keys = ['SET_TENSION', 'SET_LENGTH', 'SET_COUNT']
current_editable_index = 0

label_map = {
    'SET_TENSION':'SET TENSION',
    'SET_LENGTH':'SET LENGTH',
    'SET_COUNT':'SET COUNT',
    'ACTUAL_TENSION':'TENS ADC',
    'WIND_LENGTH':'WIND LENGTH',
    'RX_STATUS':'RX STATUS',
}

# v2.4 layout-only adjustment:
# Right-side read-only value boxes move right by 4% of screen width.
# On a 1024px-wide display this is about 41px.
RIGHT_MONITOR_VALUE_X_SHIFT = 0.04

def render_fitted_actual_tension(value, rect, color):
    """Fit a four-digit actual tension into the existing monitor value box.

    Use the normal font if it fits. On smaller screens reduce the font size,
    preserving the existing right alignment. Never clip digits off the right.
    """
    text = str(value)
    max_width = max(1, rect.width - 10)
    surface = font.render(text, True, color)
    if surface.get_width() <= max_width:
        return surface

    for size in range(max(12, FONT_SIZE - 1), 11, -1):
        candidate = pygame.font.Font(None, size).render(text, True, color)
        if candidate.get_width() <= max_width:
            return candidate

    # Very narrow displays: retain all digits rather than clip them.
    if surface.get_width() > max_width:
        target_height = max(1, round(surface.get_height() * max_width / surface.get_width()))
        return pygame.transform.smoothscale(surface, (max_width, target_height))
    return surface


# TEST UI is available only when --test-mode is supplied.
test_panel_open = False
raw_hex_text = ""
raw_input_active = False


def on_pad(val):
    global current_editable_index
    key = editable_field_keys[current_editable_index]

    if val.isdigit():
        fields[key] += val
    elif val == 'CLR':
        fields[key] = ''
    elif val == 'ENT':
        current_editable_index = (current_editable_index + 1) % len(editable_field_keys)
    elif val == 'ADJ':
        send_phase3_sensadj()
    elif val == 'RESET':
        # This GUI is intentionally 1ch. The legacy Phase3 packet still carries
        # CH2 fields, therefore send zero for CH2 without exposing it in the UI.
        send_phase3_reset(
            int(fields['SET_LENGTH'] or 0),
            int(fields['SET_TENSION'] or 0),
            0,
            0,
        )
    elif val == 'SEND':
        # Same policy as RESET: CH2 remains protocol-compatible but unused.
        send_phase3_tension(
            int(fields['SET_TENSION'] or 0),
            0,
        )


# ==========================================
# 3. 汎用UIクラス定義
# ==========================================
class Button:
    def __init__(self, rect_ratio, label, action, color=(70,130,180), text_font=None):
        self.rect = pygame.Rect(
            rect_ratio[0] * SCREEN_W, rect_ratio[1] * SCREEN_H,
            rect_ratio[2] * SCREEN_W, rect_ratio[3] * SCREEN_H
        )
        self.label = label
        self.action = action
        self.pressed = False
        self.base_color = color
        self.text_font = text_font or font

    def draw(self, surf):
        color = (30,90,140) if self.pressed else self.base_color
        pygame.draw.rect(surf, color, self.rect, 0, 5)
        txt = self.text_font.render(self.label, True, (255,255,255))
        surf.blit(txt, (self.rect.centerx - txt.get_width()//2, self.rect.centery - txt.get_height()//2))

    def handle(self, pos, is_down):
        if is_down and self.rect.collidepoint(pos):
            self.pressed = True
        elif (not is_down) and self.pressed:
            self.pressed = False
            if self.rect.collidepoint(pos):
                self.action()


class Dropdown:
    def __init__(self, rect_ratio, font):
        self.rect = pygame.Rect(
            rect_ratio[0] * SCREEN_W, rect_ratio[1] * SCREEN_H,
            rect_ratio[2] * SCREEN_W, rect_ratio[3] * SCREEN_H
        )
        self.font = font
        self.options = [STARTUP_PORT]
        self.active_option = 0
        self.expanded = False
        self.refresh_ports()

    def refresh_ports(self):
        current = self.options[self.active_option] if self.options else STARTUP_PORT
        ports = serial.tools.list_ports.comports()
        self.options = order_ports([p.device for p in ports], STARTUP_PORT)
        new_options = self.options
        self.active_option = self.options.index(current) if current in self.options else 0

    def draw(self, surf):
        color = (50, 50, 60) if not self.expanded else (80, 80, 90)
        pygame.draw.rect(surf, color, self.rect, 0, 3)
        txt = self.font.render(self.options[self.active_option], True, (255, 255, 255))
        surf.blit(txt, (self.rect.x + 10, self.rect.centery - txt.get_height()//2))
        if self.expanded:
            for i, opt in enumerate(self.options):
                opt_rect = self.rect.copy()
                opt_rect.y += self.rect.height * (i + 1)
                pygame.draw.rect(surf, (40, 40, 45), opt_rect)
                opt_txt = self.font.render(opt, True, (255, 255, 255))
                surf.blit(opt_txt, (opt_rect.x + 10, opt_rect.centery - opt_txt.get_height()//2))

    def handle(self, pos):
        if self.rect.collidepoint(pos):
            self.expanded = not self.expanded
            if self.expanded:
                self.refresh_ports()
            return None
        if self.expanded:
            for i in range(len(self.options)):
                opt_rect = self.rect.copy()
                opt_rect.y += self.rect.height * (i + 1)
                if opt_rect.collidepoint(pos):
                    self.active_option = i
                    self.expanded = False
                    return self.options[i]
            self.expanded = False
        return None


# ==========================================
# 4. 配置
# ==========================================
buttons = []
buttons.append(Button((0.05, 0.05, 0.15, 0.08), 'SAFE ON',  lambda: send_phase3_safe(True)))
buttons.append(Button((0.22, 0.05, 0.15, 0.08), 'SAFE OFF', lambda: send_phase3_safe(False)))
buttons.append(Button((0.85, 0.02, 0.12, 0.06), 'EXIT', lambda: pygame.event.post(pygame.event.Event(pygame.QUIT)), color=(180, 50, 50)))
# v3.7: reboot this Pi (opens a confirmation dialog; see section 4b).
# v3.9: EXIT / REBOOT / WIFI SCAN / WIFI SETUP are evenly spaced (pitch 0.13).
buttons.append(Button((0.85, 0.15, 0.12, 0.06), 'REBOOT', lambda: open_reboot_confirm(), color=(190, 110, 40)))
# v3.8: live Wi-Fi signal monitor (two steps below REBOOT; see section 4c).
WIFI_BTN_COLOR_OFF = (50, 110, 130)
WIFI_BTN_COLOR_ON = (50, 140, 90)
wifi_button = Button((0.85, 0.28, 0.12, 0.06), 'WIFI SCAN', lambda: toggle_wifi_monitor(),
                     color=WIFI_BTN_COLOR_OFF, text_font=small_font)
buttons.append(wifi_button)
# v3.9: Wi-Fi setup page (SSID list + on-screen keyboard; see section 4d).
buttons.append(Button((0.85, 0.41, 0.12, 0.06), 'WIFI SETUP', lambda: open_wsetup(),
                      color=(110, 90, 150), text_font=small_font))

pad_layout = [['7','8','9','CLR'],['4','5','6','ENT'],['1','2','3','SEND'],['0','ADJ','RESET','']]
for r, row in enumerate(pad_layout):
    for c, lbl in enumerate(row):
        if not lbl:
            continue
        buttons.append(Button((0.62 + c*0.09, 0.52 + r*0.11, 0.08, 0.10), lbl, lambda l=lbl: on_pad(l)))

port_dropdown = Dropdown((0.40, 0.05, 0.25, 0.08), font)


def log_note(text):
    """Write an operator mark into the evidence log (no effect on control)."""
    if logger is not None:
        logger.note(text)
        NTC_1A_utils.out(f"[LOG] NOTE {text}")
    else:
        NTC_1A_utils.out("[LOG] off: mark not recorded")


# ==========================================
# 4b. Pi REBOOT (v3.7)
# ==========================================
REBOOT_CONFIRM_TIMEOUT_S = 10.0   # confirmation dialog closes itself
REBOOT_WAIT_TIMEOUT_S = 60.0      # accepted but SIGTERM never came -> resume UI
REBOOT_COMMANDS = (
    ['systemctl', 'reboot', '--no-ask-password'],
    ['sudo', '-n', 'systemctl', 'reboot', '--no-ask-password'],
)

reboot_confirm_open = False
reboot_confirm_deadline = 0.0
reboot_requested = False          # True once the OS accepted the reboot
reboot_requested_at = 0.0


def open_reboot_confirm():
    global reboot_confirm_open, reboot_confirm_deadline
    reboot_confirm_open = True
    reboot_confirm_deadline = time.monotonic() + REBOOT_CONFIRM_TIMEOUT_S


def close_reboot_confirm():
    global reboot_confirm_open
    reboot_confirm_open = False


def run_reboot():
    """Ask the OS to reboot the Pi. The GUI is NOT torn down here: when the OS
    accepts, systemd stops this session with SIGTERM and the handler below
    ends the main loop normally (stop_serial, logger.stop, fsync). If the OS
    refuses, the GUI keeps running and reports why."""
    global reboot_confirm_open, reboot_requested, reboot_requested_at
    reboot_confirm_open = False
    last_error = 'no command ran'
    for cmd in REBOOT_COMMANDS:
        try:
            r = subprocess.run(cmd, stdin=subprocess.DEVNULL, capture_output=True,
                               text=True, timeout=10)
        except Exception as e:  # missing binary, timeout, ...
            last_error = f"{cmd[0]}: {e}"
            continue
        if r.returncode == 0:
            log_note('REBOOT')
            NTC_1A_utils.out('[UI] REBOOT accepted by OS')
            reboot_requested = True
            reboot_requested_at = time.monotonic()
            return True
        tail = (r.stderr or r.stdout or '').strip().splitlines()
        last_error = f"{' '.join(cmd[:2])} rc={r.returncode} {tail[-1] if tail else ''}".strip()
    NTC_1A_utils.out(f"[UI] REBOOT FAILED: {last_error[:60]}")
    return False


reboot_confirm_buttons = [
    Button((0.29, 0.52, 0.19, 0.10), 'CANCEL', close_reboot_confirm, color=(70, 90, 110)),
    Button((0.52, 0.52, 0.19, 0.10), 'REBOOT', run_reboot, color=(190, 70, 50)),
]
REBOOT_DIALOG_RECT = pygame.Rect(int(SCREEN_W*0.25), int(SCREEN_H*0.30),
                                 int(SCREEN_W*0.50), int(SCREEN_H*0.40))


# ==========================================
# 4c. WIFI signal monitor (v3.8)
# ==========================================
WIFI_IFACE = 'wlan0'
WIFI_POLL_S = 1.0          # one reading per second while the monitor is on
WIFI_STALE_S = 5.0         # no new reading for this long -> show STALE
WIFI_IW_CANDIDATES = ('iw', '/usr/sbin/iw')

wifi_monitor_on = False
wifi_stop_event = threading.Event()
wifi_state = {'status': 'OFF'}   # replaced as a whole by the worker thread
wifi_thread = None


def _first(pattern, text):
    m = re.search(pattern, text, re.M)
    return m.group(1) if m else None


def parse_wifi(link_text, station_text):
    """Parse `iw dev <if> link` and `iw dev <if> station dump` text.
    Returns a dict, or None when there is no signal value (not connected)."""
    link = link_text or ''
    sta = station_text or ''
    sig = _first(r'^\s*signal:\s*(-?\d+)', sta) or _first(r'^\s*signal:\s*(-?\d+)', link)
    if sig is None:
        return None
    rx = _first(r'rx bitrate:\s*([\d.]+)', sta) or _first(r'rx bitrate:\s*([\d.]+)', link)
    tx = _first(r'tx bitrate:\s*([\d.]+)', sta) or _first(r'tx bitrate:\s*([\d.]+)', link)
    failed = _first(r'tx failed:\s*(\d+)', sta)
    return {
        'signal': int(sig),
        'rx': float(rx) if rx else None,
        'tx': float(tx) if tx else None,
        'failed': int(failed) if failed else None,
        'ssid': _first(r'^\s*SSID:\s*(.+?)\s*$', link),
        'freq': _first(r'^\s*freq:\s*(\d+)', link),
    }


def _run_iw(args):
    """Run iw and return its stdout, or None if iw cannot be run at all."""
    for exe in WIFI_IW_CANDIDATES:
        try:
            r = subprocess.run([exe] + args, stdin=subprocess.DEVNULL, capture_output=True,
                               text=True, timeout=3)
            return r.stdout or ''
        except FileNotFoundError:
            continue
        except Exception:
            return None
    return None


def _wifi_worker(stop_event):
    global wifi_state
    smin = smax = base_failed = None
    while not stop_event.is_set():
        link = _run_iw(['dev', WIFI_IFACE, 'link'])
        sta = _run_iw(['dev', WIFI_IFACE, 'station', 'dump'])
        if link is None and sta is None:
            new_state = {'status': 'NO IW', 't': time.time()}
        else:
            info = parse_wifi(link, sta)
            if info is None:
                new_state = {'status': 'NO LINK', 't': time.time()}
            else:
                sig = info['signal']
                smin = sig if smin is None else min(smin, sig)
                smax = sig if smax is None else max(smax, sig)
                if base_failed is None and info['failed'] is not None:
                    base_failed = info['failed']
                delta = None
                if info['failed'] is not None and base_failed is not None:
                    delta = max(0, info['failed'] - base_failed)
                info.update({'status': 'OK', 'smin': smin, 'smax': smax,
                             'fail_delta': delta, 't': time.time()})
                new_state = info
        if stop_event.is_set():
            break
        wifi_state = new_state
        stop_event.wait(WIFI_POLL_S)


def toggle_wifi_monitor():
    global wifi_monitor_on, wifi_stop_event, wifi_state, wifi_thread
    if wifi_monitor_on:
        wifi_stop_event.set()
        wifi_monitor_on = False
        wifi_state = {'status': 'OFF'}
        wifi_button.label = 'WIFI SCAN'
        wifi_button.base_color = WIFI_BTN_COLOR_OFF
    else:
        wifi_stop_event = threading.Event()
        wifi_state = {'status': 'WAIT'}
        wifi_thread = threading.Thread(target=_wifi_worker, args=(wifi_stop_event,), daemon=True)
        wifi_thread.start()
        wifi_monitor_on = True
        wifi_button.label = 'WIFI STOP'
        wifi_button.base_color = WIFI_BTN_COLOR_ON


def wifi_quality(dbm):
    """Rough guide for this project (not a standard)."""
    if dbm >= -60:
        return 'GOOD', (110, 210, 120)
    if dbm >= -70:
        return 'FAIR', (230, 210, 90)
    if dbm >= -80:
        return 'WEAK', (240, 150, 60)
    return 'BAD', (235, 80, 80)


# Readout panel: left of the WIFI SCAN button, above the key pad (y < 0.52),
# right of the monitor value boxes (x > 0.57).
WIFI_PANEL_RECT = pygame.Rect(int(SCREEN_W*0.60), int(SCREEN_H*0.27),
                              int(SCREEN_W*0.24), int(SCREEN_H*0.23))


def draw_wifi_panel(surf):
    if not wifi_monitor_on:
        return
    st = wifi_state
    r = WIFI_PANEL_RECT
    pygame.draw.rect(surf, (36, 36, 42), r, 0, 6)
    pygame.draw.rect(surf, (90, 110, 125), r, 2, 6)
    x = r.x + 8
    y = r.y + 5
    status = st.get('status')
    if status == 'OK' and time.time() - st.get('t', 0) > WIFI_STALE_S:
        status = 'STALE'
    line_h = int(SCREEN_H * 0.037)
    if status == 'OK':
        label, col = wifi_quality(st['signal'])
        big = font.render(f"{st['signal']} dBm", True, col)
        surf.blit(big, (x, y))
        tag = small_font.render(label, True, col)
        surf.blit(tag, (r.right - tag.get_width() - 8, y + (big.get_height() - tag.get_height()) // 2))
        y += big.get_height() + 3
        rx = '--' if st.get('rx') is None else f"{st['rx']:.0f}"
        tx = '--' if st.get('tx') is None else f"{st['tx']:.0f}"
        delta = '--' if st.get('fail_delta') is None else f"+{st['fail_delta']}"
        lines = [
            (st.get('ssid') or '-')[:22],
            f"{st.get('freq') or '--'} MHz  {rx}/{tx} Mb/s",
            f"range {st['smin']}..{st['smax']}",
            f"tx failed {delta}",
        ]
        for line in lines:
            surf.blit(tiny_font.render(line, True, (200, 200, 205)), (x, y))
            y += line_h
    else:
        msg = {'WAIT': ('measuring...', (200, 200, 205)),
               'NO LINK': ('NO LINK', (235, 80, 80)),
               'NO IW': ('iw not found', (235, 80, 80)),
               'STALE': ('STALE', (240, 150, 60))}.get(status, (str(status), (200, 200, 205)))
        surf.blit(font.render(msg[0], True, msg[1]), (x, y))



# ==========================================
# 4d. WIFI SETUP (v3.9): SSID list + on-screen ANSI keyboard + nmcli
# ==========================================
WSETUP_IDLE_S = 180.0            # close by itself after this long without input
WSETUP_SCAN_TIMEOUT_S = 25
WSETUP_CMD_TIMEOUT_S = 20
WSETUP_CONNECT_TIMEOUT_S = 45
WSETUP_MAX_ROWS = 6
SSID_MAX_BYTES = 32
PSK_MIN, PSK_MAX = 8, 63
SHIFT_MAP = dict(zip("`1234567890-=[]\\;',./", '~!@#$%^&*()_+{}|:"<>?'))
GREEN, RED, ORANGE, GREY = (110, 210, 120), (235, 80, 80), (240, 150, 60), (200, 200, 205)

wsetup_open = False
wsetup_mode = 'LIST'             # 'LIST' (pick a network) or 'INPUT' (SSID / password)
wsetup_nets = []                 # [{'ssid','signal','sec','dbm','in_use'}]
wsetup_scroll = 0
wsetup_ssid = ''
wsetup_pass = ''
wsetup_sec = None                # None = typed by hand (security unknown), else nmcli SECURITY text
wsetup_focus = 'PASS'
wsetup_show_pass = False
wsetup_shift = False
wsetup_caps = False
wsetup_msg = ''
wsetup_msg_color = GREY
wsetup_busy = ''                 # '' or a label such as 'scanning'
wsetup_last_input = 0.0
wsetup_token = 0                 # a worker result is used only if its token is still current
wsetup_pressed_key = None
wsetup_row_down = None


def _split_terse(line):
    """Split one `nmcli -t` line on unescaped ':' and undo the escaping."""
    parts = re.split(r'(?<!\\):', line)
    return [p.replace('\\:', ':').replace('\\\\', '\\') for p in parts]


def parse_nm_wifi_list(text):
    """Parse `nmcli -t -f IN-USE,SSID,SIGNAL,SECURITY,FREQ dev wifi list`.
    2.4 GHz only, hidden (empty) SSIDs dropped, duplicate SSIDs merged to the
    strongest, sorted strongest first. dbm is the usual NetworkManager
    percent<->dBm approximation (percent = 2 * (dBm + 100))."""
    best = {}
    for line in (text or '').splitlines():
        f = _split_terse(line)
        if len(f) < 5:
            continue
        in_use, ssid, sig, sec, freq = f[0], f[1], f[2], f[3], f[4]
        if not ssid:
            continue
        try:
            sig_i = max(0, min(100, int(sig)))
        except ValueError:
            sig_i = 0
        m = re.match(r'\s*(\d+)', freq)
        if m and int(m.group(1)) >= 3000:
            continue
        sec = (sec or '').strip()
        if sec == '--':
            sec = ''
        cur = best.get(ssid)
        if cur is None or sig_i > cur['signal']:
            best[ssid] = {'ssid': ssid, 'signal': sig_i, 'sec': sec,
                          'dbm': sig_i // 2 - 100, 'in_use': in_use.strip() == '*'}
    return sorted(best.values(), key=lambda n: (-n['signal'], n['ssid']))


def wsetup_key_mgmt(sec):
    """'open' | 'psk' | 'sae' | 'unsupported' from an nmcli SECURITY string."""
    tokens = (sec or '').upper().split()
    if not tokens:
        return 'open'
    if '802.1X' in tokens or 'WEP' in tokens:
        return 'unsupported'
    if 'WPA3' in tokens and not ({'WPA', 'WPA1', 'WPA2'} & set(tokens)):
        return 'sae'
    return 'psk'


# NetworkManager's names for the labels used here ('psk' is not a valid value).
NM_KEY_MGMT = {'psk': 'wpa-psk', 'sae': 'sae'}


def wsetup_kind():
    if wsetup_sec is None:                        # typed by hand
        return 'psk' if wsetup_pass else 'open'
    return wsetup_key_mgmt(wsetup_sec)


def wsetup_pass_editable():
    return wsetup_sec is None or wsetup_key_mgmt(wsetup_sec) in ('psk', 'sae')


def _redact(text, secrets):
    for sec in secrets:
        if sec:
            text = text.replace(sec, '***')
    return text


def _nm_run(args, timeout, secrets=()):
    """Run nmcli; retry once with `sudo -n` only when it is refused for lack of
    privilege. Returns (ok, stdout, message); message never contains a secret."""
    for attempt, prefix in enumerate(([], ['sudo', '-n'])):
        cmd = prefix + ['nmcli'] + args
        try:
            r = subprocess.run(cmd, stdin=subprocess.DEVNULL, capture_output=True,
                               text=True, timeout=timeout)
        except FileNotFoundError:
            return False, '', f'{cmd[0]}: not found'
        except subprocess.TimeoutExpired:
            return False, '', 'timed out'
        except Exception as e:
            return False, '', type(e).__name__
        if r.returncode == 0:
            return True, r.stdout or '', ''
        text = (r.stderr or r.stdout or '').strip()
        low = text.lower()
        if attempt == 0 and any(k in low for k in ('privilege', 'not authorized', 'permission', 'polkit')):
            continue
        tail = text.splitlines()[-1] if text else f'rc={r.returncode}'
        return False, '', _redact(tail, secrets)[:70]
    return False, '', 'sudo refused'


def _wsetup_set_msg(text, color=GREY):
    global wsetup_msg, wsetup_msg_color
    wsetup_msg, wsetup_msg_color = text, color


def _wsetup_start(label, fn):
    """Run fn(token) in a background thread; one job at a time."""
    global wsetup_busy
    if wsetup_busy:
        return False
    wsetup_busy = label
    token = wsetup_token

    def run():
        global wsetup_busy
        try:
            fn(token)
        except Exception as e:  # never let a worker kill the GUI
            if token == wsetup_token:
                _wsetup_set_msg(f'error: {type(e).__name__}', RED)
        finally:
            wsetup_busy = ''

    threading.Thread(target=run, daemon=True).start()
    return True


def _wsetup_scan_worker(token):
    global wsetup_nets, wsetup_scroll
    fields = ['-t', '-f', 'IN-USE,SSID,SIGNAL,SECURITY,FREQ', 'dev', 'wifi', 'list']
    ok, out, msg = _nm_run(fields + ['--rescan', 'yes'], WSETUP_SCAN_TIMEOUT_S)
    if not ok:                                    # a forced rescan needs more rights
        ok, out, msg = _nm_run(fields, WSETUP_SCAN_TIMEOUT_S)
    if token != wsetup_token:
        return
    if not ok:
        _wsetup_set_msg(f'scan failed: {msg}', RED)
        return
    wsetup_nets = parse_nm_wifi_list(out)
    wsetup_scroll = 0
    _wsetup_set_msg(f'{len(wsetup_nets)} network(s), 2.4 GHz' if wsetup_nets
                    else 'no 2.4 GHz network found', GREY if wsetup_nets else ORANGE)


def wsetup_rescan():
    _wsetup_start('scanning', _wsetup_scan_worker)


def _wsetup_save_worker(token, ssid, pw, kind, connect):
    secrets = (pw,)
    ok, out, msg = _nm_run(['-t', '-f', 'NAME', 'connection', 'show'], WSETUP_CMD_TIMEOUT_S)
    names = set(_split_terse(l)[0] for l in out.splitlines()) if ok else set()
    if ssid in names and kind == 'psk':           # same network again: just update the password
        ok, _o, msg = _nm_run(['connection', 'modify', ssid, 'wifi-sec.key-mgmt', 'wpa-psk',
                               'wifi-sec.psk', pw], WSETUP_CMD_TIMEOUT_S, secrets)
    else:
        if ssid in names:                         # other kind: replace the profile
            ok, _o, msg = _nm_run(['connection', 'delete', ssid], WSETUP_CMD_TIMEOUT_S, secrets)
            if not ok:
                if token == wsetup_token:
                    _wsetup_set_msg(f'save failed: {msg}', RED)
                return
        args = ['connection', 'add', 'type', 'wifi', 'ifname', WIFI_IFACE, 'con-name', ssid,
                'ssid', ssid, 'connection.autoconnect', 'yes']
        if kind in ('psk', 'sae'):
            args += ['wifi-sec.key-mgmt', NM_KEY_MGMT[kind], 'wifi-sec.psk', pw]
        ok, _o, msg = _nm_run(args, WSETUP_CMD_TIMEOUT_S, secrets)
    if not ok:
        if token == wsetup_token:
            _wsetup_set_msg(f'save failed: {msg}', RED)
        NTC_1A_utils.out('[UI] WIFI SETUP: save failed')
        return
    NTC_1A_utils.out('[UI] WIFI SETUP: profile saved')
    if not connect:
        if token == wsetup_token:
            _wsetup_set_msg('SAVED (not connected yet)', GREEN)
        return
    ok, _o, msg = _nm_run(['-w', str(WSETUP_CONNECT_TIMEOUT_S - 5), 'connection', 'up', ssid],
                          WSETUP_CONNECT_TIMEOUT_S, secrets)
    NTC_1A_utils.out('[UI] WIFI SETUP: connected' if ok else '[UI] WIFI SETUP: connect failed')
    if token == wsetup_token:
        _wsetup_set_msg('CONNECTED' if ok else f'saved, but connect failed: {msg}',
                        GREEN if ok else ORANGE)


def wsetup_validate():
    """Returns (kind, error). kind is None when the input cannot be used."""
    if not wsetup_ssid:
        return None, 'SSID is empty'
    if len(wsetup_ssid.encode('utf-8')) > SSID_MAX_BYTES:
        return None, 'SSID is longer than 32 bytes'
    kind = wsetup_kind()
    if kind == 'unsupported':
        return None, 'not supported (802.1X / WEP)'
    if kind in ('psk', 'sae') and not (PSK_MIN <= len(wsetup_pass) <= PSK_MAX):
        return None, f'password must be {PSK_MIN}-{PSK_MAX} characters'
    return kind, ''


def wsetup_submit(connect):
    if wsetup_busy:
        return
    kind, err = wsetup_validate()
    if kind is None:
        _wsetup_set_msg(err, RED)
        return
    ssid = wsetup_ssid
    pw = wsetup_pass if kind in ('psk', 'sae') else ''
    _wsetup_set_msg('')
    _wsetup_start('connecting' if connect else 'saving',
                  lambda tok: _wsetup_save_worker(tok, ssid, pw, kind, connect))


def open_wsetup():
    global wsetup_open, wsetup_mode, wsetup_token, wsetup_last_input
    global wsetup_nets, wsetup_scroll, wsetup_ssid, wsetup_pass, wsetup_sec
    global wsetup_show_pass, wsetup_shift, wsetup_caps
    wsetup_open = True
    wsetup_mode = 'LIST'
    wsetup_token += 1
    wsetup_last_input = time.monotonic()
    wsetup_nets, wsetup_scroll = [], 0
    wsetup_ssid = wsetup_pass = ''
    wsetup_sec = None
    wsetup_show_pass = wsetup_shift = wsetup_caps = False
    _wsetup_set_msg('')
    wsetup_rescan()


def close_wsetup():
    global wsetup_open, wsetup_token, wsetup_pass, wsetup_ssid
    wsetup_open = False
    wsetup_token += 1                              # ignore any result still on its way
    wsetup_pass = ''                               # do not keep the password around
    wsetup_ssid = ''


def wsetup_back():
    global wsetup_mode, wsetup_pass
    wsetup_mode = 'LIST'
    wsetup_pass = ''
    _wsetup_set_msg('')


def wsetup_manual():
    global wsetup_mode, wsetup_ssid, wsetup_pass, wsetup_sec, wsetup_focus, wsetup_show_pass
    wsetup_mode = 'INPUT'
    wsetup_ssid = wsetup_pass = ''
    wsetup_sec = None
    wsetup_focus = 'SSID'
    wsetup_show_pass = False
    _wsetup_set_msg('type the SSID, then the password')


def wsetup_select(idx):
    global wsetup_mode, wsetup_ssid, wsetup_pass, wsetup_sec, wsetup_focus, wsetup_show_pass
    if not (0 <= idx < len(wsetup_nets)):
        return
    net = wsetup_nets[idx]
    if wsetup_key_mgmt(net['sec']) == 'unsupported':
        _wsetup_set_msg('not supported (802.1X / WEP)', RED)
        return
    wsetup_mode = 'INPUT'
    wsetup_ssid, wsetup_pass, wsetup_sec = net['ssid'], '', net['sec']
    wsetup_show_pass = False
    wsetup_focus = 'PASS' if wsetup_pass_editable() else ''
    _wsetup_set_msg('' if wsetup_pass_editable() else 'open network: no password')


def wsetup_scroll_by(delta):
    global wsetup_scroll
    top = max(0, len(wsetup_nets) - WSETUP_MAX_ROWS)
    wsetup_scroll = max(0, min(top, wsetup_scroll + delta))


def wsetup_toggle_show():
    global wsetup_show_pass
    wsetup_show_pass = not wsetup_show_pass


def wsetup_type(ch):
    global wsetup_ssid, wsetup_pass
    if wsetup_focus == 'SSID' and wsetup_sec is None:
        if len((wsetup_ssid + ch).encode('utf-8')) <= SSID_MAX_BYTES:
            wsetup_ssid += ch
    elif wsetup_focus == 'PASS' and wsetup_pass_editable():
        if len(wsetup_pass) < PSK_MAX:
            wsetup_pass += ch
    _wsetup_set_msg('')


def wsetup_backspace():
    global wsetup_ssid, wsetup_pass
    if wsetup_focus == 'SSID' and wsetup_sec is None:
        wsetup_ssid = wsetup_ssid[:-1]
    elif wsetup_focus == 'PASS' and wsetup_pass_editable():
        wsetup_pass = wsetup_pass[:-1]


def wsetup_clear_field():
    global wsetup_ssid, wsetup_pass
    if wsetup_focus == 'SSID' and wsetup_sec is None:
        wsetup_ssid = ''
    elif wsetup_focus == 'PASS' and wsetup_pass_editable():
        wsetup_pass = ''


def wsetup_enter():
    global wsetup_focus
    if wsetup_focus == 'SSID' and wsetup_pass_editable():
        wsetup_focus = 'PASS'


def _ws_layout():
    """ANSI (US) keyboard: 5 rows x 15 units wide, in the lower half of the screen."""
    rows = [
        (0.0, [(c, 1.0) for c in "`1234567890-="] + [('BKSP', 2.0)]),
        (0.5, [(c, 1.0) for c in "qwertyuiop[]"] + [('\\', 1.5)]),
        (0.75, [(c, 1.0) for c in "asdfghjkl;'"] + [('ENTER', 2.25)]),
        (0.0, [('SHIFT', 2.25)] + [(c, 1.0) for c in "zxcvbnm,./"] + [('SHIFT', 2.75)]),
        (0.0, [('CAPS', 1.75), ('SPACE', 11.25), ('CLR', 2.0)]),
    ]
    u = SCREEN_W * 0.92 / 15.0
    x0 = SCREEN_W * 0.04
    kh = SCREEN_H * 0.095
    y0 = SCREEN_H * 0.43
    pitch = SCREEN_H * 0.105
    keys = []
    for r, (off, items) in enumerate(rows):
        x = x0 + off * u
        for label, wu in items:
            rect = pygame.Rect(int(x), int(y0 + r * pitch), max(8, int(wu * u) - 3), int(kh))
            keys.append({'rect': rect, 'kind': 'char' if len(label) == 1 else label, 'base': label})
            x += wu * u
    return keys


wsetup_keys = _ws_layout()


def _ws_char_for(base):
    if base.isalpha():
        return base.upper() if (wsetup_shift != wsetup_caps) else base
    return SHIFT_MAP.get(base, base) if wsetup_shift else base


def wsetup_press_key(k):
    global wsetup_shift, wsetup_caps
    kind = k['kind']
    if kind == 'char':
        wsetup_type(_ws_char_for(k['base']))
        wsetup_shift = False                       # SHIFT applies to one key
    elif kind == 'SPACE':
        wsetup_type(' ')
    elif kind == 'BKSP':
        wsetup_backspace()
    elif kind == 'CLR':
        wsetup_clear_field()
    elif kind == 'ENTER':
        wsetup_enter()
    elif kind == 'SHIFT':
        wsetup_shift = not wsetup_shift
    elif kind == 'CAPS':
        wsetup_caps = not wsetup_caps


# --- geometry shared by drawing and touch handling -----------------------------------------
def _ws_row_rect(i):
    return pygame.Rect(int(SCREEN_W*0.04), int(SCREEN_H*(0.14 + i*0.10)),
                       int(SCREEN_W*0.80), int(SCREEN_H*0.09))


SSID_RECT = pygame.Rect(int(SCREEN_W*0.04), int(SCREEN_H*0.115), int(SCREEN_W*0.66), int(SCREEN_H*0.08))
PASS_RECT = pygame.Rect(int(SCREEN_W*0.04), int(SCREEN_H*0.235), int(SCREEN_W*0.66), int(SCREEN_H*0.08))

wsetup_buttons_list = [
    Button((0.86, 0.14, 0.10, 0.12), 'UP', lambda: wsetup_scroll_by(-3), color=(70, 75, 90), text_font=small_font),
    Button((0.86, 0.61, 0.10, 0.12), 'DN', lambda: wsetup_scroll_by(3), color=(70, 75, 90), text_font=small_font),
    Button((0.04, 0.82, 0.22, 0.10), 'RESCAN', wsetup_rescan, color=(60, 110, 130), text_font=small_font),
    Button((0.30, 0.82, 0.22, 0.10), 'MANUAL', wsetup_manual, color=(80, 90, 120), text_font=small_font),
    Button((0.74, 0.82, 0.22, 0.10), 'CLOSE', close_wsetup, color=(120, 80, 70), text_font=small_font),
]
wsetup_buttons_input = [
    Button((0.72, 0.115, 0.12, 0.08), 'BACK', wsetup_back, color=(120, 80, 70), text_font=small_font),
    Button((0.72, 0.235, 0.12, 0.08), 'SHOW', wsetup_toggle_show, color=(80, 90, 120), text_font=small_font),
    Button((0.04, 0.335, 0.20, 0.075), 'SAVE', lambda: wsetup_submit(False), color=(60, 110, 130), text_font=small_font),
    Button((0.27, 0.335, 0.26, 0.075), 'CONNECT NOW', lambda: wsetup_submit(True), color=(50, 140, 90), text_font=small_font),
]


def wsetup_click(pos, is_down):
    global wsetup_pressed_key, wsetup_row_down, wsetup_focus, wsetup_last_input
    wsetup_last_input = time.monotonic()
    if wsetup_mode == 'LIST':
        for b in wsetup_buttons_list:
            b.handle(pos, is_down)
        idx = None
        for i in range(WSETUP_MAX_ROWS):
            if wsetup_scroll + i < len(wsetup_nets) and _ws_row_rect(i).collidepoint(pos):
                idx = wsetup_scroll + i
        if is_down:
            wsetup_row_down = idx
        else:
            if idx is not None and idx == wsetup_row_down:
                wsetup_select(idx)
            wsetup_row_down = None
    else:
        for b in wsetup_buttons_input:
            b.handle(pos, is_down)
        if is_down:
            if SSID_RECT.collidepoint(pos) and wsetup_sec is None:
                wsetup_focus = 'SSID'
            elif PASS_RECT.collidepoint(pos) and wsetup_pass_editable():
                wsetup_focus = 'PASS'
            wsetup_pressed_key = next((k for k in wsetup_keys if k['rect'].collidepoint(pos)), None)
        else:
            k, wsetup_pressed_key = wsetup_pressed_key, None
            if k is not None and k['rect'].collidepoint(pos):
                wsetup_press_key(k)


def wsetup_keydown(ev):
    """Physical keyboard (optional) and ESC while the page is open."""
    global wsetup_last_input, wsetup_focus
    wsetup_last_input = time.monotonic()
    if ev.key == pygame.K_ESCAPE:
        if wsetup_mode == 'INPUT':
            wsetup_back()
        else:
            close_wsetup()
    elif wsetup_mode == 'INPUT':
        if ev.key == pygame.K_BACKSPACE:
            wsetup_backspace()
        elif ev.key == pygame.K_RETURN:
            wsetup_enter()
        elif ev.key == pygame.K_TAB:
            if wsetup_focus == 'SSID' and wsetup_pass_editable():
                wsetup_focus = 'PASS'
            elif wsetup_focus == 'PASS' and wsetup_sec is None:
                wsetup_focus = 'SSID'
        elif ev.unicode and len(ev.unicode) == 1 and 32 <= ord(ev.unicode) <= 126:
            wsetup_type(ev.unicode)


# --- drawing -------------------------------------------------------------------------------------
def _fit_tail(text, fnt, max_w):
    while text and fnt.size(text)[0] > max_w:
        text = text[1:]
    return text


def _fit_head(text, fnt, max_w):
    while text and fnt.size(text)[0] > max_w:
        text = text[:-1]
    return text


def _ws_draw_field(surf, rect, text, masked, focused, editable):
    pygame.draw.rect(surf, (22, 22, 26) if editable else (46, 46, 52), rect, 0, 4)
    pygame.draw.rect(surf, (255, 255, 255) if focused else (110, 110, 120), rect, 2, 4)
    shown = ('*' * len(text)) if masked else text
    if focused and int(time.time() * 2) % 2 == 0:
        shown += '|'
    shown = _fit_tail(shown, font, rect.w - 14)
    t = font.render(shown, True, (255, 255, 255) if editable else (170, 170, 175))
    surf.blit(t, (rect.x + 7, rect.centery - t.get_height() // 2))


def draw_wsetup(surf):
    surf.fill((28, 28, 34))
    title = font.render('WIFI SETUP', True, (255, 255, 255))
    surf.blit(title, (int(SCREEN_W * 0.04), int(SCREEN_H * 0.025)))
    sub = tiny_font.render('2.4 GHz only (Pi Zero 2 W)', True, (150, 150, 155))
    surf.blit(sub, (int(SCREEN_W * 0.04) + title.get_width() + 14, int(SCREEN_H * 0.04)))

    if wsetup_mode == 'LIST':
        status = (wsetup_busy + '...') if wsetup_busy else wsetup_msg
        color = ORANGE if wsetup_busy else wsetup_msg_color
        surf.blit(tiny_font.render(status[:70], True, color), (int(SCREEN_W * 0.04), int(SCREEN_H * 0.098)))
        for i in range(WSETUP_MAX_ROWS):
            n = wsetup_scroll + i
            if n >= len(wsetup_nets):
                break
            net = wsetup_nets[n]
            r = _ws_row_rect(i)
            ok = wsetup_key_mgmt(net['sec']) != 'unsupported'
            pygame.draw.rect(surf, (52, 56, 70) if ok else (40, 40, 44), r, 0, 5)
            info = f"~{net['dbm']} dBm  {net['sec'] or 'OPEN'}"
            it = tiny_font.render(info, True, (190, 190, 195) if ok else (120, 120, 125))
            surf.blit(it, (r.right - it.get_width() - 10, r.centery - it.get_height() // 2))
            name = ('* ' if net['in_use'] else '') + net['ssid']
            name = _fit_head(name, font, r.w - it.get_width() - 30)
            nt = font.render(name, True, (255, 255, 255) if ok else (130, 130, 135))
            surf.blit(nt, (r.x + 10, r.centery - nt.get_height() // 2))
        if len(wsetup_nets) > WSETUP_MAX_ROWS:
            pos = tiny_font.render(f'{wsetup_scroll + 1}-{min(len(wsetup_nets), wsetup_scroll + WSETUP_MAX_ROWS)}'
                                   f'/{len(wsetup_nets)}', True, (170, 170, 175))
            surf.blit(pos, (int(SCREEN_W * 0.86), int(SCREEN_H * 0.40)))
        for b in wsetup_buttons_list:
            b.draw(surf)
    else:
        surf.blit(tiny_font.render('SSID', True, (170, 170, 175)), (SSID_RECT.x, SSID_RECT.y - int(SCREEN_H * 0.03)))
        surf.blit(tiny_font.render('PASSWORD', True, (170, 170, 175)), (PASS_RECT.x, PASS_RECT.y - int(SCREEN_H * 0.03)))
        _ws_draw_field(surf, SSID_RECT, wsetup_ssid, False, wsetup_focus == 'SSID', wsetup_sec is None)
        editable = wsetup_pass_editable()
        _ws_draw_field(surf, PASS_RECT, wsetup_pass if editable else '', not wsetup_show_pass,
                       wsetup_focus == 'PASS', editable)
        for b in wsetup_buttons_input:
            b.draw(surf)
        status = (wsetup_busy + '...') if wsetup_busy else wsetup_msg
        color = ORANGE if wsetup_busy else wsetup_msg_color
        surf.blit(tiny_font.render(_fit_head(status, tiny_font, int(SCREEN_W * 0.43)), True, color),
                  (int(SCREEN_W * 0.55), int(SCREEN_H * 0.355)))
        for k in wsetup_keys:
            kind = k['kind']
            active = (kind == 'SHIFT' and wsetup_shift) or (kind == 'CAPS' and wsetup_caps)
            if k is wsetup_pressed_key:
                col = (30, 90, 140)
            elif active:
                col = (60, 130, 90)
            else:
                col = (70, 75, 90) if kind == 'char' else (52, 58, 76)
            pygame.draw.rect(surf, col, k['rect'], 0, 4)
            if kind == 'char':
                ch = _ws_char_for(k['base'])
                txt = font.render(ch, True, (255, 255, 255))
            else:
                txt = small_font.render(kind, True, (230, 230, 235))
            surf.blit(txt, (k['rect'].centerx - txt.get_width() // 2, k['rect'].centery - txt.get_height() // 2))



def _graceful_signal(signum, frame):
    """SIGTERM/SIGHUP (e.g. system shutdown) end the GUI like EXIT does."""
    raise SystemExit


for _sig_name in ('SIGTERM', 'SIGHUP'):
    try:
        signal.signal(getattr(signal, _sig_name), _graceful_signal)
    except (AttributeError, ValueError, OSError):
        pass


# Operator marks for remote-site analysis (e.g. correlate noise with motor state).
# Placed in the free band between the setting fields and the on-screen log.
buttons.append(Button((0.22, 0.49, 0.10, 0.07), 'RUN',  lambda: log_note('RUN'),  color=(60,110,80),  text_font=small_font))
buttons.append(Button((0.33, 0.49, 0.10, 0.07), 'STOP', lambda: log_note('STOP'), color=(120,80,70), text_font=small_font))
buttons.append(Button((0.44, 0.49, 0.10, 0.07), 'MARK', lambda: log_note('MARK'), color=(80,90,120), text_font=small_font))


def toggle_test_panel():
    global test_panel_open, raw_input_active
    if not TEST_MODE_ENABLED:
        return
    test_panel_open = not test_panel_open
    if not test_panel_open:
        raw_input_active = False


test_toggle = Button((0.05, 0.49, 0.13, 0.07), 'TEST', toggle_test_panel, color=(90,90,110), text_font=small_font)

TEST_PANEL_RECT = pygame.Rect(int(SCREEN_W*0.03), int(SCREEN_H*0.43), int(SCREEN_W*0.54), int(SCREEN_H*0.21))
RAW_RECT = pygame.Rect(int(SCREEN_W*0.23), int(SCREEN_H*0.565), int(SCREEN_W*0.24), int(SCREEN_H*0.052))

test_buttons = [
    Button((0.05, 0.455, 0.13, 0.052), 'CHK NG', send_test_checksum_ng, color=(100,80,80), text_font=tiny_font),
    Button((0.20, 0.455, 0.13, 0.052), 'CMD 99', send_test_invalid_command, color=(100,80,80), text_font=tiny_font),
    Button((0.35, 0.455, 0.13, 0.052), 'EARLY', send_test_reset_early_detect, color=(100,80,80), text_font=tiny_font),
    Button((0.05, 0.525, 0.13, 0.052), 'STAGE1', send_test_stage1_false_packet, color=(100,80,80), text_font=tiny_font),
]


def raw_send_action():
    global raw_hex_text
    if raw_hex_text.strip():
        send_raw_hex(raw_hex_text)


test_buttons.append(Button((0.49, 0.565, 0.07, 0.052), 'RAW', raw_send_action, color=(90,80,110), text_font=tiny_font))


# ==========================================
# 5. メインループ
# ==========================================
try:
    while True:
        # Refresh right-side read-only fields from one coherent telemetry snapshot.
        telemetry = get_telemetry()
        fields['ACTUAL_TENSION'] = '---' if telemetry['actual_tension'] is None else str(telemetry['actual_tension'])
        fields['WIND_LENGTH'] = '---' if telemetry['wind_length'] is None else str(telemetry['wind_length'])
        fields['RX_STATUS'] = telemetry['rx_status']

        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                raise SystemExit

            if ev.type == pygame.KEYDOWN and wsetup_open:
                wsetup_keydown(ev)
                continue

            if ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE:
                    if reboot_confirm_open:
                        close_reboot_confirm()
                    elif reboot_requested:
                        pass
                    elif test_panel_open:
                        test_panel_open = False
                        raw_input_active = False
                    else:
                        raise SystemExit
                elif test_panel_open and raw_input_active:
                    if ev.key == pygame.K_RETURN:
                        raw_send_action()
                    elif ev.key == pygame.K_BACKSPACE:
                        raw_hex_text = raw_hex_text[:-1]
                    else:
                        ch = ev.unicode.upper()
                        if ch in "0123456789ABCDEF ":
                            raw_hex_text += ch

            click_pos = None
            is_down = False
            if ev.type in (pygame.MOUSEBUTTONDOWN, pygame.MOUSEBUTTONUP):
                click_pos = ev.pos
                is_down = (ev.type == pygame.MOUSEBUTTONDOWN)
            elif ev.type in (pygame.FINGERDOWN, pygame.FINGERUP):
                click_pos = (int(ev.x * SCREEN_W), int(ev.y * SCREEN_H))
                is_down = (ev.type == pygame.FINGERDOWN)

            if click_pos:
                if reboot_requested:
                    continue
                if wsetup_open:
                    wsetup_click(click_pos, is_down)
                    continue
                if reboot_confirm_open:
                    for b in reboot_confirm_buttons:
                        b.handle(click_pos, is_down)
                    continue
                if TEST_MODE_ENABLED:
                    test_toggle.handle(click_pos, is_down)

                if TEST_MODE_ENABLED and test_panel_open:
                    if is_down:
                        raw_input_active = RAW_RECT.collidepoint(click_pos)
                    for b in test_buttons:
                        b.handle(click_pos, is_down)
                    if TEST_PANEL_RECT.collidepoint(click_pos):
                        continue

                new_port = port_dropdown.handle(click_pos)
                if new_port:
                    NTC_1A_utils.out(f"[UI] Port change: {new_port}")
                    stop_serial()
                    start_serial_thread(new_port)

                if not port_dropdown.expanded:
                    for b in buttons:
                        b.handle(click_pos, is_down)

                    # Right ACTUAL/MONITOR fields are never selectable.
                    if is_down:
                        for editable_index, key in enumerate(editable_field_keys):
                            idx = field_keys.index(key)
                            col, row = idx // 3, idx % 3
                            fx = int(SCREEN_W * (0.05 + col * 0.25))
                            fy = int(SCREEN_H * (0.15 + row * 0.11))
                            field_rect = pygame.Rect(
                                fx + int(SCREEN_W*0.15), fy,
                                int(SCREEN_W*0.08), int(SCREEN_H*0.08)
                            )
                            if field_rect.collidepoint(click_pos):
                                current_editable_index = editable_index

        if reboot_confirm_open and time.monotonic() > reboot_confirm_deadline:
            close_reboot_confirm()
        if wsetup_open and not wsetup_busy and time.monotonic() - wsetup_last_input > WSETUP_IDLE_S:
            close_wsetup()
            NTC_1A_utils.out('[UI] WIFI SETUP: closed (idle)')
        if reboot_requested and time.monotonic() - reboot_requested_at > REBOOT_WAIT_TIMEOUT_S:
            reboot_requested = False
            NTC_1A_utils.out('[UI] REBOOT: no shutdown signal within 60 s, UI resumed')

        # --- 描画処理 ---
        screen.fill((30,30,35))
        for b in buttons:
            b.draw(screen)

        current_key = editable_field_keys[current_editable_index]

        for idx, key in enumerate(field_keys):
            col, row = idx // 3, idx % 3
            lx = int(SCREEN_W * (0.05 + col * 0.25))
            fy = int(SCREEN_H * (0.15 + row * 0.11))
            screen.blit(font.render(label_map[key], True, (200,200,200)), (lx, fy + 5))

            value_x = lx + int(SCREEN_W*0.15)
            if key not in editable_field_keys:
                value_x += int(SCREEN_W * RIGHT_MONITOR_VALUE_X_SHIFT)

            rect = pygame.Rect(
                value_x, fy,
                int(SCREEN_W*0.08), int(SCREEN_H*0.08)
            )

            is_editable = key in editable_field_keys
            is_current = key == current_key
            if is_current:
                border_color = (255,255,255)
            elif is_editable:
                border_color = (100,100,100)
            else:
                border_color = (75,110,125)
            pygame.draw.rect(screen, border_color, rect, 2)

            if is_current:
                value_color = (255,255,0)
            elif key == 'RX_STATUS':
                value_color = (210,230,230)
            else:
                value_color = (255,255,255)

            if key == 'ACTUAL_TENSION':
                val_txt = render_fitted_actual_tension(fields[key], rect, value_color)
            else:
                val_txt = font.render(fields[key], True, value_color)
            screen.blit(val_txt, (rect.right - val_txt.get_width() - 5, rect.y + 5))

        for i, line in enumerate(list(log_lines)):
            screen.blit(small_font.render(line, True, (150,150,150)), (int(SCREEN_W*0.05), int(SCREEN_H*0.65) + i*int(SCREEN_H*0.04)))

        port_dropdown.draw(screen)

        draw_wifi_panel(screen)

        # Evidence-log indicator (top band, between the port selector and EXIT).
        if logger is None:
            log_label, log_color = 'LOG OFF', (140, 140, 140)
        elif logger.healthy:
            log_label, log_color = logger.status_text(), (120, 200, 120)
        else:
            log_label, log_color = logger.status_text(), (230, 90, 90)
        screen.blit(tiny_font.render(log_label, True, log_color),
                    (int(SCREEN_W * 0.67), int(SCREEN_H * 0.075)))

        if TEST_MODE_ENABLED:
            test_toggle.draw(screen)

        if TEST_MODE_ENABLED and test_panel_open:
            pygame.draw.rect(screen, (42,42,48), TEST_PANEL_RECT, 0, 6)
            pygame.draw.rect(screen, (105,105,120), TEST_PANEL_RECT, 2, 6)
            title = tiny_font.render("Phase3 parser regression tests", True, (220,220,220))
            screen.blit(title, (TEST_PANEL_RECT.x + 8, TEST_PANEL_RECT.y + 4))
            for b in test_buttons:
                b.draw(screen)

            pygame.draw.rect(screen, (25,25,28), RAW_RECT, 0, 3)
            pygame.draw.rect(screen, (255,255,255) if raw_input_active else (120,120,120), RAW_RECT, 2, 3)
            visible_raw = raw_hex_text[-34:]
            raw_txt = tiny_font.render(visible_raw or "HEX...", True, (255,255,0) if raw_input_active else (190,190,190))
            screen.blit(raw_txt, (RAW_RECT.x + 5, RAW_RECT.centery - raw_txt.get_height()//2))

        if wsetup_open:
            draw_wsetup(screen)

        if reboot_confirm_open:
            dim = pygame.Surface((SCREEN_W, SCREEN_H), pygame.SRCALPHA)
            dim.fill((0, 0, 0, 160))
            screen.blit(dim, (0, 0))
            pygame.draw.rect(screen, (45, 45, 52), REBOOT_DIALOG_RECT, 0, 8)
            pygame.draw.rect(screen, (190, 110, 40), REBOOT_DIALOG_RECT, 3, 8)
            t1 = font.render('Reboot this Raspberry Pi?', True, (255, 255, 255))
            screen.blit(t1, (REBOOT_DIALOG_RECT.centerx - t1.get_width()//2, REBOOT_DIALOG_RECT.y + int(SCREEN_H*0.05)))
            t2 = tiny_font.render('GUI and logging stop. ESP32 / TC106 keep running.', True, (210, 210, 210))
            screen.blit(t2, (REBOOT_DIALOG_RECT.centerx - t2.get_width()//2, REBOOT_DIALOG_RECT.y + int(SCREEN_H*0.14)))
            left = max(0, int(reboot_confirm_deadline - time.monotonic()) + 1)
            t3 = tiny_font.render(f'Auto-cancel in {left} s', True, (160, 160, 160))
            screen.blit(t3, (REBOOT_DIALOG_RECT.centerx - t3.get_width()//2, REBOOT_DIALOG_RECT.y + int(SCREEN_H*0.19)))
            for b in reboot_confirm_buttons:
                b.draw(screen)
        elif reboot_requested:
            screen.fill((20, 20, 24))
            t1 = font.render('REBOOTING...', True, (255, 200, 90))
            screen.blit(t1, (SCREEN_W//2 - t1.get_width()//2, SCREEN_H//2 - t1.get_height()//2))

        pygame.display.flip()
        clock.tick(30)

except SystemExit:
    pass
finally:
    wifi_stop_event.set()
    wsetup_token += 1
    stop_serial()
    comm.set_log_hook(None)
    if logger is not None:
        logger.stop()
    try:
        os.sync()   # push the closed CSV to disk before the OS goes down
    except (AttributeError, OSError):
        pass
    pygame.quit()
    sys.exit(0)
