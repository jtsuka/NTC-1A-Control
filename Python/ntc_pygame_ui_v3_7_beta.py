#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ntc_pygame_ui_v3_6_beta.py
# NTC-1A 1ch メインコントローラ GUI。
#
# v3.7 BETA (2026-11-25; BASE = v3.6 BETA, SHA256 cf23e596145e70bd):
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

GUI_REVISION = "v3.7-beta"
GUI_BASE = "v3.6-beta"

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
pygame.display.set_caption("NTC-1A Main Controller UI v3.7 BETA")

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
buttons.append(Button((0.85, 0.09, 0.12, 0.06), 'REBOOT', lambda: open_reboot_confirm(), color=(190, 110, 40)))

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
