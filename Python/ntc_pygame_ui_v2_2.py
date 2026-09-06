#!/usr/bin/env python3
# ntc_pygame_ui_v2_2.py
# Original controller-like layout preserved.
# Phase3 regression-test UI is hidden unless started with --test-mode.

import pygame, sys
import argparse
import serial.tools.list_ports
from collections import deque
import NTC_1A_utils
from NTC_1A_serial_comm_v1_3_3 import (
    start_serial_thread, stop_serial,
    send_phase3_reset, send_phase3_tension, send_phase3_safe, send_phase3_sensadj,
    send_test_checksum_ng, send_test_invalid_command,
    send_test_reset_early_detect, send_test_stage1_false_packet,
    send_raw_hex,
)

# ==========================================
# 0. 起動オプション
# ==========================================
parser = argparse.ArgumentParser(description="NTC-1A Universal UI")
parser.add_argument(
    "--test-mode",
    action="store_true",
    help="Enable the hidden Phase3 parser regression-test UI."
)
parser.add_argument(
    "--port",
    default="/dev/serial0",
    help="Serial port to open at startup (default: /dev/serial0)."
)
args = parser.parse_args()

TEST_MODE_ENABLED = args.test_mode
STARTUP_PORT = args.port

# ==========================================
# 1. ログ・通信の初期設定
# ==========================================
log_lines = deque(maxlen=8)
def log_callback(msg):
    log_lines.append(msg)
NTC_1A_utils.out = log_callback

start_serial_thread(STARTUP_PORT)
NTC_1A_utils.out(f"[UI] mode={'TEST' if TEST_MODE_ENABLED else 'NORMAL'} port={STARTUP_PORT}")

# ==========================================
# 2. Pygame 基本設定
# ==========================================
pygame.init()
info = pygame.display.Info()
SCREEN_W, SCREEN_H = info.current_w, info.current_h
screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
pygame.display.set_caption("NTC-1A Universal UI")

FONT_SIZE = int(SCREEN_H * 0.05)
font = pygame.font.Font(None, FONT_SIZE)
small_font = pygame.font.Font(None, int(FONT_SIZE * 0.7))
tiny_font = pygame.font.Font(None, max(16, int(FONT_SIZE * 0.55)))
clock = pygame.time.Clock()

fields = {'CH1_TENSION':'','CH1_LENGTH':'','CH1_COUNT':'','CH2_TENSION':'','CH2_LENGTH':'','CH2_COUNT':''}
field_keys = list(fields.keys())
current_field = 0
label_map = {k: k.replace('_', ' ') for k in field_keys}

# TEST UI is available only when --test-mode is supplied.
# Without that option, the original GUI remains visually unchanged.
test_panel_open = False
raw_hex_text = ""
raw_input_active = False


def on_pad(val):
    global current_field
    key = field_keys[current_field]
    if val.isdigit(): fields[key] += val
    elif val == 'CLR': fields[key] = ''
    elif val == 'ENT': current_field = (current_field + 1) % len(field_keys)
    elif val == 'ADJ':
        send_phase3_sensadj()
    elif val == 'RESET':
        send_phase3_reset(
            int(fields['CH1_LENGTH'] or 0),
            int(fields['CH1_TENSION'] or 0),
            int(fields['CH2_LENGTH'] or 0),
            int(fields['CH2_TENSION'] or 0),
        )
    elif val == 'SEND':
        send_phase3_tension(
            int(fields['CH1_TENSION'] or 0),
            int(fields['CH2_TENSION'] or 0),
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
        self.label = label; self.action = action; self.pressed = False
        self.base_color = color
        self.text_font = text_font or font
    def draw(self, surf):
        color = (30,90,140) if self.pressed else self.base_color
        pygame.draw.rect(surf, color, self.rect, 0, 5)
        txt = self.text_font.render(self.label, True, (255,255,255))
        surf.blit(txt, (self.rect.centerx - txt.get_width()//2, self.rect.centery - txt.get_height()//2))
    def handle(self, pos, is_down):
        if is_down and self.rect.collidepoint(pos): self.pressed = True
        elif (not is_down) and self.pressed:
            self.pressed = False
            if self.rect.collidepoint(pos): self.action()


class Dropdown:
    def __init__(self, rect_ratio, font):
        self.rect = pygame.Rect(rect_ratio[0] * SCREEN_W, rect_ratio[1] * SCREEN_H, rect_ratio[2] * SCREEN_W, rect_ratio[3] * SCREEN_H)
        self.font = font; self.options = [STARTUP_PORT]; self.active_option = 0; self.expanded = False; self.refresh_ports()
    def refresh_ports(self):
        current = self.options[self.active_option] if self.options else "/dev/serial0"
        ports = serial.tools.list_ports.comports(); new_options = [p.device for p in ports]
        if "/dev/serial0" not in new_options:
            new_options.insert(0, "/dev/serial0")
        if STARTUP_PORT not in new_options:
            new_options.insert(0, STARTUP_PORT)
        self.options = new_options
        self.active_option = self.options.index(current) if current in self.options else 0
    def draw(self, surf):
        color = (50, 50, 60) if not self.expanded else (80, 80, 90); pygame.draw.rect(surf, color, self.rect, 0, 3)
        txt = self.font.render(self.options[self.active_option], True, (255, 255, 255)); surf.blit(txt, (self.rect.x + 10, self.rect.centery - txt.get_height()//2))
        if self.expanded:
            for i, opt in enumerate(self.options):
                opt_rect = self.rect.copy(); opt_rect.y += self.rect.height * (i + 1); pygame.draw.rect(surf, (40, 40, 45), opt_rect)
                opt_txt = self.font.render(opt, True, (255, 255, 255)); surf.blit(opt_txt, (opt_rect.x + 10, opt_rect.centery - opt_txt.get_height()//2))
    def handle(self, pos):
        if self.rect.collidepoint(pos): self.expanded = not self.expanded; (self.refresh_ports() if self.expanded else None); return None
        if self.expanded:
            for i in range(len(self.options)):
                opt_rect = self.rect.copy(); opt_rect.y += self.rect.height * (i + 1)
                if opt_rect.collidepoint(pos): self.active_option = i; self.expanded = False; return self.options[i]
            self.expanded = False
        return None


# ==========================================
# 4. 配置（既存配置は変更しない）
# ==========================================
buttons = []
buttons.append(Button((0.05, 0.05, 0.15, 0.08), 'SAFE ON',  lambda: send_phase3_safe(True)))
buttons.append(Button((0.22, 0.05, 0.15, 0.08), 'SAFE OFF', lambda: send_phase3_safe(False)))
buttons.append(Button((0.85, 0.02, 0.12, 0.06), 'EXIT', lambda: pygame.event.post(pygame.event.Event(pygame.QUIT)), color=(180, 50, 50)))

pad_layout = [['7','8','9','CLR'],['4','5','6','ENT'],['1','2','3','SEND'],['0','ADJ','RESET','']]
for r, row in enumerate(pad_layout):
    for c, lbl in enumerate(row):
        if not lbl: continue
        buttons.append(Button((0.62 + c*0.09, 0.52 + r*0.11, 0.08, 0.10), lbl, lambda l=lbl: on_pad(l)))

port_dropdown = Dropdown((0.40, 0.05, 0.25, 0.08), font)

# Only one small permanent addition, placed in unused left-middle space.
def toggle_test_panel():
    global test_panel_open, raw_input_active
    if not TEST_MODE_ENABLED:
        return
    test_panel_open = not test_panel_open
    if not test_panel_open:
        raw_input_active = False

test_toggle = Button((0.05, 0.49, 0.13, 0.07), 'TEST', toggle_test_panel, color=(90,90,110), text_font=small_font)

# Test panel is an overlay; it does not move any original UI element.
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
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT: raise SystemExit

            if ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE:
                    if test_panel_open:
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
                # TEST controls are enabled only with --test-mode.
                if TEST_MODE_ENABLED:
                    test_toggle.handle(click_pos, is_down)

                if TEST_MODE_ENABLED and test_panel_open:
                    if is_down:
                        raw_input_active = RAW_RECT.collidepoint(click_pos)
                    for b in test_buttons:
                        b.handle(click_pos, is_down)
                    # Clicks inside the overlay are consumed so original controls underneath cannot fire.
                    if TEST_PANEL_RECT.collidepoint(click_pos):
                        continue

                new_port = port_dropdown.handle(click_pos)
                if new_port:
                    NTC_1A_utils.out(f"[UI] Port change: {new_port}")
                    stop_serial(); start_serial_thread(new_port)

                if not port_dropdown.expanded:
                    for b in buttons: b.handle(click_pos, is_down)
                    if is_down:
                        for idx in range(len(field_keys)):
                            col, row = idx // 3, idx % 3
                            fx = int(SCREEN_W * (0.05 + col * 0.25))
                            fy = int(SCREEN_H * (0.15 + row * 0.11))
                            field_rect = pygame.Rect(fx + int(SCREEN_W*0.15), fy, int(SCREEN_W*0.08), int(SCREEN_H*0.08))
                            if field_rect.collidepoint(click_pos):
                                current_field = idx

        # --- 描画処理 ---
        screen.fill((30,30,35))
        for b in buttons: b.draw(screen)

        for idx, key in enumerate(field_keys):
            col, row = idx // 3, idx % 3
            lx = int(SCREEN_W * (0.05 + col * 0.25))
            fy = int(SCREEN_H * (0.15 + row * 0.11))
            screen.blit(font.render(label_map[key], True, (200,200,200)), (lx, fy + 5))
            rect = pygame.Rect(lx + int(SCREEN_W*0.15), fy, int(SCREEN_W*0.08), int(SCREEN_H*0.08))
            pygame.draw.rect(screen, (255,255,255) if idx==current_field else (100,100,100), rect, 2)
            val_txt = font.render(fields[key], True, (255,255,0) if idx==current_field else (255,255,255))
            screen.blit(val_txt, (rect.right - val_txt.get_width() - 5, rect.y + 5))

        for i, line in enumerate(list(log_lines)):
            screen.blit(small_font.render(line, True, (150,150,150)), (int(SCREEN_W*0.05), int(SCREEN_H*0.65) + i*int(SCREEN_H*0.04)))

        port_dropdown.draw(screen)

        if TEST_MODE_ENABLED:
            test_toggle.draw(screen)

        if TEST_MODE_ENABLED and test_panel_open:
            pygame.draw.rect(screen, (42,42,48), TEST_PANEL_RECT, 0, 6)
            pygame.draw.rect(screen, (105,105,120), TEST_PANEL_RECT, 2, 6)
            title = tiny_font.render("Phase3 parser regression tests", True, (220,220,220))
            screen.blit(title, (TEST_PANEL_RECT.x + 8, TEST_PANEL_RECT.y + 4))
            for b in test_buttons: b.draw(screen)

            pygame.draw.rect(screen, (25,25,28), RAW_RECT, 0, 3)
            pygame.draw.rect(screen, (255,255,255) if raw_input_active else (120,120,120), RAW_RECT, 2, 3)
            visible_raw = raw_hex_text[-34:]
            raw_txt = tiny_font.render(visible_raw or "HEX...", True, (255,255,0) if raw_input_active else (190,190,190))
            screen.blit(raw_txt, (RAW_RECT.x + 5, RAW_RECT.centery - raw_txt.get_height()//2))

        pygame.display.flip()
        clock.tick(30)

except SystemExit:
    pass
finally:
    stop_serial()
    pygame.quit()
    sys.exit(0)
