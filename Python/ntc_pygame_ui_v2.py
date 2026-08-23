#!/usr/bin/env python3
# ntc_pygame_ui_v2.py - デバッグ支援機能追加版 (EXITボタン・Esc終了)

import pygame, sys
import serial.tools.list_ports
from collections import deque
import NTC_1A_utils
from NTC_1A_serial_comm import (
    start_serial_thread, stop_serial,
    send_phase3_reset, send_phase3_tension, send_phase3_safe, send_phase3_stop,
)

# ==========================================
# 1. ログ・通信の初期設定
# ==========================================
log_lines = deque(maxlen=8)
def log_callback(msg):
    log_lines.append(msg)
NTC_1A_utils.out = log_callback

start_serial_thread('/dev/serial0')

# ==========================================
# 2. Pygame 基本設定
# ==========================================
pygame.init()
info = pygame.display.Info()

# デバッグ用に少し小さめのウィンドウにする場合はここを調整
# SCREEN_W, SCREEN_H = 1024, 600  # 固定サイズにする場合
SCREEN_W, SCREEN_H = info.current_w, info.current_h

# Windows/Macで背面アプリが見えるようにするには、解像度を少し下げて実行するか、
# pygame.RESIZABLE フラグを検討してください
screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
pygame.display.set_caption("NTC-1A Universal UI")

FONT_SIZE = int(SCREEN_H * 0.05)
font = pygame.font.Font(None, FONT_SIZE)
small_font = pygame.font.Font(None, int(FONT_SIZE * 0.7))
clock = pygame.time.Clock()

fields = {'CH1_TENSION':'','CH1_LENGTH':'','CH1_COUNT':'','CH2_TENSION':'','CH2_LENGTH':'','CH2_COUNT':''}
field_keys = list(fields.keys())
current_field = 0
label_map = {k: k.replace('_', ' ') for k in field_keys}

def on_pad(val):
    global current_field
    key = field_keys[current_field]
    if val.isdigit(): fields[key] += val
    elif val == 'CLR': fields[key] = ''
    elif val == 'ENT': current_field = (current_field + 1) % len(field_keys)
    elif val == 'ADJ':
        send_phase3_stop()
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
    def __init__(self, rect_ratio, label, action, color=(70,130,180)):
        self.rect = pygame.Rect(
            rect_ratio[0] * SCREEN_W, rect_ratio[1] * SCREEN_H,
            rect_ratio[2] * SCREEN_W, rect_ratio[3] * SCREEN_H
        )
        self.label = label; self.action = action; self.pressed = False
        self.base_color = color
    def draw(self, surf):
        color = (30,90,140) if self.pressed else self.base_color
        pygame.draw.rect(surf, color, self.rect, 0, 5)
        txt = font.render(self.label, True, (255,255,255))
        surf.blit(txt, (self.rect.centerx - txt.get_width()//2, self.rect.centery - txt.get_height()//2))
    def handle(self, pos, is_down):
        if is_down and self.rect.collidepoint(pos): self.pressed = True
        elif (not is_down) and self.pressed:
            self.pressed = False
            if self.rect.collidepoint(pos): self.action()

class Dropdown:
    # (既存のDropdownクラスの内容。変更なしのため省略して実装してください)
    def __init__(self, rect_ratio, font):
        self.rect = pygame.Rect(rect_ratio[0] * SCREEN_W, rect_ratio[1] * SCREEN_H, rect_ratio[2] * SCREEN_W, rect_ratio[3] * SCREEN_H)
        self.font = font; self.options = ["/dev/serial0"]; self.active_option = 0; self.expanded = False; self.refresh_ports()
    def refresh_ports(self):
        ports = serial.tools.list_ports.comports(); new_options = [p.device for p in ports]
        if "/dev/serial0" not in new_options: new_options.insert(0, "/dev/serial0")
        self.options = new_options
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
# 4. 配置
# ==========================================
buttons = []
buttons.append(Button((0.05, 0.05, 0.15, 0.08), 'SAFE ON',  lambda: send_phase3_safe(True)))
buttons.append(Button((0.22, 0.05, 0.15, 0.08), 'SAFE OFF', lambda: send_phase3_safe(False)))

# --- [追加] 終了ボタン (右上に赤色で配置) ---
buttons.append(Button((0.85, 0.02, 0.12, 0.06), 'EXIT', lambda: pygame.event.post(pygame.event.Event(pygame.QUIT)), color=(180, 50, 50)))

pad_layout = [['7','8','9','CLR'],['4','5','6','ENT'],['1','2','3','SEND'],['0','ADJ','RESET','']]
for r, row in enumerate(pad_layout):
    for c, lbl in enumerate(row):
        if not lbl: continue
        buttons.append(Button((0.62 + c*0.09, 0.52 + r*0.11, 0.08, 0.10), lbl, lambda l=lbl: on_pad(l)))

port_dropdown = Dropdown((0.40, 0.05, 0.25, 0.08), font)

# ==========================================
# 5. メインループ
# ==========================================
try:
    while True:
        for ev in pygame.event.get():
            # 終了イベントのハンドリング
            if ev.type == pygame.QUIT: raise SystemExit
            
            # --- [追加] キーボード操作 ---
            if ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE: raise SystemExit  # Escキーで終了
                # Alt+TabやCmd+TabはOS標準機能として動作しますが、
                # ウィンドウを最小化したい場合は pygame.display.iconify() を呼ぶキーを作ることも可能です。

            click_pos = None
            is_down = False

            if ev.type in (pygame.MOUSEBUTTONDOWN, pygame.MOUSEBUTTONUP):
                click_pos = ev.pos
                is_down = (ev.type == pygame.MOUSEBUTTONDOWN)
            elif ev.type in (pygame.FINGERDOWN, pygame.FINGERUP):
                click_pos = (int(ev.x * SCREEN_W), int(ev.y * SCREEN_H))
                is_down = (ev.type == pygame.FINGERDOWN)

            if click_pos:
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
        pygame.display.flip()
        clock.tick(30)

except SystemExit: pass
finally:
    # ここでシリアルポートを確実に閉じる処理が行われます 
    stop_serial()
    pygame.quit()
    sys.exit(0)