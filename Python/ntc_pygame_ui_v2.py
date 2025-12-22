#!/usr/bin/env python3
import pygame
import sys
import NTC_1A_utils
from NTC_1A_serial_comm import start_serial_thread, send_packet, send_packet_raw, build_packet

# --- シリアル初期化 ---
SERIAL_PORT = '/dev/serial0'
start_serial_thread(SERIAL_PORT)

# --- Pygame 初期化 ---
pygame.init()
SCREEN_W, SCREEN_H = 1024, 600
screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
pygame.mouse.set_visible(True) # デバッグ用に表示（本番はFalse）
font = pygame.font.Font(None, 32)
clock = pygame.time.Clock()

# --- ログ管理 ---
log_lines = []
def log_callback(msg):
    log_lines.append(msg)
    if len(log_lines) > 8: log_lines.pop(0)
NTC_1A_utils.out = log_callback

# --- データフィールド ---
fields = {
    'CH1_TENSION': '', 'CH1_LENGTH': '', 'CH1_COUNT': '',
    'CH2_TENSION': '', 'CH2_LENGTH': '', 'CH2_COUNT': ''
}
field_keys = list(fields.keys())
current_field = 0

# --- GUI レイアウト設定 ---
LABEL_W, LABEL_H = 150, 40
FIELD_W, FIELD_H = 120, 40
COL_GAP = 120
LABEL_Y_START = 100
LABEL_Y_GAP = 60
label_map = {k: k.replace('_', ' ') for k in field_keys}

class Button:
    def __init__(self, rect, label, action, color=(70,130,180)):
        self.rect = pygame.Rect(rect)
        self.label = label
        self.action = action
        self.color = color
        self.pressed = False

    def draw(self, surf):
        c = [max(0, x-40) for x in self.color] if self.pressed else self.color
        pygame.draw.rect(surf, c, self.rect, 0, 5)
        pygame.draw.rect(surf, (200,200,200), self.rect, 2, 5)
        txt = font.render(self.label, True, (255,255,255))
        surf.blit(txt, (self.rect.centerx - txt.get_width()//2, self.rect.centery - txt.get_height()//2))

    def handle(self, pos, is_down):
        if is_down:
            if self.rect.collidepoint(pos): self.pressed = True
        else:
            if self.pressed and self.rect.collidepoint(pos):
                self.action()
            self.pressed = False

buttons = []
# コマンドボタン配置
def cmd_reset(): send_packet([0x04, 0, 0, 0, 0])
def cmd_stop():  send_packet([0x05, 0, 0, 0, 0])

# テンキー処理
def on_pad(val):
    global current_field
    key = field_keys[current_field]
    if val.isdigit(): fields[key] += val
    elif val == 'CLR': fields[key] = ''
    elif val == 'ENT': current_field = (current_field + 1) % len(field_keys)
    elif val == 'SEND':
        t = int(fields['CH1_TENSION'] or 0)
        send_packet([0x01, t & 0xFF, 0, 0, 0]) # TENSION設定

# ボタン生成
btn_x = 50
buttons.append(Button((btn_x, 20, 150, 50), 'SAFE ON', lambda: send_packet_raw([0xF0, 1])))
buttons.append(Button((btn_x + 170, 20, 150, 50), 'SAFE OFF', lambda: send_packet_raw([0xF0, 0])))

pad_layout = [
    ['7','8','9','CLR'],
    ['4','5','6','ENT'],
    ['1','2','3','SEND'],
    ['0','STOP','RESET','']
]

for r, row in enumerate(pad_layout):
    for c, lbl in enumerate(row):
        if not lbl: continue
        act = cmd_stop if lbl=='STOP' else cmd_reset if lbl=='RESET' else lambda l=lbl: on_pad(l)
        buttons.append(Button((500 + c*110, 320 + r*65, 100, 60), lbl, act))

# --- メインループ ---
running = True
while running:
    pos = None
    is_down = False
    
    for ev in pygame.event.get():
        if ev.type == pygame.QUIT: running = False
        if ev.type == pygame.MOUSEBUTTONDOWN:
            pos = ev.pos; is_down = True
        if ev.type == pygame.MOUSEBUTTONUP:
            pos = ev.pos; is_down = False

    if pos:
        for b in buttons: b.handle(pos, is_down)
        # フィールド選択
        for idx in range(len(field_keys)):
            col, row = idx // 3, idx % 3
            fx = 50 + col * (LABEL_W + FIELD_W + 50) + LABEL_W
            fy = LABEL_Y_START + row * LABEL_Y_GAP
            if pygame.Rect(fx, fy, FIELD_W, FIELD_H).collidepoint(pos):
                current_field = idx

    # 描画
    screen.fill((30, 30, 35))
    for b in buttons: b.draw(screen)

    # フィールド描画
    for idx, key in enumerate(field_keys):
        col, row = idx // 3, idx % 3
        lx = 50 + col * (LABEL_W + FIELD_W + 50)
        fy = LABEL_Y_START + row * LABEL_Y_GAP
        screen.blit(font.render(label_map[key], True, (200,200,200)), (lx, fy + 10))
        
        rect = pygame.Rect(lx + LABEL_W, fy, FIELD_W, FIELD_H)
        color = (255, 255, 255) if idx == current_field else (80, 80, 80)
        pygame.draw.rect(screen, color, rect, 2, 3)
        val_txt = font.render(fields[key], True, (255, 255, 0) if idx == current_field else (255, 255, 255))
        screen.blit(val_txt, (rect.right - val_txt.get_width() - 5, rect.y + 10))

    # ログ表示
    for i, line in enumerate(log_lines):
        screen.blit(font.render(line, True, (150, 150, 150)), (50, 450 + i * 18))

    pygame.display.flip()
    clock.tick(30)

pygame.quit()
