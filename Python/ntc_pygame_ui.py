#!/usr/bin/env python3
import pygame
import sys
import NTC_1A_utils
from NTC_1A_serial_comm import start_serial_thread, send_packet, send_packet_raw, build_packet

# --- シリアル初期化 ---
SERIAL_PORT = '/dev//dev/serial0'  # 必宜変更
#SERIAL_PORT = 'COM1'  # 必宜変更
start_serial_thread(SERIAL_PORT)

# --- Pygame 初期化 ---
pygame.init()
SCREEN_W, SCREEN_H = 1024, 600
screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
pygame.mouse.set_visible(False)
font = pygame.font.Font(None, 32)
clock = pygame.time.Clock()

# --- ログコールバック ---
log_lines = []
def log_callback(msg):
    log_lines.append(msg)
    if len(log_lines) > 5:
        log_lines.pop(0)
NTC_1A_utils.out = log_callback

# --- フィールド設定 ---
fields = {
    'CH1_TENSION': '', 'CH1_LENGTH': '', 'CH1_COUNT': '',
    'CH2_TENSION': '', 'CH2_LENGTH': '', 'CH2_COUNT': ''
}
field_keys = list(fields.keys())
current_field = 0
LABEL_W, LABEL_H = 150, 30
FIELD_W, FIELD_H = 100, 30
LABEL_X = 30
FIELD_X = LABEL_X + LABEL_W + 20
LABEL_Y_START = 120
LABEL_Y_GAP = 70

# ラベル表示用（日本語表記）
label_map = {
    'CH1_TENSION': 'CH1 テンション',
    'CH1_LENGTH': 'CH1 長さ',
    'CH1_COUNT': 'CH1 カウント',
    'CH2_TENSION': 'CH2 テンション',
    'CH2_LENGTH': 'CH2 長さ',
    'CH2_COUNT': 'CH2 カウント'
}

# --- ボタン定義 ---
class Button:
    def __init__(self, rect, label, action):
        self.rect = pygame.Rect(rect)
        self.label = label
        self.action = action
        self.pressed = False
    def draw(self, surf):
        color = (70,130,180) if not self.pressed else (30,90,140)
        pygame.draw.rect(surf, color, self.rect)
        text = font.render(self.label, True, (255,255,255))
        tx = self.rect.x + (self.rect.w - text.get_width())//2
        ty = self.rect.y + (self.rect.h - text.get_height())//2
        surf.blit(text, (tx, ty))
    def handle(self, pos, is_down):
        if is_down and self.rect.collidepoint(pos):
            self.pressed = True
        elif not is_down and self.pressed:
            self.pressed = False
            if self.rect.collidepoint(pos):
                self.action()

buttons = []
# SAFE ON/OFF
buttons.append(Button((50,20,200,60), 'SAFE ON', lambda: send_packet_raw([0xF0,1])))
buttons.append(Button((280,20,200,60), 'SAFE OFF', lambda: send_packet_raw([0xF0,0])))
# TEST1-5
for i in range(5):
    buttons.append(Button((520 + i*60,20,50,60), f'T{i+1}', lambda idx=i: send_packet_raw([0x01,0x06,0x05,0,0,0] if idx==0 else build_packet([0x01,100,0,0,0]))))
# テンキー定義
pad = [['7','8','9','CLR'], ['4','5','6','ENTER'], ['1','2','3','SEND'], ['0','STOP','RESET','NONE']]
def on_pad(label):
    global current_field
    key = field_keys[current_field]
    if label.isdigit():
        fields[key] += label
    elif label == 'CLR':
        fields[key] = ''
    elif label == 'ENTER':
        current_field = (current_field + 1) % len(field_keys)
    elif label == 'SEND':
        for ch in (1,2):
            t = int(fields[f'CH{ch}_TENSION'] or 0)
            l = int(float(fields[f'CH{ch}_LENGTH'] or 0) * 10)
            c = int(fields[f'CH{ch}_COUNT'] or 0)
            send_packet([0x01, t&0xFF,0,0,0])
            send_packet([0x02, l&0xFF,(l>>8)&0xFF,(l>>16)&0xFF,0])
            send_packet([0x03, c&0xFF,(c>>8)&0xFF,(c>>16)&0xFF,0])
    elif label == 'RESET':
        send_packet([0x04,0,0,0,0])
    elif label == 'STOP':
        send_packet([0x05,0,0,0,0])

for r, row in enumerate(pad):
    for c, lbl in enumerate(row):
        if lbl == 'NONE': continue
        x = 50 + c*200
        y = 300 + r*70
        buttons.append(Button((x,y,180,60), lbl, lambda l=lbl: on_pad(l)))

# --- メインループ ---
while True:
    for ev in pygame.event.get():
        if ev.type == pygame.FINGERDOWN or ev.type == pygame.MOUSEBUTTONDOWN:
            pos = (int(ev.x * SCREEN_W), int(ev.y * SCREEN_H)) if hasattr(ev, 'x') else ev.pos
            is_down = True
        elif ev.type == pygame.FINGERUP or ev.type == pygame.MOUSEBUTTONUP:
            pos = (int(ev.x * SCREEN_W), int(ev.y * SCREEN_H)) if hasattr(ev, 'x') else ev.pos
            is_down = False
        else:
            pos = None; is_down = False
        if pos and isinstance(pos, tuple) and len(pos) == 2:
            for b in buttons:
                b.handle(pos, is_down)

            # 入力フィールドがタップされたら current_field を更新
            for idx, key in enumerate(field_keys):
                col, row = idx // 3, idx % 3
                fx = FIELD_X + col * 350
                fy = LABEL_Y_START + row * LABEL_Y_GAP
                rect = pygame.Rect(fx, fy, FIELD_W, FIELD_H)
                if rect.collidepoint(pos):
                    current_field = idx

    screen.fill((20,20,20))
    # ボタン描画
    for b in buttons:
        b.draw(screen)
    # ラベルと入力フィールド描画
    for idx, key in enumerate(field_keys):
        col, row = idx//3, idx%3
        lx = LABEL_X + col*350; ly = LABEL_Y_START + row*LABEL_Y_GAP
        # ラベル
        #lbl_surf = font.render(key.replace('_',' '), True, (200,200,200))
        lbl_surf = font.render(label_map.get(key, key), True, (200,200,200))
        screen.blit(lbl_surf, (lx, ly))
        # フィールド枠
        fx = FIELD_X + col*350; fy = LABEL_Y_START + row*LABEL_Y_GAP
        rect = pygame.Rect(fx, fy, FIELD_W, FIELD_H)
        pygame.draw.rect(screen, (255,255,255) if idx==current_field else (100,100,100), rect, 2)
        # 値右寄せ
        val_surf = font.render(fields[key], True, (255,255,0) if idx==current_field else (200,200,200))
        vx = fx + FIELD_W - val_surf.get_width() - 10
        vy = fy + (FIELD_H - val_surf.get_height())//2
        screen.blit(val_surf, (vx, vy))
    # ログ描画
    for i, line in enumerate(log_lines):
        screen.blit(font.render(line, True, (180,180,180)), (50, 500 + i*20))

    pygame.display.flip()
    clock.tick(30)

