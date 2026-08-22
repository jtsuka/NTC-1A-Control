#!/usr/bin/env python3
import pygame
import sys
from collections import deque
import NTC_1A_utils
from NTC_1A_serial_comm import start_serial_thread, send_packet

# --- ログ管理 (スレッド安全) ---
log_lines = deque(maxlen=8)
def log_callback(msg):
    log_lines.append(msg)
NTC_1A_utils.out = log_callback

# --- シリアル開始 ---
SERIAL_PORT = '/dev/serial0'
start_serial_thread(SERIAL_PORT)

# --- Pygame 初期化 ---
pygame.init()
SCREEN_W, SCREEN_H = 1024, 600
screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
pygame.mouse.set_visible(True)
font = pygame.font.Font(None, 32)
clock = pygame.time.Clock()

fields = {'CH1_TENSION':'','CH1_LENGTH':'','CH1_COUNT':'','CH2_TENSION':'','CH2_LENGTH':'','CH2_COUNT':''}
field_keys = list(fields.keys())
current_field = 0
label_map = {
    'CH1_TENSION': 'CH1 TENSION', 'CH1_LENGTH': 'CH1 LENGTH', 'CH1_COUNT': 'CH1 COUNT',
    'CH2_TENSION': 'CH2 TENSION', 'CH2_LENGTH': 'CH2 LENGTH', 'CH2_COUNT': 'CH2 COUNT'
}

class Button:
    def __init__(self, rect, label, action):
        self.rect = pygame.Rect(rect); self.label = label; self.action = action; self.pressed = False
    def draw(self, surf):
        color = (30,90,140) if self.pressed else (70,130,180)
        pygame.draw.rect(surf, color, self.rect, 0, 5)
        txt = font.render(self.label, True, (255,255,255))
        surf.blit(txt, (self.rect.centerx - txt.get_width()//2, self.rect.centery - txt.get_height()//2))
    def handle(self, pos, is_down):
        if is_down and self.rect.collidepoint(pos): self.pressed = True
        elif not is_down and self.pressed:
            self.pressed = False
            if self.rect.collidepoint(pos): self.action()

def on_pad(val):
    global current_field
    key = field_keys[current_field]
    if val.isdigit(): fields[key] += val
    elif val == 'CLR': fields[key] = ''
    elif val == 'ENT': current_field = (current_field + 1) % len(field_keys)
    elif val == 'STOP': send_packet([0x05, 0, 0, 0, 0])  # STOP CMD
    elif val == 'RESET': send_packet([0x04, 0, 0, 0, 0]) # RESET CMD
    elif val == 'SEND':
        t = int(fields['CH1_TENSION'] or 0)
        send_packet([0x01, t & 0xFF, 0, 0, 0]) # TENSION SEND

buttons = []
buttons.append(Button((50, 20, 150, 50), 'SAFE ON', lambda: send_packet([0xF0, 1, 0, 0, 0])))
buttons.append(Button((220, 20, 150, 50), 'SAFE OFF', lambda: send_packet([0xF0, 0, 0, 0, 0])))

pad_layout = [['7','8','9','CLR'],['4','5','6','ENT'],['1','2','3','SEND'],['0','STOP','RESET','']]
for r, row in enumerate(pad_layout):
    for c, lbl in enumerate(row):
        if not lbl: continue
        buttons.append(Button((500 + c*110, 320 + r*65, 100, 60), lbl, lambda l=lbl: on_pad(l)))

# --- メインループ ---
while True:
    pos = None; is_down = False
    for ev in pygame.event.get():
        if ev.type == pygame.QUIT: pygame.quit(); sys.exit()
        if ev.type == pygame.MOUSEBUTTONDOWN: pos = ev.pos; is_down = True
        if ev.type == pygame.MOUSEBUTTONUP: pos = ev.pos; is_down = False

    if pos:
        for b in buttons: b.handle(pos, is_down)
        for idx in range(len(field_keys)):
            fx, fy = 50 + (idx//3) * 320 + 150, 120 + (idx%3) * 70
            if pygame.Rect(fx, fy, 100, 30).collidepoint(pos): current_field = idx

    screen.fill((30, 30, 35))
    for b in buttons: b.draw(screen)
    for idx, key in enumerate(field_keys):
        lx, fy = 50 + (idx//3) * 320, 120 + (idx%3) * 70
        screen.blit(font.render(label_map[key], True, (200,200,200)), (lx, fy))
        rect = pygame.Rect(lx + 150, fy, 100, 30)
        pygame.draw.rect(screen, (255,255,255) if idx==current_field else (100,100,100), rect, 2)
        val_txt = font.render(fields[key], True, (255,255,0) if idx==current_field else (255,255,255))
        screen.blit(val_txt, (rect.right - val_txt.get_width() - 5, rect.y))

    for i, line in enumerate(list(log_lines)):
        screen.blit(font.render(line, True, (150,150,150)), (50, 420 + i * 20))

    pygame.display.flip()
    clock.tick(30)