#!/usr/bin/env python3
import pygame, sys
import serial.tools.list_ports  # ポート一覧取得用
from collections import deque
import NTC_1A_utils
from NTC_1A_serial_comm import start_serial_thread, send_packet, stop_serial

# ==========================================
# 1. ログ・通信の初期設定
# ==========================================
log_lines = deque(maxlen=8)
def log_callback(msg):
    log_lines.append(msg)
NTC_1A_utils.out = log_callback

# 初期ポートで開始（Dropdownのデフォルトと合わせる）
start_serial_thread('/dev/serial0')

# ==========================================
# 2. Pygame 基本設定
# ==========================================
pygame.init()
SCREEN_W, SCREEN_H = 1024, 600
screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
font = pygame.font.Font(None, 32)
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
    elif val == 'STOP':  send_packet([0x05, 0, 0, 0, 0])
    elif val == 'RESET': send_packet([0x04, 0, 0, 0, 0])
    elif val == 'SEND':
        t = int(fields['CH1_TENSION'] or 0)
        send_packet([0x01, t & 0xFF, 0, 0, 0])

# ==========================================
# 3. UIクラス定義 (使用する前に定義する)
# ==========================================
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
        elif (not is_down) and self.pressed:
            self.pressed = False
            if self.rect.collidepoint(pos): self.action()

class Dropdown:
    def __init__(self, rect, font):
        self.rect = pygame.Rect(rect)
        self.font = font
        self.options = ["/dev/serial0"] # デフォルト
        self.active_option = 0
        self.expanded = False
        self.refresh_ports()

    def refresh_ports(self):
        """システムに接続されているシリアルポートを再スキャン"""
        ports = serial.tools.list_ports.comports()
        new_options = [p.device for p in ports]
        if "/dev/serial0" not in new_options:
            new_options.insert(0, "/dev/serial0")
        self.options = new_options
        if self.active_option >= len(self.options):
            self.active_option = 0

    def draw(self, surf):
        color = (50, 50, 60) if not self.expanded else (80, 80, 90)
        pygame.draw.rect(surf, color, self.rect, 0, 3)
        pygame.draw.rect(surf, (200, 200, 200), self.rect, 1, 3)
        txt = self.font.render(self.options[self.active_option], True, (255, 255, 255))
        surf.blit(txt, (self.rect.x + 10, self.rect.centery - txt.get_height()//2))

        if self.expanded:
            for i, opt in enumerate(self.options):
                opt_rect = self.rect.copy()
                opt_rect.y += self.rect.height * (i + 1)
                pygame.draw.rect(surf, (40, 40, 45), opt_rect)
                pygame.draw.rect(surf, (150, 150, 150), opt_rect, 1)
                opt_txt = self.font.render(opt, True, (255, 255, 255))
                surf.blit(opt_txt, (opt_rect.x + 10, opt_rect.centery - opt_txt.get_height()//2))

    def handle(self, pos):
        if self.rect.collidepoint(pos):
            self.expanded = not self.expanded
            if self.expanded: self.refresh_ports()
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
# 4. UIインスタンス生成
# ==========================================
# メイン画面のボタンリスト
buttons = []
buttons.append(Button((50, 20, 150, 50), 'SAFE ON',  lambda: send_packet([0xF0, 1, 0, 0, 0])))
buttons.append(Button((220, 20, 150, 50), 'SAFE OFF', lambda: send_packet([0xF0, 0, 0, 0, 0])))

pad_layout = [['7','8','9','CLR'],['4','5','6','ENT'],['1','2','3','SEND'],['0','STOP','RESET','']]
for r, row in enumerate(pad_layout):
    for c, lbl in enumerate(row):
        if not lbl: continue
        buttons.append(Button((500 + c*110, 320 + r*65, 100, 60), lbl, lambda l=lbl: on_pad(l)))

# ポート選択ドロップダウン
port_dropdown = Dropdown((400, 20, 250, 50), font)

# ==========================================
# 5. メインループ
# ==========================================
try:
    while True:
        pos=None; is_down=False
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT: raise SystemExit
            if ev.type == pygame.MOUSEBUTTONDOWN:
                pos=ev.pos; is_down=True
            if ev.type == pygame.MOUSEBUTTONUP:
                pos=ev.pos; is_down=False
        
        if pos:
            # ドロップダウンのクリック判定を優先
            new_port = port_dropdown.handle(pos)
            if new_port:
                NTC_1A_utils.out(f"[UI] Port change: {new_port}")
                stop_serial()
                start_serial_thread(new_port)
            
            # ドロップダウンが閉じていれば他のボタンの判定をする
            if not port_dropdown.expanded and is_down:
                for b in buttons: b.handle(pos, is_down)
                for idx in range(len(field_keys)):
                    fx, fy = 50 + (idx//3) * 320 + 150, 100 + (idx%3) * 60
                    if pygame.Rect(fx, fy, 120, 40).collidepoint(pos): current_field = idx
        
        # 描画
        screen.fill((30,30,35))
        for b in buttons: b.draw(screen)
        
        for idx, key in enumerate(field_keys):
            lx, fy = 50 + (idx//3)*320, 100 + (idx%3)*60
            screen.blit(font.render(label_map[key], True, (200,200,200)), (lx, fy + 10))
            rect = pygame.Rect(lx+150, fy, 120, 40)
            pygame.draw.rect(screen, (255,255,255) if idx==current_field else (100,100,100), rect, 2)
            val_txt = font.render(fields[key], True, (255,255,0) if idx==current_field else (255,255,255))
            screen.blit(val_txt, (rect.right - val_txt.get_width() - 5, rect.y + 10))
            
        for i, line in enumerate(list(log_lines)):
            screen.blit(font.render(line, True, (150,150,150)), (50, 420 + i*20))
            
        # ドロップダウンは最前面に表示するため最後に描画
        port_dropdown.draw(screen)

        pygame.display.flip()
        clock.tick(30)

except SystemExit: pass
finally:
    stop_serial(); pygame.quit(); sys.exit(0)