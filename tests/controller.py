import pygame
import sys

# 初始化Pygame
pygame.init()

# 设置窗口参数
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 600
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("Gamepad Input Visualizer")

# 初始化游戏手柄
pygame.joystick.init()
joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
for joystick in joysticks:
    joystick.init()

# 颜色定义
COLORS = {
    "background": (30, 30, 30),
    "button": (100, 100, 100),
    "button_pressed": (0, 255, 0),
    "axis": (200, 0, 0),
    "text": (255, 255, 255)
}

# 主循环
def main():
    clock = pygame.time.Clock()
    
    while True:
        # 处理事件
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        screen.fill(COLORS["background"])
        
        if joysticks:
            joystick = joysticks[0]  # 使用第一个连接的手柄
            num_buttons = joystick.get_numbuttons()
            num_axes = joystick.get_numaxes()
            num_hats = joystick.get_numhats()

            # 绘制按钮
            button_spacing = 40
            for i in range(num_buttons):
                x = 50 + (i % 10) * button_spacing
                y = 50 + (i // 10) * button_spacing
                color = COLORS["button_pressed"] if joystick.get_button(i) else COLORS["button"]
                pygame.draw.circle(screen, color, (x, y), 10)
                # 显示按钮编号
                font = pygame.font.Font(None, 20)
                text = font.render(str(i), True, COLORS["text"])
                screen.blit(text, (x-5, y+15))

            # 绘制摇杆和扳机
            axis_positions = [
                (300, 200),  # 左摇杆
                (500, 200),  # 右摇杆
                (200, 400),  # 左扳机
                (600, 400)   # 右扳机
            ]
            
            # 左摇杆（轴0和1）
            lx = joystick.get_axis(0)
            ly = joystick.get_axis(1)
            # 右摇杆（轴2和3）
            rx = joystick.get_axis(2)
            ry = joystick.get_axis(3)
            # 扳机（轴4和5）
            lt = (joystick.get_axis(4) + 1) / 2  # 标准化到0-1
            rt = (joystick.get_axis(5) + 1) / 2  # 标准化到0-1

            # 绘制摇杆
            for i, (x, y) in enumerate(axis_positions[:2]):
                pygame.draw.rect(screen, COLORS["axis"], (x-50, y-50, 100, 100), 2)
                pos = (lx, ly) if i == 0 else (rx, ry)
                dot_x = x + int(pos[0] * 40)
                dot_y = y + int(pos[1] * 40)
                pygame.draw.circle(screen, COLORS["button_pressed"], (dot_x, dot_y), 8)

            # 绘制扳机进度条
            for i, (x, y) in enumerate(axis_positions[2:]):
                value = lt if i == 0 else rt
                pygame.draw.rect(screen, COLORS["axis"], (x-20, y-100, 40, 100), 2)
                fill_height = int(value * 100)
                pygame.draw.rect(screen, COLORS["button_pressed"], 
                                 (x-18, y - 100 + (100 - fill_height), 36, fill_height))

        # 显示提示文字
        font = pygame.font.Font(None, 24)
        if not joysticks:
            text = font.render("No controller connected!", True, COLORS["text"])
            screen.blit(text, (WINDOW_WIDTH//2-100, WINDOW_HEIGHT//2))
        else:
            text = font.render("Buttons", True, COLORS["text"])
            screen.blit(text, (50, 20))
            text = font.render("Left Stick", True, COLORS["text"])
            screen.blit(text, (300-50, 150))
            text = font.render("Right Stick", True, COLORS["text"])
            screen.blit(text, (500-50, 150))
            text = font.render("Triggers", True, COLORS["text"])
            screen.blit(text, (WINDOW_WIDTH//2-30, 350))

        pygame.display.flip()
        clock.tick(60)

if __name__ == "__main__":
    main()