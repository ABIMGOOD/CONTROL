import pygame
import math
import csv
import os
from datetime import datetime

pygame.init()

WIDTH = 1200
HEIGHT = 900  # extra height for telemetry panel at bottom
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Robot Simulation — Telemetry View")
clock = pygame.time.Clock()

# ── Fonts ──────────────────────────────────────────────────────────────────
FONT_MONO  = pygame.font.SysFont("Courier New", 15)
FONT_TITLE = pygame.font.SysFont("Courier New", 17, bold=True)

# ── Colours ────────────────────────────────────────────────────────────────
WHITE      = (255, 255, 255)
BLACK      = (10,  10,  10)
DARK_PANEL = (18,  22,  34)
MID_PANEL  = (30,  36,  54)
GRID_COL   = (45,  52,  75)
GREEN      = (0,   210, 120)
YELLOW     = (255, 210, 60)
CYAN       = (60,  200, 255)
ORANGE     = (255, 140, 50)
RED        = (255, 80,  80)
HEADER_COL = (140, 160, 220)
VALUE_COL  = (220, 235, 255)
ROBOT_COL  = (0,   200, 80)
TRAIL_COL  = (0,   120, 50)

# ── Simulation area & panel ────────────────────────────────────────────────
SIM_HEIGHT  = 300          # top portion = simulation canvas
PANEL_Y     = SIM_HEIGHT   # telemetry panel starts here
PANEL_H     = HEIGHT - SIM_HEIGHT

# ── Robot State ────────────────────────────────────────────────────────────
x     = 200.0
y     = 240.0
theta = 0.0

wheel_base   = 60
left_speed   = 0.0
right_speed  = 0.0

# ── Motor Parameters ───────────────────────────────────────────────────────
tau              = 0.5
dt               = 0.016
left_motor_gain  = 1.3
right_motor_gain = 1.0   # 10% stronger → robot curves left

# ── Controller ─────────────────────────────────────────────────────────────
Kc           = 30.0
target_speed = 110.0

# ── Trail ──────────────────────────────────────────────────────────────────
trail = []
MAX_TRAIL = 400

# ── CSV logging ────────────────────────────────────────────────────────────
timestamp   = datetime.now().strftime("%Y%m%d_%H%M%S")
csv_path    = os.path.join(os.path.dirname(__file__), f"telemetry_{timestamp}.csv")
CSV_HEADERS = ["frame", "time_s",
               "Kc", "target_speed",

               "error_L", "error_R",
               "command_L", "command_R",
               "left_speed", "right_speed",
               "v", "w", "theta_deg",
               "x", "y"]

csv_file   = open(csv_path, "w", newline="")
csv_writer = csv.DictWriter(csv_file, fieldnames=CSV_HEADERS)
csv_writer.writeheader()

frame_count  = 0
LOG_INTERVAL = 3   # write a CSV row every N frames (≈20 rows/sec)

# ── Helpers ────────────────────────────────────────────────────────────────
def draw_panel_bg():
    """Draw the dark telemetry panel background with a subtle grid."""
    pygame.draw.rect(screen, DARK_PANEL, (0, PANEL_Y, WIDTH, PANEL_H))
    # top border line
    pygame.draw.line(screen, CYAN, (0, PANEL_Y), (WIDTH, PANEL_Y), 2)
    # vertical grid dividers
    for gx in range(0, WIDTH, 225):
        pygame.draw.line(screen, GRID_COL, (gx, PANEL_Y+4), (gx, HEIGHT-4), 1)

def draw_bar(label, value, max_val, color, bx, by, bw=160, bh=10):
    """Horizontal mini progress bar with label."""
    pct  = max(0.0, min(1.0, abs(value) / max_val))
    surf = FONT_MONO.render(f"{label}", True, HEADER_COL)
    screen.blit(surf, (bx, by))
    bar_y = by + 16
    pygame.draw.rect(screen, MID_PANEL,  (bx, bar_y, bw, bh))
    pygame.draw.rect(screen, color,      (bx, bar_y, int(bw * pct), bh))
    pygame.draw.rect(screen, GRID_COL,   (bx, bar_y, bw, bh), 1)
    val_s = FONT_MONO.render(f"{value:7.2f}", True, VALUE_COL)
    screen.blit(val_s, (bx + bw + 6, bar_y - 2))

def labeled(label, value, color, lx, ly, fmt=".2f"):
    s1 = FONT_TITLE.render(f"{label}", True, HEADER_COL)
    s2 = FONT_MONO.render(f"{value:{fmt}}", True, color)
    screen.blit(s1, (lx, ly))
    screen.blit(s2, (lx, ly + 18))

def draw_telemetry(error_L, error_R, command_L, command_R, v, w):
    draw_panel_bg()

    # ── Section titles ───────────────────────────────────────────
    titles = [
        (12,   "CONTROLLER"),
        (237,  "MOTOR CMD"),
        (462,  "WHEEL SPEEDS"),
        (687,  "KINEMATICS"),
    ]
    for tx, label in titles:
        t = FONT_TITLE.render(label, True, CYAN)
        screen.blit(t, (tx, PANEL_Y + 8))

    base_y = PANEL_Y + 30

    # ── Column 1 : Controller ────────────────────────────────────
    labeled("Kc",           Kc,           YELLOW, 12, base_y)
    labeled("Target spd",   target_speed, YELLOW, 12, base_y + 40)
    draw_bar("Error  L", error_L, target_speed, RED,    12, base_y + 80)
    draw_bar("Error  R", error_R, target_speed, ORANGE, 12, base_y + 110)

    # ── Column 2 : Commands ──────────────────────────────────────
    draw_bar("Cmd  L", command_L, Kc * target_speed, CYAN,  237, base_y + 10)
    draw_bar("Cmd  R", command_R, Kc * target_speed, GREEN, 237, base_y + 40)

    # ── Column 3 : Wheel speeds ──────────────────────────────────
    draw_bar("Left  spd", left_speed,  target_speed * 1.2, GREEN,  462, base_y + 10)
    draw_bar("Right spd", right_speed, target_speed * 1.2, YELLOW, 462, base_y + 40)
    labeled("Δ speed", right_speed - left_speed, ORANGE, 462, base_y + 75)

    # ── Column 4 : Kinematics ────────────────────────────────────
    labeled("v  (lin)", v,                  CYAN,   687, base_y)
    labeled("ω  (ang)", w,                  ORANGE, 687, base_y + 40)
    labeled("θ  (deg)", math.degrees(theta),YELLOW, 687, base_y + 80, fmt=".1f")

    # ── Bottom status bar ────────────────────────────────────────
    status = (f"  frame {frame_count:05d}   "
              f"x={x:6.1f}  y={y:6.1f}   "
              f"CSV → {os.path.basename(csv_path)}")
    st = FONT_MONO.render(status, True, GRID_COL)
    screen.blit(st, (6, HEIGHT - 18))


# ── Main loop ──────────────────────────────────────────────────────────────
running = True

while running:
    clock.tick(60)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # ── Controller ────────────────────────────────────────────────
    error_L   = target_speed - left_speed
    error_R   = target_speed - right_speed
    command_L = Kc * error_L
    command_R = Kc * error_R

    # ── Motor dynamics ────────────────────────────────────────────
    left_speed  += dt * ((left_motor_gain  * command_L - left_speed)  / tau)
    right_speed += dt * ((right_motor_gain * command_R - right_speed) / tau)

    # ── Robot kinematics ──────────────────────────────────────────
    v = (left_speed + right_speed) / 2
    w = (right_speed - left_speed) / wheel_base

    theta += w * dt
    x     += v * math.cos(theta) * dt
    y     += v * math.sin(theta) * dt

    # Wrap within sim area
    x = x % WIDTH
    y = y % SIM_HEIGHT

    # ── Trail ─────────────────────────────────────────────────────
    trail.append((int(x), int(y)))
    if len(trail) > MAX_TRAIL:
        trail.pop(0)

    # ── CSV logging ───────────────────────────────────────────────
    frame_count += 1
    if frame_count % LOG_INTERVAL == 0:
        csv_writer.writerow({
            "frame":        frame_count,
            "time_s":       round(frame_count * dt, 4),
            "Kc":           Kc,
            "target_speed": target_speed,
            "error_L":      round(error_L,   4),
            "error_R":      round(error_R,   4),
            "command_L":    round(command_L, 4),
            "command_R":    round(command_R, 4),
            "left_speed":   round(left_speed,  4),
            "right_speed":  round(right_speed, 4),
            "v":            round(v,     4),
            "w":            round(w,     4),
            "theta_deg":    round(math.degrees(theta), 4),
            "x":            round(x, 2),
            "y":            round(y, 2),
        })
        csv_file.flush()

    # ── Draw simulation area ──────────────────────────────────────
    screen.fill(BLACK)

    # Draw trail
    if len(trail) > 1:
        for i in range(1, len(trail)):
            alpha = int(80 * i / len(trail))
            c = (0, max(40, alpha * 2), max(20, alpha))
            pygame.draw.line(screen, c, trail[i-1], trail[i], 2)

    # Robot body
    pygame.draw.circle(screen, ROBOT_COL, (int(x), int(y)), 20)
    pygame.draw.circle(screen, WHITE,     (int(x), int(y)), 20, 2)

    # Heading arrow
    hx = x + 30 * math.cos(theta)
    hy = y + 30 * math.sin(theta)
    pygame.draw.line(screen, YELLOW, (int(x), int(y)), (int(hx), int(hy)), 3)

    # ── Draw telemetry panel ──────────────────────────────────────
    draw_telemetry(error_L, error_R, command_L, command_R, v, w)

    pygame.display.flip()

# ── Cleanup ────────────────────────────────────────────────────────────────
csv_file.close()
print(f"\n✅ Telemetry saved → {csv_path}")
pygame.quit()