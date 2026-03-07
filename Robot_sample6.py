import pygame
import math
import csv
import os
from datetime import datetime

pygame.init()

WIDTH = 900
HEIGHT = 740
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Robot Simulation — Heading Feedback Control")
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
MAGENTA    = (220, 80,  220)
HEADER_COL = (140, 160, 220)
VALUE_COL  = (220, 235, 255)
ROBOT_COL  = (0,   200, 80)

# ── Simulation area & panel ────────────────────────────────────────────────
SIM_HEIGHT = 480
PANEL_Y    = SIM_HEIGHT
PANEL_H    = HEIGHT - SIM_HEIGHT

# ── Robot State ────────────────────────────────────────────────────────────
x     = 200.0
y     = 240.0
theta = 0.0

wheel_base  = 60
left_speed  = 0.0
right_speed = 0.0

# ── Motor Parameters ───────────────────────────────────────────────────────
tau              = 0.5
dt               = 0.016
left_motor_gain  = 1.0
right_motor_gain = 1.1   # 10% stronger — heading controller must fight this

# ── Speed Controller ───────────────────────────────────────────────────────
Kc           = 2.0
target_speed = 100.0

# ── Heading Controller ─────────────────────────────────────────────────────
# The desired heading the robot should maintain (radians).
# 0 = pointing right.  Change this to send the robot in any direction.
target_heading = 0.0

# Kh — heading gain.
# This scales how hard the controller corrects for heading error.
# Too small → drifts. Too large → oscillates / wiggles.
Kh = 40.0

# ── Trail ──────────────────────────────────────────────────────────────────
trail = []
MAX_TRAIL = 500

# ── CSV logging ────────────────────────────────────────────────────────────
timestamp   = datetime.now().strftime("%Y%m%d_%H%M%S")
csv_path    = os.path.join(os.path.dirname(__file__), f"telemetry_{timestamp}.csv")
CSV_HEADERS = ["frame", "time_s",
               "Kc", "Kh", "target_speed", "target_heading_deg",
               "heading_error_deg", "heading_correction",
               "error_L", "error_R",
               "command_L", "command_R",
               "left_speed", "right_speed",
               "v", "w", "theta_deg",
               "x", "y"]

csv_file   = open(csv_path, "w", newline="")
csv_writer = csv.DictWriter(csv_file, fieldnames=CSV_HEADERS)
csv_writer.writeheader()

frame_count  = 0
LOG_INTERVAL = 3

# ── Helpers ────────────────────────────────────────────────────────────────
def angle_wrap(a):
    """Wrap an angle into [-π, π] so the controller always takes the short way round."""
    return (a + math.pi) % (2 * math.pi) - math.pi

def draw_panel_bg():
    pygame.draw.rect(screen, DARK_PANEL, (0, PANEL_Y, WIDTH, PANEL_H))
    pygame.draw.line(screen, CYAN, (0, PANEL_Y), (WIDTH, PANEL_Y), 2)
    for gx in range(0, WIDTH, 180):
        pygame.draw.line(screen, GRID_COL, (gx, PANEL_Y+4), (gx, HEIGHT-4), 1)

def draw_bar(label, value, max_val, color, bx, by, bw=150, bh=10):
    pct  = max(0.0, min(1.0, abs(value) / max(max_val, 0.001)))
    surf = FONT_MONO.render(label, True, HEADER_COL)
    screen.blit(surf, (bx, by))
    bar_y = by + 16
    pygame.draw.rect(screen, MID_PANEL, (bx, bar_y, bw, bh))
    pygame.draw.rect(screen, color,     (bx, bar_y, int(bw * pct), bh))
    pygame.draw.rect(screen, GRID_COL,  (bx, bar_y, bw, bh), 1)
    val_s = FONT_MONO.render(f"{value:7.2f}", True, VALUE_COL)
    screen.blit(val_s, (bx + bw + 6, bar_y - 2))

def labeled(label, value, color, lx, ly, fmt=".2f"):
    s1 = FONT_TITLE.render(label,          True, HEADER_COL)
    s2 = FONT_MONO.render(f"{value:{fmt}}", True, color)
    screen.blit(s1, (lx, ly))
    screen.blit(s2, (lx, ly + 18))

def draw_target_heading_arrow():
    """Draw a dashed arc / arrow showing the desired heading direction."""
    cx, cy = WIDTH - 60, 60
    r = 40
    # Desired direction arrow (cyan)
    dx = cx + r * math.cos(target_heading)
    dy = cy + r * math.sin(target_heading)
    pygame.draw.circle(screen, GRID_COL, (cx, cy), r, 1)
    pygame.draw.line(screen, CYAN, (cx, cy), (int(dx), int(dy)), 2)
    # Current heading arrow (yellow)
    hx = cx + r * math.cos(theta)
    hy = cy + r * math.sin(theta)
    pygame.draw.line(screen, YELLOW, (cx, cy), (int(hx), int(hy)), 2)
    label = FONT_MONO.render("tgt θ", True, CYAN)
    screen.blit(label, (cx - 18, cy + r + 4))

def draw_telemetry(heading_error, heading_correction,
                   error_L, error_R, command_L, command_R, v, w):
    draw_panel_bg()

    titles = [
        (10,  "HEADING CTRL"),
        (190, "SPEED CTRL"),
        (370, "MOTOR CMD"),
        (550, "WHEEL SPEEDS"),
        (730, "KINEMATICS"),
    ]
    for tx, label in titles:
        t = FONT_TITLE.render(label, True, CYAN)
        screen.blit(t, (tx, PANEL_Y + 8))

    base_y = PANEL_Y + 30

    # ── Col 1 : Heading controller ───────────────────────────────
    labeled("Kh",          Kh,                              MAGENTA, 10, base_y)
    labeled("Tgt θ (°)",   math.degrees(target_heading),    CYAN,    10, base_y + 40, fmt=".1f")
    draw_bar("θ error °",  math.degrees(heading_error), 180, MAGENTA, 10, base_y + 85)
    draw_bar("correction", heading_correction, Kh * math.pi, RED,    10, base_y + 115)

    # ── Col 2 : Speed controller ─────────────────────────────────
    labeled("Kc",          Kc,           YELLOW, 190, base_y)
    labeled("Tgt spd",     target_speed, YELLOW, 190, base_y + 40)
    draw_bar("Error  L", error_L, target_speed, RED,    190, base_y + 85)
    draw_bar("Error  R", error_R, target_speed, ORANGE, 190, base_y + 115)

    # ── Col 3 : Commands ─────────────────────────────────────────
    draw_bar("Cmd  L", command_L, Kc * target_speed, CYAN,  370, base_y + 10)
    draw_bar("Cmd  R", command_R, Kc * target_speed, GREEN, 370, base_y + 40)

    # ── Col 4 : Wheel speeds ─────────────────────────────────────
    draw_bar("Left  spd", left_speed,  target_speed * 1.2, GREEN,  550, base_y + 10)
    draw_bar("Right spd", right_speed, target_speed * 1.2, YELLOW, 550, base_y + 40)
    labeled("Δ speed", right_speed - left_speed, ORANGE, 550, base_y + 75)

    # ── Col 5 : Kinematics ───────────────────────────────────────
    labeled("v  (lin)", v,                   CYAN,   730, base_y)
    labeled("ω  (ang)", w,                   ORANGE, 730, base_y + 40)
    labeled("θ  (deg)", math.degrees(theta), YELLOW, 730, base_y + 80, fmt=".1f")

    # ── Status bar ───────────────────────────────────────────────
    status = (f"  frame {frame_count:05d}   x={x:6.1f}  y={y:6.1f}   "
              f"CSV → {os.path.basename(csv_path)}")
    screen.blit(FONT_MONO.render(status, True, GRID_COL), (6, HEIGHT - 18))


# ── Main loop ──────────────────────────────────────────────────────────────
running = True

while running:
    clock.tick(60)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        # ── Live target-heading adjustment ────────────────────────
        # Press LEFT / RIGHT arrows to rotate the desired heading by 5°
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                target_heading = angle_wrap(target_heading - math.radians(5))
            if event.key == pygame.K_RIGHT:
                target_heading = angle_wrap(target_heading + math.radians(5))

    # ══════════════════════════════════════════════════════════════
    #  HEADING FEEDBACK CONTROLLER
    # ══════════════════════════════════════════════════════════════
    #
    #  1. Compute heading error — always the shortest angular path
    #     so the robot never spins the long way round.
    #
    heading_error = angle_wrap(target_heading - theta)
    #
    #  2. Scale by Kh to produce a correction signal.
    #     Positive error  → robot is pointing LEFT of target
    #                      → speed up RIGHT wheel, slow LEFT wheel
    #     Negative error  → robot is pointing RIGHT of target
    #                      → speed up LEFT wheel, slow RIGHT wheel
    #
    heading_correction = Kh * heading_error
    #
    #  ─────────────────────────────────────────────────────────────
    #  SPEED CONTROLLER  (negative feedback, same as before)
    #  but each wheel's target is now nudged by the heading term:
    #
    #    left  target  =  base speed  −  correction
    #    right target  =  base speed  +  correction
    #
    #  When the robot drifts left  → heading_error > 0
    #    → correction > 0  → right wheel sped up, left slowed → turns right ✓
    #  When the robot drifts right → heading_error < 0
    #    → correction < 0  → left wheel sped up, right slowed → turns left  ✓
    # ══════════════════════════════════════════════════════════════

    left_target  = target_speed - heading_correction
    right_target = target_speed + heading_correction

    error_L   = left_target  - left_speed
    error_R   = right_target - right_speed
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
            "frame":               frame_count,
            "time_s":              round(frame_count * dt, 4),
            "Kc":                  Kc,
            "Kh":                  Kh,
            "target_speed":        target_speed,
            "target_heading_deg":  round(math.degrees(target_heading), 2),
            "heading_error_deg":   round(math.degrees(heading_error),  4),
            "heading_correction":  round(heading_correction, 4),
            "error_L":             round(error_L,    4),
            "error_R":             round(error_R,    4),
            "command_L":           round(command_L,  4),
            "command_R":           round(command_R,  4),
            "left_speed":          round(left_speed,  4),
            "right_speed":         round(right_speed, 4),
            "v":                   round(v,  4),
            "w":                   round(w,  4),
            "theta_deg":           round(math.degrees(theta), 4),
            "x":                   round(x, 2),
            "y":                   round(y, 2),
        })
        csv_file.flush()

    # ── Drawing ───────────────────────────────────────────────────
    screen.fill(BLACK)

    # Trail
    if len(trail) > 1:
        for i in range(1, len(trail)):
            alpha = int(80 * i / len(trail))
            pygame.draw.line(screen,
                             (0, max(40, alpha * 2), max(20, alpha)),
                             trail[i-1], trail[i], 2)

    # Target heading compass (top-right corner)
    draw_target_heading_arrow()

    # Robot body
    pygame.draw.circle(screen, ROBOT_COL, (int(x), int(y)), 20)
    pygame.draw.circle(screen, WHITE,     (int(x), int(y)), 20, 2)

    # Current heading arrow (yellow)
    hx = x + 30 * math.cos(theta)
    hy = y + 30 * math.sin(theta)
    pygame.draw.line(screen, YELLOW, (int(x), int(y)), (int(hx), int(hy)), 3)

    # Target heading arrow (cyan, from robot)
    tx = x + 30 * math.cos(target_heading)
    ty = y + 30 * math.sin(target_heading)
    pygame.draw.line(screen, CYAN, (int(x), int(y)), (int(tx), int(ty)), 2)

    # Telemetry panel
    draw_telemetry(heading_error, heading_correction,
                   error_L, error_R, command_L, command_R, v, w)

    pygame.display.flip()

# ── Cleanup ────────────────────────────────────────────────────────────────
csv_file.close()
print(f"\n✅ Telemetry saved → {csv_path}")
pygame.quit()
