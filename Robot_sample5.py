"""
=============================================================================
  ROBOT SIMULATION  —  Real-Time Control Panel + Live Telemetry
=============================================================================

  WHAT THIS PROGRAM DOES:
  ─────────────────────────
  It simulates a two-wheeled robot (like a Roomba) on screen.
  The robot has two motors — left and right.
  A "controller" tells each motor how fast to spin.
  Because the right motor is slightly stronger, the robot curves.

  LAYOUT:
  ─────────
  left side  = simulation canvas (robot moves here)
  right side = control panel (sliders + live numbers + live charts)

  CONTROLS:
  ──────────
  Drag any slider with the mouse to change parameters live.
  R  → reset the robot to starting position
  C  → clear the trail
  Q  → quit
=============================================================================
"""

import pygame
import math
import csv
import os
from datetime import datetime
from collections import deque   # deque = a list that auto-drops old values when full


# ─────────────────────────────────────────────────────────────────────────────
#  SECTION 1 — PYGAME INITIALISATION
# ─────────────────────────────────────────────────────────────────────────────
pygame.init()

TOTAL_W = 1280
TOTAL_H = 720
SIM_W   = 780       # simulation canvas width
SIM_H   = TOTAL_H
PANEL_X = SIM_W     # panel starts where sim ends
PANEL_W = TOTAL_W - SIM_W

screen = pygame.display.set_mode((TOTAL_W, TOTAL_H))
pygame.display.set_caption("Robot Sim — Live Control Panel")
clock = pygame.time.Clock()


# ─────────────────────────────────────────────────────────────────────────────
#  SECTION 2 — FONTS
# ─────────────────────────────────────────────────────────────────────────────
FONT_SM    = pygame.font.SysFont("Courier New", 13)
FONT_MD    = pygame.font.SysFont("Courier New", 15)
FONT_BOLD  = pygame.font.SysFont("Courier New", 15, bold=True)
FONT_TITLE = pygame.font.SysFont("Courier New", 20, bold=True)


# ─────────────────────────────────────────────────────────────────────────────
#  SECTION 3 — COLOUR PALETTE  (R, G, B)  each channel 0-255
# ─────────────────────────────────────────────────────────────────────────────
C_BG        = (12,  15,  25)
C_PANEL     = (20,  25,  40)
C_PANEL2    = (28,  34,  55)
C_BORDER    = (50,  60,  90)
C_CYAN      = (60,  210, 255)
C_GREEN     = (50,  220, 120)
C_YELLOW    = (255, 210, 60)
C_ORANGE    = (255, 150, 50)
C_RED       = (255, 80,  80)
C_WHITE     = (230, 235, 255)
C_DIM       = (80,  95,  130)
C_ROBOT     = (40,  200, 100)
C_SLIDER_BG = (35,  42,  65)
C_SLIDER_FG = (60,  210, 255)
C_SLIDER_KN = (200, 220, 255)


# ─────────────────────────────────────────────────────────────────────────────
#  SECTION 4 — SIMULATION PARAMETERS
#  Stored in a dict so sliders can read/write them by key name.
# ─────────────────────────────────────────────────────────────────────────────
params = {
    "Kc":               2.0,    # proportional controller gain
    "target_speed":   100.0,    # desired wheel speed (pixels/sec)
    "left_motor_gain":  1.0,    # left motor strength multiplier
    "right_motor_gain": 1.1,    # right motor strength (10% stronger => curves)
    "tau":              0.5,    # motor time constant (higher = slower spin-up)
}

# Each slider: (screen label, params key, min value, max value)
SLIDER_DEFS = [
    ("Kc  (controller gain)",  "Kc",               0.1,  20.0),
    ("Target Speed  (px/s)",   "target_speed",      10.0, 300.0),
    ("Left Motor Gain",        "left_motor_gain",   0.5,  2.0),
    ("Right Motor Gain",       "right_motor_gain",  0.5,  2.0),
    ("Tau  (motor lag)",       "tau",               0.05, 2.0),
]


# ─────────────────────────────────────────────────────────────────────────────
#  SECTION 5 — ROBOT STATE
# ─────────────────────────────────────────────────────────────────────────────
robot = {
    "x":           150.0,
    "y":           300.0,
    "theta":         0.0,   # heading in radians (0 = right)
    "left_speed":    0.0,
    "right_speed":   0.0,
}

WHEEL_BASE = 60     # distance between wheels in pixels
DT         = 0.016  # time step = 1/60 seconds


# ─────────────────────────────────────────────────────────────────────────────
#  SECTION 6 — HISTORY BUFFERS (scrolling chart data)
#  deque(maxlen=N) automatically drops the oldest item when it fills up.
# ─────────────────────────────────────────────────────────────────────────────
HISTORY_LEN = 200

history = {k: deque(maxlen=HISTORY_LEN) for k in
           ["left_speed", "right_speed", "error_L", "error_R",
            "command_L",  "command_R"]}

trail = deque(maxlen=500)   # robot position trail


# ─────────────────────────────────────────────────────────────────────────────
#  SECTION 7 — CSV LOGGING
# ─────────────────────────────────────────────────────────────────────────────
ts       = datetime.now().strftime("%Y%m%d_%H%M%S")
csv_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        f"telemetry_{ts}.csv")
CSV_COLS = ["frame","time_s","Kc","target_speed","left_gain","right_gain",
            "tau","error_L","error_R","command_L","command_R",
            "left_speed","right_speed","v","w","theta_deg","x","y"]

csv_file   = open(csv_path, "w", newline="")
csv_writer = csv.DictWriter(csv_file, fieldnames=CSV_COLS)
csv_writer.writeheader()
frame_num  = 0


# ─────────────────────────────────────────────────────────────────────────────
#  SECTION 8 — SLIDER CLASS
#
#  A Slider lets the user drag a knob left/right to change a parameter.
#  It reads and writes directly to the `params` dict.
# ─────────────────────────────────────────────────────────────────────────────
class Slider:
    def __init__(self, label, key, min_val, max_val, x, y, w=200, h=8):
        self.label    = label
        self.key      = key
        self.min_val  = min_val
        self.max_val  = max_val
        self.x        = x      # left edge of track
        self.y        = y      # top edge of track
        self.w        = w      # track width
        self.h        = h      # track height
        self.dragging = False

    @property
    def value(self):
        """Always reads live from shared params dict."""
        return params[self.key]

    def _val_to_px(self, val):
        """Convert a data value to a pixel x position on the track."""
        frac = (val - self.min_val) / (self.max_val - self.min_val)
        return int(self.x + frac * self.w)

    def _px_to_val(self, px):
        """Convert a pixel x position to a data value. Clamped to [min, max]."""
        frac = (px - self.x) / self.w
        frac = max(0.0, min(1.0, frac))
        return self.min_val + frac * (self.max_val - self.min_val)

    def handle_event(self, event):
        """
        MOUSEBUTTONDOWN  — check if click hit the knob → start dragging
        MOUSEBUTTONUP    — stop dragging
        MOUSEMOTION      — if dragging, update params[key]
        """
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            kx = self._val_to_px(self.value)
            if pygame.Rect(kx - 8, self.y - 6, 16, self.h + 12).collidepoint(event.pos):
                self.dragging = True
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            self.dragging = False
        elif event.type == pygame.MOUSEMOTION and self.dragging:
            params[self.key] = round(self._px_to_val(event.pos[0]), 3)

    def draw(self, surface):
        # Label above track
        surface.blit(FONT_SM.render(self.label, True, C_DIM),
                     (self.x, self.y - 18))
        # Background track
        pygame.draw.rect(surface, C_SLIDER_BG,
                         (self.x, self.y, self.w, self.h), border_radius=4)
        # Filled (coloured) portion up to knob
        kx = self._val_to_px(self.value)
        if kx > self.x:
            pygame.draw.rect(surface, C_SLIDER_FG,
                             (self.x, self.y, kx - self.x, self.h), border_radius=4)
        # Knob rectangle
        knob_col = C_WHITE if self.dragging else C_SLIDER_KN
        pygame.draw.rect(surface, knob_col,
                         (kx - 5, self.y - 4, 10, self.h + 8), border_radius=3)
        # Numeric value to the right
        surface.blit(FONT_MD.render(f"{self.value:.3f}", True, C_CYAN),
                     (self.x + self.w + 10, self.y - 3))


# ─────────────────────────────────────────────────────────────────────────────
#  SECTION 9 — BUILD SLIDER LIST
# ─────────────────────────────────────────────────────────────────────────────
SLIDER_START_Y = 60
SLIDER_SPACING = 58
SLIDER_X       = PANEL_X + 20
SLIDER_W       = PANEL_W - 80

sliders = [
    Slider(label, key, lo, hi,
           SLIDER_X,
           SLIDER_START_Y + i * SLIDER_SPACING,
           SLIDER_W)
    for i, (label, key, lo, hi) in enumerate(SLIDER_DEFS)
]


# ─────────────────────────────────────────────────────────────────────────────
#  SECTION 10 — DRAWING HELPERS
# ─────────────────────────────────────────────────────────────────────────────

def draw_label_value(surface, label, value, color, x, y, fmt=".2f"):
    """Two-line block: small dim label on top, bright value below."""
    surface.blit(FONT_SM.render(label, True, C_DIM),         (x, y))
    surface.blit(FONT_BOLD.render(f"{value:{fmt}}", True, color), (x, y + 16))


def draw_live_chart(surface, data_a, data_b, lbl_a, lbl_b,
                    col_a, col_b, rect, y_min, y_max, title=""):
    """
    Scrolling two-line chart inside rect.
    data_a / data_b : deques of floats
    col_a  / col_b  : line colours
    y_min  / y_max  : value range mapped to chart height
    """
    rx, ry, rw, rh = rect
    pygame.draw.rect(surface, (15, 20, 35), rect, border_radius=4)
    pygame.draw.rect(surface, C_BORDER,     rect, 1, border_radius=4)

    if title:
        surface.blit(FONT_SM.render(title, True, C_DIM), (rx + 4, ry + 3))

    def vy(v):
        """Value  →  pixel y  (bottom of chart = low value)"""
        frac = max(0.0, min(1.0, (v - y_min) / max(y_max - y_min, 1)))
        return int(ry + rh - frac * rh)

    def vx(i, n):
        """Sample index  →  pixel x  (scrolls left as new data arrives)"""
        return int(rx + (i / max(n - 1, 1)) * rw)

    for data, col in [(data_a, col_a), (data_b, col_b)]:
        pts = list(data)
        if len(pts) < 2:
            continue
        n   = len(pts)
        coords = [(vx(i, n), vy(pts[i])) for i in range(n)]
        pygame.draw.lines(surface, col, False, coords, 2)

    # Legend
    lx, ly = rx + 4, ry + rh - 18
    pygame.draw.rect(surface, col_a, (lx,      ly, 8, 8))
    pygame.draw.rect(surface, col_b, (lx + 60, ly, 8, 8))
    surface.blit(FONT_SM.render(lbl_a, True, col_a), (lx + 10,     ly - 1))
    surface.blit(FONT_SM.render(lbl_b, True, col_b), (lx + 70, ly - 1))


def draw_sim_bg(surface):
    """Dark background + subtle dot grid for the simulation canvas."""
    pygame.draw.rect(surface, C_BG, (0, 0, SIM_W, SIM_H))
    for gx in range(0, SIM_W, 40):
        for gy in range(0, SIM_H, 40):
            pygame.draw.circle(surface, (22, 28, 45), (gx, gy), 1)


def reset_robot():
    """Return robot to start position and zero all state."""
    robot.update({"x": 150.0, "y": 300.0, "theta": 0.0,
                  "left_speed": 0.0, "right_speed": 0.0})
    trail.clear()
    for buf in history.values():
        buf.clear()


# ─────────────────────────────────────────────────────────────────────────────
#  SECTION 11 — MAIN LOOP  (runs 60× per second)
# ─────────────────────────────────────────────────────────────────────────────
running = True

while running:

    # ── Timing ───────────────────────────────────────────────────────────────
    clock.tick(60)
    frame_num += 1

    # ── Events ───────────────────────────────────────────────────────────────
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:   running = False
            if event.key == pygame.K_r:   reset_robot()
            if event.key == pygame.K_c:   trail.clear()
        for slider in sliders:
            slider.handle_event(event)

    # ── Read params (snapshot so values are consistent this whole frame) ──────
    Kc               = params["Kc"]
    target_speed     = params["target_speed"]
    left_motor_gain  = params["left_motor_gain"]
    right_motor_gain = params["right_motor_gain"]
    tau              = params["tau"]
    left_speed       = robot["left_speed"]
    right_speed      = robot["right_speed"]

    # ── CONTROLLER  (proportional / P-controller) ─────────────────────────
    #
    #  error   = how far the wheel speed is from target
    #  command = Kc * error  →  the signal sent to the motor driver
    #
    #  When speed == target  →  error = 0  →  command = 0  (motor rests)
    #  When speed < target   →  positive error  →  positive push
    #  This automatic self-correction is called NEGATIVE FEEDBACK.
    # ─────────────────────────────────────────────────────────────────────────
    error_L   = target_speed - left_speed
    error_R   = target_speed - right_speed
    command_L = Kc * error_L
    command_R = Kc * error_R

    # ── MOTOR DYNAMICS  (first-order lag) ─────────────────────────────────
    #
    #  Real motors ramp up gradually, not instantly.
    #  tau controls how fast — small tau = snappy, large tau = sluggish.
    #
    #  Discrete formula:  new_speed += DT * ( (gain*cmd - speed) / tau )
    #  This is equivalent to the differential equation:
    #      tau * d(speed)/dt = gain*cmd - speed
    # ─────────────────────────────────────────────────────────────────────────
    left_speed  += DT * ((left_motor_gain  * command_L - left_speed)  / tau)
    right_speed += DT * ((right_motor_gain * command_R - right_speed) / tau)
    robot["left_speed"]  = left_speed
    robot["right_speed"] = right_speed

    # ── DIFFERENTIAL DRIVE KINEMATICS ─────────────────────────────────────
    #
    #  v = linear velocity  = average of both wheels  → how fast forward
    #  w = angular velocity = (right - left) / wheelbase  → how fast turning
    #
    #  right > left  →  w is positive  →  robot turns LEFT
    #  left  > right →  w is negative  →  robot turns RIGHT
    #
    #  Position update:
    #      theta  +=  w * DT                    (turn a little each frame)
    #      x      +=  v * cos(theta) * DT       (move in x direction)
    #      y      +=  v * sin(theta) * DT       (move in y direction)
    # ─────────────────────────────────────────────────────────────────────────
    v = (left_speed + right_speed) / 2.0
    w = (right_speed - left_speed) / WHEEL_BASE
    robot["theta"] += w * DT
    robot["x"]     += v * math.cos(robot["theta"]) * DT
    robot["y"]     += v * math.sin(robot["theta"]) * DT
    robot["x"]      = robot["x"] % SIM_W   # wrap so robot stays on screen
    robot["y"]      = robot["y"] % SIM_H

    # ── Update history buffers ────────────────────────────────────────────
    history["left_speed"].append(left_speed)
    history["right_speed"].append(right_speed)
    history["error_L"].append(error_L)
    history["error_R"].append(error_R)
    history["command_L"].append(command_L)
    history["command_R"].append(command_R)
    trail.append((int(robot["x"]), int(robot["y"])))

    # ── CSV logging ───────────────────────────────────────────────────────
    if frame_num % 3 == 0:
        csv_writer.writerow({
            "frame": frame_num,  "time_s": round(frame_num * DT, 4),
            "Kc": Kc,  "target_speed": target_speed,
            "left_gain": left_motor_gain,  "right_gain": right_motor_gain,
            "tau": tau,
            "error_L":    round(error_L,   4),  "error_R":   round(error_R,   4),
            "command_L":  round(command_L, 4),  "command_R": round(command_R, 4),
            "left_speed": round(left_speed, 4), "right_speed": round(right_speed, 4),
            "v": round(v, 4),  "w": round(w, 4),
            "theta_deg": round(math.degrees(robot["theta"]), 3),
            "x": round(robot["x"], 2),  "y": round(robot["y"], 2),
        })
        csv_file.flush()

    # =========================================================================
    #  DRAW EVERYTHING
    #  Order matters: things drawn later appear on top.
    # =========================================================================

    # ── Simulation canvas ─────────────────────────────────────────────────
    draw_sim_bg(screen)

    # Trail (faded path)
    pts = list(trail)
    for i in range(1, len(pts)):
        b = int(40 + 160 * (i / len(pts)))
        pygame.draw.line(screen, (0, b // 2, b // 3), pts[i-1], pts[i], 2)

    # Robot body + outline
    rx, ry = int(robot["x"]), int(robot["y"])
    pygame.draw.circle(screen, C_ROBOT, (rx, ry), 20)
    pygame.draw.circle(screen, C_WHITE, (rx, ry), 20, 2)

    # Heading arrow
    hx = robot["x"] + 28 * math.cos(robot["theta"])
    hy = robot["y"] + 28 * math.sin(robot["theta"])
    pygame.draw.line(screen, C_YELLOW, (rx, ry), (int(hx), int(hy)), 3)
    pygame.draw.circle(screen, C_YELLOW, (int(hx), int(hy)), 4)

    # Sim-area border + corner info
    pygame.draw.line(screen, C_BORDER, (SIM_W, 0), (SIM_W, SIM_H), 2)
    for li, txt in enumerate([
        f"x={robot['x']:6.1f}  y={robot['y']:6.1f}",
        f"theta={math.degrees(robot['theta'])%360:.1f} deg",
        f"v={v:.1f}  omega={w:.4f}",
        "",
        "R=reset  C=clear  Q=quit",
    ]):
        screen.blit(FONT_SM.render(txt, True, C_DIM), (8, 8 + li * 16))

    # ── Panel background ──────────────────────────────────────────────────
    pygame.draw.rect(screen, C_PANEL, (PANEL_X, 0, PANEL_W, TOTAL_H))
    screen.blit(FONT_TITLE.render("CONTROL PANEL", True, C_CYAN),
                (PANEL_X + 14, 14))
    pygame.draw.line(screen, C_BORDER,
                     (PANEL_X + 10, 42), (PANEL_X + PANEL_W - 10, 42), 1)

    # ── Sliders ───────────────────────────────────────────────────────────
    for s in sliders:
        s.draw(screen)

    # Divider after sliders
    div_y = SLIDER_START_Y + len(sliders) * SLIDER_SPACING + 10
    pygame.draw.line(screen, C_BORDER,
                     (PANEL_X + 10, div_y), (PANEL_X + PANEL_W - 10, div_y), 1)

    # ── Live telemetry numbers ────────────────────────────────────────────
    #  These re-render every frame so you always see the current values.
    ty     = div_y + 12
    col1   = PANEL_X + 14
    col2   = PANEL_X + PANEL_W // 2 + 5

    screen.blit(FONT_BOLD.render("LIVE TELEMETRY", True, C_CYAN), (col1, ty))
    ty += 24

    # Two-column grid of values
    readouts = [
        ("Error  L",    error_L,    C_RED,    col1),
        ("Error  R",    error_R,    C_ORANGE, col2),
        ("Command L",   command_L,  C_CYAN,   col1),
        ("Command R",   command_R,  C_YELLOW, col2),
        ("Left Speed",  left_speed, C_GREEN,  col1),
        ("Right Speed", right_speed,C_YELLOW, col2),
        ("Lin Vel  v",  v,          C_WHITE,  col1),
        ("Ang Vel  w",  w,          C_ORANGE, col2),
    ]
    ROW_H = 40
    for idx, (lbl, val, col, cx) in enumerate(readouts):
        row_y = ty + (idx // 2) * ROW_H
        if (idx // 2) % 2 == 0:
            pygame.draw.rect(screen, C_PANEL2,
                             (PANEL_X + 4, row_y - 2, PANEL_W - 8, ROW_H - 2),
                             border_radius=3)
        draw_label_value(screen, lbl, val, col, cx, row_y)

    ty += (len(readouts) // 2) * ROW_H + 8

    # ── Live charts ───────────────────────────────────────────────────────
    pygame.draw.line(screen, C_BORDER,
                     (PANEL_X + 10, ty), (PANEL_X + PANEL_W - 10, ty), 1)
    ty += 8
    screen.blit(FONT_BOLD.render("LIVE CHARTS", True, C_CYAN), (col1, ty))
    ty += 20

    CW = PANEL_W - 20
    CH = 72

    # Chart 1 — Wheel Speeds
    draw_live_chart(screen,
                    history["left_speed"], history["right_speed"],
                    "L-spd", "R-spd", C_GREEN, C_YELLOW,
                    pygame.Rect(PANEL_X + 10, ty, CW, CH),
                    0, max(target_speed * 1.3, 10), "Wheel Speeds")
    ty += CH + 8

    # Chart 2 — Errors
    draw_live_chart(screen,
                    history["error_L"], history["error_R"],
                    "Err-L", "Err-R", C_RED, C_ORANGE,
                    pygame.Rect(PANEL_X + 10, ty, CW, CH),
                    0, max(target_speed * 1.1, 10), "Errors")
    ty += CH + 8

    # Chart 3 — Commands
    draw_live_chart(screen,
                    history["command_L"], history["command_R"],
                    "Cmd-L", "Cmd-R", C_CYAN, C_YELLOW,
                    pygame.Rect(PANEL_X + 10, ty, CW, CH),
                    0, max(Kc * target_speed * 1.1, 10), "Commands")

    # ── Flip (double-buffer swap — shows everything drawn this frame) ─────
    pygame.display.flip()


# ─────────────────────────────────────────────────────────────────────────────
#  SECTION 12 — CLEANUP
# ─────────────────────────────────────────────────────────────────────────────
csv_file.close()
print(f"\nDone. Telemetry saved to: {csv_path}")
pygame.quit()