import pygame
import numpy as np
import math

pygame.init()

WIDTH = 900
HEIGHT = 600

screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Robot Simulation")
clock = pygame.time.Clock()

# Robot state in pygame
x = 200
y = 300 #x,y the robot pixel position on the screen                                                                                                                                                                                                                                         
theta = 0  # the robot heading angle in radians, 0 means facing right, pi/2 means facing down

wheel_base = 60 # the distance betwen the 2 wheels

left_speed = 0
right_speed = 0 #how fat each wheel is spinning

# Motor parameters
tau = 0.5          # motor time constant - how quickly the motor responds higher = slower response
dt = 0.016         # (60hz) - 60 updates per second the time between each update

left_motor_gain = 1.0
right_motor_gain = 1.1  # 10% stronger

# Controller gain
Kc = 22  # how agressively the controller responds to error, Higher = Faster but can be potentially unstable
# Kc = 20  # large loop case

target_speed = 100  #the desired speed i want for both wheels (100 pixels/sec)

# Simulation loop
running = True 

while running:
    clock.tick(60) #everything within the loop runs 60 times per second

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # --- Controller calculations (negative feedback) ---

    #     This is a proportional controller
    error_L = target_speed - left_speed
    error_R = target_speed - right_speed

    command_L = Kc * error_L
    command_R = Kc * error_R  #these signal is sent to the motor
    # As the wheel seeds up maybe left for example, as the error reduces, the command reduces and the motor balances

    # --- Motor dynamics ---
    left_speed  += dt * ((left_motor_gain  * command_L - left_speed)  / tau)
    right_speed += dt * ((right_motor_gain * command_R - right_speed) / tau)

    # --- Robot motion ---
    #  Differential Drive Kinematics for a 2 wheeled robot
    v = (left_speed + right_speed) / 2           # Linear speed = average of both wheels
    w = (right_speed - left_speed) / wheel_base  # Angular speed = difference of wheel speed / wheel base

    theta += w * dt #heading angle, updated in each frame

    x += v * math.cos(theta) * dt # position x, updated each frame based on heading angle
    y += v * math.sin(theta) * dt # position y, updated each frame based on heading angle

    # Wrap position so robot stays on screen
    x = x % WIDTH 
    y = y % HEIGHT

    # --- Drawing ---
    screen.fill((255, 255, 255))   # clear screen first

    pygame.draw.circle(screen, (0, 200, 0), (int(x), int(y)), 20)

    heading_x = x + 30 * math.cos(theta)
    heading_y = y + 30 * math.sin(theta)

    pygame.draw.line(screen, (255, 180, 0), (int(x), int(y)),
                     (int(heading_x), int(heading_y)), 3)

    pygame.display.flip()   # flip AFTER drawing

pygame.quit()