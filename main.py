import math
import pygame

pygame.init()
screen = pygame.display.set_mode((600, 600))
clock = pygame.time.Clock()
font = pygame.font.SysFont("helvetica", 18)

centre = (300, 300)
tilt_x = 20.0
tilt_y = 20.0
length = 260

swing_x = 0.0
total_x = 0.0

swing_y = 0.0
total_y = 0.0

kp = 0.1
ki = 0.01
kd = 0.4
limit = 0.5
gravity = 1.0
held = False

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.MOUSEBUTTONDOWN:
            held = True
        if event.type == pygame.MOUSEBUTTONUP:
            held = False

    # x brain
    error_x = 0 - tilt_x
    push_x = kp * error_x + ki * total_x - kd * swing_x
    if abs(push_x) < limit:
        total_x += error_x
    push_x = max(-limit, min(limit, push_x))

    # y brain
    error_y = 0 - tilt_y
    push_y = kp * error_y + ki * total_y - kd * swing_y
    if abs(push_y) < limit:
        total_y += error_y
    push_y = max(-limit, min(limit, push_y))

    # physics (cuz im mathing)
    if held:
        mx, my = pygame.mouse.get_pos()
        new_x = math.degrees(math.asin(max(-1, min(1, (mx - centre[0]) / length))))
        new_y = math.degrees(math.asin(max(-1, min(1, (my - centre[1]) / length))))
        swing_x = new_x - tilt_x
        swing_y = new_y - tilt_y
        tilt_x = new_x
        tilt_y = new_y
    else:
        swing_x += gravity * math.sin(math.radians(tilt_x))
        swing_x += push_x
        tilt_x += swing_x
        swing_y += gravity * math.sin(math.radians(tilt_y))
        swing_y += push_y
        tilt_y += swing_y

    dot_x = centre[0] + length * math.sin(math.radians(tilt_x))
    dot_y = centre[1] + length * math.sin(math.radians(tilt_y))

    noz_x = centre[0] + push_x / limit * 60
    noz_y = centre[1] + push_y / limit * 60
    
    gim_x = push_x / limit * 15
    gim_y = push_y / limit * 15
    
    screen.fill((20, 20, 20))
    pygame.draw.circle(screen, (60, 60, 60), centre, 130, 2)
    pygame.draw.line(screen, (255, 140, 0), centre, (noz_x, noz_y), 8)
    pygame.draw.circle(screen, (255, 255, 255), (dot_x, dot_y), 12)

    #text (im polished)
    text = font.render(f"tilt   x {round(tilt_x, 1)}  y {round(tilt_y, 1)}", True, (200, 200, 200))
    screen.blit(text, (20, 20))
    text = font.render(f"gimbal x {round(gim_x, 1)}  y {round(gim_y, 1)}", True, (200, 200, 200))
    screen.blit(text, (20, 44))
    
    pygame.display.flip()
    clock.tick(60)