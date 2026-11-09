import math
import pygame

pygame.init()
screen = pygame.display.set_mode((600, 600))
clock = pygame.time.Clock()
font = pygame.font.SysFont("helvetica", 20)

angle = 20.0
pivot = (300, 450)
length = 250
swing = 0.0
total = 0.0

kp = 0.1
ki = 0.01
kd = 0.4
limit = 0.5
held = False
gravity = 1.0 #woah gravity

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.MOUSEBUTTONDOWN:
            held = True
        if event.type == pygame.MOUSEBUTTONUP:
            held = False

    # pid
    error = 0 - angle
    push = kp * error + ki * total - kd * swing
    if abs(push) < limit:
        total += error
    push = max(-limit, min(limit, push))

    # physics
    if held:
        mx, my = pygame.mouse.get_pos()
        new_angle = math.degrees(math.atan2(mx - pivot[0], pivot[1] - my))
        swing = new_angle - angle
        angle = new_angle
    else:
        swing += gravity * math.sin(math.radians(angle))
        swing += push
        angle += swing

    tip_x = pivot[0] + length * math.sin(math.radians(angle))
    tip_y = pivot[1] - length * math.cos(math.radians(angle))

    gimbal = push / limit * 15
    noz = math.radians(angle + 180 + gimbal)
    noz_x = pivot[0] + 60 * math.sin(noz)
    noz_y = pivot[1] - 60 * math.cos(noz)

    screen.fill((20, 20, 20))
    
    text = font.render(f"angle {angle:6.1f}", True, (200, 200, 200))
    screen.blit(text, (20, 20))
    
    text = font.render(f"gimbal thing {gimbal:6.1f}", True, (200, 200, 200))
    screen.blit(text, (20, 50))
    
    pygame.draw.line(screen, (255, 140, 0), pivot, (noz_x, noz_y), 12)
    pygame.draw.line(screen, (255, 255, 255), pivot, (tip_x, tip_y), 16)

    pygame.display.flip()
    clock.tick(60)