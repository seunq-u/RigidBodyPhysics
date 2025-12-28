import pygame
import numpy as np
from core.world import World
from core.body import RigidBody
from core.shape import Box, Sphere
import config

# ---- helpers ----
SCALE = 400.0  # pixels per meter
W, H = 900, 600

def to_screen(p):
    x, y = float(p[0]), float(p[1])
    return int(W/2 + x*SCALE), int(H/2 - y*SCALE)

def draw_circle(screen, body, color=(240,240,240)):
    c = to_screen(body.r)
    r = int(body.shape.R * SCALE)
    pygame.draw.circle(screen, color, c, r, width=2)

def draw_obb(screen, body, color=(200,200,200)):
    w, h = body.shape.w, body.shape.h
    hw, hh = 0.5*w, 0.5*h
    th = float(body.theta)
    c, s = np.cos(th), np.sin(th)
    ux = np.array([c, s])
    uy = np.array([-s, c])
    center = body.r[:2]

    pts = []
    for sx, sy in [(-hw,-hh),(hw,-hh),(hw,hh),(-hw,hh)]:
        p = center + sx*ux + sy*uy
        pts.append(to_screen([p[0], p[1], 0.0]))
    pygame.draw.polygon(screen, color, pts, width=2)

# ---- scene ----
world = World()

floor = RigidBody(
    shape=Box(5.0, 0.5, 1.0),
    density=config.Density.Solid.Material.Iron.Steel,
    is_static=True,
    restitution=0.1,
    friction=0.8,
    r=[0.0, -1.0, 0.0],
)

ball = RigidBody(
    shape=Sphere(0.1),
    density=1000.0,
    r=[-0.5, 0.7, 0.0],
    v=[0.2, 0.0, 0.0],
    omega=[0.0, 0.0, 10.0],
    restitution=0.4,
    friction=0.3,
)

world.add_body(floor)
world.add_body(ball)

# ---- pygame loop ----
pygame.init()
screen = pygame.display.set_mode((W, H))
clock = pygame.time.Clock()

running = True
dt = 1.0/120.0

while running:
    for e in pygame.event.get():
        if e.type == pygame.QUIT:
            running = False

    # 여러 substep으로 안정화(충돌 있을 때 특히)
    for _ in range(2):
        world.step(dt/2, solver_iterations=20)

    screen.fill((20, 20, 20))
    draw_obb(screen, floor, (120, 180, 120))
    draw_circle(screen, ball, (220, 220, 220))
    pygame.display.flip()
    clock.tick(60)

pygame.quit()
