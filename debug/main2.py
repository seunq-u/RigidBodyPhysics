import pygame
import numpy as np

from core.world import World
from core.body import RigidBody
from core.shape import Box, Sphere
from core.collision import collide
import config

# =========================
# Camera
# =========================
class Camera:
    def __init__(self, center=(0.0, 0.0), zoom=400.0):
        self.center = np.array(center, dtype=float)  # world xy at screen center
        self.zoom = float(zoom)

    def world_to_screen(self, p_xy, screen_wh):
        W, H = screen_wh
        x, y = float(p_xy[0]), float(p_xy[1])
        sx = W/2 + (x - self.center[0]) * self.zoom
        sy = H/2 - (y - self.center[1]) * self.zoom
        return int(sx), int(sy)

    def pan(self, dx, dy):
        self.center[0] += dx
        self.center[1] += dy

    def zoom_at(self, factor):
        self.zoom = max(20.0, min(5000.0, self.zoom * factor))


# =========================
# Draw helpers
# =========================
def draw_contact_debug(screen, cam, e, screen_wh, n_scale=0.2):
    # contact points
    for (cx, cy) in e.contacts:
        c = cam.world_to_screen((cx, cy), screen_wh)
        pygame.draw.circle(screen, (255, 80, 80), c, 4)

        nx, ny = e.normal
        tip = (cx + nx * n_scale, cy + ny * n_scale)
        t = cam.world_to_screen(tip, screen_wh)
        pygame.draw.line(screen, (255, 80, 80), c, t, 2)

def draw_sphere(screen, cam, body, screen_wh, color=(230, 230, 230)):
    c = cam.world_to_screen(body.r[:2], screen_wh)
    Rpx = int(body.shape.R * cam.zoom)

    pygame.draw.circle(screen, color, c, Rpx, width=2)

    # 회전 막대(원 위에)
    th = float(body.theta)
    end_xy = body.r[:2] + body.shape.R * np.array([np.cos(th), np.sin(th)], dtype=float)
    p2 = cam.world_to_screen(end_xy, screen_wh)
    pygame.draw.line(screen, (200, 200, 80), c, p2, 3)

def draw_box(screen, cam, body, screen_wh, color=(170, 220, 170)):
    w, h = float(body.shape.w), float(body.shape.h)
    hw, hh = 0.5*w, 0.5*h
    th = float(body.theta)
    c, s = np.cos(th), np.sin(th)
    ux = np.array([c, s], dtype=float)
    uy = np.array([-s, c], dtype=float)
    center = body.r[:2]

    pts = []
    for sx, sy in [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]:
        p = center + sx*ux + sy*uy
        pts.append(cam.world_to_screen(p, screen_wh))
    pygame.draw.polygon(screen, color, pts, width=2)

    # 방향 막대(박스 로컬 x축)
    p2 = center + hw * ux
    pygame.draw.line(screen, (200, 200, 80),
                     cam.world_to_screen(center, screen_wh),
                     cam.world_to_screen(p2, screen_wh), 3)


# =========================
# Main
# =========================
def main():
    pygame.init()
    W, H = 1000, 700
    screen = pygame.display.set_mode((W, H))
    clock = pygame.time.Clock()

    cam = Camera(center=(0.0, 0.0), zoom=400.0)
    follow = True  # 공 따라가기 토글

    world = World()

    floor = RigidBody(
        shape=Box(600.0, 0.6, 1.0),
        density=config.Density.Solid.Material.Iron.Steel,
        is_static=True,
        restitution=0.1,
        friction=0.9,
        r=[0.0, -1.0, 0.0],
    )

    ball = RigidBody(
        shape=Sphere(0.12),
        density=1000.0,
        r=[-0.8, 0.8, 123.0],
        v=[2.5, -1.0, 999.0],
        omega=[9.0, 2.0, 18.0],
        restitution=0.35,
        friction=0.35,
    )

    box = RigidBody(
        shape=Box(0.5, 0.3, 1.0),
        density=7850.0,
        r=[0.6, 0.2, 0.0],
        v=[-0.5, 0.0, 0.0],
        omega=[0.0, 0.0, -2.0],
        restitution=0.2,
        friction=0.6,
    )

    world.add_body(floor)
    world.add_body(ball)
    world.add_body(box)

    dt = 1.0 / 120.0
    running = True

    while running:
        # ----- events -----
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.KEYDOWN:
                if e.key == pygame.K_f:
                    follow = not follow
                elif e.key in (pygame.K_EQUALS, pygame.K_PLUS):
                    cam.zoom_at(1.15)
                elif e.key == pygame.K_MINUS:
                    cam.zoom_at(1.0 / 1.15)
            elif e.type == pygame.MOUSEWHEEL:
                cam.zoom_at(1.15 if e.y > 0 else 1.0 / 1.15)

        keys = pygame.key.get_pressed()
        pan_speed = 0.20 / cam.zoom * 300.0  # 화면 줌에 따라 체감 일정하게
        if keys[pygame.K_LEFT]:  cam.pan(-pan_speed, 0.0)
        if keys[pygame.K_RIGHT]: cam.pan(+pan_speed, 0.0)
        if keys[pygame.K_UP]:    cam.pan(0.0, +pan_speed)
        if keys[pygame.K_DOWN]:  cam.pan(0.0, -pan_speed)

        # ----- simulate (substeps for stability) -----
        for _ in range(2):
            world.step(dt/2.0, solver_iterations=12)

        # ----- camera follow -----
        if follow:
            cam.center = ball.r[:2].copy()

        # ----- compute collisions for debug draw (post-solve snapshot) -----
        debug_events = []
        bodies = world.bodies
        for i in range(len(bodies)):
            for j in range(i+1, len(bodies)):
                ev = collide(bodies[i], bodies[j])
                if ev is not None:
                    debug_events.append(ev)

        # ----- render -----
        screen.fill((18, 18, 22))

        for b in world.bodies:
            if isinstance(b.shape, Sphere):
                draw_sphere(screen, cam, b, (W, H))
            elif isinstance(b.shape, Box):
                draw_box(screen, cam, b, (W, H))

        # collision debug overlay
        for ev in debug_events:
            draw_contact_debug(screen, cam, ev, (W, H), n_scale=0.18)

        # small UI hint
        font = pygame.font.SysFont(None, 20)
        text = font.render("Arrows: pan | MouseWheel/+/-: zoom | F: follow toggle", True, (200, 200, 200))
        screen.blit(text, (12, 10))

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()
