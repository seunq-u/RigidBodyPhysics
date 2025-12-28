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
    def __init__(self, center=(0.0, 0.0), zoom=450.0):
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
        self.zoom = max(30.0, min(8000.0, self.zoom * factor))


# =========================
# Naming
# =========================
def assign_names(bodies):
    counts = {"circle": 0, "box": 0, "floor": 0}
    for b in bodies:
        if getattr(b, "name", None):
            continue
        if isinstance(b.shape, Sphere):
            counts["circle"] += 1
            b.name = f"circle{counts['circle']}"
        elif isinstance(b.shape, Box):
            # 정적이면 floor로
            if bool(getattr(b, "is_static", False)):
                counts["floor"] += 1
                b.name = f"floor{counts['floor']}"
            else:
                counts["box"] += 1
                b.name = f"box{counts['box']}"
        else:
            b.name = f"body{len(counts)+1}"


# =========================
# Draw helpers
# =========================
def draw_crosshair(screen, color=(110, 110, 130)):
    W, H = screen.get_size()
    cx, cy = W//2, H//2
    L = 10
    pygame.draw.line(screen, color, (cx - L, cy), (cx + L, cy), 2)
    pygame.draw.line(screen, color, (cx, cy - L), (cx, cy + L), 2)

def draw_text(screen, font, x, y, s, color=(220, 220, 220)):
    surf = font.render(s, True, color)
    screen.blit(surf, (x, y))
    return y + surf.get_height() + 2

def draw_contact_debug(screen, cam, e, screen_wh, n_scale=0.18):
    for (cx, cy) in e.contacts:
        c = cam.world_to_screen((cx, cy), screen_wh)
        pygame.draw.circle(screen, (255, 80, 80), c, 4)
        nx, ny = e.normal
        tip = (cx + nx * n_scale, cy + ny * n_scale)
        t = cam.world_to_screen(tip, screen_wh)
        pygame.draw.line(screen, (255, 80, 80), c, t, 2)

def draw_sphere(screen, cam, body, screen_wh):
    c = cam.world_to_screen(body.r[:2], screen_wh)
    Rpx = max(1, int(body.shape.R * cam.zoom))
    pygame.draw.circle(screen, (235, 235, 235), c, Rpx, width=2)

    # 회전 막대(원 위)
    th = float(body.theta)
    end_xy = body.r[:2] + body.shape.R * np.array([np.cos(th), np.sin(th)], dtype=float)
    p2 = cam.world_to_screen(end_xy, screen_wh)
    pygame.draw.line(screen, (200, 200, 80), c, p2, 3)

    # 이름
    if hasattr(body, "name"):
        pygame.draw.circle(screen, (235, 235, 235), c, 2)
        name_pos = (c[0] + 8, c[1] - 18)
        screen.blit(pygame.font.SysFont(None, 20).render(body.name, True, (200, 200, 200)), name_pos)

def draw_box(screen, cam, body, screen_wh):
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
    color = (120, 190, 120) if bool(getattr(body, "is_static", False)) else (200, 200, 200)
    pygame.draw.polygon(screen, color, pts, width=2)

    # 방향 막대(로컬 x축)
    p2 = center + hw * ux
    pygame.draw.line(screen, (200, 200, 80),
                     cam.world_to_screen(center, screen_wh),
                     cam.world_to_screen(p2, screen_wh), 3)

    # 이름
    if hasattr(body, "name"):
        c_scr = cam.world_to_screen(center, screen_wh)
        name_pos = (c_scr[0] + 8, c_scr[1] - 18)
        screen.blit(pygame.font.SysFont(None, 20).render(body.name, True, (200, 200, 200)), name_pos)


# =========================
# Main
# =========================
def main():
    pygame.init()

    # 16:9
    W, H = 1280, 720
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("2.5D Physics Debug View")

    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", 18)

    cam = Camera(center=(0.0, 0.0), zoom=450.0)

    # follow control
    follow_enabled = True
    follow_index = 0

    # world
    world = World()

    floor = RigidBody(
        shape=Box(800.0, 0.7, 1.0),
        density=config.Density.Solid.Material.Iron.Steel,
        is_static=True,
        restitution=0.05,
        friction=0.9,
        r=[0.0, -1.4, 0.0],
    )

    ball1 = RigidBody(
        shape=Sphere(0.14),
        density=1000.0,
        r=[-1.2, 1.1, 999.0],
        v=[3.0, -1.0, 999.0],
        omega=[0.0, 0.0, 16.0],
        restitution=0.35,
        friction=0.35,
    )

    ball2 = RigidBody(
        shape=Sphere(0.10),
        density=1000.0,
        r=[1.0, 0.8, 0.0],
        v=[-2.0, -0.5, 0.0],
        omega=[0.0, 0.0, -10.0],
        restitution=0.30,
        friction=0.30,
    )

    box1 = RigidBody(
        shape=Box(0.6, 0.35, 1.0),
        density=7850.0,
        r=[0.6, 0.3, 0.0],
        v=[-0.6, 0.0, 0.0],
        omega=[0.0, 0.0, -2.0],
        restitution=0.2,
        friction=0.6,
    )

    world.add_body(floor)
    world.add_body(ball1)
    world.add_body(ball2)
    world.add_body(box1)

    assign_names(world.bodies)

    dt = 1.0 / 120.0
    running = True

    while running:
        # -------- input --------
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.KEYDOWN:
                # follow enable/disable
                if e.key == pygame.K_f:
                    follow_enabled = not follow_enabled

                # follow target change (only changes target; does not toggle off)
                if e.key == pygame.K_TAB:
                    follow_index = (follow_index + 1) % len(world.bodies)

                # zoom
                if e.key in (pygame.K_EQUALS, pygame.K_PLUS):
                    cam.zoom_at(1.15)
                elif e.key == pygame.K_MINUS:
                    cam.zoom_at(1.0 / 1.15)
            elif e.type == pygame.MOUSEWHEEL:
                cam.zoom_at(1.15 if e.y > 0 else 1.0 / 1.15)

        keys = pygame.key.get_pressed()
        pan_speed = 1.8 / cam.zoom * 320.0  # feel stable w.r.t zoom
        if keys[pygame.K_LEFT]:  cam.pan(-pan_speed, 0.0)
        if keys[pygame.K_RIGHT]: cam.pan(+pan_speed, 0.0)
        if keys[pygame.K_UP]:    cam.pan(0.0, +pan_speed)
        if keys[pygame.K_DOWN]:  cam.pan(0.0, -pan_speed)

        # -------- simulate --------
        for _ in range(2):
            world.step(dt/2.0, solver_iterations=12)

        # -------- follow camera --------
        if follow_enabled and len(world.bodies) > 0:
            target = world.bodies[follow_index]
            cam.center = np.asarray(target.r[:2], dtype=float).copy()

        # -------- collision debug snapshot --------
        debug_events = []
        bodies = world.bodies
        for i in range(len(bodies)):
            for j in range(i+1, len(bodies)):
                ev = collide(bodies[i], bodies[j])
                if ev is not None:
                    debug_events.append(ev)

        # -------- render --------
        screen.fill((18, 18, 22))

        for b in world.bodies:
            if isinstance(b.shape, Sphere):
                draw_sphere(screen, cam, b, (W, H))
            elif isinstance(b.shape, Box):
                draw_box(screen, cam, b, (W, H))

        for ev in debug_events:
            draw_contact_debug(screen, cam, ev, (W, H))

        draw_crosshair(screen)

        # -------- HUD --------
        y = 8
        follow_name = getattr(world.bodies[follow_index], "name", "?") if world.bodies else "-"
        y = draw_text(screen, font, 10, y, f"Camera center: ({cam.center[0]:.3f}, {cam.center[1]:.3f})   zoom={cam.zoom:.1f}")
        y = draw_text(screen, font, 10, y, f"Follow: {'ON' if follow_enabled else 'OFF'}   target=[TAB] -> {follow_name}   toggle=[F]")
        y = draw_text(screen, font, 10, y, "Controls: arrows=pan  mousewheel/+/-=zoom  TAB=next target  F=toggle follow")
        y += 6

        # bodies states
        for b in world.bodies:
            name = getattr(b, "name", "body")
            st = b.state()
            y = draw_text(
                screen, font, 10, y,
                f"{name:>7}: r=({st[0]: .3f},{st[1]: .3f}) v=({st[3]: .3f},{st[4]: .3f}) w={st[8]: .3f} th={st[9]: .3f}"
            )

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()


if __name__ == "__main__":
    main()
