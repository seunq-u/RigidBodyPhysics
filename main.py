import pygame
import numpy as np

from core.world import World
from core.body import RigidBody
from core.shape import Box, Sphere
from core.collision import collide
from core.background import Background
import config


class Camera:
    def __init__(self, center=(0.0, 0.0), zoom=450.0):
        self.center = np.array(center, dtype=float)
        self.zoom = float(zoom)

    def world_to_screen(self, p_xy, screen_wh):
        W, H = screen_wh
        x, y = float(p_xy[0]), float(p_xy[1])
        sx = W / 2 + (x - self.center[0]) * self.zoom
        sy = H / 2 - (y - self.center[1]) * self.zoom
        return int(sx), int(sy)

    def screen_to_world(self, s_xy, screen_wh):
        W, H = screen_wh
        sx, sy = float(s_xy[0]), float(s_xy[1])
        x = (sx - W / 2) / self.zoom + self.center[0]
        y = -(sy - H / 2) / self.zoom + self.center[1]
        return np.array([x, y], dtype=float)

    def pan(self, dx, dy):
        self.center[0] += dx
        self.center[1] += dy

    def zoom_at(self, factor):
        self.zoom = max(30.0, min(8000.0, self.zoom * factor))


def assign_names(bodies):
    counts = {"circle": 0, "box": 0, "floor": 0, "wall": 0}
    for b in bodies:
        if getattr(b, "name", None):
            continue
        if isinstance(b.shape, Sphere):
            counts["circle"] += 1
            b.name = f"circle{counts['circle']}"
        elif isinstance(b.shape, Box):
            if bool(getattr(b, "is_static", False)):
                if float(b.shape.w) < 1.0:
                    counts["wall"] += 1
                    b.name = f"wall{counts['wall']}"
                else:
                    counts["floor"] += 1
                    b.name = f"floor{counts['floor']}"
            else:
                counts["box"] += 1
                b.name = f"box{counts['box']}"
        else:
            b.name = "body"


def dynamic_indices(bodies):
    return [i for i, b in enumerate(bodies) if not bool(getattr(b, "is_static", False))]


def draw_crosshair(screen, color=(110, 110, 130)):
    W, H = screen.get_size()
    cx, cy = W // 2, H // 2
    L = 10
    pygame.draw.line(screen, color, (cx - L, cy), (cx + L, cy), 2)
    pygame.draw.line(screen, color, (cx, cy - L), (cx, cy + L), 2)


def draw_text(screen, font, x, y, s, color=(220, 220, 220)):
    surf = font.render(s, True, color)
    screen.blit(surf, (x, y))
    return y + surf.get_height() + 2


def draw_contact_debug(screen, cam, e, screen_wh, font_small, n_scale=0.18):
    nx, ny = e.normal
    for (cx, cy) in e.contacts:
        c = cam.world_to_screen((cx, cy), screen_wh)
        pygame.draw.circle(screen, (255, 80, 80), c, 4)
        tip = (cx + nx * n_scale, cy + ny * n_scale)
        t = cam.world_to_screen(tip, screen_wh)
        pygame.draw.line(screen, (255, 80, 80), c, t, 2)
        txt = f"pen={float(e.penetration):.4f} n=({nx:+.2f},{ny:+.2f})"
        screen.blit(font_small.render(txt, True, (255, 120, 120)), (c[0] + 6, c[1] + 4))


def draw_sphere(screen, cam, body, screen_wh, font_name, selected=False):
    c = cam.world_to_screen(body.r[:2], screen_wh)
    Rpx = max(1, int(body.shape.R * cam.zoom))
    col = (255, 240, 140) if selected else (235, 235, 235)
    pygame.draw.circle(screen, col, c, Rpx, width=3 if selected else 2)

    th = float(body.theta)
    end_xy = body.r[:2] + body.shape.R * np.array([np.cos(th), np.sin(th)], dtype=float)
    p2 = cam.world_to_screen(end_xy, screen_wh)
    pygame.draw.line(screen, (200, 200, 80), c, p2, 3)

    name = getattr(body, "name", "")
    if name:
        screen.blit(font_name.render(name, True, (200, 200, 200)), (c[0] + 8, c[1] - 18))


def draw_box(screen, cam, body, screen_wh, font_name, selected=False):
    w, h = float(body.shape.w), float(body.shape.h)
    hw, hh = 0.5 * w, 0.5 * h
    th = float(body.theta)
    c, s = np.cos(th), np.sin(th)
    ux = np.array([c, s], dtype=float)
    uy = np.array([-s, c], dtype=float)
    center = body.r[:2]

    pts = []
    for sx, sy in [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]:
        p = center + sx * ux + sy * uy
        pts.append(cam.world_to_screen(p, screen_wh))

    base = (120, 190, 120) if bool(getattr(body, "is_static", False)) else (200, 200, 200)
    col = (255, 240, 140) if selected else base
    pygame.draw.polygon(screen, col, pts, width=3 if selected else 2)

    p2 = center + hw * ux
    pygame.draw.line(
        screen, (200, 200, 80),
        cam.world_to_screen(center, screen_wh),
        cam.world_to_screen(p2, screen_wh),
        3
    )

    name = getattr(body, "name", "")
    if name:
        c_scr = cam.world_to_screen(center, screen_wh)
        screen.blit(font_name.render(name, True, (200, 200, 200)), (c_scr[0] + 8, c_scr[1] - 18))


def pick_body(world, cam, mouse_pos, screen_wh, max_px=22):
    m_world = cam.screen_to_world(mouse_pos, screen_wh)
    best = None
    best_d2 = float("inf")
    for b in world.bodies:
        if bool(getattr(b, "is_static", False)):
            continue
        p = np.asarray(b.r[:2], dtype=float)
        p_scr = cam.world_to_screen(p, screen_wh)
        dx = mouse_pos[0] - p_scr[0]
        dy = mouse_pos[1] - p_scr[1]
        if dx*dx + dy*dy > max_px*max_px:
            continue
        d2 = float(np.dot(m_world - p, m_world - p))
        if d2 < best_d2:
            best_d2 = d2
            best = b
    return best, m_world


def apply_impulse(body, J_xy, contact_world_xy):
    """Apply impulse J at contact point, updates v and omega_z."""
    if body is None or bool(getattr(body, "is_static", False)):
        return
    J = np.asarray(J_xy, dtype=float).reshape(2)
    if np.allclose(J, 0.0):
        return

    m = float(body.mass)
    I = float(body.Izz)
    if (not np.isfinite(m)) or m <= 0.0:
        return

    body.v[0] += float(J[0] / m)
    body.v[1] += float(J[1] / m)

    if np.isfinite(I) and I > 0.0:
        r = np.asarray(contact_world_xy, dtype=float).reshape(2) - np.asarray(body.r[:2], dtype=float)
        tau_z = float(r[0]*J[1] - r[1]*J[0])
        body.omega[2] += float(tau_z / I)


def snapshot_states(world):
    return [b.state().copy() for b in world.bodies]


def restore_states(world, snap):
    for b, s in zip(world.bodies, snap):
        b.set_state(s)


def add_walls(world, half_width=15, wall_thickness=1, wall_height=7.0, y_center=1.2, density=7850.0):
    wl = RigidBody(
        shape=Box(wall_thickness, wall_height, 1.0),
        density=density,
        is_static=True,
        restitution=0.05,
        friction=0.9,
        r=[-half_width - wall_thickness/2.0, y_center, 0.0],
    )
    wr = RigidBody(
        shape=Box(wall_thickness, wall_height, 1.0),
        density=density,
        is_static=True,
        restitution=0.05,
        friction=0.9,
        r=[+half_width + wall_thickness/2.0, y_center, 0.0],
    )
    world.add_body(wl)
    world.add_body(wr)


def main():
    pygame.init()
    W, H = 1600, 900
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("2.5D Physics Debug View")

    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", 18)
    font_small = pygame.font.SysFont("consolas", 14)
    font_name = pygame.font.SysFont(None, 20)

    cam = Camera(center=(0.0, 0.0), zoom=35.0)

    # follow
    follow_enabled = True
    smooth_follow = False
    follow_alpha = 0.12
    follow_index = 0

    # HUD / pause
    hud_enabled = True
    paused = False
    step_once = False

    # toggles
    gravity_on = True
    drag_on = True
    smooth_follow = False

    # impulse behavior
    impulse_equal_dv = True 

    # selection
    selected_body = None

    # mouse interaction pause control
    mouse_paused = False
    paused_before_mouse = False

    # dragging modes
    dragging_left = False    # impulse drag
    drag_body = None
    drag_start = None
    drag_contact = None
    drag_gain = 25.0

    world = World(background=Background(Density=config.Density.Liquid.Water.Pure, Absolute_Viscosity=config.Absolute_Viscosity.PureWater, Kinematic_Viscosity=config.Kinematic_Viscosity.PureWater))
    # world = World()
    floor = RigidBody(
        shape=Box(3000.0, 1, 1.0),
        density=config.Density.Solid.Material.Iron.Steel,
        is_static=True,
        restitution=0.05,
        friction=0.9,
        r=[0.0, -1.4, 0.0],
    )
    world.add_body(floor)
    add_walls(world)

    # dynamic bodies (예시)
    ball1 = RigidBody(
        shape=Sphere(1),
        density=config.Density.Solid.Wood.Quercus_Acutissima,
        r=[-1.8, 1.8, 0.0],
        v=[5.5, 3.0, 0.0],
        omega=[0.0, 0.0, 18.0],
        restitution=0.35,
        friction=0.35,
    )
    box = RigidBody(
        shape=Box(1.6, 1.35, 2.0),
        density=config.Density.Solid.Material.Iron.Cast,
        r=[1.7, 10.3, 0.0],
        v=[-3.3, -4.0, 0.0],
        omega=[0.0, 0.0, -2.0],
        restitution=0.2,
        friction=0.6,
    )
    box2 = RigidBody(
        shape=Box(1, 1, 2.0),
        density=config.Density.Solid.Wood.Cherry,
        r=[-12.7, 3.3, 0.0],
        v=[25.3, 0.0, 0.0],
        omega=[0.0, 0.0, -10.0],
        restitution=0.2,
        friction=0.6,
    )
    ball2 = RigidBody(
        shape=Sphere(0.5),
        density=config.Density.Solid.Rock.Sedimentary.Limestone,
        r=[5.0, 2.8, 0.0],
        v=[-50.0, -3.5, 0.0],
        omega=[0.0, 0.0, -10.0],
        restitution=0.2,
        friction=0.6,
    )
    ball3 = RigidBody(
        shape=Sphere(0.7),
        density=config.Density.Solid.Material.Common.Aluminum,
        r=[-8.0, 1.2, 0.0],
        v=[-15.0, +15.0, 0.0],
        omega=[0.0, 0.0, +10.0],
        restitution=0.2,
        friction=0.6,
    )
    world.add_body(ball1)
    world.add_body(ball2)
    world.add_body(ball3)
    world.add_body(box)
    world.add_body(box2)

    assign_names(world.bodies)

    initial = snapshot_states(world)
    initial_cam = cam.center.copy()
    initial_zoom = cam.zoom

    dt = 1.0 / 120.0
    running = True

    while running:
        dyn_idx = dynamic_indices(world.bodies)
        if dyn_idx:
            follow_index = max(0, min(follow_index, len(dyn_idx) - 1))

        # -------- events --------
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False

            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_h:
                    hud_enabled = not hud_enabled
                elif ev.key == pygame.K_SPACE:
                    paused = not paused
                elif ev.key == pygame.K_n:
                    step_once = True
                elif ev.key == pygame.K_f:
                    follow_enabled = not follow_enabled
                elif ev.key == pygame.K_TAB:
                    if dyn_idx:
                        follow_index = (follow_index + 1) % len(dyn_idx)
                elif ev.key == pygame.K_r:
                    restore_states(world, initial)
                    cam.center = initial_cam.copy()
                    cam.zoom = initial_zoom
                    selected_body = None
                elif ev.key == pygame.K_g:
                    gravity_on = not gravity_on
                elif ev.key == pygame.K_d:
                    drag_on = not drag_on
                elif ev.key == pygame.K_s:
                    smooth_follow = not smooth_follow
                elif ev.key == pygame.K_j:
                    impulse_equal_dv = not impulse_equal_dv
                elif ev.key in (pygame.K_EQUALS, pygame.K_PLUS):
                    cam.zoom_at(1.15)
                elif ev.key == pygame.K_MINUS:
                    cam.zoom_at(1.0 / 1.15)

            elif ev.type == pygame.MOUSEWHEEL:
                cam.zoom_at(1.15 if ev.y > 0 else 1.0 / 1.15)

            elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button in (1, ):
                # click -> auto pause
                if not mouse_paused:
                    paused_before_mouse = paused
                    paused = True
                    mouse_paused = True

                b, hit = pick_body(world, cam, ev.pos, (W, H))
                if b is not None:
                    selected_body = b

                    # 드래그한 물체를 팔로우 대상으로 자동 설정 + 스냅
                    if follow_enabled and dyn_idx:
                        bi = world.bodies.index(b)
                        if bi in dyn_idx:
                            follow_index = dyn_idx.index(bi)
                            cam.center = np.asarray(b.r[:2], dtype=float).copy()

                    drag_body = b
                    drag_start = hit.copy()
                    drag_contact = hit.copy()
                    dragging_left = True

            elif ev.type == pygame.MOUSEBUTTONUP and ev.button in (1, ):
                # left release -> impulse
                if ev.button == 1 and dragging_left and drag_body is not None:
                    end = cam.screen_to_world(ev.pos, (W, H))
                    d = end - drag_start  
                    if impulse_equal_dv:
                        J = float(drag_gain) * float(drag_body.mass) * d
                    else:
                        J = float(drag_gain) * d
                    apply_impulse(drag_body, J, drag_contact)
                dragging_left = False
                drag_body = None
                drag_start = None
                drag_contact = None
                if mouse_paused:
                    paused = paused_before_mouse
                    mouse_paused = False

        # -------- manual pan --------
        keys = pygame.key.get_pressed()
        pan_speed = 0.05 / cam.zoom * 320.0
        if keys[pygame.K_LEFT]:  cam.pan(-pan_speed, 0.0)
        if keys[pygame.K_RIGHT]: cam.pan(+pan_speed, 0.0)
        if keys[pygame.K_UP]:    cam.pan(0.0, +pan_speed)
        if keys[pygame.K_DOWN]:  cam.pan(0.0, -pan_speed)

        # -------- simulate --------
        do_sim = (not paused) or step_once
        if do_sim:
            for _ in range(2):
                world.step(dt / 2.0, solver_iterations=20,
                            enable_gravity=gravity_on, enable_drag=drag_on)
            step_once = False

        # -------- follow camera --------
        dyn_idx = dynamic_indices(world.bodies)
        if follow_enabled and dyn_idx:
            target = world.bodies[dyn_idx[follow_index]]
            desired = np.asarray(target.r[:2], dtype=float)
            if smooth_follow:
                cam.center = (1.0 - follow_alpha) * cam.center + follow_alpha * desired
            else:
                cam.center = desired.copy()

        # -------- collision debug snapshot --------
        debug_events = []
        bodies = world.bodies
        for i in range(len(bodies)):
            for j in range(i + 1, len(bodies)):
                e = collide(bodies[i], bodies[j])
                if e is not None:
                    debug_events.append(e)

        # -------- render --------
        screen.fill((18, 18, 22))

        for b in world.bodies:
            sel = (b is selected_body)
            if isinstance(b.shape, Sphere):
                draw_sphere(screen, cam, b, (W, H), font_name, selected=sel)
            elif isinstance(b.shape, Box):
                draw_box(screen, cam, b, (W, H), font_name, selected=sel)

        # drag guide line (left)
        if dragging_left and drag_body is not None and drag_start is not None:
            mpos = pygame.mouse.get_pos()
            start_scr = cam.world_to_screen(drag_start, (W, H))
            pygame.draw.line(screen, (80, 200, 255), start_scr, mpos, 2)
            pygame.draw.circle(screen, (80, 200, 255), start_scr, 5, 1)

        for e in debug_events:
            draw_contact_debug(screen, cam, e, (W, H), font_small)

        draw_crosshair(screen)

        # -------- HUD --------
        if hud_enabled:
            y = 8
            target_name = "-"
            if dyn_idx:
                target_name = getattr(world.bodies[dyn_idx[follow_index]], "name", "?")

            y = draw_text(screen, font, 10, y, f"Camera: ({cam.center[0]:.3f}, {cam.center[1]:.3f}) zoom={cam.zoom:.1f}")
            y = draw_text(screen, font, 10, y, f"Follow[F]={follow_enabled} target[TAB]={target_name} smooth[S]={smooth_follow}")
            y = draw_text(screen, font, 10, y, f"Sim[SPACE]={'PAUSED' if paused else 'RUN'} Step[N] HUD[H] Reset[R]")
            y = draw_text(screen, font, 10, y, f"Forces: G[G]={gravity_on} D[D]={drag_on}  ImpulseMode[J]={'equal_dv' if impulse_equal_dv else 'equal_J'}")
            y = draw_text(screen, font, 10, y, "Mouse: L-drag=impulse  R-drag=move(body)  (click pauses temporarily)")

            y += 6
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
