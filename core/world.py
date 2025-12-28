# core/world.py
from __future__ import annotations

import typing
from core.body import Body
from core.integrator import rk4_step
from core.collision import collide, resolve
from core.background import Background
import config


class World:
    """시뮬레이션 세계 (2.5D)"""
    def __init__(
            self,
            background: Background | None = None,
            C_rot: float = config.C_rot ):
        self.background = background or Background(
            config.Density.Gas.Air,
            config.Absolute_Viscosity.Air,
            config.Kinematic_Viscosity.Air,
        )
        self.C_rot = float(C_rot)
        self.bodies: typing.List[Body] = []

    def add_body(self, body: Body) -> None:
        self.bodies.append(body)

    def step(self, dt: float, solver_iterations: int = 12, enable_gravity: bool = True, enable_drag: bool = True) -> None:
        dt = float(dt)
        
        # 적분( 정적 물체 제외 )
        for body in self.bodies:
            if bool(getattr(body, "is_static", False)):
                continue
            Y_next = rk4_step(t=0.0, h=dt, body=body, background=self.background, C_rot=self.C_rot, enable_gravity=enable_gravity, enable_drag=enable_drag)
            body.set_state(Y_next)
            
        # 충돌 보정
        n = len(self.bodies)
        for _ in range(max(1, int(solver_iterations))):
            for i in range(n):
                for j in range(i + 1, n):
                    e = collide(self.bodies[i], self.bodies[j])
                    if e is not None:
                        resolve(e, iterations=1)