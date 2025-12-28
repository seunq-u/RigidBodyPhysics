from core.world import World
from core.body import RigidBody
from core.shape import Box, Sphere
import config
import numpy as np

w = World()

# 바닥(정적)
floor = RigidBody(
    shape=Box(5.0, 0.5, 1.0),
    density=config.Density.Solid.Material.Iron.Steel,
    is_static=True,
    restitution=0.1,
    friction=0.8,
)
floor.r[:] = np.array([0.0, -1.0, 0.0])

# 공(동적)
ball = RigidBody(
    shape=Sphere(0.1),
    density=1000.0,
    r=[0, 0.5, 999],      # z 이상값 넣어도 0으로 강제
    v=[1.0, -1.0, 3.0],   # vz도 0으로 강제
    omega=[9.0, 2.0, 20.0], # wx,wy 0으로 강제, wz만 유지
    restitution=0.4,
    friction=0.3,
)

w.add_body(floor)
w.add_body(ball)

for _ in range(300):
    w.step(0.01, solver_iterations=12)

print(ball.r, ball.v, ball.omega, ball.theta)
