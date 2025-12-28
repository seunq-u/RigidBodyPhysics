from core.body import RigidBody
from core.shape import Box
from core.reynolds import reynolds_from_mu
from core.drag_coefficient import CD
from core.force import Force, Torque
import config
k = RigidBody(shape=Box(0.02,0.02,0.02), density=config.Density.Solid.Material.Iron.Steel, r=[2,2,2], v=[3,3,3], omega=[1,1,1], theta=0)
print(k)
Re = reynolds_from_mu(k.state()[3:6], k.L, config.Density.Gas.Air, config.Absolute_Viscosity.air)
print(f'레이놀즈 수: {Re}')
C_D = CD.cube(Re)
print(f'항력 계수: {C_D}')

g = Force.gravity(k)
b = Force.buoyancy(config.Density.Gas.Air, k)
l = Force.lift(config.Density.Gas.Air, k)
d = Force.drag(config.Density.Gas.Air, k, Re)
t = Torque.Damping(config.C_rot, k)
print(f"중력: {g}\n부력: {b}\n양력: {l}\n항력: {d}\n회전감쇠토크: {t}")