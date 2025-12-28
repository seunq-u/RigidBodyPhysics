# core/integrator.py

# 수치 해석의 룽게-쿠타 4차(RK4) 방법을 기반으로 만든 적분기 모듈 모음
# 시간 t에 대한 선형 함수 Y를 4번 적분하여 적당히 가중치를 부과해 평균을 내는 냄

# core/integrator.py
import numpy
from core.force import Force, Torque
from core.reynolds import reynolds_from_mu  # 또는 from_nu
from core.body import Body
from config import C_rot
from core.background import Background

def f(Y: numpy.ndarray, t: float, body: Body, background: Background, C_rot: float, enable_gravity: bool = True, enable_drag: bool = True) -> numpy.ndarray:
    Y = numpy.asarray(Y, dtype=float)
    Y_save = body.state().copy()
    body.set_state(Y)
    
    density_fluid = background.density
    mu = background.absolute_viscosity
    
    dr_dt = body.v.copy()
    
    Re = reynolds_from_mu(dr_dt, body.L, density_fluid, mu)
    
    Fg = Force.gravity(body) if enable_gravity else numpy.zeros(3, dtype=float)
    Fb = Force.buoyancy(background.density, body) if enable_gravity else numpy.zeros(3, dtype=float)  # 중력 off면 부력도 off
    Fl = Force.lift(background.density, body)
    Re = reynolds_from_mu(body.v, body.L, background.density, background.absolute_viscosity)
    Fd = Force.drag(background.density, body, Re) if enable_drag else numpy.zeros(3, dtype=float)
    F = Fg + Fb + Fl + Fd
    
    dv_dt = F / body.mass
    
    tau_D = Torque.Damping(C_rot, body)
    domega_dt = numpy.zeros(3, dtype=float)
    if body.Izz > 0.0: domega_dt[2] = tau_D[2] / body.Izz
    dtheta_dt = float(body.omega[2])
    
    # 2.5D 제약
    dr_dt[2] = 0.0
    dv_dt[2] = 0.0
    domega_dt[0] = 0.0
    domega_dt[1] = 0.0
    
    dY = numpy.array(
        [dr_dt[0], dr_dt[1], dr_dt[2],
        dv_dt[0], dv_dt[1], dv_dt[2],
        domega_dt[0], domega_dt[1], domega_dt[2],
        dtheta_dt],
        dtype=float
    )
    body.set_state(Y_save)
    return dY


def rk4_step(t, h, body: Body, background: Background, C_rot: float = C_rot, enable_gravity: bool = True, enable_drag: bool = True) -> numpy.ndarray:
    """단일 RK4 스텝을 수행하는 함수
        Y : 현재 상태 벡터 (Y_i)
        t : 현재 시간 (t_i)
        h : 시간 간격 (dt)
    """
    Y = body.state()
    k1 = h * f(Y, t, body, background, C_rot, enable_gravity, enable_drag) # 시간 간격 x 속도 = 거리, 시간 간격 * 가속도 = 속도
    k2 = h * f(Y + k1/2, t + h/2, body, background, C_rot, enable_gravity, enable_drag)
    k3 = h * f(Y + k2/2, t + h/2, body, background, C_rot, enable_gravity, enable_drag)
    k4 = h * f(Y + k3, t + h, body, background, C_rot, enable_gravity, enable_drag)
    Y_next = Y + (k1 + 2*k2 + 2*k3 + k4) / 6 # 거리와 속도 변화 평균
    return Y_next

