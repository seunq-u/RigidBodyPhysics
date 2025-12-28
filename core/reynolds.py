# core/reynolds.py

import numpy as np
import math

# def reynolds_from_nu(v, L, nu, eps: float = 1e-12):
#     """
#     동점성계수로 레이놀즈수 계산 (Re = |v| * L / nu)
#     v : 3차원 속도 벡터 (m/s)
#     L : 폭
#     nu: kinematic viscosity (m^2/s)
#     """
#     speed = math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
#     if nu <= 0:
#         raise ValueError("nu must be > 0")
#     if L < 0:
#         raise ValueError("L must be >= 0")
#     if speed < eps:
#         return 0.0
#     return (speed * L) / nu


def reynolds_from_mu(v, L, density_fluid, mu, eps: float = 1e-12):
    """
    점성 계수로 레이놀즈 수 계산 (Re = density_fluid * |v| * L / mu)
    v  : 3차원 속도 벡터 (m/s)
    L  : 폭 (m)
    density_fluid: 유체 밀도 (kg/m^3)
    mu : 점성 계수 (Pa*s = kg/(m*s))
    """
    speed = math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
    if mu <= 0:
        raise ValueError("mu must be > 0")
    if density_fluid <= 0:
        raise ValueError("density fluid must be > 0")
    if L < 0:
        raise ValueError("L must be >= 0")
    if speed < eps:
        return 0.0
    return (density_fluid * speed * L) / mu
