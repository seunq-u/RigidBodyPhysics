# core/background.py

class Background:
    def __init__(self, Density, Absolute_Viscosity, Kinematic_Viscosity):
        self.density = Density
        self.absolute_viscosity = Absolute_Viscosity
        self.kinematic_viscosity = Kinematic_Viscosity

# core/body.py

from __future__ import annotations

import math
from dataclasses import dataclass, field
from abc import ABC, abstractmethod
from typing import Callable, Optional
import numpy

from core.shape import Shape, Sphere, Box
from core.drag_coefficient import CD

Vec3 = numpy.ndarray # (3,)

class Body(ABC):
    """상태 벡터를 가진 물체 추상 클래스"""
    @abstractmethod
    def state(self) -> numpy.ndarray: """상태 벡터 반환."""; raise NotImplementedError
    @abstractmethod
    def set_state(self, y: numpy.ndarray) -> None: """상태 벡터 설정"""; raise NotImplementedError
    @abstractmethod
    def drag_coefficient(self, Re: float) -> None: """항력 계수 반환"""; raise NotImplementedError
    @abstractmethod
    def proj_area(self) -> None: """속도 벡터 방향의 투영 단면적은 회전시 계속 바뀌므로 그때그때 계산"""; raise NotImplementedError

CdFn = Callable[[float], float]  # (Re) -> Cd

@dataclass
class RigidBody(Body):
    """
    2.5D 전용 강체
        - 3차원 벡터를 기본적으로 포함함
        - 회전각과 각속도는 z축에서만 정의됨, 다른 축은 0.0
        !! 따라서 z축 위치와 속도 및 각속도(x축, y축)는 0.0으로 정의해야함(다른 값 넣으면 _enforce_2p5d_constraints를 통해 강제로 조정됨)
        - 부피, 질량 등은 자동으로 계산됨. 만약 직접 질량을 조정하고 싶으면 CustomRigidBody를 사용하면 됨
    상태벡터는 다음과 같이 구성됨 [x,y,z, vx,vy,vz, ωx,ωy,ωz, theta]
    각각 위치벡터, 병진속도벡터, 각속도벡터, 회전각임
    """
    shape: Shape
    density: float
    
    # 물질 상태
    is_static: bool = False
    restitution: float = 0.2
    friction: float = 0.5
    
    # 상태
    r: Vec3 = field(default_factory=lambda: numpy.zeros(3, dtype=float)) # 3차원 위치 벡터
    v: Vec3 = field(default_factory=lambda: numpy.zeros(3, dtype=float)) # 3차원 속도 벡터
    omega: Vec3 = field(default_factory=lambda: numpy.zeros(3, dtype=float)) # 각속도 [rad/s]
    theta: float = 0.0 # 회전각 [rad]
    
    # 파생 속성
    volume: float = field(init=False) # 부피
    mass: float = field(init=False) # 질량
    Izz: float = field(init=False) # z축 관성 모멘트
    A0: float = field(init=False) # 기준 면적
    L: float = field(init=False) # 길이
    Rspin: float = field(init=False) # 스핀파라미터용 길이
    _cd_fn: Optional[CdFn] = field(init=False, default=None)
    
    def __post_init__(self) -> None:
        
        self.r = numpy.asarray(self.r, dtype=float).reshape(3)
        self.v = numpy.asarray(self.v, dtype=float).reshape(3)
        self.omega = numpy.asarray(self.omega, dtype=float).reshape(3)
        
        self.volume = float(self.shape.volume())
        self.A0 = float(self.shape.ref_area())
        self.L = float(self.shape.char_length())
        self.Rspin = float(self.shape.spin_radius())
        
        if self.is_static:
            self.mass = float("inf")
            self.Izz = float("inf")
        else:
            self.mass = float(self.density) * self.volume
            self.Izz = float(self.shape.Izz_from_mass(self.mass))
        
        self._enforce_2p5d_constraints()
        self._bind_drag_coefficient()
        
    def _enforce_2p5d_constraints(self) -> None:
        # z축 이동 제거
        self.r[2] = 0.0
        self.v[2] = 0.0
        # x,y 축 회전 제거
        self.omega[0] = 0.0
        self.omega[1] = 0.0
    
    def _bind_drag_coefficient(self) -> None:
        if isinstance(self.shape, Sphere):
            self._cd_fn = lambda Re: CD.sphere(Re)
            return
        if isinstance(self.shape, Box):
            w, h, d = float(self.shape.w), float(self.shape.h), float(self.shape.d)
            if math.isclose(w, h, rel_tol=0.0, abs_tol=1e-12) and math.isclose(h, d, rel_tol=0.0, abs_tol=1e-12):
                self._cd_fn = lambda Re: CD.cube(Re)
            else:
                self._cd_fn = lambda Re: CD.box(Re)
            return
        raise TypeError(f"Unsupported shape type: {type(self.shape)}")
    
    def drag_coefficient(self, Re: float) -> float:
        if self._cd_fn is None:
            raise RuntimeError("Cd function is not bound.")
        return float(self._cd_fn(float(Re)))
    
    def proj_area(self) -> float:
        return float(self.shape.proj_area(self.theta, self.v))
    
    def state(self) -> numpy.ndarray:
        return numpy.array([
            self.r[0], self.r[1], self.r[2],
            self.v[0], self.v[1], self.v[2],
            self.omega[0], self.omega[1], self.omega[2],
            float(self.theta),
        ], dtype=float)
        
    def set_state(self, y: numpy.ndarray) -> None:
        y = numpy.asarray(y, dtype=float).reshape(-1)
        self.r[:] = y[0:3]
        self.v[:] = y[3:6]
        self.omega[:] = y[6:9]
        self.theta = float(y[9])
        self._enforce_2p5d_constraints()

@dataclass
class CustomRigidBody(RigidBody):
    mass_override: Optional[float] = None
    Izz_override: Optional[float] = None

    def __post_init__(self) -> None:
        self.r = numpy.asarray(self.r, dtype=float).reshape(3)
        self.v = numpy.asarray(self.v, dtype=float).reshape(3)
        self.omega = numpy.asarray(self.omega, dtype=float).reshape(3)

        self.volume = float(self.shape.volume())
        self.A0 = float(self.shape.ref_area())
        self.L = float(self.shape.char_length())
        self.Rspin = float(self.shape.spin_radius())

        if self.is_static:
            self.mass = float("inf")
            self.Izz = float("inf")
        else:
            self.mass = float(self.mass_override) if self.mass_override is not None else float(self.density) * self.volume
            if self.Izz_override is not None:
                self.Izz = float(self.Izz_override)
            else:
                self.Izz = float(self.shape.Izz_from_mass(self.mass))

        self._enforce_2p5d_constraints()
        self._bind_drag_coefficient()

# core/collision.py
# 2.5D 충돌 감지 및 보정
from __future__ import annotations

import math
import numpy as np
from core.shape import Sphere, Box
from core.event import Event

EPS = 1e-12

def _v2(x) -> np.ndarray:
    v = np.asarray(x, dtype=float)
    return v.reshape(2)

def _norm(v: np.ndarray) -> float:
    return float(np.hypot(v[0], v[1]))

def _unit(v: np.ndarray) -> np.ndarray:
    n = _norm(v)
    if n < EPS:
        return np.array([1.0, 0.0], dtype=float)
    return v / n

def _cross2(a: np.ndarray, b: np.ndarray) -> float:
    return float(a[0]*b[1] - a[1]*b[0])

def _rot_axes(theta: float):
    c, s = math.cos(theta), math.sin(theta)
    ux = np.array([c, s], dtype=float)
    uy = np.array([-s, c], dtype=float)
    return ux, uy

def _half_extents(box: Box):
    return 0.5*float(box.w), 0.5*float(box.h)

def _proj_radius(axis: np.ndarray, ux: np.ndarray, uy: np.ndarray, hx: float, hy: float) -> float:
    return hx*abs(float(np.dot(axis, ux))) + hy*abs(float(np.dot(axis, uy)))

def collide(a, b) -> Event | None:
    sa, sb = a.shape, b.shape
    if isinstance(sa, Sphere) and isinstance(sb, Sphere):
        return _ss(a, b)
    if isinstance(sa, Sphere) and isinstance(sb, Box):
        return _so(a, b)          # normal: A(sphere)->B(box)
    if isinstance(sa, Box) and isinstance(sb, Sphere):
        e = _so(b, a)             # normal: B(sphere)->A(box)
        if e is None: return None
        e.body_a, e.body_b = a, b
        e.normal = (-e.normal[0], -e.normal[1])  # A(box)->B(sphere)
        return e
    if isinstance(sa, Box) and isinstance(sb, Box):
        return _oo(a, b)
    return None

def _ss(a, b) -> Event | None:
    ca = _v2(a.r[:2]); cb = _v2(b.r[:2])
    ra = float(a.shape.R); rb = float(b.shape.R)

    d = cb - ca
    dist = _norm(d)
    pen = (ra + rb) - dist
    if pen <= 0.0:
        return None

    n = _unit(d)  # A->B
    contact = ca + n*(ra - 0.5*pen)

    e = Event(a, b)
    e.normal = (float(n[0]), float(n[1]))
    e.penetration = float(pen)
    e.contacts = [(float(contact[0]), float(contact[1]))]
    return e

def _so(sphere_body, box_body) -> Event | None:
    """Sphere vs OBB (2D), normal points sphere->box (A->B)."""
    c = _v2(sphere_body.r[:2])
    r = float(sphere_body.shape.R)

    bc = _v2(box_body.r[:2])
    hx, hy = _half_extents(box_body.shape)
    ux, uy = _rot_axes(float(box_body.theta))

    rel = c - bc
    lx = float(np.dot(rel, ux))
    ly = float(np.dot(rel, uy))

    inside = (-hx <= lx <= hx) and (-hy <= ly <= hy)

    cx = max(-hx, min(hx, lx))
    cy = max(-hy, min(hy, ly))
    closest = bc + cx*ux + cy*uy

    if not inside:
        d = closest - c                 # sphere->box
        dist = _norm(d)
        pen = r - dist
        if pen <= 0.0:
            return None
        n = _unit(d)
        contact = closest
    else:
        dx = hx - abs(lx)
        dy = hy - abs(ly)
        if dx < dy:
            sgn = 1.0 if lx >= 0.0 else -1.0
            n = -sgn * ux              # sphere->box
            contact = bc + (sgn*hx)*ux + ly*uy
            pen = r + dx
        else:
            sgn = 1.0 if ly >= 0.0 else -1.0
            n = -sgn * uy              # sphere->box
            contact = bc + lx*ux + (sgn*hy)*uy
            pen = r + dy

    e = Event(sphere_body, box_body)
    e.normal = (float(n[0]), float(n[1]))
    e.penetration = float(pen)
    e.contacts = [(float(contact[0]), float(contact[1]))]
    return e

def _oo(a, b) -> Event | None:
    ca = _v2(a.r[:2]); cb = _v2(b.r[:2])
    hxA, hyA = _half_extents(a.shape)
    hxB, hyB = _half_extents(b.shape)
    uxA, uyA = _rot_axes(float(a.theta))
    uxB, uyB = _rot_axes(float(b.theta))

    d = cb - ca
    axes = (uxA, uyA, uxB, uyB)

    min_overlap = float("inf")
    best_axis = None
    best_from_A = True
    best_is_x = True

    for idx, axis0 in enumerate(axes):
        axis = _unit(axis0)
        ra = _proj_radius(axis, uxA, uyA, hxA, hyA)
        rb = _proj_radius(axis, uxB, uyB, hxB, hyB)
        dist = abs(float(np.dot(d, axis)))
        overlap = (ra + rb) - dist
        if overlap <= 0.0:
            return None
        if overlap < min_overlap:
            min_overlap = overlap
            n = axis if float(np.dot(d, axis)) >= 0.0 else -axis  # A->B
            best_axis = n
            best_from_A = (idx < 2)
            best_is_x = (idx % 2 == 0)

    n = best_axis if best_axis is not None else np.array([1.0, 0.0], dtype=float)

    # manifold (max 2 contacts) via clipping
    if best_from_A:
        ref_c, inc_c = ca, cb
        ref_ux, ref_uy = uxA, uyA
        inc_ux, inc_uy = uxB, uyB
        ref_hx, ref_hy = hxA, hyA
        inc_hx, inc_hy = hxB, hyB
        normal = n                  # ref->inc
    else:
        ref_c, inc_c = cb, ca
        ref_ux, ref_uy = uxB, uyB
        inc_ux, inc_uy = uxA, uyA
        ref_hx, ref_hy = hxB, hyB
        inc_hx, inc_hy = hxA, hyA
        normal = -n                 # ref->inc

    if best_is_x:
        ref_face_n = ref_ux if float(np.dot(normal, ref_ux)) >= 0.0 else -ref_ux
        side = ref_uy
        extent_n, extent_t = ref_hx, ref_hy
    else:
        ref_face_n = ref_uy if float(np.dot(normal, ref_uy)) >= 0.0 else -ref_uy
        side = ref_ux
        extent_n, extent_t = ref_hy, ref_hx

    ref_face_center = ref_c + ref_face_n * extent_n
    v1 = ref_face_center - side * extent_t
    v2 = ref_face_center + side * extent_t

    cand = [inc_ux, -inc_ux, inc_uy, -inc_uy]
    dots = [float(np.dot(ref_face_n, c)) for c in cand]
    inc_face_n = cand[int(np.argmin(dots))]

    if abs(float(np.dot(inc_face_n, inc_ux))) > abs(float(np.dot(inc_face_n, inc_uy))):
        inc_side = inc_uy
        inc_extent_n, inc_extent_t = inc_hx, inc_hy
        inc_n_dir = inc_ux if float(np.dot(inc_face_n, inc_ux)) >= 0.0 else -inc_ux
    else:
        inc_side = inc_ux
        inc_extent_n, inc_extent_t = inc_hy, inc_hx
        inc_n_dir = inc_uy if float(np.dot(inc_face_n, inc_uy)) >= 0.0 else -inc_uy

    inc_face_center = inc_c + inc_n_dir * inc_extent_n
    i1 = inc_face_center - inc_side * inc_extent_t
    i2 = inc_face_center + inc_side * inc_extent_t

    def clip(p1, p2, n_clip, c_clip):
        out = []
        d1 = float(np.dot(n_clip, p1)) - c_clip
        d2 = float(np.dot(n_clip, p2)) - c_clip
        if d1 <= 0.0: out.append(p1)
        if d2 <= 0.0: out.append(p2)
        if d1*d2 < 0.0:
            t = d1 / (d1 - d2)
            out.append(p1 + t*(p2 - p1))
        if len(out) == 0:
            return None
        if len(out) == 1:
            return (out[0], out[0])
        return (out[0], out[1])

    seg = clip(i1, i2,  side, float(np.dot(side, v2)))
    if seg is None: return None
    seg = clip(seg[0], seg[1], -side, float(np.dot(-side, v1)))
    if seg is None: return None
    c1, c2 = seg

    contacts = []
    for p in (c1, c2):
        sep = float(np.dot(ref_face_n, p - ref_face_center))
        if sep <= 1e-9:
            contacts.append((float(p[0]), float(p[1])))
    if not contacts:
        mid = 0.5*(c1+c2)
        contacts = [(float(mid[0]), float(mid[1]))]

    e = Event(a, b)
    e.normal = (float(n[0]), float(n[1]))
    e.penetration = float(min_overlap)
    e.contacts = contacts
    return e

def resolve(event: Event, iterations: int = 1, slop: float = 1e-3, percent: float = 0.6) -> None:
    a = event.body_a
    b = event.body_b

    e = min(float(getattr(a, "restitution", 0.2)), float(getattr(b, "restitution", 0.2)))
    mu = math.sqrt(max(float(getattr(a, "friction", 0.5)), 0.0) * max(float(getattr(b, "friction", 0.5)), 0.0))

    n = _unit(_v2(event.normal))
    contacts = [_v2(c) for c in event.contacts] or [0.5*(_v2(a.r[:2]) + _v2(b.r[:2]))]
    m = len(contacts)

    ma = float(getattr(a, "mass", 0.0)); mb = float(getattr(b, "mass", 0.0))
    Ia = float(getattr(a, "Izz", 0.0));  Ib = float(getattr(b, "Izz", 0.0))
    static_a = bool(getattr(a, "is_static", False))
    static_b = bool(getattr(b, "is_static", False))

    inv_ma = 0.0 if static_a or ma <= 0.0 or math.isinf(ma) else 1.0/ma
    inv_mb = 0.0 if static_b or mb <= 0.0 or math.isinf(mb) else 1.0/mb
    inv_Ia = 0.0 if static_a or Ia <= 0.0 or math.isinf(Ia) else 1.0/Ia
    inv_Ib = 0.0 if static_b or Ib <= 0.0 or math.isinf(Ib) else 1.0/Ib

    def w_cross_r(w, r):
        return np.array([-w*r[1], w*r[0]], dtype=float)

    for _ in range(max(1, int(iterations))):
        for c in contacts:
            ra = c - _v2(a.r[:2])
            rb = c - _v2(b.r[:2])

            va = _v2(a.v[:2]); vb = _v2(b.v[:2])
            wa = float(a.omega[2]); wb = float(b.omega[2])

            rv = (vb + w_cross_r(wb, rb)) - (va + w_cross_r(wa, ra))  # vB - vA
            vel_n = float(np.dot(rv, n))
            if vel_n > 0.0:
                continue

            ra_cn = _cross2(ra, n)
            rb_cn = _cross2(rb, n)
            denom = inv_ma + inv_mb + (ra_cn*ra_cn)*inv_Ia + (rb_cn*rb_cn)*inv_Ib
            if denom < EPS:
                continue

            j = -(1.0 + e) * vel_n / denom
            j /= m
            impulse = j * n

            va -= inv_ma * impulse
            vb += inv_mb * impulse
            wa -= inv_Ia * _cross2(ra, impulse)
            wb += inv_Ib * _cross2(rb, impulse)

            rv2 = (vb + w_cross_r(wb, rb)) - (va + w_cross_r(wa, ra))
            t = rv2 - float(np.dot(rv2, n)) * n
            tmag = _norm(t)
            if tmag > EPS:
                t = t / tmag
                ra_ct = _cross2(ra, t)
                rb_ct = _cross2(rb, t)
                denom_t = inv_ma + inv_mb + (ra_ct*ra_ct)*inv_Ia + (rb_ct*rb_ct)*inv_Ib
                if denom_t > EPS:
                    jt = -float(np.dot(rv2, t)) / denom_t
                    jt = max(-j*mu, min(j*mu, jt))
                    imp_t = jt * t

                    va -= inv_ma * imp_t
                    vb += inv_mb * imp_t
                    wa -= inv_Ia * _cross2(ra, imp_t)
                    wb += inv_Ib * _cross2(rb, imp_t)

            a.v[0], a.v[1] = float(va[0]), float(va[1])
            b.v[0], b.v[1] = float(vb[0]), float(vb[1])
            a.omega[2] = float(wa)
            b.omega[2] = float(wb)

    _positional_correction(a, b, n, float(event.penetration), inv_ma, inv_mb, slop, percent)
    _enforce_2p5d(a); _enforce_2p5d(b)

def _positional_correction(a, b, n, penetration, inv_ma, inv_mb, slop, percent):
    if penetration <= 0.0:
        return
    denom = inv_ma + inv_mb
    if denom < EPS:
        return
    corr_mag = max(penetration - slop, 0.0) / denom * percent
    corr = corr_mag * n
    a.r[0] -= inv_ma * float(corr[0]); a.r[1] -= inv_ma * float(corr[1])
    b.r[0] += inv_mb * float(corr[0]); b.r[1] += inv_mb * float(corr[1])

def _enforce_2p5d(body):
    body.r[2] = 0.0
    body.v[2] = 0.0
    body.omega[0] = 0.0
    body.omega[1] = 0.0

# core/drag_coefficient.py
# 레이놀즈 수에 따른 항력계수를 반환하는 함수 모음
# 공기/물에 대해 구, 직육면체, 정육면체의 항력계수 반환

import math

class CD:
    """레이놀즈 수에 따른 항력계수를 반환하는 함수 모음"""
    @staticmethod
    def sphere(Re: float) -> float:
        """
        레이놀즈 수에 따른 구의 항력 계수
        Args:
            Re (float): 레이놀즈 수
        Returns:
            float: 항력 계수 (C_D)
        """
        Re = float(Re)
        if Re <= 0.0:
            return float("inf")
        if Re < 1.0:
            return 24.0 / Re
        if Re < 2.0e5:
            return 0.47
        if Re < 4.0e5:
            return 0.47 + (0.10 - 0.47) * ((Re - 2.0e5) / (2.0e5))
        return 0.1

    @staticmethod
    def cube(Re: float) -> float:
        """정육면체 항력 계수 반환
        Args:
            Re (float): 레이놀즈 수
        Returns:
            float: 항력 계수 (C_D)
        """
        Re = float(Re)
        if Re <= 0.0:
            return float("inf")
        if Re < 1.0:
            return min(24.0 / Re, 10.0)
        return 1.05

    @staticmethod
    def box(Re: float) -> float:
        """
        직육면체 항력 계수 반환
        Args:
            Re (float): 레이놀즈 수
        Returns:
            float: 항력 계수 (C_D)
        """
        Re = float(Re)
        if Re <= 0.0:
            return float("inf")
        if Re < 1.0:
            return min(24.0 / Re, 10.0)
        return 1.20


# core/event.py
from core.body import Body

class Event:
    """충돌 정보 저장"""
    def __init__(self, body_a: Body, body_b: Body):
        self.body_a = body_a
        self.body_b = body_b
        self.contacts = []  # 접촉점 리스트
        self.normal = (0.0, 0.0)  # 충돌 법선
        self.penetration = 0.0  # 침투 깊이

# core/force.py
# 각종 힘을 계산하는 함수 모음 (중력Fg, 부력Fb, 항력Fd, 양력Fl(마그누스힘) 및 회전 감쇠 토크 torque_damper)

import numpy
import math

import config
from core.shape import *
from core.body import *

class Force:
    """병진 운동에 관한 힘 계산 함수 모음"""
    @staticmethod
    def gravity(body: Body) -> numpy.ndarray:
        """중력 계산

        Args:
            body (Body): 물체 (RigidBody, CustomRigidBody)

        Returns:
            numpy.ndarray: 물체에 작용하는 y축(수직)방향 중력(무게)
        """
        return numpy.array([0, -body.mass * config.g, 0], dtype=float)

    @staticmethod
    def buoyancy(density_fluid: float, body: Body) -> numpy.ndarray:
        """부력 계산(완전 잠긴 상태 기준임)

        Args:
            density_fluid (float): 유체 밀도
            body (Body): 물체 (RigidBody, CustomRigidBody)

        Returns:
            numpy.ndarray: 부력
        """
        return numpy.array([0, density_fluid * body.volume * config.g, 0], dtype=float)

    @staticmethod
    def drag(density_fluid: float, body: Body, Re: float) -> numpy.ndarray:
        """항력 계산

        Args:
            density_fluid (float): 유체 밀도
            body (Body): 물체 (RigidBody, CustomRigidBody)
            Re (float): 레이놀즈 수

        Returns:
            numpy.ndarray: 항력
        """
        return -0.5 * density_fluid * body.proj_area() * body.drag_coefficient(Re) * numpy.linalg.norm(body.v) * body.v

    @staticmethod
    def lift(density_fluid: float, 
                body: Body,
                # R: float,
                # velocity: numpy.ndarray,
                # omega: numpy.ndarray,
                # h: float = -1,
                # d: float = -1,
                k: float = 1.2,
                CL_max: float = 0.5,
                eps: float = 1e-8,
            ) -> numpy.ndarray:
        """표준형 양력(마그누스 힘) F = 0.5*밀도*단면적*CL*v^2 * n_perp 2.5D(평면 운동 + z축 회전) 가정 omega는 스칼라(omega_z)만 사용. shape = Box일 때, h와 d를 입력하지 않으면 양력 벡터 [0,0,0]을 반환함

        Args:
            density_fluid (float): 유체 밀도
            body (Body): 물체 (RigidBody, CustomRigidBody)
            # R (float): 대표길이(구는 반지름, 육면체는 min(w+h)/2 로 입력)
            # velocity (numpy.ndarray): 속도 벡터([vx, vy, vz])
            # omega (numpy.ndarray): 각속도 벡터([ωx, ωy, ωz])
            # h(float, optional): (육면체인 경우) y축 방향 길이 Defaults to -1
            # d(float, optional): (육면체인 경우) z축 방향 길이 Defaults to -1
            k (float, optional): 양력계수 계산용 상수. Defaults to 1.2.
            CL_max (float, optional): 양력계수. Defaults to 0.5.
            eps (float, optional): _description_. Defaults to 1e-8.

        Returns:
            numpy.ndarray: 양력
        """
        state = body.state()
        velocity = state[3:6]
        vx, vy = state[3], state[4]
        v2 = vx*vx + vy*vy
        if v2 < eps*eps:
                return numpy.zeros(3, dtype=float)

        v = math.sqrt(v2)
        omega_z = state[8]

        # 스핀 파라미터 S
        S = (omega_z * body.Rspin) / (v + eps)

        # 양력 계수 CL (선형 + 포화)
        CL = k * S
        if CL > CL_max:
            CL = CL_max
        elif CL < -CL_max:
            CL = -CL_max

        # 기준면적 A0
        A0 = body.A0

        # 힘 크기
        Fmag = 0.5 * density_fluid * A0 * CL * v2

        # 수직 단위벡터 (vx,vy)에 수직
        n_perp_x = -vy / v
        n_perp_y =  vx / v

        return numpy.array([Fmag * n_perp_x, Fmag * n_perp_y, 0.0], dtype=float)


class Torque:
    """회전 운동에 관한 토크(돌림 힘) 계산 함수 모음"""
    @staticmethod
    def Damping(C_rot: float, body: Body) -> numpy.ndarray:
        """회전 감쇠 토크 계산 (고속에서 비선형 기준임)
            회전 감쇠율 x |각속도| x 각속도

        Args:
            C_rot (float): 감쇠율
            body (Body): 물체 (RigidBody, CustomRigidBody)

        Returns:
            numpy.ndarray: 토크
        """
        return  -C_rot * numpy.linalg.norm(body.omega) * body.omega


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

def f(Y: numpy.ndarray, t: float, body: Body, background: Background, C_rot: float ) -> numpy.ndarray:
    Y = numpy.asarray(Y, dtype=float)
    Y_save = body.state().copy()
    body.set_state(Y)
    
    density_fluid = background.density
    mu = background.absolute_viscosity
    
    dr_dt = body.v.copy()
    
    Re = reynolds_from_mu(dr_dt, body.L, density_fluid, mu)
    
    Fg = Force.gravity(body)
    Fb = Force.buoyancy(density_fluid, body)
    Fl = Force.lift(density_fluid, body)
    Fd = Force.drag(density_fluid, body, Re)
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


def rk4_step(t, h, body: Body, background: Background, C_rot: float = C_rot) -> numpy.ndarray:
    """단일 RK4 스텝을 수행하는 함수
        Y : 현재 상태 벡터 (Y_i)
        t : 현재 시간 (t_i)
        h : 시간 간격 (dt)
    """
    Y = body.state()
    k1 = h * f(Y, t, body, background, C_rot) # 시간 간격 x 속도 = 거리, 시간 간격 * 가속도 = 속도
    k2 = h * f(Y + k1/2, t + h/2, body, background, C_rot)
    k3 = h * f(Y + k2/2, t + h/2, body, background, C_rot)
    k4 = h * f(Y + k3, t + h, body, background, C_rot)
    Y_next = Y + (k1 + 2*k2 + 2*k3 + k4) / 6 # 거리와 속도 변화 평균
    return Y_next


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



# core/share.py

from __future__ import annotations

import math
from dataclasses import dataclass
from abc import ABC, abstractmethod
import numpy

Vec3 = numpy.ndarray  # convention: shape (3,), dtype float


class Shape(ABC):
    """물체의 기하학적 형태 추상 클래스"""
    @abstractmethod
    def volume(self) -> float: """부피 [m^3]"""; raise NotImplementedError
    @abstractmethod
    def ref_area(self) -> float: """항력계수/항력/양력 계산에 쓰이는 기준 면적(A0) [m^2]"""; raise NotImplementedError
    @abstractmethod
    def char_length(self) -> float: """레이놀즈 수 계산에 쓰이는 물체의 길이 [m]"""; raise NotImplementedError
    @abstractmethod
    def spin_radius(self) -> float: """스핀파라미터 계산에 쓰이는 물체의 길이 [m]"""; raise NotImplementedError
    @abstractmethod
    def Izz_from_mass(self, mass: float) -> float: """z축 관성 모멘트 [kg*m^2]""" ; raise NotImplementedError
    @abstractmethod
    def proj_area(self, yaw_rad: float, velocity: numpy.ndarray) -> float: """투영 단면적 [m^2]""" ; raise NotImplementedError

@dataclass(frozen=True)
class Sphere(Shape):
    R: float  # radius [m]
    def volume(self) -> float: return (4.0 / 3.0) * math.pi * self.R**3
    def ref_area(self) -> float: return math.pi * self.R**2
    def char_length(self) -> float: return 2.0 * self.R
    def spin_radius(self) -> float: return self.R
    def Izz_from_mass(self, mass: float) -> float: return (2.0 / 5.0) * mass * self.R**2
    def proj_area(self, yaw_rad: float, velocity: numpy.ndarray) -> float: return math.pi * self.R**2

@dataclass(frozen=True)
class Box(Shape):
    w: float
    h: float
    d: float
    def volume(self) -> float: return self.w * self.h * self.d
    def ref_area(self) -> float: return self.h * self.d
    def char_length(self) -> float: return max(self.w, self.h)
    def spin_radius(self) -> float: return 0.5 * min(self.w, self.h)
    def Izz_from_mass(self, mass: float) -> float: return (1.0 / 12.0) * mass * (self.w**2 + self.h**2)
    
    def proj_area(self, yaw_rad: float, velocity: numpy.ndarray) -> float:
        vx, vy = float(velocity[0]), float(velocity[1])
        if vx == 0.0 and vy == 0.0:
            return self.h * self.d
        phi = math.atan2(vy, vx)
        beta = phi - float(yaw_rad)
        b = abs(beta) % math.pi
        if b > math.pi / 2.0:
            b = math.pi - b
        return self.d * (abs(self.w * math.sin(b)) + abs(self.h * math.cos(b)))



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

    def step(self, dt: float, solver_iterations: int = 8) -> None:
        dt = float(dt)
        
        # 적분( 정적 물체 제외 )
        for body in self.bodies:
            if bool(getattr(body, "is_static", False)):
                continue
            Y_next = rk4_step(t=0.0, h=dt, body=body, background=self.background, C_rot=self.C_rot)
            body.set_state(Y_next)
            
        # 충돌 보정
        n = len(self.bodies)
        for _ in range(max(1, int(solver_iterations))):
            for i in range(n):
                for j in range(i + 1, n):
                    e = collide(self.bodies[i], self.bodies[j])
                    if e is not None:
                        resolve(e, iterations=1)