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
    def state(self) -> numpy.ndarray: """상태 벡터 반환"""; raise NotImplementedError
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
    3D 전용 강체
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
