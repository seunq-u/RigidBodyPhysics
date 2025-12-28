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