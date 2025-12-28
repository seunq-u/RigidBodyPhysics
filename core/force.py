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
        """표준형 양력(마그누스 힘) F = 0.5*밀도*단면적*CL*v^2 * n_perp 2.5D(평면 운동 + z축 회전) 가정 omega는 스칼라(omega_z)만 사용.

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