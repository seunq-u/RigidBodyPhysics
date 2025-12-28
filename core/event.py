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
