# core/drag_coefficient.py
# 레이놀즈 수에 따른 항력계수를 반환하는 함수 모음
# 공기/물에 대해 구, 직육면체, 정육면체의 항력계수 반환

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