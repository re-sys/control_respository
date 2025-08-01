#!/usr/bin/env python3
"""
逆运动学核心文件
包含腿部运动学计算
"""

import math
from typing import Optional, Tuple

class LegKinematics:
    def __init__(self, l3=0.2, l4=0.16):
        self.l3 = l3; self.l4 = l4  # 大腿/小腿长度

    def inverse_kinematics(self, l0: float, theta: float) -> Optional[Tuple[float, float]]:
        """ 计算关节角度 (极坐标版本) """
        offset = 0.2 if l0 > 0.35 else 0
        theta_inside = math.acos((self.l4**2 + l0**2 - self.l3**2) / (2*self.l4*l0))-offset
        return theta + theta_inside, theta - theta_inside 