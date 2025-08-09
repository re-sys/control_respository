#!/usr/bin/env python3
"""
常量定义文件
包含关节名称、腿枚举、参数类等
"""

import math
from enum import Enum

# ------------------- 硬件常量定义 -------------------
JOINT_NAMES = [
    "FL_thigh_joint_i", "FL_thigh_joint_o",  # 0,1 (左前)
    "FR_thigh_joint_i", "FR_thigh_joint_o",  # 2,3 (右前) 
    "waist_joint",                           # 4 (不驱动)
    "RL_thigh_joint_i", "RL_thigh_joint_o",  # 5,6 (左后)
    "RR_thigh_joint_i", "RR_thigh_joint_o"   # 7,8 (右后)
]

class Leg:
    FL = 0; FR = 1; RL = 2; RR = 3  # 四条腿枚举

class Params:
    def __init__(self):
        self.leg_length = 0.20
        self.pitch_angle = math.radians(-90)

        self.swing_height = 0.01
        self.running_stride = 0.1
        self.max_turn_stride = 0.1
        self.smallest_period = 0.4

class StateType(Enum):
    IDLE = "idle"           # 静止状态
    WALK = "walk"           # 对角步态
    FLIP = "flip"           # 空翻状态
    JUMP = "jump"           # 前跳状态
    STAIR_JUMP = "stair_jump"  # 台阶跳跃状态
    ERROR = "error"         # 错误状态
    RECOVERY = "recovery"   # 恢复状态 
    NAVIGATION = "navigation"