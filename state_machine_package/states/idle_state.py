#!/usr/bin/env python3
"""
静止状态文件
"""

import numpy as np
from typing import Dict, Tuple
from base_state import State
from constants import StateType, Leg

class IdleState(State):
    def __init__(self, controller: 'StateMachineController'):
        super().__init__(controller)
        self.smooth_factor = 1.0  # 平滑因子，控制平滑速度
        self.last_target_cmd = None  # 记录上一次的目标指令
        
    def enter(self):
        self.controller.get_logger().info("进入静止状态")
        # 初始化目标指令为当前关节位置
        self.last_target_cmd = self.controller.joint_position.copy()
        
    def update(self):
        # 计算目标腿位置
        angle = self.controller.pitch_angle 
        result = {
            Leg.FL: (self.controller.leg_length, angle),
            Leg.FR: (self.controller.leg_length, angle),
            Leg.RL: (self.controller.leg_length, angle),
            Leg.RR: (self.controller.leg_length, angle)
        }
        
        # 获取目标关节指令
        target_cmd = self.controller.compute_joint_cmd_from_leg_targets(result)
        
        # 如果这是第一次调用，直接返回目标指令
        if self.last_target_cmd is None:
            self.last_target_cmd = target_cmd.copy()
            return target_cmd
        
        # 实时平滑：基于当前关节位置和目标位置进行平滑过渡
        current_position = self.controller.joint_position
        smoothed_cmd = current_position + self.smooth_factor * (target_cmd - current_position)
        
        # 更新上一次的目标指令
        self.last_target_cmd = target_cmd.copy()
        
        return smoothed_cmd
        
    def can_transition_to(self, new_state: StateType) -> bool:
        return True  # 静止状态可以切换到任何状态
        
    def is_finished(self) -> bool:
        return False  # 静止状态永不完成 