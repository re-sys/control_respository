#!/usr/bin/env python3
"""
对角步态状态文件
"""

import math
from typing import Dict, Tuple
from base_state import State
from constants import StateType, Leg
class WalkState(State):
    def __init__(self, controller: 'StateMachineController'):
        super().__init__(controller)
        # 步态参数
        self.z_swing = self.controller.params.swing_height
        self.stride = self.controller.params.running_stride
        self.max_turn_offset = self.controller.params.max_turn_stride
        
        self.phase_amble_offsets = [0.0, 0.5, 0.625, 0.125]
        self.phase_trot_offsets = [0.0, 0.5, 0.5, 0.0]
        self.phase_walk_offsets = [0.0, 0.25, 0.5, 0.75]
        self.phase_offsets = self.phase_trot_offsets
        self.largest_period = 1.0
        self.smallest_period = self.controller.params.smallest_period
        self.dt = 0.01
        self.swing_time = 0.5
        
    def enter(self):
        self.controller.get_logger().info("进入对角步态状态")
        self.current_phase = 0.25
        
    def update(self) -> Dict[int, Tuple[float, float]]:
        # 更新相位
        period = self.largest_period + (self.smallest_period - self.largest_period) * min(math.hypot(self.controller.cmd_vel[0], self.controller.cmd_vel[1]), 1)
        phase_step = self.dt / period
        self.current_phase = (self.current_phase + phase_step) % 1.0
        
        # 生成各腿轨迹
        leg_targets = {}
        for leg_id in [Leg.FL, Leg.FR, Leg.RL, Leg.RR]:
            leg_targets[leg_id] = self._get_foot_target(leg_id)
        
        return self.controller.compute_joint_cmd_from_leg_targets(leg_targets)
        
    def _get_foot_target(self, leg_id: int) -> Tuple[float, float]:
        """ 生成单腿轨迹 """
        phase = (self.current_phase + self.phase_offsets[leg_id]) % 1.0
        vx = self.controller.cmd_vel[0]
        vy = self.controller.cmd_vel[1]
        abs_vx = abs(vx)
        
        stride = self.stride
        
        if vy != 0:
            angular_stride = vy * self.max_turn_offset
            if leg_id in [Leg.FR, Leg.RR]:  # 右腿
                stride = self.stride * abs_vx + angular_stride
            else:  # 左腿
                stride = self.stride * abs_vx - angular_stride
                
        stride *= 1 if vx >= 0 else -1
        half_stride = stride * 0.5
        angle = self.controller.pitch_angle
        z_base = self.controller.leg_length * math.sin(angle)
        x_base = self.controller.leg_length * math.cos(angle)
        
        if phase < self.swing_time:  # 摆动相
            progress = phase / self.swing_time
            theta = 2.0 * math.pi * progress
            z = z_base + self.z_swing * (1-math.cos(theta)) * 0.5
            x = x_base - half_stride + stride * (progress - math.sin(theta)/(2.0 * math.pi))
        else:  # 支撑相
            progress = (phase - self.swing_time) / (1-self.swing_time)
            z = z_base
            x = x_base + half_stride - progress * stride

        l = min(max(math.hypot(x, z),0.13),0.34)
        theta = math.atan2(z, x)
        return l, theta
        
    def can_transition_to(self, new_state: StateType) -> bool:
        return self.is_finished()  # 步态状态可以切换到任何状态
        
    def is_finished(self) -> bool:
        return self.controller.cmd_vel[0] == 0 and self.controller.cmd_vel[1] == 0  # 步态状态永不完成 