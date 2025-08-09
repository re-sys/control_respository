#!/usr/bin/env python3
"""
台阶跳跃状态文件
基于JumpState，但给后腿添加偏置以适应台阶跳跃
"""

import math
from typing import Dict, Tuple, List
from .jump_state import JumpState
from constants import StateType, Leg

class StairJumpState(JumpState):
    def __init__(self, controller: 'StateMachineController'):
        super().__init__(controller)
        self.step_idx = 0
        self.rear_leg_offset = 0.05  # 后腿偏置，可以根据需要调整
        
    def enter(self):
        self.step_idx = 0
        self.controller.pitch_angle = math.radians(-110)
        self.front_pitch_angle = math.radians(-110)
        self.rear_pitch_angle = math.radians(-100)
        self.trajectory = self._generate_stair_jump_trajectory()
        self.controller.get_logger().info("进入台阶跳跃状态")
        self.controller.pitch_angle = math.radians(-110)
        
    def _generate_stair_jump_trajectory(self) -> List[List[Tuple[float, float]]]:
        """ 生成台阶跳跃轨迹，后腿添加偏置 """
        trajectory = []
        
        # 定义关键参数
        leg_start_length = self.controller.leg_length
        leg_length_prepare_front = 0.12
        leg_length_prepare_rear = 0.12 + self.rear_leg_offset  # 后腿添加偏置
        leg_length_extend_front = 0.35-self.rear_leg_offset

        leg_length_extend_rear = 0.35  # 后腿添加偏置
        leg_end_length = 0.12
        
        angle_land = 2*math.radians(-100)-self.rear_pitch_angle
        
        # 时间参数
        self.t_prepare = 0.2
        self.t_jump = 0.2
        self.t_short = 0.1
        self.t_land = 0.1
        self.t_recover = 0.5

        # 1. 准备阶段 - 后腿添加偏置
        for i in range(int(self.t_prepare * 100)):
            progress = i / (self.t_prepare * 100)
            lf = leg_start_length + (leg_length_prepare_front - leg_start_length) * progress
            lr = leg_start_length + (leg_length_prepare_rear - leg_start_length) * progress
            trajectory.append([(lf, self.front_pitch_angle)] * 2 + [(lr, self.rear_pitch_angle)] * 2)
        
        # 2. 起跳阶段 - 后腿添加偏置
        for i in range(int(self.t_jump * 100)):
            trajectory.append([(leg_length_extend_front, self.front_pitch_angle)] * 2 + 
                              [(leg_length_extend_rear, self.rear_pitch_angle)] * 2)
        
        # 3. 收腿阶段 - 后腿添加偏置
        for i in range(int(self.t_short * 100)):
            trajectory.append([(leg_length_prepare_front, self.controller.pitch_angle)] * 2 + 
                              [(leg_length_prepare_rear, self.controller.pitch_angle)] * 2)
        
        # 4. 落地阶段 - 后腿添加偏置
        for i in range(int(self.t_land * 100)):
            progress = i / (self.t_land * 100)
            lf = leg_length_prepare_front + (leg_end_length - leg_length_prepare_front) * progress
            lr = leg_length_prepare_rear + (leg_end_length - leg_length_prepare_rear) * progress
            front_angle = self.front_pitch_angle + (angle_land - self.front_pitch_angle) * progress
            rear_angle = self.rear_pitch_angle + (angle_land - self.rear_pitch_angle) * progress
            trajectory.append([(lf, front_angle)] * 2 + [(lr, rear_angle)] * 2)
            
        # 5. 恢复阶段 - 后腿添加偏置
        for i in range(int(self.t_recover * 100)):
            progress = i / (self.t_recover * 100)
            lf = leg_end_length + (leg_start_length - leg_end_length) * progress
            lr = leg_end_length + (leg_start_length - leg_end_length) * progress
            angle = angle_land + (math.radians(-90) - angle_land) * progress
            trajectory.append([(lf, angle)] * 2 + [(lr, angle)] * 2)
        
        return trajectory
