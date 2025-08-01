#!/usr/bin/env python3
"""
前跳状态文件
"""

import math
from typing import Dict, Tuple, List
from base_state import State
from constants import StateType, Leg

class JumpState(State):
    def __init__(self, controller: 'StateMachineController'):
        super().__init__(controller)
        self.trajectory = self._generate_jump_trajectory()
        self.step_idx = 0
        
    def enter(self):
        self.step_idx = 0
        self.controller.get_logger().info("进入前跳状态")
        
    def _generate_jump_trajectory(self) -> List[List[Tuple[float, float]]]:
        """ 生成前跳轨迹 """
        trajectory = []
        
        # 定义关键参数
        leg_start_length = self.controller.leg_length
        leg_length_prepare = 0.10
        leg_length_extend = 0.35
        leg_end_length = 0.14
        
        angle_land = 2*math.radians(-90)-self.controller.pitch_angle
        
        # 时间参数
        self.t_prepare = 0.3
        self.t_jump = 0.15
        self.t_short = 0.1
        self.t_land = 0.1
        self.t_recover = 0.6

        # 1. 准备阶段
        for i in range(int(self.t_prepare * 100)):
            progress = i / (self.t_prepare * 100)
            l = leg_start_length + (leg_length_prepare - leg_start_length) * progress
            trajectory.append([(l, self.controller.pitch_angle)] * 4)
        
        # 2. 起跳阶段
        for i in range(int(self.t_jump * 100)):
            trajectory.append([(leg_length_extend, self.controller.pitch_angle)] * 4)
        
        # 3. 收腿阶段
        for i in range(int(self.t_short * 100)):
            trajectory.append([(leg_length_prepare, self.controller.pitch_angle)] * 4)
        
        # 4. 落地阶段
        for i in range(int(self.t_land * 100)):
            progress = i / (self.t_land * 100)
            l = leg_length_prepare + (leg_end_length - leg_length_prepare) * progress
            angle = self.controller.pitch_angle + (angle_land - self.controller.pitch_angle) * progress
            trajectory.append([(l, angle)] * 4)
            
        # 5. 恢复阶段
        for i in range(int(self.t_recover * 100)):
            progress = i / (self.t_recover * 100)
            l = leg_end_length + (leg_start_length - leg_end_length) * progress
            angle = angle_land + (self.controller.pitch_angle - angle_land) * progress
            trajectory.append([(l, angle)] * 4)
        
        return trajectory
        
    def update(self) -> Dict[int, Tuple[float, float]]:           
        leg_xz = self.trajectory[self.step_idx]
        self.step_idx += 1
        
        # 转换为极坐标
        result = {}
        for leg_id, (l, angle) in enumerate(leg_xz):
            result[leg_id] = (l, angle)
        cmd = self.controller.compute_joint_cmd_from_leg_targets(result)

        return cmd
        
    def can_transition_to(self, new_state: StateType) -> bool:
        return self.is_finished() or new_state == StateType.ERROR  # 只有完成才能切换
        
    def is_finished(self) -> bool:
        return self.step_idx >= len(self.trajectory) 