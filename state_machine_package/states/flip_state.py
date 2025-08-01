#!/usr/bin/env python3
"""
空翻状态文件
"""

import math
from typing import Dict, Tuple, List
from base_state import State
from constants import StateType, Leg

class FlipState(State):
    def __init__(self, controller: 'StateMachineController'):
        super().__init__(controller)
        self.trajectory = self._generate_flip_trajectory()
        self.trajectory_length = len(self.trajectory)
        self.step_idx = 0
        
    def enter(self):
        self.step_idx = 0
        self.controller.get_logger().info("进入空翻状态")
        
    def _generate_flip_trajectory(self) -> List[List[Tuple[float, float]]]:
        """ 生成空翻轨迹 """
        trajectory = []
        
        # 角度定义
        angle_phase0 = math.radians(-90)
        angle_phase1 = math.radians(-60)
        angle_phase2_front = math.radians(10)
        angle_phase2_back = math.radians(-180)
        angle_phase3_back = math.radians(-270)
        angle_phase4_front = math.radians(90)

        angle_inc = self.controller.pitch_angle - angle_phase0
        
        # 长度定义
        leg_start_length = self.controller.leg_length
        leg_length_prepare_back = 0.14
        leg_length_prepare_front = 0.14
        leg_length_recover = 0.3
        leg_length_extend = 0.36
        leg_end_length = 0.16
        
        # 时间参数
        self.t_phase0 = 0.6        # 下蹲阶段
        self.t_phase1 = 0.35       # 后腿蹬伸+前腿旋转30°
        self.t_phase2 = 0.35     # 后腿收腿+前腿旋转90°
        self.t_phase3 = 0.3        # 前腿蹬伸+后腿旋转
        self.t_phase4 = 0.2        # 前腿收腿
        self.t_phase5 = 0.4        # 恢复初始状态

        self.first_io_flip_index = round(self.t_phase0*100+self.t_phase1*100)
        self.second_io_flip_index = round(self.t_phase0*100+self.t_phase1*100+self.t_phase2*100+self.t_phase3*100)
        
        # 阶段0：下蹲
        for i in range(int(self.t_phase0 * 100)):
            progress = i / (self.t_phase0 * 100)
            length_back = leg_start_length + (leg_length_prepare_back - leg_start_length) * progress
            length_front = leg_start_length + (leg_length_prepare_front - leg_start_length) * progress
            trajectory.append([(length_back, angle_phase0), (length_front, angle_phase0)])
        
        # 阶段1：后腿蹬伸+前腿旋转
        for i in range(int(self.t_phase1 * 100)):
            progress = i / (self.t_phase1 * 100)
            front_angle = angle_phase0 + (angle_phase1 - angle_phase0) * progress
            trajectory.append([
                (leg_length_prepare_front, front_angle),  # Front
                (leg_length_extend, angle_phase0),  # Rear
            ])
        
        # 阶段2：后腿收腿+前腿继续旋转
        for i in range(int(self.t_phase2 * 100)):
            progress = i / (self.t_phase2 * 100)
            front_angle = angle_phase1 + (angle_phase2_front - angle_phase1) * progress
            back_angle = angle_phase0 + (angle_phase2_back - angle_phase0) * progress
            back_length = leg_length_recover 
            trajectory.append([
                (leg_length_prepare_front, front_angle),  # Front
                (back_length, back_angle),       # Rear
            ])
        
        # 阶段3：前腿蹬伸+后腿旋转
        for i in range(int(self.t_phase3 * 100)):
            progress = i / (self.t_phase3 * 100)
            back_angle = angle_phase2_back + (angle_phase3_back - angle_phase2_back) * progress
            back_length = leg_length_recover + (leg_end_length - leg_length_recover) * progress
            trajectory.append([
                (leg_length_extend, angle_phase2_front),  # Front
                (back_length, back_angle)            # Rear
            ])
        
        # 阶段4：前腿收腿
        for i in range(int(self.t_phase4 * 100)):
            progress = i / (self.t_phase4 * 100)
            leg_front_length = leg_length_recover + (leg_end_length - leg_length_recover) * progress
            front_angle = angle_phase2_front + (angle_phase4_front - angle_phase2_front) * progress
            trajectory.append([
                (leg_front_length, front_angle),      # Front
                (leg_end_length, angle_phase3_back)  # Rear
            ])
        
        # 阶段5：恢复初始状态
        for i in range(int(self.t_phase5 * 100)):
            progress = i / (self.t_phase5 * 100)
            length = leg_end_length + (leg_start_length - leg_end_length) * progress
            back_angle = angle_phase3_back + angle_inc * progress
            front_angle = angle_phase2_front + angle_inc * progress
            trajectory.append([(length, front_angle), (length, back_angle)])
        
        return trajectory
        
    def update(self) -> Dict[int, Tuple[float, float]]:
        if self.step_idx >= self.trajectory_length:
            return {}
            
        leg_polar_coords = self.trajectory[self.step_idx]
        
        if self.step_idx == self.first_io_flip_index:
            if not self.controller.back_state:
                self.controller.flip_io_leg[1] = False if self.controller.flip_io_leg[1] else True
            else:
                self.controller.flip_io_leg[0] = False if self.controller.flip_io_leg[0] else True
                
        if self.step_idx == self.second_io_flip_index:
            if not self.controller.back_state:
                self.controller.flip_io_leg[0] = False if self.controller.flip_io_leg[0] else True
            else:
                self.controller.flip_io_leg[1] = False if self.controller.flip_io_leg[1] else True
                self.controller.get_logger().info(f"back_state: {self.controller.back_state}, flip_io_leg: {self.controller.flip_io_leg}")
        self.step_idx += 1

        result = {
            Leg.FL: leg_polar_coords[0],
            Leg.FR: leg_polar_coords[0],
            Leg.RL: leg_polar_coords[1],
            Leg.RR: leg_polar_coords[1]
        }
        return self.controller.compute_joint_cmd_from_leg_targets(result)
        
    def can_transition_to(self, new_state: StateType) -> bool:
        if self.step_idx == self.trajectory_length:
            self.controller.back_state = not self.controller.back_state
        return self.is_finished()  # 只有完成才能切换
        
    def is_finished(self) -> bool:
        return self.step_idx >= self.trajectory_length 