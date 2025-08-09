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
        self.step_idx = 0
        
    def enter(self):
        self.step_idx = 0
        self.controller.pitch_angle = math.radians(-115)
        self.front_pitch_angle = math.radians(-115)
        self.rear_pitch_angle = math.radians(-115)
        self.trajectory = self._generate_jump_trajectory()
        self.controller.get_logger().info("进入前跳状态")
        self.controller.pitch_angle = math.radians(-90)
        
    def _generate_jump_trajectory(self) -> List[List[Tuple[float, float]]]:
        """ 生成前跳轨迹 """
        trajectory = []
        
        # 定义关键参数
        leg_start_length = self.controller.leg_length
        leg_length_prepare_front = 0.12
        leg_length_prepare_rear = 0.16
        leg_length_extend_front = 0.35
        leg_length_extend_rear = 0.35
        leg_end_length = 0.12
        
        angle_land = 2*math.radians(-90)-self.rear_pitch_angle
        
        # 时间参数
        # self.t_gather = 1.5    # 腿部收拢阶段时间
        self.t_prepare = 0.2
        self.t_jump = 0.2
        self.t_short = 0.1
        self.t_land = 0.1
        self.t_recover = 0.5

        # # 0. 腿部收拢阶段 - 让前腿后退靠近后腿
        # front_gather_angle = math.radians(-110)  # 前腿向后转更多（更负的角度）
        # rear_gather_angle = math.radians(-70)   # 后腿稍微向前一点
        # for i in range(int(self.t_gather * 100)):
        #     progress = i / (self.t_gather * 100)
        #     # 前腿从初始角度向后转到收拢角度
        #     front_angle_current = self.controller.pitch_angle + (front_gather_angle - self.controller.pitch_angle) * progress
        #     # 后腿从初始角度稍微向前调整
        #     rear_angle_current = self.controller.pitch_angle + (rear_gather_angle - self.controller.pitch_angle) * progress
        #     trajectory.append([(leg_start_length, front_angle_current)] * 2 + [(leg_start_length, rear_angle_current)] * 2)

        # 1. 准备阶段
        for i in range(int(self.t_prepare * 100)):
            progress = i / (self.t_prepare * 100)
            lf = leg_start_length + (leg_length_prepare_front - leg_start_length) * progress
            lr = leg_start_length + (leg_length_prepare_rear - leg_start_length) * progress
            trajectory.append([(lf, self.front_pitch_angle)] * 2 + [(lr, self.rear_pitch_angle)] * 2)
        
        # 2. 起跳阶段
        for i in range(int(self.t_jump * 100)):
            trajectory.append([(leg_length_extend_front, self.front_pitch_angle)] * 2 + 
                              [(leg_length_extend_rear, self.rear_pitch_angle)] * 2)
        
        # 3. 收腿阶段
        for i in range(int(self.t_short * 100)):
            trajectory.append([(leg_length_prepare_front, self.controller.pitch_angle)] * 4)
        
        # 4. 落地阶段
        for i in range(int(self.t_land * 100)):
            progress = i / (self.t_land * 100)
            lf = leg_length_prepare_front + (leg_end_length - leg_length_prepare_front) * progress
            lr = leg_length_prepare_rear + (leg_end_length - leg_length_prepare_rear) * progress
            front_angle = self.front_pitch_angle + (angle_land - self.front_pitch_angle) * progress
            rear_angle = self.rear_pitch_angle + (angle_land - self.rear_pitch_angle) * progress
            trajectory.append([(lf, front_angle)] * 2 + [(lr, rear_angle)] * 2)
            
        # 5. 恢复阶段
        for i in range(int(self.t_recover * 100)):
            progress = i / (self.t_recover * 100)
            l = leg_end_length + (leg_start_length - leg_end_length) * progress
            angle = angle_land + (math.radians(-90) - angle_land) * progress
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