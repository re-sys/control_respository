#!/usr/bin/env python3
"""
navigation
"""
import math
from typing import Dict, Tuple
from base_state import State
from constants import StateType, Leg

class Navigation_straight_line(State):
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
        self.largest_period = 0.5
        self.smallest_period = self.controller.params.smallest_period
        self.dt = 0.01
        self.swing_time = 0.5
        self.vx = 0.0
        self.vy = 0.0
        self.controller.get_logger().info(f"""
        self.z_swing: {self.z_swing}
        self.stride: {self.stride}
        """)
        # PID控制器参数 - 用于保持直线运动
        self.k_p = 2.5 # 比例增益
        self.k_i = 0.01 # 积分增益  
        self.k_d = 0.05  # 微分增益
        
        # PID控制器状态变量
        self.yaw_error_integral = 0.0
        self.last_yaw_error = 0.0
        self.target_yaw = 0.0  # 目标偏航角（直线方向）
        
        # 固定前进速度
        self.forward_speed = 1  # 固定前进速度 m/s
        
    def enter(self):
        self.z_swing = 0.05
        self.stride = 0.1
        self.max_turn_offset = 0.03
        self.controller.get_logger().info(f"""
        self.z_swing: {self.z_swing}
        self.stride: {self.stride}
        self.max_turn_offset: {self.max_turn_offset}
        """)
        self.controller.get_logger().info("进入直线导航状态")
        self.current_phase = 0.25
        # 记录进入状态时的偏航角作为目标方向
        self.target_yaw = self.controller.pitch
        self.yaw_error_integral = 0.0
        self.last_yaw_error = 0.0
        
    def update(self) -> Dict[int, Tuple[float, float]]:
        # 计算偏航角误差
        yaw_error = self._normalize_angle(self.controller.pitch - self.target_yaw)
        
        # PID控制计算
        self.yaw_error_integral += yaw_error * self.dt
        yaw_error_derivative = (yaw_error - self.last_yaw_error) / self.dt
        
        # 计算角速度修正
        angular_correction = (self.k_p * yaw_error + 
                            self.k_i * self.yaw_error_integral + 
                            self.k_d * yaw_error_derivative)
        # 限制积分项防止积分饱和
        self.yaw_error_integral = max(min(self.yaw_error_integral, 0.8), -0.8)
        #需要將兩個速度正規化到[-1,1]之間
        self.vx = max(min(self.forward_speed, 1), -1)
        self.vy = max(min(angular_correction, 1), -1)
        self.controller.get_logger().info(f"vx: {self.vx},vy: {self.vy},error: {yaw_error}",throttle_duration_sec=0.5)
        self.controller.get_logger().info(f"integral: {self.yaw_error_integral},derivative: {yaw_error_derivative}",throttle_duration_sec=0.5)
        # 更新相位
        period = self.largest_period + (self.smallest_period - self.largest_period) * min(math.hypot(self.vx, self.vy), 1)
        phase_step = self.dt / period
        self.current_phase = (self.current_phase + phase_step) % 1.0
        
        # 生成各腿轨迹
        leg_targets = {}
        for leg_id in [Leg.FL, Leg.FR, Leg.RL, Leg.RR]:
            leg_targets[leg_id] = self._get_foot_target(leg_id)
        
        # 更新上一次误差
        self.last_yaw_error = yaw_error
        self.controller.get_logger().info(f"stride: {self.stride},vx: {self.vx},vy: {self.vy}",throttle_duration_sec=0.5)
        
        return self.controller.compute_joint_cmd_from_leg_targets(leg_targets)
        
    def _normalize_angle(self, angle):
        """ 将角度标准化到 [-pi, pi] 范围 """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
        
    def _get_foot_target(self, leg_id: int) -> Tuple[float, float]:
        """ 生成单腿轨迹 """
        phase = (self.current_phase + self.phase_offsets[leg_id]) % 1.0
        vx = self.vx
        vy = self.vy
        stride = self.stride

        if vy != 0:
            angular_stride = vy * self.max_turn_offset
            if leg_id in [Leg.FR, Leg.RR]:  # 右腿
                stride = self.stride * vx + angular_stride
            else:  # 左腿
                stride = self.stride * vx - angular_stride
                
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
        return True # 步态状态可以切换到任何状态
        
    def is_finished(self) -> bool:
        return False  # 步态状态永不完成 