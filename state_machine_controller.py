#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool, Int8
from std_msgs.msg import Float32MultiArray
import math
import numpy as np
from abc import ABC, abstractmethod
from typing import Dict, List, Tuple, Optional
from enum import Enum
from functools import lru_cache
import time

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

class StateType(Enum):
    IDLE = "idle"           # 静止状态
    TROT = "trot"           # 对角步态
    FLIP = "flip"           # 空翻状态
    JUMP = "jump"           # 前跳状态
    ERROR = "error"         # 错误状态
    RECOVERY = "recovery"   # 恢复状态

# Pre-computed constants for performance
PI = math.pi
PI_2 = PI / 2
PI_4 = PI / 4
DEG_TO_RAD = PI / 180.0
RAD_TO_DEG = 180.0 / PI

# ------------------- 逆运动学核心 (优化版本) -------------------
class LegKinematics:
    def __init__(self, l3=0.2, l4=0.16):
        self.l3 = l3
        self.l4 = l4
        self.l3_sq = l3 * l3
        self.l4_sq = l4 * l4
        self._ik_cache = {}
        self._cache_hits = 0
        self._cache_misses = 0
        
    def inverse_kinematics(self, l0: float, theta: float) -> Optional[Tuple[float, float]]:
        """ 计算关节角度 (极坐标版本) - 带缓存优化 """
        # 使用量化值作为缓存键以提高缓存命中率
        l0_quantized = round(l0 * 1000) / 1000
        theta_quantized = round(theta * 1000) / 1000
        cache_key = (l0_quantized, theta_quantized)
        
        if cache_key in self._ik_cache:
            self._cache_hits += 1
            return self._ik_cache[cache_key]
        
        self._cache_misses += 1
        
        # 优化计算
        l0_sq = l0 * l0
        denominator = 2 * self.l4 * l0
        
        if abs(denominator) < 1e-6:
            return None
            
        cos_theta_inside = (self.l4_sq + l0_sq - self.l3_sq) / denominator
        
        # 数值稳定性检查
        if cos_theta_inside > 1.0:
            cos_theta_inside = 1.0
        elif cos_theta_inside < -1.0:
            cos_theta_inside = -1.0
            
        theta_inside = math.acos(cos_theta_inside)
        result = (theta + theta_inside, theta - theta_inside)
        
        # 缓存结果 (限制缓存大小)
        if len(self._ik_cache) < 10000:
            self._ik_cache[cache_key] = result
            
        return result
    
    def clear_cache(self):
        """ 清理缓存 """
        self._ik_cache.clear()
        self._cache_hits = 0
        self._cache_misses = 0
    
    def get_cache_stats(self):
        """ 获取缓存统计 """
        total = self._cache_hits + self._cache_misses
        hit_rate = self._cache_hits / total if total > 0 else 0
        return {
            'hits': self._cache_hits,
            'misses': self._cache_misses,
            'hit_rate': hit_rate,
            'cache_size': len(self._ik_cache)
        }

# ------------------- 状态基类 -------------------
class State(ABC):
    def __init__(self, controller: 'StateMachineController'):
        self.controller = controller
        
    @abstractmethod
    def enter(self):
        """ 进入状态时的初始化 """
        pass
        
    @abstractmethod
    def update(self) -> np.ndarray:
        """ 更新状态，返回9维关节角度数组 """
        pass
        
    @abstractmethod
    def can_transition_to(self, new_state: StateType) -> bool:
        """ 是否可以切换到新状态 """
        pass
        
    @abstractmethod
    def is_finished(self) -> bool:
        """ 状态是否完成 """
        pass

# ------------------- 静止状态 (优化版本) -------------------
class IdleState(State):
    def __init__(self, controller: 'StateMachineController'):
        super().__init__(controller)
        self._cached_result = None
        self._last_pitch_angle = None
        self._last_leg_length = None
        
    def enter(self):
        self.controller.get_logger().info("进入静止状态")
        self._cached_result = None  # 清除缓存
        
    def update(self) -> np.ndarray:
        # 缓存优化：如果参数没有变化，直接返回缓存结果
        if (self._cached_result is not None and 
            self._last_pitch_angle == self.controller.pitch_angle and
            self._last_leg_length == self.controller.leg_length):
            return self._cached_result
        
        # 计算新的关节角度
        angle = self.controller.pitch_angle
        leg_targets = {
            Leg.FL: (self.controller.leg_length, angle),
            Leg.FR: (self.controller.leg_length, angle),
            Leg.RL: (self.controller.leg_length, angle),
            Leg.RR: (self.controller.leg_length, angle)
        }
        
        result = self.controller.compute_joint_cmd_from_leg_targets(leg_targets)
        
        # 更新缓存
        self._cached_result = result
        self._last_pitch_angle = self.controller.pitch_angle
        self._last_leg_length = self.controller.leg_length
        
        return result
        
    def can_transition_to(self, new_state: StateType) -> bool:
        return True  # 静止状态可以切换到任何状态
        
    def is_finished(self) -> bool:
        return False  # 静止状态永不完成

# ------------------- 对角步态状态 (优化版本) -------------------
class TrotState(State):
    def __init__(self, controller: 'StateMachineController'):
        super().__init__(controller)
        self.current_phase = 0.0
        # 步态参数
        self.z_swing = 0.04
        self.stride = 0.1
        self.max_turn_offset = 0.04
        self.phase_offsets = np.array([0.0, 0.5, 0.5, 0.0])
        self.largest_period = 1.0
        self.smallest_period = 0.2
        self.dt = 0.01
        
        # 预计算常量
        self._PI_2 = PI_2
        self._z_swing_half = self.z_swing * 0.5
        self._stride_half = self.stride * 0.5
        self._two_pi = 2.0 * PI
        
    def enter(self):
        self.controller.get_logger().info("进入对角步态状态")
        
    def update(self) -> np.ndarray:
        # 更新相位
        vel_magnitude = math.hypot(self.controller.cmd_vel[0], self.controller.cmd_vel[1])
        period = self.largest_period + (self.smallest_period - self.largest_period) * min(vel_magnitude, 1)
        phase_step = self.dt / period
        self.current_phase = (self.current_phase + phase_step) % 1.0
        
        # 生成各腿轨迹
        leg_targets = {}
        for leg_id in [Leg.FL, Leg.FR, Leg.RL, Leg.RR]:
            leg_targets[leg_id] = self._get_foot_target(leg_id)
        
        return self.controller.compute_joint_cmd_from_leg_targets(leg_targets)
        
    def _get_foot_target(self, leg_id: int) -> Tuple[float, float]:
        """ 生成单腿轨迹 (优化版本) """
        phase = (self.current_phase + self.phase_offsets[leg_id]) % 1.0
        vx = self.controller.cmd_vel[0]
        vy = self.controller.cmd_vel[1]
        abs_vx = abs(vx)
        
        # 优化计算
        stride = self.stride
        if vy != 0:
            angular_stride = vy * self.max_turn_offset
            if leg_id in [Leg.FR, Leg.RR]:  # 右腿
                stride = self.stride * abs_vx + angular_stride
            else:  # 左腿
                stride = self.stride * abs_vx - angular_stride
                
        stride *= 1 if vx >= 0 else -1
        half_stride = stride * 0.5
        
        # 预计算基础位置
        angle = -self._PI_2
        sin_angle = math.sin(angle)
        cos_angle = math.cos(angle)
        z_base = self.controller.leg_length * sin_angle
        x_base = self.controller.leg_length * cos_angle
        
        if phase < 0.5:  # 摆动相
            progress = phase * 2
            theta = self._two_pi * progress
            cos_theta = math.cos(theta)
            sin_theta = math.sin(theta)
            z = z_base + self._z_swing_half * (1 - cos_theta)
            x = x_base - half_stride + stride * (progress - sin_theta / self._two_pi)
        else:  # 支撑相
            progress = (phase - 0.5) * 2
            z = z_base
            x = x_base + half_stride - progress * stride
            
        l = min(max(math.hypot(x, z), 0.12), 0.34)
        theta = math.atan2(z, x)
        return l, theta
        
    def can_transition_to(self, new_state: StateType) -> bool:
        return self.is_finished()
        
    def is_finished(self) -> bool:
        return self.controller.cmd_vel[0] == 0 and self.controller.cmd_vel[1] == 0

# ------------------- 空翻状态 (优化版本) -------------------
class FlipState(State):
    def __init__(self, controller: 'StateMachineController'):
        super().__init__(controller)
        self.trajectory = None
        self.trajectory_length = 0
        self.step_idx = 0
        self._precompute_trajectory()
        
    def enter(self):
        self.step_idx = 0
        self.controller.get_logger().info("进入空翻状态")
        
    def _precompute_trajectory(self):
        """ 预计算空翻轨迹 (优化版本) """
        trajectory = []
        
        # 角度定义 (预计算)
        angle_phase0 = -PI_2
        angle_phase1 = -PI / 3
        angle_phase2_front = PI / 18
        angle_phase2_back = -PI
        angle_phase3_back = -3 * PI / 2
        angle_phase4_front = PI_2
        angle_inc = self.controller.pitch_angle - angle_phase0
        
        # 长度定义
        leg_start_length = self.controller.leg_length
        leg_length_prepare = 0.14
        leg_length_extend = 0.36
        leg_end_length = 0.14
        
        # 时间参数
        t_phase0 = 0.6
        t_phase1 = 0.3
        t_phase2 = 0.35
        t_phase3 = 0.2
        t_phase4 = 0.4
        t_phase5 = 0.4

        self.first_io_flip_index = round((t_phase0 + t_phase1) * 100)
        self.second_io_flip_index = round((t_phase0 + t_phase1 + t_phase2 + t_phase3) * 100)
        
        # 使用numpy进行批量计算以提高性能
        steps_phase0 = int(t_phase0 * 100)
        steps_phase1 = int(t_phase1 * 100)
        steps_phase2 = int(t_phase2 * 100)
        steps_phase3 = int(t_phase3 * 100)
        steps_phase4 = int(t_phase4 * 100)
        steps_phase5 = int(t_phase5 * 100)
        
        # 阶段0：下蹲
        for i in range(steps_phase0):
            progress = i / steps_phase0
            length = leg_start_length + (leg_length_prepare - leg_start_length) * progress
            trajectory.append([(length, angle_phase0)] * 2)
        
        # 阶段1：后腿蹬伸+前腿旋转
        for i in range(steps_phase1):
            progress = i / steps_phase1
            front_angle = angle_phase0 + (angle_phase1 - angle_phase0) * progress
            trajectory.append([
                (leg_length_prepare, front_angle),
                (leg_length_extend, angle_phase0),
            ])
        
        # 阶段2：后腿收腿+前腿继续旋转
        for i in range(steps_phase2):
            progress = i / steps_phase2
            front_angle = angle_phase1 + (angle_phase2_front - angle_phase1) * progress
            back_angle = angle_phase0 + (angle_phase2_back - angle_phase0) * progress
            back_length = leg_length_extend + (leg_end_length - leg_length_extend) * progress
            trajectory.append([
                (leg_length_prepare, front_angle),
                (back_length, back_angle),
            ])
        
        # 阶段3：前腿蹬伸+后腿旋转
        for i in range(steps_phase3):
            progress = i / steps_phase3
            back_angle = angle_phase2_back + (angle_phase3_back - angle_phase2_back) * progress
            trajectory.append([
                (leg_length_extend, angle_phase2_front),
                (leg_end_length, back_angle)
            ])
        
        # 阶段4：前腿旋转
        for i in range(steps_phase4):
            progress = i / steps_phase4
            front_angle = angle_phase2_front + (angle_phase4_front - angle_phase2_front) * progress
            trajectory.append([
                (leg_end_length, front_angle),
                (leg_end_length, angle_phase3_back)
            ])
        
        # 阶段5：恢复初始状态
        for i in range(steps_phase5):
            progress = i / steps_phase5
            length = leg_end_length + (leg_start_length - leg_end_length) * progress
            back_angle = angle_phase3_back + angle_inc * progress
            front_angle = angle_phase4_front + angle_inc * progress
            trajectory.append([(length, front_angle), (length, back_angle)])
        
        self.trajectory = trajectory
        self.trajectory_length = len(trajectory)
        
    def update(self) -> np.ndarray:
        if self.step_idx >= self.trajectory_length:
            return np.zeros(9)
            
        leg_polar_coords = self.trajectory[self.step_idx]
        self.step_idx += 1
        
        # 处理IO翻转逻辑
        if self.step_idx == self.first_io_flip_index:
            if not self.controller.back_state:
                self.controller.flip_io_leg[1] = not self.controller.flip_io_leg[1]
            else:
                self.controller.flip_io_leg[0] = not self.controller.flip_io_leg[0]
                
        if self.step_idx == self.second_io_flip_index:
            if not self.controller.back_state:
                self.controller.flip_io_leg[0] = not self.controller.flip_io_leg[0]
            else:
                self.controller.flip_io_leg[1] = not self.controller.flip_io_leg[1]

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
        return self.is_finished()
        
    def is_finished(self) -> bool:
        return self.step_idx >= self.trajectory_length

# ------------------- 前跳状态 (优化版本) -------------------
class JumpState(State):
    def __init__(self, controller: 'StateMachineController'):
        super().__init__(controller)
        self.trajectory = None
        self.step_idx = 0
        self._precompute_trajectory()
        
    def enter(self):
        self.step_idx = 0
        self.controller.get_logger().info("进入前跳状态")
        
    def _precompute_trajectory(self):
        """ 预计算前跳轨迹 (优化版本) """
        trajectory = []
        
        # 定义关键参数
        leg_start_length = self.controller.leg_length
        leg_length_prepare = 0.14
        leg_length_extend = 0.36
        leg_end_length = 0.14
        
        angle_land = 2 * (-PI_2) - self.controller.pitch_angle
        
        # 时间参数
        t_prepare = 0.2
        t_jump = 0.15
        t_short = 0.1
        t_land = 0.1
        t_recover = 0.6
        
        # 预计算步数
        steps_prepare = int(t_prepare * 100)
        steps_jump = int(t_jump * 100)
        steps_short = int(t_short * 100)
        steps_land = int(t_land * 100)
        steps_recover = int(t_recover * 100)
        
        # 1. 准备阶段
        for i in range(steps_prepare):
            progress = i / steps_prepare
            l = leg_start_length + (leg_length_prepare - leg_start_length) * progress
            trajectory.append([(l, self.controller.pitch_angle)] * 4)
        
        # 2. 起跳阶段
        for i in range(steps_jump):
            trajectory.append([(leg_length_extend, self.controller.pitch_angle)] * 4)
        
        # 3. 收腿阶段
        for i in range(steps_short):
            trajectory.append([(leg_length_prepare, self.controller.pitch_angle)] * 4)
        
        # 4. 落地阶段
        for i in range(steps_land):
            progress = i / steps_land
            l = leg_length_prepare + (leg_end_length - leg_length_prepare) * progress
            angle = self.controller.pitch_angle + (angle_land - self.controller.pitch_angle) * progress
            trajectory.append([(l, angle)] * 4)
            
        # 5. 恢复阶段
        for i in range(steps_recover):
            progress = i / steps_recover
            l = leg_end_length + (leg_start_length - leg_end_length) * progress
            angle = angle_land + (self.controller.pitch_angle - angle_land) * progress
            trajectory.append([(l, angle)] * 4)
        
        self.trajectory = trajectory
        
    def update(self) -> np.ndarray:
        if self.step_idx >= len(self.trajectory):
            return np.zeros(9)
            
        leg_xz = self.trajectory[self.step_idx]
        self.step_idx += 1
        
        # 转换为极坐标
        result = {}
        for leg_id, (l, angle) in enumerate(leg_xz):
            result[leg_id] = (l, angle)
        return self.controller.compute_joint_cmd_from_leg_targets(result)
        
    def can_transition_to(self, new_state: StateType) -> bool:
        return self.is_finished()
        
    def is_finished(self) -> bool:
        return self.step_idx >= len(self.trajectory)

# ------------------- 恢复状态 (优化版本) -------------------
class RecoveryState(State):
    def __init__(self, controller: 'StateMachineController'):
        super().__init__(controller)
        self.start_cmd = None
        self.target_cmd = None
        self.steps = 100
        self.current_step = 0
        self._cached_target = None

    def enter(self):
        self.start_cmd = self.controller.last_joint_cmd.copy()
        # 缓存目标关节角度
        if self._cached_target is None:
            self._cached_target = self.controller.states[StateType.IDLE].update()
        self.target_cmd = self._cached_target
        self.current_step = 0
        self.controller.get_logger().info("进入恢复状态，平滑过渡到静止站立")

    def update(self) -> np.ndarray:
        if self.current_step >= self.steps:
            return self.target_cmd
        alpha = self.current_step / self.steps
        cmd = (1 - alpha) * self.start_cmd + alpha * self.target_cmd
        self.current_step += 1
        return cmd

    def can_transition_to(self, new_state: StateType) -> bool:
        return self.is_finished() and new_state == StateType.IDLE

    def is_finished(self) -> bool:
        return self.current_step >= self.steps

# ------------------- 错误状态 -------------------
class ErrorState(State):
    def __init__(self, controller: 'StateMachineController'):
        super().__init__(controller)
        
    def enter(self):
        self.controller.get_logger().info("进入错误状态")
        
    def update(self) -> np.ndarray:
        return self.controller.last_joint_cmd
        
    def can_transition_to(self, new_state: 'StateType') -> bool:
        return new_state == StateType.RECOVERY
        
    def is_finished(self) -> bool:
        return False

# ------------------- 状态机控制器 (优化版本) -------------------
class StateMachineController(Node):
    def __init__(self):
        super().__init__('state_machine_controller')
        self.ik = LegKinematics()
        self.leg_length = 0.17
        self.pitch_angle = -PI_2
        self.back_state = False
        self.flip_io_leg = [False, False]
        self.cmd_vel = np.array([0.0, 0.0])
        self.last_joint_cmd = np.zeros(9)
        
        # 性能监控
        self.control_loop_count = 0
        self.last_performance_log = time.time()

        # 初始化状态
        self.states = {
            StateType.IDLE: IdleState(self),
            StateType.TROT: TrotState(self),
            StateType.FLIP: FlipState(self),
            StateType.JUMP: JumpState(self),
            StateType.ERROR: ErrorState(self),
            StateType.RECOVERY: RecoveryState(self)
        }
        
        # ROS接口
        self.joint_pub = self.create_publisher(JointState, '/action', 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.state_sub = self.create_subscription(Int8, '/state', self.state_callback, 10)
        self.params_sub = self.create_subscription(Float32MultiArray, '/params', self.params_callback, 10)
        self.current_state = self.states[StateType.FLIP]
        self.current_state.enter()
        self.timer = self.create_timer(0.01, self.control_loop)

        self.get_logger().info("优化版状态机控制器已启动")
        
    def cmd_vel_callback(self, msg: Twist):
        """ 处理速度指令 """
        self.cmd_vel[0] = msg.linear.x
        self.cmd_vel[1] = msg.angular.z
        
    def params_callback(self, msg: Float32MultiArray):
        """ 处理参数指令 """
        self.leg_length = 0.14 + msg.data[0] * (0.36 - 0.14)
        self.pitch_angle = (-60 + msg.data[1] * (-60)) * DEG_TO_RAD
        self.states[StateType.TROT].smallest_period = msg.data[2]
        self.states[StateType.TROT].stride = msg.data[3]
        self.states[StateType.TROT].z_swing = msg.data[4]
        
        # 清除相关缓存
        if hasattr(self.states[StateType.IDLE], '_cached_result'):
            self.states[StateType.IDLE]._cached_result = None

    def state_callback(self, msg: Int8):
        """ 处理状态切换 """
        if msg.data == 1:  # 空翻触发
            self._transition_to(StateType.FLIP)
        elif msg.data == 2:  # 前跳触发
            self._transition_to(StateType.JUMP)
        elif msg.data == 3:  # 恢复触发
            self._transition_to(StateType.RECOVERY)
            
    def _transition_to(self, new_state_type: StateType):
        """ 切换到新状态 """
        if self.current_state.can_transition_to(new_state_type):
            self.current_state = self.states[new_state_type]
            self.current_state.enter()
            self.get_logger().info(f"切换到状态: {new_state_type.value}")

    def compute_joint_cmd_from_leg_targets(self, leg_targets):
        """ 计算关节指令 (优化版本) """
        # 根据back_state选择腿的映射
        if not self.back_state:
            fl_length, fl_angle = leg_targets[Leg.FL]
            fr_length, fr_angle = leg_targets[Leg.FR]
            rl_length, rl_angle = leg_targets[Leg.RL]
            rr_length, rr_angle = leg_targets[Leg.RR]
        else:
            fl_length, fl_angle = leg_targets[Leg.RL]
            fr_length, fr_angle = leg_targets[Leg.RR]
            rl_length, rl_angle = leg_targets[Leg.FL]
            rr_length, rr_angle = leg_targets[Leg.FR]
            fl_angle += PI
            fr_angle += PI
            rl_angle -= PI
            rr_angle -= PI
            
        # 计算逆运动学
        if self.flip_io_leg[0]:
            FL_theta4, FL_theta1 = self.ik.inverse_kinematics(fl_length, fl_angle)
            FR_theta4, FR_theta1 = self.ik.inverse_kinematics(fr_length, fr_angle)
        else:
            FL_theta1, FL_theta4 = self.ik.inverse_kinematics(fl_length, fl_angle)
            FR_theta1, FR_theta4 = self.ik.inverse_kinematics(fr_length, fr_angle)
            
        if self.flip_io_leg[1]:
            RL_theta4, RL_theta1 = self.ik.inverse_kinematics(rl_length, rl_angle)
            RR_theta4, RR_theta1 = self.ik.inverse_kinematics(rr_length, rr_angle)
        else:
            RL_theta1, RL_theta4 = self.ik.inverse_kinematics(rl_length, rl_angle)
            RR_theta1, RR_theta4 = self.ik.inverse_kinematics(rr_length, rr_angle)
            
        # 构建关节指令数组
        joint_cmd = np.zeros(9)
        joint_cmd[0] = -FL_theta1
        joint_cmd[1] = FL_theta4 + PI
        joint_cmd[2] = FR_theta1
        joint_cmd[3] = -FR_theta4 - PI
        joint_cmd[5] = -RL_theta4 - PI
        joint_cmd[6] = RL_theta1
        joint_cmd[7] = RR_theta4 + PI
        joint_cmd[8] = -RR_theta1
        
        return joint_cmd

    def control_loop(self):
        """ 主控制循环 (优化版本) """
        start_time = time.time()
        
        # 更新当前状态
        if isinstance(self.current_state, ErrorState):
            return

        # 检查是否需要切换到步态状态
        if self.cmd_vel[0] != 0 or self.cmd_vel[1] != 0:
            self._transition_to(StateType.TROT)

        # 更新当前状态
        cmd = self.current_state.update()
        
        # 发布指令
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = cmd.tolist()

        # 安全检查
        if np.max(np.abs(cmd - self.last_joint_cmd)) < PI * 0.8:
            self.joint_pub.publish(msg)
            self.last_joint_cmd = cmd.copy()
        else:
            self.get_logger().info("关节角度变化过大，不发布指令")
            self._transition_to(StateType.ERROR)

        # 检查状态是否完成
        if self.current_state.is_finished():
            self.get_logger().info(f"finished, back_state: {self.back_state}, flip_io_leg: {self.flip_io_leg}")
            self._transition_to(StateType.IDLE)
            
        # 性能监控
        self.control_loop_count += 1
        current_time = time.time()
        if current_time - self.last_performance_log > 10.0:  # 每10秒记录一次
            loop_time = (current_time - start_time) * 1000  # 转换为毫秒
            cache_stats = self.ik.get_cache_stats()
            self.get_logger().info(f"性能统计 - 控制循环时间: {loop_time:.2f}ms, "
                                 f"IK缓存命中率: {cache_stats['hit_rate']:.2%}, "
                                 f"缓存大小: {cache_stats['cache_size']}")
            self.last_performance_log = current_time

# ------------------- 主函数 -------------------
def main(args=None):
    rclpy.init(args=args)
    controller = StateMachineController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("控制器安全停止")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 