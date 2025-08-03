#!/usr/bin/env python3
from types import NoneType
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Quaternion
from std_msgs.msg import Float32, Bool, Int8
from std_msgs.msg import Float32MultiArray
import math
import numpy as np
from abc import ABC, abstractmethod
from typing import Dict, List, Tuple, Optional
from enum import Enum
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
    ERROR = "error"         # 错误状态
    RECOVERY = "recovery"   # 恢复状态


# ------------------- 逆运动学核心 -------------------
class LegKinematics:
    def __init__(self, l3=0.2, l4=0.16):
        self.l3 = l3; self.l4 = l4  # 大腿/小腿长度

    def inverse_kinematics(self, l0: float, theta: float) -> Optional[Tuple[float, float]]:
        """ 计算关节角度 (极坐标版本) """
        offset = 0.2 if l0 > 0.35 else 0
        theta_inside = math.acos((self.l4**2 + l0**2 - self.l3**2) / (2*self.l4*l0))-offset
        return theta + theta_inside, theta - theta_inside

# ------------------- 状态基类 -------------------
class State(ABC):
    def __init__(self, controller: 'StateMachineController'):
        self.controller = controller
        
        
    @abstractmethod
    def enter(self):
        """ 进入状态时的初始化 """
        pass
        
    @abstractmethod
    def update(self) -> Dict[int, Tuple[float, float]]:
        """ 更新状态，返回各腿的目标位置 (leg_id -> (length, angle)) """
        pass
        
    @abstractmethod
    def can_transition_to(self, new_state: StateType) -> bool:
        """ 是否可以切换到新状态 """
        pass
        
    @abstractmethod
    def is_finished(self) -> bool:
        """ 状态是否完成 """
        pass

# ------------------- 静止状态 -------------------
class IdleState(State):
    def __init__(self, controller: 'StateMachineController'):
        super().__init__(controller)
        
    def enter(self):
        self.controller.get_logger().info("进入静止状态")
        
    def update(self) -> Dict[int, Tuple[float, float]]:
        angle = self.controller.pitch_angle 
        result = {
            Leg.FL: (self.controller.leg_length, angle),
            Leg.FR: (self.controller.leg_length, angle),
            Leg.RL: (self.controller.leg_length, angle),
            Leg.RR: (self.controller.leg_length, angle)
        }
        # self.controller.get_logger().info(f"back_state: {self.controller.back_state}, flip_io_leg: {self.controller.flip_io_leg}")
        return self.controller.compute_joint_cmd_from_leg_targets(result)
    def can_transition_to(self, new_state: StateType) -> bool:
        return True  # 静止状态可以切换到任何状态
        
    def is_finished(self) -> bool:
        return False  # 静止状态永不完成

# ------------------- 对角步态状态 -------------------
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
        # progress = phase / self.swing_time
        # theta = 2.0 * math.pi * progress
        # x = x_base - half_stride * math.cos(theta)
        # z = z_base + half_stride * math.sin(theta)

        l = min(max(math.hypot(x, z),0.13),0.34)
        theta = math.atan2(z, x)
        return l, theta
        
    def can_transition_to(self, new_state: StateType) -> bool:
        return self.is_finished()  # 步态状态可以切换到任何状态
        
    def is_finished(self) -> bool:
        return self.controller.cmd_vel[0] == 0 and self.controller.cmd_vel[1] == 0  # 步态状态永不完成

# ------------------- 空翻状态 -------------------
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
        # self.controller.get_logger().info(f"step_idx: {self.step_idx},phase0+phase1+phase2+phase3: {(self.t_phase0+self.t_phase1+self.t_phase2+self.t_phase3) * 100}")
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

# ------------------- 前跳状态 -------------------
class JumpState(State):
    def __init__(self, controller: 'StateMachineController'):
        super().__init__(controller)
        # self.kp_value_jump = [200.0, 200.0, 200.0, 200.0, 400.0, 200.0, 200.0, 200.0, 200.0]  # 起跳设置的 Kp 值
        # self.kd_value_jump = [1.0, 1.0, 1.0, 1.0, 8.0, 1.0, 1.0, 1.0, 1.0]  # 起跳设置的 Kd 值
        # self.kp_value_land = [25.0, 25.0, 25.0, 25.0, 400.0, 25.0, 25.0, 25.0, 25.0]  # 着陆设置的 Kp 值
        # self.kd_value_land = [0.5, 0.5, 0.5, 0.5, 8.0, 0.5, 0.5, 0.5, 0.5]  # 着陆设置的 Kd 值
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
        # 计算前倾和后仰的x偏移量
        
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
        # if self.step_idx == 1:
        #     msg = JointState()
        #     msg.header.stamp = self.controller.get_clock().now().to_msg()
        #     msg.name = JOINT_NAMES
        #     kp_efforts = [25.0] * len(JOINT_NAMES)
        #     kd_efforts = [0.5] * len(JOINT_NAMES)
        #     for index in range(len(JOINT_NAMES)):
        #         kp_efforts[index] = self.kp_value_jump[index]
        #         kd_efforts[index] = self.kd_value_jump[index]
        #     msg.effort =kp_efforts # 使用 effort 字段来传输 Kp/Kd
        #     self.controller.kp_pub.publish(msg)
        #     msg.effort = kd_efforts
        #     self.controller.kd_pub.publish(msg)
        #     self.controller.get_logger().info(
        #         f"发布指令: 所有电机KpKd值已设置",
        #         throttle_duration_sec=5 # 每5秒最多打印一次，避免刷屏
        #     )
        # elif self.step_idx == self.t_prepare * 100 + self.t_jump * 100 + self.t_short * 100:
        #     msg = JointState()
        #     msg.header.stamp = self.controller.get_clock().now().to_msg()
        #     msg.name = JOINT_NAMES
        #     kp_efforts = [25.0] * len(JOINT_NAMES)
        #     kd_efforts = [0.5] * len(JOINT_NAMES)
        #     for index in range(len(JOINT_NAMES)):
        #         kp_efforts[index] = self.kp_value_land[index]
        #         kd_efforts[index] = self.kd_value_land[index]
        #     msg.effort = kp_efforts
        #     self.controller.kp_pub.publish(msg)
        #     msg.effort = kd_efforts
        #     self.controller.kd_pub.publish(msg)
        #     self.controller.get_logger().info(
        #         f"发布指令: 所有电机KpKd值已设置",
        #         throttle_duration_sec=5 # 每5秒最多打印一次，避免刷屏
        #     )
        for leg_id, (l, angle) in enumerate(leg_xz):
            result[leg_id] = (l, angle)
        cmd = self.controller.compute_joint_cmd_from_leg_targets(result)

        return cmd
        
    def can_transition_to(self, new_state: StateType) -> bool:
        return self.is_finished() or new_state == StateType.ERROR  # 只有完成才能切换
        
    def is_finished(self) -> bool:
        return self.step_idx >= len(self.trajectory)

class RecoveryState(State):
    def __init__(self, controller: 'StateMachineController'):
        super().__init__(controller)
        self.start_cmd = None
        self.target_cmd = None
        self.steps = 100  # 平滑步数
        self.current_step = 0

    def enter(self):
        self.start_cmd = self.controller.joint_position.copy()
        self.controller.publish_kp_value(20.0)
        # 生成Idle目标关节角度
        self.target_cmd = self.controller.states[StateType.IDLE].update()
        self.current_step = 0
        self.controller.get_logger().info("进入恢复状态，平滑过渡到静止站立")

    def update(self):
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

class ErrorState(State):
    def __init__(self, controller: 'StateMachineController'):
        super().__init__(controller)
        
    def enter(self):
        self.controller.get_logger().info("进入错误状态")
        self.controller.publish_kp_value(0.0)
        self.controller.publish_joint_cmd(np.zeros(9))
        self.controller.back_state = False
        self.controller.flip_io_leg = [False, False]
        
        
    def update(self) -> Dict[int, Tuple[float, float]]:
        return None
        
    def can_transition_to(self, new_state: 'StateType') -> bool:
        if new_state == StateType.RECOVERY:
            return True
        return False
        
    def is_finished(self) -> bool:
        return False

# ------------------- 状态机控制器 -------------------
class StateMachineController(Node):
    def __init__(self, params: Params):
        super().__init__('state_machine_controller')
        self.ik = LegKinematics()
        self.params = params
        self.leg_length = params.leg_length
        self.pitch_angle = params.pitch_angle
        self.back_state = False # False: 正常状态, True: 翻转状态
        self.flip_io_leg = [False, False]  # False: 相位正常状态, True: 相位反转状态； 0：前腿， 1：后腿
        self.cmd_vel = [0.0, 0.0]
        self.last_joint_cmd = np.zeros(9)
        self.joint_position = np.zeros(9)
        self.joint_velocity = np.zeros(9)
        self.joint_effort = np.zeros(9)
        self.error_time = 0
        self.last_vel_not_change_count=0
        self.last_first_joint_velocity = 0.0
        self.kp_value = 35.0
        self.kd_value = 0.5

        # 初始化状态
        self.states = {
            StateType.IDLE: IdleState(self),
            StateType.WALK: WalkState(self),
            StateType.FLIP: FlipState(self),
            StateType.JUMP: JumpState(self),
            StateType.ERROR: ErrorState(self),
            StateType.RECOVERY: RecoveryState(self)
        }
        
        # ROS接口
        self.joint_pub = self.create_publisher(JointState, '/action', 10)
        self.kp_pub = self.create_publisher(JointState, '/kp_cmd', 10)
        self.kd_pub = self.create_publisher(JointState, '/kd_cmd', 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.state_sub = self.create_subscription(Int8, '/state', self.state_callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_state', self.joint_state_callback, 10)
        self.params_sub = self.create_subscription(Float32MultiArray, '/params', self.params_callback, 10)
        # self.orientation_sub = self.create_subscription(Quaternion, '/orient', self.orientation_callback, 10)
        self.current_state = self.states[StateType.IDLE]
        self.current_state.enter()
        self.publish_kp_value(self.kp_value)
        self.publish_kd_value(self.kd_value)
        self.get_logger().info(f"kp_value: {self.kp_value}, kd_value: {self.kd_value}")
        self.timer = self.create_timer(0.01, self.control_loop)
        self.get_logger().info("状态机控制器已启动")
    
    def cmd_vel_callback(self, msg: Twist):
        """ 处理速度指令 """
        # 更新当前状态的速度指令
        self.cmd_vel = [msg.linear.x, msg.angular.z]
        
    def params_callback(self, msg: Float32MultiArray):
        """ 处理参数指令 """
        self.leg_length = 0.14 + msg.data[0]*0.2
        self.pitch_angle = math.radians(-60)+msg.data[1]*math.radians(-60)
        self.states[StateType.WALK].smallest_period = 0.4 - msg.data[2]*0.3
        self.states[StateType.WALK].stride = 0.1 + msg.data[3]*0.1
        self.states[StateType.WALK].z_swing = msg.data[4]*0.06

    def state_callback(self, msg: Int8):
        """ 处理状态切换 """
        # 检查是否需要切换到跳跃状态
        if msg.data == 1:  # 空翻触发
            self._transition_to(StateType.FLIP)
        elif msg.data == 2:  # 前跳触发
            self._transition_to(StateType.JUMP)
        elif msg.data == 3:  # 恢复触发
            self._transition_to(StateType.RECOVERY)

    def joint_state_callback(self, msg: JointState):
        """ 处理关节状态 """
        self.joint_position = np.array(msg.position)
        self.joint_velocity = np.array(msg.velocity)
        self.joint_effort = np.array(msg.effort)
        if self.last_first_joint_velocity == self.joint_velocity[0]:
            self.last_vel_not_change_count+=1
            if self.last_vel_not_change_count >10:
                self.get_logger().info("检测到关节速度停止，程序即将退出")
                # 停止所有发布者
                self.joint_pub.destroy()
                self.kp_pub.destroy()
                # 强制退出程序
                import sys
                sys.exit(0)
        else:
            self.last_vel_not_change_count=0
        self.last_first_joint_velocity = self.joint_velocity[0]

    def check_joint(self):

        if np.max(np.abs(self.joint_position - self.last_joint_cmd)) < np.pi*0.1 or np.min(np.abs(self.joint_velocity)) > 0.1:
            self.error_time = 0
        else:
            self.get_logger().info(f"超出1{np.max(np.abs(self.joint_position-self.last_joint_cmd))}")
            self.error_time += 1
        if self.error_time > 100:
            self.get_logger().info("检测到堵转，进入错误状态")
            self._transition_to(StateType.ERROR)


    # def orientation_callback(self, msg: Quaternion):
    #     """ 处理姿态信息 """
    #     x = msg.x
    #     y = msg.y
    #     z = msg.z
    #     w = msg.w
    #     # 计算当前姿态的俯仰角
    #     sinr_cosp = 2 * (w * x + y * z)
    #     cosr_cosp = 1 - 2 * (x * x + y * y)
    #     roll = math.atan2(sinr_cosp, cosr_cosp)
    #     roll_deg = math.degrees(roll)
    #     if roll_deg > 0:
    #         self.back_state = False
    #     else:
    #         self.back_state = True
    

    def publish_kp_value(self, kp_value):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.effort = [kp_value] * 9
        msg.effort[4] = 400.0
        self.kp_pub.publish(msg)
    def publish_kd_value(self, kd_value):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.effort = [kd_value] * 9
        msg.effort[4] = 8.0
        self.kd_pub.publish(msg)
        

    def _transition_to(self, new_state_type: StateType):
        """ 切换到新状态 """
        if self.current_state.can_transition_to(new_state_type):
            self.current_state = self.states[new_state_type]
            self.current_state.enter()
            self.get_logger().info(f"切换到状态: {new_state_type.value}")

    def compute_joint_cmd_from_leg_targets(self, leg_targets):
        # 复制自control_loop的IK流程，返回9维关节角度数组
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
            fl_angle += math.pi
            fr_angle += math.pi
            rl_angle -= math.pi
            rr_angle -= math.pi
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
        joint_cmd = np.zeros(9)
        joint_cmd[0] = -FL_theta1 ;joint_cmd[1] = FL_theta4 + math.pi
        joint_cmd[2] = FR_theta1 ;joint_cmd[3] = -FR_theta4 - math.pi
        joint_cmd[5] = -RL_theta4 - math.pi; joint_cmd[6] = RL_theta1
        joint_cmd[7] = RR_theta4 + math.pi; joint_cmd[8] = -RR_theta1
        return joint_cmd


    def publish_joint_cmd(self, cmd):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = cmd.tolist()
        self.joint_pub.publish(msg)

    def control_loop(self):
        """ 主控制循环 """
        # 更新当前状态
        time_start = time.time()
        if isinstance(self.current_state, ErrorState):
            return

        if self.cmd_vel[0] != 0 or self.cmd_vel[1] != 0:
            self._transition_to(StateType.WALK)

        cmd = self.current_state.update()
        # 直接获取四个腿的数据
        # 发布指令
        self.publish_joint_cmd(cmd)
        self.last_joint_cmd = cmd.copy()

        if self.current_state.is_finished():  # 状态完成
            self.get_logger().info(f"finished, back_state: {self.back_state}, flip_io_leg: {self.flip_io_leg}")
            self._transition_to(StateType.IDLE)
            return
        self.check_joint()
        time_end = time.time()
        self.get_logger().info(f"控制周期: {time_end - time_start}秒", throttle_duration_sec=5)

# ------------------- 主函数 -------------------
def main(args=None):
    rclpy.init(args=args)
    controller = StateMachineController(Params())
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("控制器安全停止")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 