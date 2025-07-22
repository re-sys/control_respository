#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math
import numpy as np
from abc import ABC, abstractmethod
from typing import Dict, List, Tuple, Optional
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

class StateType(Enum):
    IDLE = "idle"           # 静止状态
    TROT = "trot"           # 对角步态
    FLIP = "flip"           # 空翻状态
    JUMP = "jump"           # 前跳状态

# ------------------- 逆运动学核心 -------------------
class LegKinematics:
    def __init__(self, l3=0.2, l4=0.1):
        self.l3 = l3; self.l4 = l4  # 大腿/小腿长度

    def inverse_kinematics(self, l0: float, theta: float) -> Optional[Tuple[float, float]]:
        """ 计算关节角度 (极坐标版本) """
        # 工作空间检查
        if abs(self.l3 - self.l4) > l0 or l0 > (self.l3 + self.l4):
            return None
        theta_inside = math.acos((self.l4**2 + l0**2 - self.l3**2) / (2*self.l4*l0))
        return theta + theta_inside, theta - theta_inside

# ------------------- 状态基类 -------------------
class State(ABC):
    def __init__(self, controller: 'StateMachineController'):
        self.controller = controller
        self.ik = LegKinematics()
        
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
        self.leg_length = 0.17
        self.pitch_angle = 0.0
        
    def enter(self):
        self.controller.get_logger().info("进入静止状态")
        
    def update(self) -> Dict[int, Tuple[float, float]]:
        angle = self.pitch_angle - math.pi/2
        return {
            Leg.FL: (self.leg_length, angle),
            Leg.FR: (self.leg_length, angle),
            Leg.RL: (self.leg_length, angle),
            Leg.RR: (self.leg_length, angle)
        }
        
    def can_transition_to(self, new_state: StateType) -> bool:
        return True  # 静止状态可以切换到任何状态
        
    def is_finished(self) -> bool:
        return False  # 静止状态永不完成

# ------------------- 对角步态状态 -------------------
class TrotState(State):
    def __init__(self, controller: 'StateMachineController'):
        super().__init__(controller)
        self.current_phase = 0.0
        self.cmd_vel = [0.0, 0.0]
        self.leg_length = 0.17
        
        # 步态参数
        self.z_swing = 0.04
        self.stride = 0.1
        self.max_turn_offset = 0.04
        self.phase_offsets = [0.0, 0.5, 0.5, 0.0]
        self.largest_period = 1.0
        self.smallest_period = 0.2
        self.dt = 0.01
        
    def enter(self):
        self.controller.get_logger().info("进入对角步态状态")
        
    def update(self) -> Dict[int, Tuple[float, float]]:
        # 更新相位
        period = self.largest_period + (self.smallest_period - self.largest_period) * min(math.hypot(self.cmd_vel[0], self.cmd_vel[1]), 1)
        phase_step = self.dt / period
        self.current_phase = (self.current_phase + phase_step) % 1.0
        
        # 生成各腿轨迹
        leg_targets = {}
        for leg_id in [Leg.FL, Leg.FR, Leg.RL, Leg.RR]:
            leg_targets[leg_id] = self._get_foot_target(leg_id)
            
        return leg_targets
        
    def _get_foot_target(self, leg_id: int) -> Tuple[float, float]:
        """ 生成单腿轨迹 """
        phase = (self.current_phase + self.phase_offsets[leg_id]) % 1.0
        abs_vx = abs(self.cmd_vel[0])
        
        stride = self.stride
        if self.cmd_vel[1] != 0:
            angular_stride = self.cmd_vel[1] * self.max_turn_offset
            if leg_id in [Leg.FR, Leg.RR]:  # 右腿
                stride = self.stride * abs_vx + angular_stride
            else:  # 左腿
                stride = self.stride * abs_vx - angular_stride
                
        stride *= 1 if self.cmd_vel[0] >= 0 else -1
        half_stride = stride * 0.5
        angle = -math.pi/2
        z_base = self.leg_length * math.sin(angle)
        x_base = self.leg_length * math.cos(angle)
        
        if phase < 0.5:  # 摆动相
            progress = phase * 2
            theta = 2.0 * math.pi * progress
            z = z_base + self.z_swing * (1-math.cos(theta)) * 0.5
            x = x_base - half_stride + stride * (progress - math.sin(theta)/(2.0 * math.pi))
        else:  # 支撑相
            progress = (phase - 0.5) * 2
            z = z_base
            x = x_base + half_stride - progress * stride
            
        l = math.hypot(x, z)
        theta = math.atan2(z, x)
        return l, theta
        
    def can_transition_to(self, new_state: StateType) -> bool:
        return True  # 步态状态可以切换到任何状态
        
    def is_finished(self) -> bool:
        return False  # 步态状态永不完成

# ------------------- 空翻状态 -------------------
class FlipState(State):
    def __init__(self, controller: 'StateMachineController'):
        super().__init__(controller)
        self.trajectory = self._generate_flip_trajectory()
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
        angle_phase2_front = math.radians(20)
        angle_phase3_back = math.radians(-225)
        angle_phase4_front = math.radians(45)
        
        # 长度定义
        leg_start_length = 0.17
        leg_length_prepare = 0.12
        leg_length_extend = 0.29
        leg_end_length = 0.14
        
        # 时间参数
        t_phase0 = 0.6
        t_phase1 = 0.5
        t_phase2 = 0.6
        t_phase3 = 0.4
        t_phase4 = 0.4
        
        # 阶段0：下蹲
        for i in range(int(t_phase0 * 100)):
            progress = i / (t_phase0 * 100)
            length = leg_start_length + (leg_length_prepare - leg_start_length) * progress
            trajectory.append([(length, angle_phase0)] * 4)
        
        # 阶段1：后腿蹬伸+前腿旋转
        for i in range(int(t_phase1 * 100)):
            progress = i / (t_phase1 * 100)
            front_angle = angle_phase0 + (angle_phase1 - angle_phase0) * progress
            trajectory.append([
                (leg_length_prepare, front_angle),  # FL
                (leg_length_prepare, front_angle),  # FR
                (leg_length_extend, angle_phase0),  # RL
                (leg_length_extend, angle_phase0)   # RR
            ])
        
        # 阶段2：后腿收腿+前腿继续旋转
        for i in range(int(t_phase2 * 100)):
            progress = i / (t_phase2 * 100)
            front_angle = angle_phase1 + (angle_phase2_front - angle_phase1) * progress
            back_length = leg_length_extend + (leg_end_length - leg_length_extend) * progress
            trajectory.append([
                (leg_length_prepare, front_angle),  # FL
                (leg_length_prepare, front_angle),  # FR
                (back_length, angle_phase0),       # RL
                (back_length, angle_phase0)        # RR
            ])
        
        # 阶段3：前腿蹬伸+后腿旋转
        for i in range(int(t_phase3 * 100)):
            progress = i / (t_phase3 * 100)
            back_angle = angle_phase0 + (angle_phase3_back - angle_phase0) * progress
            trajectory.append([
                (leg_length_extend, angle_phase2_front),  # FL
                (leg_length_extend, angle_phase2_front),  # FR
                (leg_end_length, back_angle),            # RL
                (leg_end_length, back_angle)             # RR
            ])
        
        # 阶段4：前腿旋转
        for i in range(int(t_phase4 * 100)):
            progress = i / (t_phase4 * 100)
            front_angle = angle_phase2_front + (angle_phase4_front - angle_phase2_front) * progress
            trajectory.append([
                (leg_end_length, front_angle),      # FL
                (leg_end_length, front_angle),      # FR
                (leg_end_length, angle_phase3_back), # RL
                (leg_end_length, angle_phase3_back)  # RR
            ])
        
        return trajectory
        
    def update(self) -> Dict[int, Tuple[float, float]]:
        if self.step_idx >= len(self.trajectory):
            return {}
            
        leg_polar_coords = self.trajectory[self.step_idx]
        self.step_idx += 1
        
        return {
            Leg.FL: leg_polar_coords[0],
            Leg.FR: leg_polar_coords[1],
            Leg.RL: leg_polar_coords[2],
            Leg.RR: leg_polar_coords[3]
        }
        
    def can_transition_to(self, new_state: StateType) -> bool:
        return self.is_finished()  # 只有完成才能切换
        
    def is_finished(self) -> bool:
        return self.step_idx >= len(self.trajectory)

# ------------------- 前跳状态 -------------------
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
        z_start = -0.18
        z_prepare = -0.12
        z_jump = -0.29
        z_land = -0.18
        angle = math.radians(20)
        
        # 计算前倾和后仰的x偏移量
        x_forward = -abs(z_prepare) * math.sin(angle)
        z_forward = -abs(z_prepare) * math.cos(angle)
        x_forward_long = x_forward * (abs(z_jump) / abs(z_prepare))
        z_forward_long = z_forward * (abs(z_jump) / abs(z_prepare))
        x_backward = abs(z_prepare) * math.sin(angle)
        z_backward = -abs(z_prepare) * math.cos(angle)
        
        # 时间参数
        t_prepare = 0.2
        t_jump = 0.15
        t_short = 0.1
        t_land = 0.1
        t_recover = 0.6
        
        # 1. 准备阶段
        for i in range(int(t_prepare * 100)):
            progress = i / (t_prepare * 100)
            x = progress * x_forward
            z = z_start + (z_forward - z_start) * progress
            trajectory.append([(x, z)] * 4)
        
        # 2. 起跳阶段
        for i in range(int(t_jump * 100)):
            trajectory.append([(x_forward_long, z_forward_long)] * 4)
        
        # 3. 收腿阶段
        for i in range(int(t_short * 100)):
            trajectory.append([(x_forward, z_forward)] * 4)
        
        # 4. 落地阶段
        for i in range(int(t_land * 100)):
            progress = i / (t_land * 100)
            x = x_forward + (x_backward - x_forward) * progress
            z = z_forward + (z_backward - z_forward) * progress
            trajectory.append([(x, z)] * 4)
            
        # 5. 恢复阶段
        for i in range(int(t_recover * 100)):
            progress = i / (t_recover * 100)
            x = x_backward * (1 - progress)
            z = z_backward + (z_land - z_backward) * progress
            trajectory.append([(x, z)] * 4)
        
        return trajectory
        
    def update(self) -> Dict[int, Tuple[float, float]]:
        if self.step_idx >= len(self.trajectory):
            return {}
            
        leg_xz = self.trajectory[self.step_idx]
        self.step_idx += 1
        
        # 转换为极坐标
        result = {}
        for leg_id, (x, z) in enumerate(leg_xz):
            l = math.hypot(x, z)
            theta = math.atan2(z, x)
            result[leg_id] = (l, theta)
            
        return result
        
    def can_transition_to(self, new_state: StateType) -> bool:
        return self.is_finished()  # 只有完成才能切换
        
    def is_finished(self) -> bool:
        return self.step_idx >= len(self.trajectory)

# ------------------- 状态机控制器 -------------------
class StateMachineController(Node):
    def __init__(self):
        super().__init__('state_machine_controller')
        
        # 初始化状态
        self.states = {
            StateType.IDLE: IdleState(self),
            StateType.TROT: TrotState(self),
            StateType.FLIP: FlipState(self),
            StateType.JUMP: JumpState(self)
        }
        self.current_state = self.states[StateType.IDLE]
        self.current_state.enter()
        
        # ROS接口
        self.joint_pub = self.create_publisher(JointState, '/action', 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.height_sub = self.create_subscription(Float32, '/height', self.height_callback, 10)
        self.timer = self.create_timer(0.01, self.control_loop)
        
        self.get_logger().info("状态机控制器已启动")
        
    def cmd_vel_callback(self, msg: Twist):
        """ 处理速度指令 """
        # 更新当前状态的速度指令
        if isinstance(self.current_state, TrotState):
            self.current_state.cmd_vel = [msg.linear.x, msg.angular.z]
            
        # 状态切换逻辑
        self._handle_state_transition(msg)
        
    def height_callback(self, msg: Float32):
        """ 处理高度指令 """
        height = msg.data
        leg_length = 0.16 + height * (0.27 - 0.16)
        
        # 更新各状态的腿长
        for state in self.states.values():
            if hasattr(state, 'leg_length'):
                state.leg_length = leg_length
                
    def _handle_state_transition(self, msg: Twist):
        """ 处理状态切换 """
        # 检查是否需要切换到跳跃状态
        if msg.angular.y > 0.5:  # 空翻触发
            self._transition_to(StateType.FLIP)
        elif msg.angular.y < -0.5:  # 前跳触发
            self._transition_to(StateType.JUMP)
        elif msg.linear.x != 0 or msg.angular.z != 0:  # 有速度指令
            self._transition_to(StateType.TROT)
        else:  # 无速度指令
            self._transition_to(StateType.IDLE)
            
    def _transition_to(self, new_state_type: StateType):
        """ 切换到新状态 """
        if self.current_state.can_transition_to(new_state_type):
            self.current_state = self.states[new_state_type]
            self.current_state.enter()
            self.get_logger().info(f"切换到状态: {new_state_type.value}")
            
    def control_loop(self):
        """ 主控制循环 """
        # 更新当前状态
        leg_targets = self.current_state.update()
        
        if not leg_targets:  # 状态完成
            self._transition_to(StateType.IDLE)
            return
            
        # 直接获取四个腿的数据
        fl_length, fl_angle = leg_targets[Leg.FL]
        fr_length, fr_angle = leg_targets[Leg.FR]
        rl_length, rl_angle = leg_targets[Leg.RL]
        rr_length, rr_angle = leg_targets[Leg.RR]
        
        # 分别计算IK
        FL_theta1, FL_theta4 = self.ik.inverse_kinematics(fl_length, fl_angle)
        FR_theta1, FR_theta4 = self.ik.inverse_kinematics(fr_length, fr_angle)
        RL_theta1, RL_theta4 = self.ik.inverse_kinematics(rl_length, rl_angle)
        RR_theta1, RR_theta4 = self.ik.inverse_kinematics(rr_length, rr_angle)
        
            
        # 直接赋值给cmd数组
        joint_cmd = [0.0] * 9
        joint_cmd[0] = -FL_theta1 ;joint_cmd[1] = FL_theta4 + math.pi  # FL_thigh_joint_o
        joint_cmd[2] = FR_theta1 ;joint_cmd[3] = -FR_theta4 - math.pi  # FR_thigh_joint_o
        joint_cmd[5] = -RL_theta4 - math.pi  ;joint_cmd[6] = RL_theta1 ;
        joint_cmd[7] = RR_theta4 + math.pi  ;joint_cmd[8] = -RR_theta1  # RR_thigh_joint_o
        
        # 发布指令
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = joint_cmd
        self.joint_pub.publish(msg)

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