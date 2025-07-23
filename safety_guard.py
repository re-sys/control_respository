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
from std_srvs.srv import Trigger

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


# ------------------- 逆运动学核心 -------------------
class LegKinematics:
    def __init__(self, l3=0.2, l4=0.16):
        self.l3 = l3; self.l4 = l4  # 大腿/小腿长度

    def inverse_kinematics(self, l0: float, theta: float, io_flip: bool) -> Optional[Tuple[float, float]]:
        """ 计算关节角度 (极坐标版本) """
        theta_inside = math.acos((self.l4**2 + l0**2 - self.l3**2) / (2*self.l4*l0))
        
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
        return {
            Leg.FL: (self.controller.leg_length, angle),
            Leg.FR: (self.controller.leg_length, angle),
            Leg.RL: (self.controller.leg_length, angle),
            Leg.RR: (self.controller.leg_length, angle)
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
        period = self.largest_period + (self.smallest_period - self.largest_period) * min(math.hypot(self.controller.cmd_vel[0], self.controller.cmd_vel[1]), 1)
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
        if self.controller.cmd_vel[1] != 0:
            angular_stride = self.controller.cmd_vel[1] * self.max_turn_offset
            if leg_id in [Leg.FR, Leg.RR]:  # 右腿
                stride = self.stride * abs_vx + angular_stride
            else:  # 左腿
                stride = self.stride * abs_vx - angular_stride
                
        stride *= 1 if self.controller.cmd_vel[0] >= 0 else -1
        half_stride = stride * 0.5
        angle = -math.pi/2
        z_base = self.controller.leg_length * math.sin(angle)
        x_base = self.controller.leg_length * math.cos(angle)
        
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
        
        # 长度定义
        leg_start_length = self.controller.leg_length
        leg_length_prepare = 0.14
        leg_length_extend = 0.36
        leg_end_length = 0.14
        
        # 时间参数
        t_phase0 = 0.6        # 下蹲阶段
        t_phase1 = 0.3       # 后腿蹬伸+前腿旋转30°
        t_phase2 = 0.35     # 后腿收腿+前腿旋转90°
        t_phase3 = 0.2        # 前腿蹬伸+后腿旋转
        t_phase4 = 0.4        # 前腿旋转
        
        # 阶段0：下蹲
        for i in range(int(t_phase0 * 100)):
            progress = i / (t_phase0 * 100)
            length = leg_start_length + (leg_length_prepare - leg_start_length) * progress
            trajectory.append([(length, angle_phase0)] * 2)
        
        # 阶段1：后腿蹬伸+前腿旋转
        for i in range(int(t_phase1 * 100)):
            progress = i / (t_phase1 * 100)
            front_angle = angle_phase0 + (angle_phase1 - angle_phase0) * progress
            trajectory.append([
                (leg_length_prepare, front_angle),  # Front
                (leg_length_extend, angle_phase0),  # Rear
            ])
        
        # 阶段2：后腿收腿+前腿继续旋转
        for i in range(int(t_phase2 * 100)):
            progress = i / (t_phase2 * 100)
            front_angle = angle_phase1 + (angle_phase2_front - angle_phase1) * progress
            back_angle = angle_phase0 + (angle_phase2_back - angle_phase0) * progress
            back_length = leg_length_extend + (leg_end_length - leg_length_extend) * progress
            trajectory.append([
                (leg_length_prepare, front_angle),  # Front
                (back_length, back_angle),       # Rear
            ])
        
        # 阶段3：前腿蹬伸+后腿旋转
        for i in range(int(t_phase3 * 100)):
            progress = i / (t_phase3 * 100)
            back_angle = angle_phase2_back + (angle_phase3_back - angle_phase2_back) * progress
            trajectory.append([
                (leg_length_extend, angle_phase2_front),  # Front
                (leg_end_length, back_angle)            # Rear
            ])
        
        # 阶段4：前腿旋转
        for i in range(int(t_phase4 * 100)):
            progress = i / (t_phase4 * 100)
            front_angle = angle_phase2_front + (angle_phase4_front - angle_phase2_front) * progress
            trajectory.append([
                (leg_end_length, front_angle),      # Front
                (leg_end_length, angle_phase3_back)  # Rear
            ])
        
        return trajectory
        
    def update(self) -> Dict[int, Tuple[float, float]]:
        if self.step_idx >= self.trajectory_length:
            return {}
            
        leg_polar_coords = self.trajectory[self.step_idx]
        self.step_idx += 1
        if self.step_idx == (self.t_phase0+self.t_phase1) * 100:
            if not self.back_state:
                self.controller.flip_io_leg[1] = False if self.controller.flip_io_leg[1] else True
            else:
                self.controller.flip_io_leg[0] = False if self.controller.flip_io_leg[0] else True

        if self.step_idx == (self.t_phase0+self.t_phase1+self.t_phase2+self.t_phase3) * 100:
            if not self.back_state:
                self.controller.flip_io_leg[0] = False if self.controller.flip_io_leg[0] else True
            else:
                self.controller.flip_io_leg[1] = False if self.controller.flip_io_leg[1] else True

        if self.step_idx == self.trajectory_length:
            self.controller.back_state = not self.controller.back_state

        return {
            Leg.FL: leg_polar_coords[0],
            Leg.FR: leg_polar_coords[0],
            Leg.RL: leg_polar_coords[1],
            Leg.RR: leg_polar_coords[1]
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
        leg_start_length = 0.17
        leg_length_prepare = 0.14
        leg_length_extend = 0.36
        leg_end_length = 0.14
        
        angle_land = 2*math.radians(-90)-self.controller.pitch_angle
        # 计算前倾和后仰的x偏移量
        
        # 时间参数
        t_prepare = 0.2
        t_jump = 0.15
        t_short = 0.1
        t_land = 0.1
        t_recover = 0.6
        
        # 1. 准备阶段
        for i in range(int(t_prepare * 100)):
            progress = i / (t_prepare * 100)
            l = leg_start_length + (leg_length_prepare - leg_start_length) * progress
            trajectory.append([(l, self.controller.pitch_angle)] * 4)
        
        # 2. 起跳阶段
        for i in range(int(t_jump * 100)):
            trajectory.append([(leg_length_extend, self.controller.pitch_angle)] * 4)
        
        # 3. 收腿阶段
        for i in range(int(t_short * 100)):
            trajectory.append([(leg_length_prepare, self.controller.pitch_angle)] * 4)
        
        # 4. 落地阶段
        for i in range(int(t_land * 100)):
            progress = i / (t_land * 100)
            l = leg_length_prepare + (leg_end_length - leg_length_prepare) * progress
            angle = self.controller.pitch_angle + (angle_land - self.controller.pitch_angle) * progress
            trajectory.append([(l, angle)] * 4)
            
        # 5. 恢复阶段
        for i in range(int(t_recover * 100)):
            progress = i / (t_recover * 100)
            l = leg_end_length + (leg_start_length - leg_end_length) * progress
            angle = angle_land + (self.controller.pitch_angle - angle_land) * progress
            trajectory.append([(l, angle)] * 4)
        
        return trajectory
        
    def update(self) -> Dict[int, Tuple[float, float]]:
        if self.step_idx >= len(self.trajectory):
            return {}
            
        leg_xz = self.trajectory[self.step_idx]
        self.step_idx += 1
        
        # 转换为极坐标
        result = {}
        for leg_id, (l, angle) in enumerate(leg_xz):
            result[leg_id] = (l, angle)
            
        return result
        
    def can_transition_to(self, new_state: StateType) -> bool:
        return self.is_finished()  # 只有完成才能切换
        
    def is_finished(self) -> bool:
        return self.step_idx >= len(self.trajectory)

# ------------------- 错误状态 -------------------
class ErrorState(State):
    def __init__(self, controller: 'StateMachineController'):
        super().__init__(controller)
        self.locked_joint_cmd = None

    def enter(self):
        # 锁定当前关节角度
        self.locked_joint_cmd = self.controller.last_joint_cmd.copy()
        self.controller.get_logger().error("进入ERROR状态，锁定当前关节角度，僵尸站立！")

    def update(self) -> Dict[int, Tuple[float, float]]:
        # 返回锁定的关节角度（直接返回None，主循环特殊处理）
        return None

    def can_transition_to(self, new_state: StateType) -> bool:
        return False  # 错误状态不可切换

    def is_finished(self) -> bool:
        return False  # 永远不完成

# ------------------- 恢复状态 -------------------
class RecoveryState(State):
    def __init__(self, controller: 'StateMachineController'):
        super().__init__(controller)
        self.start_cmd = None
        self.target_cmd = None
        self.steps = 100  # 平滑步数
        self.current_step = 0

    def enter(self):
        self.start_cmd = self.controller.last_joint_cmd.copy()
        # 生成Idle目标关节角度
        idle_targets = self.controller.states[StateType.IDLE].update()
        self.target_cmd = self.controller.compute_joint_cmd_from_leg_targets(idle_targets)
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

# ------------------- 状态机控制器 -------------------
class StateMachineController(Node):
    def __init__(self):
        super().__init__('state_machine_controller')
        
        # 初始化状态
        self.states = {
            StateType.IDLE: IdleState(self),
            StateType.TROT: TrotState(self),
            StateType.FLIP: FlipState(self),
            StateType.JUMP: JumpState(self),
            StateType.ERROR: ErrorState(self),
            StateType.RECOVERY: RecoveryState(self)  # 新增
        }
        self.current_state = self.states[StateType.IDLE]
        self.current_state.enter()
        
        # ROS接口
        self.joint_pub = self.create_publisher(JointState, '/action', 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.state_sub = self.create_subscription(Int8, '/state', self.state_callback, 10)
        self.params_sub = self.create_subscription(Float32MultiArray, '/params', self.params_callback, 10)
        self.timer = self.create_timer(0.01, self.control_loop)
        self.recover_srv = self.create_service(Trigger, '/recover', self.recover_callback)

        self.ik = LegKinematics()
        self.leg_length = 0.17
        self.pitch_angle = math.radians(-90)
        self.back_state = False
        self.flip_io_leg = [False, False] # front, rear
        self.cmd_vel = [0.0, 0.0]
        self.last_joint_cmd = [0.0] * 9

        self.get_logger().info("状态机控制器已启动")
        
    def cmd_vel_callback(self, msg: Twist):
        """ 处理速度指令 """
        # 更新当前状态的速度指令
        self.cmd_vel = [msg.linear.x, msg.angular.z]
        
    def params_callback(self, msg: Float32MultiArray):
        """ 处理参数指令 """
        self.leg_length = msg.data[0]
        self.pitch_angle = math.radians(-60)+msg.data[1]*math.radians(-60)
                
    def state_callback(self, msg: Int8):
        """ 处理状态切换 """
        # 检查是否需要切换到跳跃状态
        if msg.data == 1:  # 空翻触发
            self._transition_to(StateType.FLIP)
        elif msg.data == 2:  # 前跳触发
            self._transition_to(StateType.JUMP)

            
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

    def recover_callback(self, request, response):
        if self.current_state == self.states[StateType.ERROR]:
            self._transition_to(StateType.RECOVERY)
            response.success = True
            response.message = "已开始恢复"
        else:
            response.success = False
            response.message = "当前不是ERROR状态，无法恢复"
        return response

    def control_loop(self):
        """ 主控制循环 """
        # 如果处于ERROR状态，持续发布锁定角度
        if self.current_state == self.states[StateType.ERROR]:
            if self.current_state.locked_joint_cmd is not None:
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = JOINT_NAMES
                msg.position = self.current_state.locked_joint_cmd
                self.joint_pub.publish(msg)
            return
        # 如果处于RECOVERY状态，平滑过渡
        if self.current_state == self.states[StateType.RECOVERY]:
            cmd = self.current_state.update()
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = JOINT_NAMES
            msg.position = cmd
            self.joint_pub.publish(msg)
            self.last_joint_cmd = cmd.copy()
            if self.current_state.is_finished():
                self._transition_to(StateType.IDLE)
            return
        # 更新当前状态
        if self.cmd_vel[0] != 0 or self.cmd_vel[1] != 0:
            self._transition_to(StateType.TROT)
        leg_targets = self.current_state.update()

        if self.current_state.is_finished():  # 状态完成
            self._transition_to(StateType.IDLE)
            return
            
        # 直接获取四个腿的数据
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
        
        # 分别计算IK
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
        
            
        # 直接赋值给cmd数组
        joint_cmd = np.zeros(9)
        joint_cmd[0] = -FL_theta1 ;joint_cmd[1] = FL_theta4 + math.pi  # FL_thigh_joint_o
        joint_cmd[2] = FR_theta1 ;joint_cmd[3] = -FR_theta4 - math.pi  # FR_thigh_joint_o
        joint_cmd[5] = -RL_theta4 - math.pi; joint_cmd[6] = RL_theta1
        joint_cmd[7] = RR_theta4 + math.pi; joint_cmd[8] = -RR_theta1  # RR_thigh_joint_o
        
        # 检查关节角度突变
        if np.max(np.abs(joint_cmd - self.last_joint_cmd)) > (np.pi / 2):
            self.get_logger().error("检测到关节角度突变 > 90°，进入ERROR状态！")
            self._transition_to(StateType.ERROR)
            return  # 不发布指令
        # 发布指令
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = joint_cmd
        self.joint_pub.publish(msg)
        self.last_joint_cmd = joint_cmd.copy()


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