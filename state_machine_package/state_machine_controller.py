#!/usr/bin/env python3
"""
状态机控制器主文件
重构后的状态机控制器，使用模块化设计
"""

import rclpy
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Quaternion
from std_msgs.msg import Float32, Bool, Int8
from std_msgs.msg import Float32MultiArray
import math
import numpy as np
import time

# 导入自定义模块
from constants import JOINT_NAMES, Leg, Params, StateType
from kinematics import LegKinematics
from states import (
    IdleState, WalkState, FlipState, JumpState, StairJumpState,
    RecoveryState, ErrorState
)
from states.navigation import Navigation_straight_line

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
        self.pitch, self.roll, self.yaw = 0, 0, 0
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
            StateType.STAIR_JUMP: StairJumpState(self),
            StateType.ERROR: ErrorState(self),
            StateType.RECOVERY: RecoveryState(self),
            StateType.NAVIGATION: Navigation_straight_line(self)
        }
        
        # ROS接口
        self.joint_pub = self.create_publisher(JointState, '/action', 10)
        self.kp_pub = self.create_publisher(JointState, '/kp_cmd', 10)
        self.kd_pub = self.create_publisher(JointState, '/kd_cmd', 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.state_sub = self.create_subscription(Int8, '/state', self.state_callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_state', self.joint_state_callback, 10)
        self.params_sub = self.create_subscription(Float32MultiArray, '/params', self.params_callback, 10)
        self.orient = self.create_subscription(Quaternion, 'orient', self.orient_callback, 10)

        self.current_state = self.states[StateType.IDLE]
        self.current_state.enter()
        self.publish_kp_value(self.kp_value)
        self.publish_kd_value(self.kd_value)
        self.get_logger().info(f"kp_value: {self.kp_value}, kd_value: {self.kd_value}")
        self.timer = self.create_timer(0.01, self.control_loop)
        self.get_logger().info("状态机控制器已启动")
    
    def orient_callback(self, msg):
        self.pitch = euler_from_quaternion([msg.x, msg.y, msg.z, msg.w])[0]
        self.roll = euler_from_quaternion([msg.x, msg.y, msg.z, msg.w])[1]
        self.yaw = euler_from_quaternion([msg.x, msg.y, msg.z, msg.w])[2]
        # self.get_logger().info(f"pitch:{self.pitch},roll:{self.roll},yaw:{self.yaw}")

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
        self.states[StateType.WALK].z_swing = 0.02 + msg.data[4]*0.09
       
    def state_callback(self, msg: Int8):
        """ 处理状态切换 """
        # 检查是否需要切换到跳跃状态
        if msg.data == 1:  # 空翻触发
            self._transition_to(StateType.FLIP)
        elif msg.data == 2:  # 前跳触发
            self._transition_to(StateType.JUMP)
        elif msg.data == 5:  # 台阶跳跃触发
            self._transition_to(StateType.STAIR_JUMP)
        elif msg.data == 3:  # 恢复触发
            self._transition_to(StateType.RECOVERY)
        elif msg.data == 4:  # 导航触发
            self._transition_to(StateType.NAVIGATION)

    def joint_state_callback(self, msg: JointState):
        """ 处理关节状态 """
        self.joint_position = np.array(msg.position)
        self.joint_velocity = np.array(msg.velocity)
        self.joint_effort = np.array(msg.effort)
        if self.last_first_joint_velocity == self.joint_velocity[0]:
            self.last_vel_not_change_count+=1
            # if self.last_vel_not_change_count >10:
            #     self.get_logger().info("检测到关节速度停止，程序即将退出")
            #     # 停止所有发布者
            #     self.joint_pub.destroy()
            #     self.kp_pub.destroy()
            #     # 强制退出程序
            #     import sys
            #     sys.exit(0)
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
        # self.check_joint()
        time_end = time.time()
        self.get_logger().info(f"控制周期: {time_end - time_start}秒", throttle_duration_sec=5) 