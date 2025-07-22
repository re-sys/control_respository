#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
from typing import Optional

# 机械参数
UPPER_LINK_LENGTH = 0.2  # l3
LOWER_LINK_LENGTH = 0.16  # l4
leg_start_length = 0.14
leg_length_extend = 0.36
leg_end_length = 0.14
angle_phase0 = math.radians(-90)
angle_phase1 = math.radians(0)
t_phase1 = 1 # 阶段1时长
t_phase2 = 1  # 阶段2时长

JOINT_NAMES = [
    "FL_thigh_joint_i", "FL_thigh_joint_o",  # 0, 1
    "FR_thigh_joint_i", "FR_thigh_joint_o",  # 2, 3
    "waist_joint",                           # 4 (不驱动)
    "RL_thigh_joint_i", "RL_thigh_joint_o",  # 5, 6
    "RR_thigh_joint_i", "RR_thigh_joint_o"   # 7, 8
]

class Leg:
    FL = 0
    FR = 1
    RL = 2
    RR = 3

class LegKinematics:
    def __init__(self, l3=UPPER_LINK_LENGTH, l4=LOWER_LINK_LENGTH):
        self.l3 = l3
        self.l4 = l4
    def inverse_kinematics(self, l0, theta) -> Optional[tuple]:
        if abs(self.l3 - self.l4) > l0 or l0 > (self.l3 + self.l4):
            return None
        theta_inside = math.acos((self.l4**2 + l0**2 - self.l3**2) / (2 * self.l4 * l0))
        theta1 = theta + theta_inside
        theta4 = theta - theta_inside
        return theta1, theta4

def generate_rear_leg_trajectory():
    """阶段1后腿蹬伸为线性插值，阶段2收腿也为线性插值，前腿静止。"""
    traj = []
    # 阶段1：后腿蹬伸（线性插值）
    for i in range(int(t_phase1 * 100)):
        progress = i / (t_phase1 * 100)
        length = leg_start_length + (leg_length_extend - leg_start_length) * progress
        angle = angle_phase0 + (angle_phase1 - angle_phase0) * progress
        RL = (length, angle)
        RR = (length, angle)
        traj.append((RL, RR))
    # 阶段2：后腿收腿
    for i in range(int(t_phase2 * 100)):
        progress = i / (t_phase2 * 100)
        length = leg_length_extend + (leg_end_length - leg_length_extend) * progress
        angle = angle_phase1
        RL = (length, angle)
        RR = (length, angle)
        traj.append((RL, RR))
    return traj

class RearLegTestNode(Node):
    def __init__(self):
        super().__init__('rear_leg_test_node')
        self.ik = LegKinematics()
        self.pub = self.create_publisher(JointState, '/action', 10)
        self.trajectory = generate_rear_leg_trajectory()
        self.step_idx = 0
        self.timer = self.create_timer(0.01, self.publish_next_step)
        self.get_logger().info("后腿阶段1/2测试节点已启动，前腿静止")

    def publish_next_step(self):
        if self.step_idx >= len(self.trajectory):
            self.get_logger().info("测试完成！")
            self.timer.cancel()
            return
        RL, RR = self.trajectory[self.step_idx]
        self.step_idx += 1
        # 前腿静止
        fl_length, fl_angle = 0.17, math.radians(-90)
        fr_length, fr_angle = 0.17, math.radians(-90)
        # 逆解
        FL_theta1, FL_theta4 = self.ik.inverse_kinematics(RL[0], RL[1])
        FR_theta1, FR_theta4 = self.ik.inverse_kinematics(RR[0], RR[1])
        RL_theta1, RL_theta4 = self.ik.inverse_kinematics(fl_length, fl_angle)
        RR_theta1, RR_theta4 = self.ik.inverse_kinematics(fr_length, fr_angle)
        # 阶段2对调后腿theta1/theta4
        phase2_start = int(t_phase1 * 100)
        if self.step_idx > phase2_start:
            FL_theta1, FL_theta4 = FL_theta4, FL_theta1
            FR_theta1, FR_theta4 = FR_theta4, FR_theta1
        # 组装指令
        cmd = [0.0] * 9
        cmd[0] = -FL_theta1; cmd[1] = FL_theta4 + math.pi
        cmd[2] = FR_theta1;  cmd[3] = -FR_theta4 - math.pi
        # cmd[4] = 0.0 # 腰部不动
        cmd[5] = -RL_theta4 - math.pi; cmd[6] = RL_theta1
        cmd[7] = RR_theta4 + math.pi;  cmd[8] = -RR_theta1
        # 发布
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = cmd
        self.pub.publish(msg)
        self.get_logger().info(f"Step {self.step_idx}: RL_len={RL[0]:.3f}, RR_len={RR[0]:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = RearLegTestNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()