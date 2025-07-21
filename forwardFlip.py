#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time
from typing import List, Tuple, Optional

# ------------------- 硬件常量定义 -------------------
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

# ------------------- 逆运动学核心 -------------------
class LegKinematics:
    def __init__(self, l3: float = 0.2, l4: float = 0.1):
        self.l3 = l3  # 大腿长度
        self.l4 = l4  # 小腿长度
        self.last_theta = 0.0

    def inverse_kinematics(self, x: float, z: float) -> Optional[Tuple[float, float]]:
        """ 逆运动学计算 (与C++版本完全一致) """
        l0 = math.sqrt(x*x + z*z)
        
        # 工作空间检查
        if abs(self.l3 - self.l4) > l0 or l0 > (self.l3 + self.l4):
            return None

        theta_inside = math.acos((self.l4**2 + l0**2 - self.l3**2) / (2 * self.l4 * l0))
        theta = math.atan2(z, x)
        if (self.last_theta-theta) < -math.pi:
            theta = theta - 2*math.pi
        elif (self.last_theta-theta) > math.pi:
            theta = theta + 2*math.pi
        self.last_theta = theta
        
        theta1 = theta + theta_inside
        theta4 = theta - theta_inside
        return theta1, theta4

# ------------------- 跳跃轨迹生成 -------------------
def generate_jump_trajectory() -> List[List[Tuple[float, float]]]:
    """ 生成两阶段跳跃轨迹：后腿蹬伸+前腿旋转 """
    
    # 前腿旋转角度（弧度）
    angle_phase0 = math.radians(-90)
    angle_phase1 = math.radians(-60)  # 第一阶段旋转30度
    angle_phase2_front = math.radians(0)  # 第二阶段旋转到90度
    angle_phase3_back = math.radians(-225)
    angle_phase4_front = math.radians(45)
    
    # 前腿长度保持
    leg_start_length = 0.17
    leg_length_prepare = 0.12
    leg_length_extend = 0.29
    leg_end_length = 0.17

    x_back_end = leg_end_length * math.cos(angle_phase3_back)
    z_back_end = leg_end_length * math.sin(angle_phase3_back)
    
    # 时间参数（单位：秒）
    t_phase0 = 0.6        # 第一阶段下蹲时长
    t_phase1 = 0.4        # 第一阶段后腿蹬伸时长
    t_phase2 = 0.5       # 第二阶段后腿收腿，前腿旋转时长
    t_phase3 = 0.4       # 第三阶段前腿蹬伸，后腿旋转时长
    t_phase4 = 0.4       # 第四阶段前腿旋转时长
    
    # 轨迹分段生成
    trajectory = []
    # 初始阶段：下蹲到leg_length_prepare
    for i in range(int(t_phase0 * 100)):
        progress = i / (t_phase0 * 100)
        x = (leg_start_length + (leg_length_prepare - leg_start_length) * progress) * math.cos(angle_phase0)
        z = (leg_start_length + (leg_length_prepare - leg_start_length) * progress) * math.sin(angle_phase0)
        trajectory.append([
            (x, z),  # FL (前左)
            (x, z),  # FR (前右)
            (x, z),  # RL (后左)
            (x, z)   # RR (后右)
        ])
    
    # 第一阶段：后腿蹬伸 + 前腿旋转30度
    for i in range(int(t_phase1 * 100)):
        progress = i / (t_phase1 * 100)
        
        # 后腿：从z_start蹬伸到z_extend
        back_z = leg_length_extend*math.sin(angle_phase0)
        back_x = 0.0  # 后腿x位置保持为0
        
        # 前腿：保持长度，旋转30度
        front_angle = angle_phase0 + (angle_phase1 - angle_phase0) * progress
        front_x = leg_length_prepare * math.cos(front_angle)
        front_z = leg_length_prepare * math.sin(front_angle)
        
        # 四条腿的位置 [FL, FR, RL, RR]
        trajectory.append([
            (front_x, front_z),  # FL (前左)
            (front_x, front_z),  # FR (前右)
            (back_x, back_z),    # RL (后左)
            (back_x, back_z)     # RR (后右)
        ])
    
    # 第二阶段：后腿收腿 + 前腿继续旋转到90度
    for i in range(int(t_phase2 * 100)):
        progress = i / (t_phase2 * 100)
        
        # 后腿：从z_extend收腿到z_recover
        back_z = -(leg_length_extend + (leg_end_length-leg_length_extend)*progress)
        back_x = 0.0  # 后腿x位置保持为0
        
        # 前腿：从30度继续旋转到90度
        front_angle = angle_phase1 + (angle_phase2_front - angle_phase1) * progress
        front_x = leg_length_prepare * math.cos(front_angle)
        front_z = leg_length_prepare * math.sin(front_angle)
        
        # 四条腿的位置 [FL, FR, RL, RR]
        trajectory.append([
            (front_x, front_z),  # FL (前左)
            (front_x, front_z),  # FR (前右)
            (back_x, back_z),    # RL (后左)
            (back_x, back_z)     # RR (后右)
        ])
    # 第三阶段： 前腿开始蹬,后腿转个方向
    for i in range(int(t_phase3 * 100)):
        progress = i / (t_phase3 * 100)
        front_x = leg_length_extend * math.cos(angle_phase2_front)
        front_z = leg_length_extend * math.sin(angle_phase2_front)

        angle = angle_phase0 + (angle_phase3_back - angle_phase0) * progress

        back_x = leg_end_length * math.cos(angle)
        back_z = leg_end_length * math.sin(angle)

        trajectory.append([
            (front_x, front_z),  # FL (前左)
            (front_x, front_z),  # FR (前右)
            (back_x, back_z),  # RL (后左)
            (back_x, back_z)   # RR (后右)
        ])
    # 第四阶段： 前腿也换个方向,后腿保持
    for i in range(int(t_phase4 * 100)):
        progress = i / (t_phase4 * 100)
        angle=angle_phase2_front+(angle_phase4_front-angle_phase2_front)*progress
        front_x = leg_end_length * math.cos(angle)
        front_z = leg_end_length * math.sin(angle)
        back_x = x_back_end
        back_z = z_back_end
        trajectory.append([
            (front_x, front_z),  # FL (前左)
            (front_x, front_z),  # FR (前右)
            (back_x, back_z),  # RL (后左)
            (back_x, back_z)   # RR (后右)
        ])

    return trajectory

# ------------------- 主控制节点 -------------------
class JumpControlNode(Node):
    def __init__(self):
        super().__init__('jump_control_node')
        self.ik_solver1 = LegKinematics()
        self.ik_solver2 = LegKinematics()
        self.ik_solver3 = LegKinematics()
        self.ik_solver4 = LegKinematics()
        self.pub = self.create_publisher(JointState, '/action', 10)
        self.trajectory = generate_jump_trajectory()
        self.timer = self.create_timer(0.01, self.publish_next_step)  # 10Hz控制
        self.step_idx = 0
        self.get_logger().info("两阶段跳跃控制节点已启动：后腿蹬伸+前腿旋转")

    def publish_next_step(self):
        """ 按顺序发布轨迹的每一步 """
        if self.step_idx >= len(self.trajectory):
            self.get_logger().info("跳跃完成！")
            self.timer.cancel()
            return
        
        leg_xz = self.trajectory[self.step_idx]
        self.step_idx += 1
        
        # 计算并发布关节指令
        cmd = [0.0] * 9
        for leg, (x, z) in enumerate(leg_xz):
            if leg == Leg.FL:
                ik_result = self.ik_solver1.inverse_kinematics(x, z)
            elif leg == Leg.FR:
                ik_result = self.ik_solver2.inverse_kinematics(x, z)
            elif leg == Leg.RL:
                ik_result = self.ik_solver3.inverse_kinematics(x, z)
            elif leg == Leg.RR:
                ik_result = self.ik_solver4.inverse_kinematics(x, z)
            if ik_result is None:
                self.get_logger().error(f"Step {self.step_idx}: Leg {leg} IK失败")
                return
            
            theta1, theta4 = ik_result
            
            # 关节方向调整
            if leg == Leg.FL:
                cmd[0] = -theta1
                cmd[1] = theta4 + math.pi
            elif leg == Leg.FR:
                cmd[2] = theta1
                cmd[3] = -theta4 - math.pi
            elif leg == Leg.RL:
                cmd[5] = -theta4 - math.pi
                cmd[6] = theta1
            elif leg == Leg.RR:
                cmd[7] = theta4 + math.pi
                cmd[8] = -theta1
        
        # 发布指令
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = cmd
        self.pub.publish(msg)
        
        # 打印当前状态
        phase1_steps = int(0.1 * 100)  # 第一阶段步数
        if self.step_idx <= phase1_steps:
            phase = "阶段1: 后腿蹬伸+前腿旋转30°"
        else:
            phase = "阶段2: 后腿收腿+前腿旋转90°"
            
        self.get_logger().info(
            f"Step {self.step_idx}: {phase} | "
            f"前腿: x={leg_xz[Leg.FL][0]:.3f}, z={leg_xz[Leg.FL][1]:.3f} | "
            f"后腿: x={leg_xz[Leg.RL][0]:.3f}, z={leg_xz[Leg.RL][1]:.3f}",
            throttle_duration_sec=0.1)

# ------------------- 主函数 -------------------
def main(args=None):
    rclpy.init(args=args)
    node = JumpControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
