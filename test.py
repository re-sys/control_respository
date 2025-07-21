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

    def inverse_kinematics(self, x: float, z: float) -> Optional[Tuple[float, float]]:
        """ 逆运动学计算 (与C++版本完全一致) """
        l0 = math.sqrt(x*x + z*z)
        
        # 工作空间检查
        if abs(self.l3 - self.l4) > l0 or l0 > (self.l3 + self.l4):
            return None

        theta_inside = math.acos((self.l4**2 + l0**2 - self.l3**2) / (2 * self.l4 * l0))
        theta = math.atan2(z, x)
        
        theta1 = theta + theta_inside
        theta4 = theta - theta_inside
        return theta1, theta4

# ------------------- 跳跃轨迹生成 -------------------
def generate_jump_trajectory() -> List[List[Tuple[float, float]]]:
    """ 生成带30度倾角的跳跃轨迹 """
    # 定义关键参数
    z_start = -0.18       # 初始高度
    z_prepare = -0.12     # 准备姿势高度
    z_jump = -0.29        # 起跳最高点
    z_land = -0.18        # 落地缓冲高度
    angle = math.radians(20)  # 30度倾角
    
    
    # 计算前倾和后仰的x偏移量
    x_forward = -abs(z_prepare) * math.sin(angle)
    z_forward = -abs(z_prepare) * math.cos(angle)

    x_forward_long = x_forward * (abs(z_jump) / abs(z_prepare))
    z_forward_long = z_forward * (abs(z_jump) / abs(z_prepare))
    
    x_backward = abs(z_prepare) * math.sin(angle)
    z_backward = -abs(z_prepare) * math.cos(angle)
    
    # 时间参数（单位：秒）
    t_prepare = 0.2      # 准备阶段时长
    t_jump = 0.15        # 起跳阶段时长
    t_short = 0.1        # 收腿阶段时长
    t_land = 0.1         # 落地阶段时长
    t_recover = 0.6
    
    # 轨迹分段生成
    trajectory = []
    
    # 1. 准备阶段 (前倾30度)
    for i in range(int(t_prepare * 100)):
        progress = i / (t_prepare * 100)
        x = progress * x_forward
        z = z_start + (z_forward - z_start) * progress
        trajectory.append([(x, z)] * 4)
    
    # 2. 起跳阶段 (保持角度，伸长到z_jump)
    for i in range(int(t_jump * 100)):
        x = x_forward_long
        z = z_forward_long
        trajectory.append([(x, z)] * 4)
    
    # 3. 收腿阶段 (后仰30度)
    for i in range(int(t_short * 100)):
        progress = i / (t_short * 100)
        x = x_forward
        z = z_forward
        trajectory.append([(x, z)] * 4)
    
    # 4. 落地阶段 (回到中立位)
    for i in range(int(t_land * 100)):
        progress = i / (t_land * 100)
        x = x_forward + (x_backward - x_forward) * progress
        z = z_forward + (z_backward - z_forward) * progress
        trajectory.append([(x, z)] * 4)
        
    for i in range(int(t_recover * 100)):
        progress = i / (t_recover * 100)
        x = x_backward * (1 - progress)
        z = z_backward + (z_land - z_backward) * progress
        trajectory.append([(x, z)] * 4)
    
    return trajectory

# ------------------- 主控制节点 -------------------
class JumpControlNode(Node):
    def __init__(self):
        super().__init__('jump_control_node')
        self.ik_solver = LegKinematics()
        self.pub = self.create_publisher(JointState, '/action', 10)
        self.trajectory = generate_jump_trajectory()
        self.timer = self.create_timer(0.01, self.publish_next_step)  # 10Hz控制
        self.step_idx = 0
        self.get_logger().info("带30度倾角的跳跃控制节点已启动")

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
            ik_result = self.ik_solver.inverse_kinematics(x, z)
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
        phase = "准备" if self.step_idx < 70 else ("起跳" if self.step_idx < 85 else ("收腿" if self.step_idx < 125 else "落地"))
        self.get_logger().info(
            f"Step {self.step_idx}: {phase} | "
            f"FL_x={leg_xz[Leg.FL][0]:.3f}, z={leg_xz[Leg.FL][1]:.3f}",
            throttle_duration_sec=0.1)

# ------------------- 主函数 -------------------
def main(args=None):
    rclpy.init(args=args)
    node = JumpControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()