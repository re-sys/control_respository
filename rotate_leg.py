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

# ------------------- 前腿旋转轨迹生成 -------------------
def generate_front_leg_rotation_trajectory() -> List[List[Tuple[float, float]]]:
    """ 生成前腿从-90度旋转到45度的轨迹，后腿保持固定位置 """
    
    # 角度参数（弧度）
    angle_start = math.radians(-90)  # 起始角度
    angle_end = math.radians(-225)     # 结束角度
    
    # 腿长度参数
    front_leg_length = 0.17          # 前腿长度
    back_leg_length = 0.17           # 后腿长度
    
    # 后腿固定位置
    front_x = 0.0                     # 前腿x位置
    front_z = -0.17                   # 前腿z位置
    
    # 时间参数（单位：秒）
    total_time = 2.0                 # 总旋转时间
    dt = 0.01                        # 时间步长
    
    # 轨迹生成
    trajectory = []
    num_steps = int(total_time / dt)
    
    for i in range(num_steps):
        progress = i / (num_steps - 1)  # 归一化进度 [0, 1]
        
        # 前腿角度插值（使用平滑的插值函数）
        # 使用sin函数实现平滑的加速和减速
        smooth_progress = 0.5 * (1 - math.cos(progress * math.pi))
        front_angle = angle_start + (angle_end - angle_start) * smooth_progress
        
        # 计算前腿位置
        back_x = front_leg_length * math.cos(front_angle)
        back_z = front_leg_length * math.sin(front_angle)
        
        # 四条腿的位置 [FL, FR, RL, RR]
        trajectory.append([
            (front_x, front_z),  # FL (前左)
            (front_x, front_z),  # FR (前右)
            (back_x, back_z),    # RL (后左)
            (back_x, back_z)     # RR (后右)
        ])
    
    return trajectory

# ------------------- 主控制节点 -------------------
class FrontLegRotationNode(Node):
    def __init__(self):
        super().__init__('front_leg_rotation_node')
        self.ik_solver = LegKinematics()
        self.pub = self.create_publisher(JointState, '/action', 10)
        self.trajectory = generate_front_leg_rotation_trajectory()
        self.timer = self.create_timer(0.01, self.publish_next_step)  # 100Hz控制
        self.step_idx = 0
        self.get_logger().info("前腿旋转控制节点已启动：从-90°旋转到45°")

    def publish_next_step(self):
        """ 按顺序发布轨迹的每一步 """
        if self.step_idx >= len(self.trajectory):
            self.get_logger().info("前腿旋转完成！")
            self.timer.cancel()
            return
        
        leg_xz = self.trajectory[self.step_idx]
        self.step_idx += 1
        
        # 计算并发布关节指令
        cmd = [0.0] * 9
        for leg, (x, z) in enumerate(leg_xz):
            ik_result = self.ik_solver.inverse_kinematics(x, z)
            if ik_result is None:
                self.get_logger().error(f"Step {self.step_idx}: Leg {leg} IK失败, x={x:.3f}, z={z:.3f}")
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
        if self.step_idx % 10 == 0:  # 每0.1秒打印一次 (10步 * 0.01秒 = 0.1秒)
            back_x, back_z = leg_xz[Leg.RL]  # 后腿位置
            back_angle = math.degrees(math.atan2(back_z, back_x))
            progress = (self.step_idx / len(self.trajectory)) * 100
            
            # 获取后腿的逆运动学角度
            ik_result = self.ik_solver.inverse_kinematics(back_x, back_z)
            if ik_result is not None:
                theta1, theta4 = ik_result
                theta1_deg = math.degrees(theta1)
                theta4_deg = math.degrees(theta4)
                
                self.get_logger().info(
                    f"进度: {progress:.1f}% | "
                    f"后腿角度: {back_angle:.1f}° | "
                    f"后腿位置: x={back_x:.3f}, z={back_z:.3f} | "
                    f"逆运动学角度: θ1={theta1_deg:.1f}°, θ4={theta4_deg:.1f}°")
            else:
                self.get_logger().error(f"逆运动学解算失败: x={back_x:.3f}, z={back_z:.3f}")

# ------------------- 主函数 -------------------
def main(args=None):
    rclpy.init(args=args)
    node = FrontLegRotationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main() 