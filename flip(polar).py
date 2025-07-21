#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time
from typing import List, Tuple, Optional

# ------------------- 硬件常量定义 -------------------
UPPER_LINK_LENGTH = 0.2  # l3, 大腿连杆长度 (m)，与 forwardFlip.py 保持一致
LOWER_LINK_LENGTH = 0.1  # l4, 小腿连杆长度 (m)，与 forwardFlip.py 保持一致

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

# ------------------- 逆运动学核心 (极坐标接口，笛卡尔核心) -------------------
class LegKinematics:
    def __init__(self, l3: float = UPPER_LINK_LENGTH, l4: float = LOWER_LINK_LENGTH):
        self.l3 = l3
        self.l4 = l4

    def inverse_kinematics(self, l0: float, theta: float) -> Optional[Tuple[float, float]]:
        """
        逆运动学计算（已简化）。
        直接使用极坐标（长度, 角度）作为 l0 和 theta 进行物理计算。
        """       
        # 工作空间检查
        if abs(self.l3 - self.l4) > l0 or l0 > (self.l3 + self.l4):
            return None

        theta_inside = math.acos((self.l4**2 + l0**2 - self.l3**2) / (2 * self.l4 * l0))
           
        theta1 = theta + theta_inside
        theta4 = theta - theta_inside
        return theta1, theta4

# ------------------- 极坐标跳跃轨迹生成 -------------------
def generate_polar_jump_trajectory() -> List[List[Tuple[float, float]]]:
    """ 生成极坐标下的两阶段跳跃轨迹 """
    
    # 角度定义（弧度）
    angle_phase0 = math.radians(-90)   # 初始角度
    angle_phase1 = math.radians(-60)   # 第一阶段前腿旋转目标
    angle_phase2_front = math.radians(0)  # 第二阶段前腿旋转目标
    angle_phase3_back = math.radians(-225) # 第三阶段后腿旋转目标
    angle_phase4_front = math.radians(45)  # 第四阶段前腿旋转目标
    
    # 长度定义（米）
    leg_start_length = 0.17
    leg_length_prepare = 0.12
    leg_length_extend = 0.29
    leg_end_length = 0.14
    
    # 时间参数（秒）
    t_phase0 = 0.6        # 下蹲阶段
    t_phase1 = 0.5       # 后腿蹬伸+前腿旋转30°
    t_phase2 = 0.6     # 后腿收腿+前腿旋转90°
    t_phase3 = 0.4        # 前腿蹬伸+后腿旋转
    t_phase4 = 0.4        # 前腿旋转
    
    trajectory = []
    
    # 阶段0：下蹲（所有腿长度变化）
    for i in range(int(t_phase0 * 100)):
        progress = i / (t_phase0 * 100)
        length = leg_start_length + (leg_length_prepare - leg_start_length) * progress
        trajectory.append([
            (length, angle_phase0),  # FL
            (length, angle_phase0),  # FR
            (length, angle_phase0),  # RL
            (length, angle_phase0)   # RR
        ])
    
    # 阶段1：后腿蹬伸+前腿旋转
    for i in range(int(t_phase1 * 100)):
        progress = i / (t_phase1 * 100)
        # 前腿：保持长度，旋转角度
        front_angle = angle_phase0 + (angle_phase1 - angle_phase0) * progress
        # 后腿：保持角度，改变长度
        back_length = leg_length_extend
        
        trajectory.append([
            (leg_length_prepare, front_angle),  # FL
            (leg_length_prepare, front_angle),  # FR
            (back_length, angle_phase0),       # RL
            (back_length, angle_phase0)        # RR
        ])
    
    # 阶段2：后腿收腿+前腿继续旋转
    for i in range(int(t_phase2 * 100)):
        progress = i / (t_phase2 * 100)
        # 前腿：继续旋转
        front_angle = angle_phase1 + (angle_phase2_front - angle_phase1) * progress
        # 后腿：收腿
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
        # 前腿：蹬伸
        front_length = leg_length_extend
        # 后腿：旋转
        back_angle = angle_phase0 + (angle_phase3_back - angle_phase0) * progress
        
        trajectory.append([
            (front_length, angle_phase2_front),  # FL
            (front_length, angle_phase2_front),  # FR
            (leg_end_length, back_angle),        # RL
            (leg_end_length, back_angle)         # RR
        ])
    
    # 阶段4：前腿旋转
    for i in range(int(t_phase4 * 100)):
        progress = i / (t_phase4 * 100)
        # 前腿：旋转
        front_angle = angle_phase2_front + (angle_phase4_front - angle_phase2_front) * progress
        
        trajectory.append([
            (leg_end_length, front_angle),  # FL
            (leg_end_length, front_angle),  # FR
            (leg_end_length, angle_phase3_back),  # RL
            (leg_end_length, angle_phase3_back)   # RR
        ])
    
    return trajectory

# ------------------- 主控制节点 -------------------
class PolarJumpControlNode(Node):
    def __init__(self):
        super().__init__('polar_jump_control_node')
        self.ik_solvers = [LegKinematics() for _ in range(4)]
        self.pub = self.create_publisher(JointState, '/action', 10)
        self.trajectory = generate_polar_jump_trajectory()
        self.timer = self.create_timer(0.01, self.publish_next_step)
        self.step_idx = 0
        self.get_logger().info("极坐标接口跳跃控制节点已启动")

    def publish_next_step(self):
        """ 发布轨迹的每一步 """
        if self.step_idx >= len(self.trajectory):
            self.get_logger().info("跳跃完成！")
            self.timer.cancel()
            return
        
        leg_polar_coords = self.trajectory[self.step_idx]
        self.step_idx += 1
        
        # 计算并发布关节指令
        cmd = [0.0] * 9
        for leg, (length, angle) in enumerate(leg_polar_coords):
            # 直接将极坐标传递给IK解算器
            ik_result = self.ik_solvers[leg].inverse_kinematics(length, angle)
            
            if ik_result is None:
                self.get_logger().error(f"Step {self.step_idx}: Leg {leg} IK失败, length={length:.3f}, angle={math.degrees(angle):.1f}°")
                # 为安全起见，若IK失败则不发布新指令
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
        self.get_logger().info(
            f"Step {self.step_idx} | "
            f"前腿: 长度={leg_polar_coords[Leg.FL][0]:.3f}m, 角度={math.degrees(leg_polar_coords[Leg.FL][1]):.1f}° | "
            f"后腿: 长度={leg_polar_coords[Leg.RL][0]:.3f}m, 角度={math.degrees(leg_polar_coords[Leg.RL][1]):.1f}°",
            throttle_duration_sec=0.1)

# ------------------- 主函数 -------------------
def main(args=None):
    rclpy.init(args=args)
    node = PolarJumpControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()