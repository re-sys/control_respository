#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time
from typing import List, Tuple, Optional

# ------------------- 硬件常量定义 -------------------
UPPER_LINK_LENGTH = 0.2  # l3, 小腿连杆长度 (m)，与 forwardFlip.py 保持一致
LOWER_LINK_LENGTH = 0.16  # l4, 大腿连杆长度 (m)，与 forwardFlip.py 保持一致

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
class Params:
    def __init__(self): 
        self.t_phase0 = 0.6        # 下蹲阶段
        self.t_phase1 = 0.35       # 后腿蹬伸+前腿旋转30°
        self.t_phase2 = 0.3     # 后腿收腿+前腿旋转90°
        self.t_phase3 = 0.25        # 前腿蹬伸+后腿旋转
        self.t_phase4 = 0.4        # 前腿旋转
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
def generate_polar_jump_trajectory(params: Params,index: int) -> List[List[Tuple[float, float]]]:
    """ 生成极坐标下的两阶段跳跃轨迹 """
    
    # 角度定义（弧度）
    if index ==1:
        angle_phase0 = math.radians(-90)   # 初始角度
        angle_phase1 = math.radians(-70)   # 第一阶段前腿旋转目标
        angle_phase2_front = math.radians(0)  # 第二阶段前腿旋转目标
        angle_phase3_back = math.radians(-270) # 第三阶段后腿旋转目标
        angle_phase4_front = math.radians(90)  # 第四阶段前腿旋转目标
    elif index==2:
        angle_phase0 = math.radians(-90+180)   # 初始角度
        angle_phase1 = math.radians(-70+180)   # 第一阶段前腿旋转目标
        angle_phase2_front = math.radians(0+180)  # 第二阶段前腿旋转目标
        angle_phase3_back = math.radians(-270+180) # 第三阶段后腿旋转目标
        angle_phase4_front = math.radians(90+180)  # 第四阶段前腿旋转目标
    
    # 长度定义（米）
    leg_start_length = 0.17
    leg_length_prepare = 0.12
    leg_length_extend = 0.35
    leg_end_length = 0.13
    
    # 时间参数（秒）
    t_phase0 = params.t_phase0         # 下蹲阶段
    t_phase1 = params.t_phase1       # 后腿蹬伸+前腿旋转30°
    t_phase2 = params.t_phase2     # 后腿收腿+前腿旋转90°
    t_phase3 = params.t_phase3        # 前腿蹬伸+后腿旋转
    t_phase4 = params.t_phase4        # 前腿旋转
    
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
        if index ==1:
            trajectory.append([
                (leg_length_prepare, front_angle),  # FL
                (leg_length_prepare, front_angle),  # FR
                (back_length, angle_phase0),       # RL
                (back_length, angle_phase0)        # RR
            ])
        elif index==2:
            trajectory.append([
                (back_length, angle_phase0),       # FL
                (back_length, angle_phase0),        # FR
                (leg_length_prepare, front_angle),  # RL
                (leg_length_prepare, front_angle),  # RR
            ])
    
    # 阶段2：后腿收腿+前腿继续旋转
    for i in range(int(t_phase2 * 100)):
        progress = i / (t_phase2 * 100)
        # 前腿：继续旋转
        front_angle = angle_phase1 + (angle_phase2_front - angle_phase1) * progress
        # 后腿：收腿
        back_length = leg_length_extend + (leg_end_length - leg_length_extend) * progress
        
        if index ==1:
            trajectory.append([
                (leg_length_prepare, front_angle),  # FL
                (leg_length_prepare, front_angle),  # FR
                (back_length, angle_phase0),       # RL
                (back_length, angle_phase0)        # RR
                ])
        elif index==2:
            trajectory.append([
                (back_length, angle_phase0),       # FL
                (back_length, angle_phase0),        # FR
                (leg_length_prepare, front_angle),  # RL
                (leg_length_prepare, front_angle),  # RR
        ])
    
    # 阶段3：前腿蹬伸+后腿旋转
    for i in range(int(t_phase3 * 100)):
        progress = i / (t_phase3 * 100)
        # 前腿：蹬伸
        front_length = leg_length_extend
        # 后腿：旋转
        back_angle = angle_phase0 + (angle_phase3_back - angle_phase0) * progress
        if index ==1:
            trajectory.append([
                (front_length, angle_phase2_front),  # FL
                (front_length, angle_phase2_front),  # FR
                (leg_end_length, back_angle),        # RL
                (leg_end_length, back_angle)         # RR
                ])
        elif index==2:
            trajectory.append([
                (leg_end_length, back_angle),        # FL
                (leg_end_length, back_angle),        # FR
                (front_length, angle_phase2_front),  # RL
                (front_length, angle_phase2_front),  # RR
            ])
    
    # 阶段4：前腿旋转
    for i in range(int(t_phase4 * 100)):
        progress = i / (t_phase4 * 100)
        # 前腿：旋转
        front_angle = angle_phase2_front + (angle_phase4_front - angle_phase2_front) * progress
        if index ==1:
            trajectory.append([
                (leg_end_length, front_angle),  # FL
                (leg_end_length, front_angle),  # FR
                (leg_end_length, angle_phase3_back),  # RL
                (leg_end_length, angle_phase3_back)   # RR
            ])
        elif index==2:
            trajectory.append([
                (leg_end_length, angle_phase3_back),  # FL
                (leg_end_length, angle_phase3_back),  # FR
                (leg_end_length, front_angle),  # RL
                (leg_end_length, front_angle)   # RR
            ])
    
    return trajectory

# ------------------- 主控制节点 -------------------
class PolarJumpControlNode(Node):
    def __init__(self):
        super().__init__('polar_jump_control_node')
        self.ik = LegKinematics()
        self.pub = self.create_publisher(JointState, '/action', 10)
        self.params = Params()
        self.trajectory_flip1 = generate_polar_jump_trajectory(self.params,1)
        self.trajecotry_flip2 = generate_polar_jump_trajectory(self.params,2)
        self.timer = self.create_timer(0.01, self.publish_next_step)
        self.step_idx = 0
        self.get_logger().info("极坐标接口跳跃控制节点已启动")
        self.flip_flag = [False, False] # front leg/rear leg

    def publish_next_step(self):
        """ 发布轨迹的每一步 """
        if self.step_idx >= len(self.trajectory_flip1):
            self.get_logger().info("跳跃完成！")
            self.timer.cancel()
            return
        
        leg_polar_coords = self.trajectory_flip1[self.step_idx]
        self.step_idx += 1
        if self.step_idx == int((self.params.t_phase0+self.params.t_phase1) * 100):
            self.flip_flag[1] = False if self.flip_flag[1] else True
        if self.step_idx == int((self.params.t_phase2+self.params.t_phase0+self.params.t_phase1+self.params.t_phase3) * 100):
            self.flip_flag[0] = False if self.flip_flag[0] else True
        
        
        # 计算并发布关节指令
        fl_length, fl_angle = leg_polar_coords[Leg.FL]
        fr_length, fr_angle = leg_polar_coords[Leg.FR]
        rl_length, rl_angle = leg_polar_coords[Leg.RL]
        rr_length, rr_angle = leg_polar_coords[Leg.RR]
        
        # 分别计算IK
        if self.flip_flag[0]:
            FL_theta4, FL_theta1 = self.ik.inverse_kinematics(fl_length, fl_angle)
            FR_theta4, FR_theta1 = self.ik.inverse_kinematics(fr_length, fr_angle)
        else:
            FL_theta1, FL_theta4 = self.ik.inverse_kinematics(fl_length, fl_angle)
            FR_theta1, FR_theta4 = self.ik.inverse_kinematics(fr_length, fr_angle)

        if self.flip_flag[1]:
            RL_theta4, RL_theta1 = self.ik.inverse_kinematics(rl_length, rl_angle)
            RR_theta4, RR_theta1 = self.ik.inverse_kinematics(rr_length, rr_angle)
        else:
            RL_theta1, RL_theta4 = self.ik.inverse_kinematics(rl_length, rl_angle)
            RR_theta1, RR_theta4 = self.ik.inverse_kinematics(rr_length, rr_angle)

        # 直接赋值给cmd数组
        joint_cmd = [0.0] * 9
        joint_cmd[0] = -FL_theta1 ;joint_cmd[1] = FL_theta4 + math.pi  # FL_thigh_joint_o
        joint_cmd[2] = FR_theta1 ;joint_cmd[3] = -FR_theta4 - math.pi  # FR_thigh_joint_o
        joint_cmd[5] = -RL_theta4 - math.pi  ;joint_cmd[6] = RL_theta1  # RL_thigh_joint_o
        joint_cmd[7] = RR_theta4 + math.pi  ;joint_cmd[8] = -RR_theta1  # RR_thigh_joint_o
        
        
        # 发布指令
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = joint_cmd
        self.pub.publish(msg)
        
        # 打印当前状态
        self.get_logger().info(
            # f"Step {self.step_idx} | "
            # f"前腿: 长度={fl_length:.3f}m, 角度={math.degrees(fl_angle):.1f}° | "
            # f"前腿theta1: ={math.degrees(FL_theta1):.1f}度, theta4角度={math.degrees(FL_theta4):.1f}° | "
            # f"后腿: 长度={rl_length:.3f}m, 角度={math.degrees(rl_angle):.1f}° | "
            f"后腿theta1: ={math.degrees(RR_theta1):.1f}度, theta4角度={math.degrees(RR_theta4):.1f}°",
            throttle_duration_sec=0.1)

# ------------------- 主函数 -------------------
def main(args=None):
    rclpy.init(args=args)
    node = PolarJumpControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()