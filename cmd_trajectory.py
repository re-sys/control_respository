#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32  # 新增导入Float32消息类型
import math
import numpy as np

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

# ------------------- 步态参数配置 -------------------
class GaitParams:
    def __init__(self):
        # 基础参数 (单位: 米/秒)

        self.z_swing = 0.03       # 抬腿高度
        self.leg_length_min = 0.14
        self.leg_length_max = 0.34
        self.z_landing_offset = 0.00
        
        self.stride = 0.1     # 最大步幅
        self.max_turn_offset = 0.04 # 最大转向偏移量
        self.max_angle = math.pi/6 # 最大角度
        # 对角步态相位 (FL/RR同相, FR/RL同相)
        self.phase_offsets = [0.0, 0.5, 0.5, 0.0]
        
        # 时间参数
        self.largest_period = 1.0         # 完整步态周期(秒)
        self.smallest_period = 0.2         # 完整步态周期(秒)
        self.dt = 0.01            # 控制周期(10ms)
        


# ------------------- 逆运动学核心 -------------------
class LegKinematics:
    def __init__(self, l3=0.2, l4=0.16):
        self.l3 = l3; self.l4 = l4  # 小腿/大腿长度

    def inverse_kinematics(self, l0: float, theta: float) -> tuple:
        """ 计算关节角度 (注意机械结构调整符号) """
        theta_inside = math.acos((self.l4**2 + l0**2 - self.l3**2) / (2*self.l4*l0))
        return theta + theta_inside, theta - theta_inside

# ------------------- 运动轨迹生成器 -------------------
class MotionGenerator:
    def __init__(self):
        self.params = GaitParams()
        self.current_phase = 0.0
        self.cmd_vel = [0.0, 0.0]      # 当前执行的指令 [linear.x, angular.z]
        self.pitch_angle = 0.0
        self.leg_length = 0.17

    def update_target_command(self, target_vx: float, target_wz: float, target_angle: float):
        """ 直接更新目标指令，不做平滑处理 """
        self.cmd_vel = [target_vx, target_wz]
        #self.pitch_angle = -self.params.max_angle + target_angle * self.params.max_angle * 2
     
    def get_foot_target(self, leg_id: int) -> tuple:
        """ 生成合成运动轨迹 """
        # 1. 基础相位计算
        phase = (self.current_phase + self.params.phase_offsets[leg_id]) % 1.0
        abs_vx = abs(self.cmd_vel[0])
        # 2. 计算合成步幅 (使用平滑后的指令)
        stride = self.params.stride
        if self.cmd_vel[1] != 0:
            angular_stride = self.cmd_vel[1] * self.params.max_turn_offset  # [-0.2, 0.2]
            
            # 3. 根据腿部分配步幅 (左右腿反向)
            if leg_id in [Leg.FR, Leg.RR]:  # 右腿
                stride =self.params.stride*abs_vx + angular_stride
            else:  # 左腿
                stride =self.params.stride*abs_vx - angular_stride

        stride *= 1 if self.cmd_vel[0] >= 0 else -1
        half_stride = stride * 0.5
        angle = self.pitch_angle - math.pi/2
        z_base = self.leg_length * math.sin(angle)
        x_base = self.leg_length * math.cos(angle)
        # 4. 轨迹生成
        if phase < 0.5:  # 摆动相 (空中)
            progress = phase * 2  # 归一化[0,1]
            theta = 2.0 * math.pi * progress
            z = z_base + self.params.z_swing * (1-math.cos(theta)) * 0.5
            x = x_base - half_stride + stride * (progress - math.sin(theta)/(2.0 * math.pi))
        else:  # 支撑相 (地面)
            progress = (phase - 0.5) * 2  # 归一化[0,1]
            z = z_base + self.params.z_landing_offset * progress * abs_vx
            x = x_base + half_stride - progress * stride

        l = math.hypot(x, z)
        theta = math.atan2(z, x)

        return l, theta
    
    def step(self):
        """ 更新步态相位 (速度自适应) """
        period = self.params.largest_period + (self.params.smallest_period - self.params.largest_period) * min(math.hypot(self.cmd_vel[0],self.cmd_vel[1]),1)
        phase_step = self.params.dt / period
        self.current_phase = (self.current_phase + phase_step) % 1.0

    
# ------------------- 主控制节点 -------------------
class LocomotionController(Node):
    def __init__(self):
        super().__init__('locomotion_controller')
        
        # 初始化组件
        self.ik = LegKinematics()
        self.motion = MotionGenerator()
        
        # ROS接口
        self.joint_pub = self.create_publisher(JointState, '/action', 10)
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.height_sub = self.create_subscription(  # 新增：高度订阅器
            Float32, '/height', self.height_callback, 10)
        self.timer = self.create_timer(self.motion.params.dt, self.control_loop)
        
        self.get_logger().info("运动控制器已启动 | 支持前进/后退/转向 (带指令平滑)")
        self.get_logger().info("高度控制器已启动 | 范围: 0.0(-0.16m) 到 1.0(-0.27m)")

    def cmd_vel_callback(self, msg: Twist):
        """ 处理速度指令 - 直接赋值给目标指令 """
        self.motion.update_target_command(msg.linear.x, msg.angular.z,msg.angular.y)
        
    
    def height_callback(self, msg: Float32):
        """ 处理高度指令 - 将height(0-1)映射到z_base(-0.16到-0.27) """
        height = msg.data
        # 线性映射: height(0-1) -> z_base(-0.16到-0.27)
        self.motion.leg_length = self.motion.params.leg_length_min + height * (self.motion.params.leg_length_max - self.motion.params.leg_length_min)
        
    def control_loop(self):
        """ 主控制循环 """
        # 更新运动相位
        self.motion.step()
        if(self.motion.cmd_vel[0] == 0 and self.motion.cmd_vel[1] == 0):
            angle = self.motion.pitch_angle - math.pi/2
            leg_targets = {
                Leg.FL: (self.motion.leg_length, angle),
                Leg.FR: (self.motion.leg_length, angle),
                Leg.RL: (self.motion.leg_length, angle),
                Leg.RR: (self.motion.leg_length, angle)
            }
        else:
            leg_targets = {
                leg: self.motion.get_foot_target(leg) 
                for leg in [Leg.FL, Leg.FR, Leg.RL, Leg.RR]
            }
        # 生成关节指令
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
        
        # 调试信息 (限频) - 显示目标指令和平滑后的指令
        if int(self.motion.current_phase * 20) % 5 == 0:
            self.get_logger().info(
                f"Phase: {self.motion.current_phase:.2f} | "
                f"Current: vx={self.motion.cmd_vel[0]:.3f}, wz={self.motion.cmd_vel[1]:.3f} | "
                f"Height: z_base={self.motion.leg_length:.4f},pitch_angle:{self.motion.pitch_angle:.4f}m",
                throttle_duration_sec=0.2)

# ------------------- 主函数 -------------------
def main(args=None):
    rclpy.init(args=args)
    controller = LocomotionController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("控制器安全停止")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
