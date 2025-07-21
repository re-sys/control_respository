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
        self.z_base = -0.17      # 站立高度(初始值)
        self.min_z_base = -0.135  # 当height=0时的z_base
        self.max_z_base = -0.27  # 当height=1时的z_base

        self.z_swing = 0.02       # 抬腿高度
        
        self.max_stride = 0.12     # 最大步幅
        self.max_turn_offset = 0.08 # 最大转向偏移量
        
        self.max_angle = math.pi/6 # 最大角度
        
        # 对角步态相位 (FL/RR同相, FR/RL同相)
        self.phase_offsets = [0.0, 0.5, 0.5, 0.0]
          
        
        # 时间参数
        self.period = 0.2         # 完整步态周期(秒)
        self.append_period_coefficient = self.period * 0.4  # 附加周期系数
        self.dt = 0.01            # 控制周期(10ms)
        
        # 指令平滑参数
        self.max_cmd_change = 0.1  # 最大指令变化量

# ------------------- 逆运动学核心 -------------------
class LegKinematics:
    def __init__(self, l3=0.2, l4=0.1):
        self.l3 = l3; self.l4 = l4  # 大腿/小腿长度

    def inverse_kinematics(self, x: float, z: float) -> tuple:
        """ 计算关节角度 (注意机械结构调整符号) """
        l0 = math.hypot(x, z)
        theta_inside = math.acos((self.l4**2 + l0**2 - self.l3**2) / (2*self.l4*l0))
        theta = math.atan2(z, x)
        return theta + theta_inside, theta - theta_inside

# ------------------- 运动轨迹生成器 -------------------
class MotionGenerator:
    def __init__(self):
        self.params = GaitParams()
        self.current_phase = 0.0
        self.cmd_vel = [0.0, 0.0]      # 当前执行的指令 [linear.x, angular.z]
        self.target_cmd_vel = [0.0, 0.0]  # 目标指令
        self.angle = 0.0
        self.x_offset = 0.0
        self.z_base = -0.17

        
    def update_target_command(self, target_vx: float, target_wz: float, target_angle: float):
        """ 直接更新目标指令，不做平滑处理 """
        self.target_cmd_vel = [target_vx, target_wz]
        self.angle = -self.params.max_angle + target_angle * self.params.max_angle * 2
    
    def smooth_command(self):
        """ 在定时器中调用，执行指令平滑处理 """
        # 特殊情况：目标指令为0时直接跳变
        if self.target_cmd_vel[0] == 0.0 and self.target_cmd_vel[1] == 0.0:
            self.cmd_vel = [0.0, 0.0]
            return
            
        # 计算速度指令差异
        vx_diff = self.target_cmd_vel[0] - self.cmd_vel[0]
        wz_diff = self.target_cmd_vel[1] - self.cmd_vel[1]
        
        # 对linear.x进行限幅处理
        new_vx = self.cmd_vel[0] + vx_diff*self.params.max_cmd_change

        # 对angular.z进行限幅处理
        new_wz = self.cmd_vel[1] + wz_diff*self.params.max_cmd_change
 
        # 更新当前指令
        self.cmd_vel = [new_vx, new_wz]
     
    def get_foot_target(self, leg_id: int) -> tuple:
        """ 生成合成运动轨迹 """
        # 1. 基础相位计算
        phase = (self.current_phase + self.params.phase_offsets[leg_id]) % 1.0
        
        # 2. 计算合成步幅 (使用平滑后的指令)
        linear_stride = self.cmd_vel[0] * self.params.max_stride  # [-0.4, 0.4]
        angular_stride = self.cmd_vel[1] * self.params.max_turn_offset  # [-0.2, 0.2]
        
        # 3. 根据腿部分配步幅 (左右腿反向)
        if leg_id in [Leg.FR, Leg.RR]:  # 右腿
            stride = linear_stride + angular_stride
        else:  # 左腿
            stride = linear_stride - angular_stride

        half_stride = stride * 0.5
        
        
        # 4. 轨迹生成
        if phase < 0.5:  # 摆动相 (空中)
            progress = phase * 2  # 归一化[0,1]
            theta = 2.0 * math.pi * progress
            z = self.z_base + self.params.z_swing * (1-math.cos(theta)) * 0.5
            x = -half_stride + stride * (progress - math.sin(theta)/(2.0 * math.pi)) - 0.5*half_stride + self.x_offset
        else:  # 支撑相 (地面)
            progress = (phase - 0.5) * 2  # 归一化[0,1]
            z = self.z_base - 0.01*progress
            x = half_stride - progress * stride - 0.5*half_stride + self.x_offset
        
        return x, z
    
    def step(self):
        """ 更新步态相位 (速度自适应) """
        append_period = (abs(self.cmd_vel[0]) + abs(self.cmd_vel[1])) * self.params.append_period_coefficient 
        phase_step = self.params.dt / (self.params.period + append_period)
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
        self.motion.params.z_base = self.motion.params.min_z_base + height * (self.motion.params.max_z_base - self.motion.params.min_z_base)
        
        self.get_logger().info(f"高度更新: height={height:.2f}, z_base={self.motion.params.z_base:.4f}m")
        
    def control_loop(self):
        """ 主控制循环 """
        # 执行指令平滑处理
        self.motion.smooth_command()
        
        # 更新运动相位
        self.motion.step()
        if(self.motion.target_cmd_vel[0] == 0 and self.motion.target_cmd_vel[1] == 0):
            x = math.sin(self.motion.angle) * abs(self.motion.params.z_base)
            z = -math.cos(self.motion.angle) * abs(self.motion.params.z_base)
            leg_targets = {
                Leg.FL: (x, z),
                Leg.FR: (x, z),
                Leg.RL: (x, z),
                Leg.RR: (x, z)
            }
            self.motion.x_offset = x
            self.motion.z_base = z

        else:
            leg_targets = {
                leg: self.motion.get_foot_target(leg) 
                for leg in [Leg.FL, Leg.FR, Leg.RL, Leg.RR]
            }
        # 计算各腿目标位置
        
        # 生成关节指令
        joint_cmd = [0.0] * 9
        for leg, (x, z) in leg_targets.items():
            theta1, theta4 = self.ik.inverse_kinematics(x, z)
            
            # 机械结构调整 (根据实际安装方向可能需要修改)
            if leg == Leg.FL:
                joint_cmd[0] = -theta1; joint_cmd[1] = theta4 + math.pi
            elif leg == Leg.FR:
                joint_cmd[2] = theta1; joint_cmd[3] = -theta4 - math.pi
            elif leg == Leg.RL:
                joint_cmd[5] = -theta4 - math.pi; joint_cmd[6] = theta1
            elif leg == Leg.RR:
                joint_cmd[7] = theta4 + math.pi; joint_cmd[8] = -theta1
        
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
                f"Target: vx={self.motion.target_cmd_vel[0]:.3f}, wz={self.motion.target_cmd_vel[1]:.3f} | "
                f"Current: vx={self.motion.cmd_vel[0]:.3f}, wz={self.motion.cmd_vel[1]:.3f} | "
                f"Height: z_base={self.motion.params.z_base:.4f}m",
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
