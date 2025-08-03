#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
import math
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64MultiArray
import numpy as np

class JointOrientSubscriber(Node):
    def __init__(self):
        super().__init__('joint_orient_subscriber')
        
        # 订阅者
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        self.orient_sub = self.create_subscription(
            Quaternion,
            '/orient',
            self.orient_callback,
            10
        )
        
        # 数据存储
        self.joint_data = None
        self.orient_data = None
        
        # 定时器，每秒打印一次
        self.timer = self.create_timer(1.0, self.print_data)
        
        self.get_logger().info("Joint and Orient Subscriber initialized")
    
    def joint_callback(self, msg):
        """处理joint_states数据"""
        self.joint_data = msg
    
    def orient_callback(self, msg):
        """处理orient数据"""
        self.orient_data = msg
    
    def quaternion_to_euler(self, quaternion):
        """将四元数转换为欧拉角（弧度）"""
        # 提取四元数分量
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        
        # 手动计算欧拉角 (ZYX顺序)
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # 转换为度数
        euler_degrees = [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]
        
        return euler_degrees
    
    def radians_to_degrees(self, radians_list):
        """将弧度列表转换为度数列表"""
        return [math.degrees(rad) for rad in radians_list]
    
    def print_data(self):
        """每秒打印一次数据"""
        print("\n" + "="*60)
        print(f"Time: {self.get_clock().now()}")
        print("="*60)
        
        # 打印关节状态数据
        if self.joint_data is not None:
            print("JOINT STATES (degrees):")
            print("-" * 30)
            for i, name in enumerate(self.joint_data.name):
                if i < len(self.joint_data.position):
                    position_deg = math.degrees(self.joint_data.position[i])
                    velocity_deg = math.degrees(self.joint_data.velocity[i]) if i < len(self.joint_data.velocity) else 0.0
                    effort = self.joint_data.effort[i] if i < len(self.joint_data.effort) else 0.0
                    
                    print(f"{name:15s}: Position={position_deg:8.2f}°, Velocity={velocity_deg:8.2f}°/s, Effort={effort:8.2f}")
        else:
            print("JOINT STATES: No data received")
        
        print()
        
        # 打印方向数据
        if self.orient_data is not None:
            print("ORIENTATION (Euler angles in degrees):")
            print("-" * 40)
            euler_degrees = self.quaternion_to_euler(self.orient_data)
            print(f"Roll (X):  {euler_degrees[0]:8.2f}°")
            print(f"Pitch (Y): {euler_degrees[1]:8.2f}°")
            print(f"Yaw (Z):   {euler_degrees[2]:8.2f}°")
            
            # 同时显示四元数
            print(f"Quaternion: x={self.orient_data.x:6.3f}, y={self.orient_data.y:6.3f}, z={self.orient_data.z:6.3f}, w={self.orient_data.w:6.3f}")
        else:
            print("ORIENTATION: No data received")
        
        print("="*60)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        subscriber = JointOrientSubscriber()
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        print("Subscriber stopped by user")
    except Exception as e:
        print(f"Subscriber error: {e}")
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 