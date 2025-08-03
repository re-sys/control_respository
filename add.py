#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

# 关节名称常量定义（与 state_machine_controller.py 保持一致）
JOINT_NAMES = [
    "FL_thigh_joint_i", "FL_thigh_joint_o",  # 0,1 (左前)
    "FR_thigh_joint_i", "FR_thigh_joint_o",  # 2,3 (右前) 
    "waist_joint",                           # 4 (不驱动)
    "RL_thigh_joint_i", "RL_thigh_joint_o",  # 5,6 (左后)
    "RR_thigh_joint_i", "RR_thigh_joint_o"   # 7,8 (右后)
]

class AddNode(Node):
    def __init__(self):
        super().__init__('add_node')
        
        # 创建发布者，发布全零的 kp 话题
        self.kp_pub = self.create_publisher(JointState, '/kp_cmd', 10)
        
        # 创建订阅者，订阅 kp 话题（可选，用于监控）
        self.kp_sub = self.create_subscription(JointState, '/kp_cmd', self.kp_callback, 10)
        
        # 创建定时器，定期发布全零的 kp 值
        self.timer = self.create_timer(0.1, self.publish_zero_kp)  # 每0.1秒发布一次
        
        self.get_logger().info("Add节点已启动，将发布全零的kp话题")
    
    def kp_callback(self, msg: JointState):
        """ 处理接收到的 kp 话题消息 """
        self.get_logger().info(f"接收到kp话题: {msg.effort}")
    
    def publish_zero_kp(self):
        """ 发布全零的 kp 值 """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        # 发布全零的 effort 值
        msg.effort = [00.0] * len(JOINT_NAMES)
        
        self.kp_pub.publish(msg)
        self.get_logger().info("已发布全零的kp话题")

def main(args=None):
    rclpy.init(args=args)
    node = AddNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Add节点安全停止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
