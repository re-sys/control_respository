#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class ZeroJointPublisher(Node):
    def __init__(self):
        super().__init__('zero_joint_publisher')
        
        # 定义关节名称（根据你的实际机器人配置修改）
        self.joint_names = [
            "FL_thigh_joint_i", "FL_thigh_joint_o",
            "FR_thigh_joint_i", "FR_thigh_joint_o",
            "waist_joint",
            "RL_thigh_joint_i", "RL_thigh_joint_o",
            "RR_thigh_joint_i", "RR_thigh_joint_o"
        ]
        
        # 创建发布者
        self.publisher = self.create_publisher(JointState, '/action', 10)
        
        # 发布零位指令
        self.publish_zero_positions()
        
        # 发布后自动关闭节点
        self.get_logger().info("已发布所有关节零位指令，节点将退出")
        rclpy.shutdown()

    def publish_zero_positions(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [0.0] * len(self.joint_names)  # 所有关节置零
        # msg.position[1] = 3.14
        
        self.publisher.publish(msg)
        self.get_logger().info(f"已发布零位指令: {msg.position}")

def main(args=None):
    rclpy.init(args=args)
    node = ZeroJointPublisher()
    rclpy.spin(node)  # 短暂运行以确保消息发布
    node.destroy_node()

if __name__ == '__main__':
    main()