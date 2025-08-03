#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class KpCommandPublisherPeriodic(Node):
    """
    一个周期性地给单个电机发送 Kp 指令的 ROS2 节点。
    此节点会以固定的频率向 /kp_cmd 话题发布一条 JointState 消息，
    为指定电机设置 Kp 值，并将其他所有电机的 Kp 值设为 0.0。
    """
    def __init__(self):
        super().__init__('kp_command_publisher_periodic')
        
        # --- 参数配置 ---
        self.motor_index = 0      # 要控制的电机索引 (1号电机: FL_thigh_joint_o)
        self.kp_value_jump = [200.0, 200.0, 200.0, 200.0, 400.0, 200.0, 200.0, 200.0, 200.0]  # 起跳设置的 Kp 值
        self.kd_value_jump = [4.0, 4.0, 4.0, 4.0, 8.0, 4.0, 4.0, 4.0, 4.0]  # 起跳设置的 Kd 值
        self.kp_value_land = [25.0, 25.0, 25.0, 25.0, 400.0, 25.0, 25.0, 25.0, 25.0]  # 着陆设置的 Kp 值
        self.kd_value_land = [0.5, 0.5, 0.5, 0.5, 8.0, 0.5, 0.5, 0.5, 0.5]  # 着陆设置的 Kd 值
        self.topic_name_kp = '/kp_cmd' # 目标话题
        self.topic_name_kd = '/kd_cmd' # 目标话题
        self.period = 1.0         # 发布周期 (秒)
        
        # 定义关节名称 (必须与 usb_bridge_node 中的顺序和数量一致)
        self.joint_names = [
            "FL_thigh_joint_i", "FL_thigh_joint_o",
            "FR_thigh_joint_i", "FR_thigh_joint_o",
            "waist_joint",
            "RL_thigh_joint_i", "RL_thigh_joint_o",
            "RR_thigh_joint_i", "RR_thigh_joint_o"
        ]
        
        # 创建发布者
        self.publisher_kp = self.create_publisher(JointState, self.topic_name_kp, 10)
        self.publisher_kd = self.create_publisher(JointState, self.topic_name_kd, 10)
        
        # 创建一个定时器，周期性地调用回调函数
        self.timer1 = self.create_timer(self.period, self.kp_callback)
        self.timer2 = self.create_timer(self.period, self.kd_callback)
        
        self.get_logger().info(
            f"节点已启动，将以 {self.period} 秒为周期向 '{self.topic_name_kp}' 话题发布 Kp 指令。"
        )

    def kp_callback(self):
        """
        定时器回调函数，构建并发布 Kp 指令。
        """
        if self.motor_index >= len(self.joint_names):
            self.get_logger().error(
                f"电机索引 {self.motor_index} 超出范围，定时器将停止发布。",
                once=True # 避免日志刷屏
            )
            return
            
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # 1. 填充所有关节名称
        msg.name = self.joint_names
        
        # 2. 初始化所有电机的 effort (Kp) 为 0.0
        efforts = [25.0] * len(self.joint_names)
        
        # 3. 为目标电机设置指定的 Kp 值
        # 设置 effort (Kp) 值
        for index in range(len(self.joint_names)):
            efforts[index] = self.kp_value_jump[index]
        msg.effort = efforts # 使用 effort 字段来传输 Kp/Kd

        # 发布消息
        self.publisher_kp.publish(msg)
        self.get_logger().info(
            f"发布Kp指令: 所有电机Kp值已设置",
            throttle_duration_sec=5 # 每5秒最多打印一次，避免刷屏
        )

    def kd_callback(self):
        """
        定时器回调函数，构建并发布 Kd 指令。
        """
        if self.motor_index >= len(self.joint_names):
            self.get_logger().error(
                f"电机索引 {self.motor_index} 超出范围，定时器将停止发布。",
                once=True # 避免日志刷屏
            )
            return
             
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # 1. 填充所有关节名称
        msg.name = self.joint_names
        
        # 2. 初始化所有电机的 effort (Kd) 为 0.0
        efforts = [0.5] * len(self.joint_names)
        
        # 3. 为目标电机设置指定的 Kd 值
        # 设置 effort (Kd) 值
        for index in range(len(self.joint_names)):
            efforts[index] = self.kd_value_jump[index]
        msg.effort = efforts

        # 发布消息
        self.publisher_kd.publish(msg)
        self.get_logger().info(
            f"发布Kd指令: 所有电机Kd值已设置",
            throttle_duration_sec=5 # 每5秒最多打印一次，避免刷屏
        )

def main(args=None):
    rclpy.init(args=args)
    node = KpCommandPublisherPeriodic()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点被用户中断")
    finally:
        # 清理
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()