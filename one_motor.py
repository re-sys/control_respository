# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# import math

# class CyclicMotorController(Node):
#     """
#     一个循环控制单个电机的ROS2节点。
#     此节点会以1秒的间隔，交替将指定电机的位置设置为3.14和0.0弧度，
#     同时保持其他所有电机在零位。
#     """
#     def __init__(self):
#         super().__init__('cyclic_motor_controller')
        
#         # --- 参数配置 ---
#         self.motor_index = 1  # 要控制的电机索引 (1号电机: FL_thigh_joint_o)
#         self.target_angle = 0.0  # 初始目标角度
#         self.max_target_angle = 6.14
#         self.period = 0.5  # 循环周期 (秒)
        
#         # 定义关节名称
#         self.joint_names = [
#             "FL_thigh_joint_i", "FL_thigh_joint_o",
#             "FR_thigh_joint_i", "FR_thigh_joint_o",
#             "waist_joint",
#             "RL_thigh_joint_i", "RL_thigh_joint_o",
#             "RR_thigh_joint_i", "RR_thigh_joint_o"
#         ]
        
#         # 创建发布者
#         self.publisher = self.create_publisher(JointState, '/action', 10)
        
#         # 创建一个定时器，周期性地调用回调函数
#         self.timer = self.create_timer(self.period, self.timer_callback)
        
#         self.get_logger().info(
#             f"节点已启动，将以 {self.period} 秒为周期循环控制电机 "
#             f"'{self.joint_names[self.motor_index]}' (索引: {self.motor_index}) "
#             f"在 0.0 和 {self.target_angle} 弧度之间运动。"
#         )

#     def timer_callback(self):
#         """
#         定时器回调函数，用于发布电机指令。
#         """
#         msg = JointState()
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.name = self.joint_names
        
#         # 初始化所有电机位置为0.0
#         positions = [0.0] * len(self.joint_names)
        
#         # 设置目标电机的位置
#         positions[self.motor_index] = self.target_angle
#         msg.position = positions
        
#         # 发布消息
#         self.publisher.publish(msg)
#         self.get_logger().info(f"发布指令: 电机 {self.motor_index} 位置设置为 {self.target_angle:.2f}")
        
#         # 切换目标角度，为下一次回调做准备
#         if math.isclose(self.target_angle, self.max_target_angle):
#             self.target_angle = 0.0
#         else:
#             self.target_angle = self.max_target_angle

# def main(args=None):
#     rclpy.init(args=args)
#     node = CyclicMotorController()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("节点被用户中断")
#     finally:
#         # 清理
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()