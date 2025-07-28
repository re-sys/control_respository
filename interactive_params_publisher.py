#!/usr/bin/env python3
"""
交互式参数发布者
用于实时调整四足机器人控制参数
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import threading
import time

class InteractiveParamsPublisher(Node):
    def __init__(self):
        super().__init__('interactive_params_publisher')
        
        # 创建发布者
        self.params_pub = self.create_publisher(Float32MultiArray, '/params', 10)
        
        # 当前参数值
        self.current_params = [
            0.0,    # leg_length_ratio (0.0-1.0)
            0.5,    # pitch_angle_ratio (0.0-1.0)
            0.5,    # smallest_period
            0.5,    # stride
            0.5    # z_swing
        ]
        
        # 创建定时器，每秒发布一次参数
        self.timer = self.create_timer(1.0, self.publish_current_params)
        
        self.get_logger().info("交互式参数发布者已启动")
        self.get_logger().info("输入 'help' 查看命令")
        
    def publish_current_params(self):
        """发布当前参数"""
        msg = Float32MultiArray()
        msg.data = self.current_params
        self.params_pub.publish(msg)
        
    def set_leg_length(self, ratio):
        """设置腿长比例"""
        self.current_params[0] = max(0.0, min(1.0, ratio))
        self.get_logger().info(f"腿长比例设置为: {self.current_params[0]:.3f}")
        
    def set_pitch_angle(self, ratio):
        """设置俯仰角比例"""
        self.current_params[1] = max(0.0, min(1.0, ratio))
        self.get_logger().info(f"俯仰角比例设置为: {self.current_params[1]:.3f}")
        
    def set_smallest_period(self, value):
        """设置最小周期"""
        self.current_params[2] = max(0.1, min(1.0, value))
        self.get_logger().info(f"最小周期设置为: {self.current_params[2]:.3f}")
        
    def set_stride(self, value):
        """设置步长"""
        self.current_params[3] = max(0.01, min(0.5, value))
        self.get_logger().info(f"步长设置为: {self.current_params[3]:.3f}")
        
    def set_z_swing(self, value):
        """设置摆动高度"""
        self.current_params[4] = max(0.01, min(0.1, value))
        self.get_logger().info(f"摆动高度设置为: {self.current_params[4]:.3f}")
        
    def show_current_params(self):
        """显示当前参数"""
        self.get_logger().info("当前参数:")
        self.get_logger().info(f"  腿长比例: {self.current_params[0]:.3f}")
        self.get_logger().info(f"  俯仰角比例: {self.current_params[1]:.3f}")
        self.get_logger().info(f"  最小周期: {self.current_params[2]:.3f}")
        self.get_logger().info(f"  步长: {self.current_params[3]:.3f}")
        self.get_logger().info(f"  摆动高度: {self.current_params[4]:.3f}")
        
    def show_help(self):
        """显示帮助信息"""
        print("\n=== 交互式参数发布者帮助 ===")
        print("命令列表:")
        print("  leg <value>     - 设置腿长比例 (0.0-1.0)")
        print("  pitch <value>   - 设置俯仰角比例 (0.0-1.0)")
        print("  period <value>  - 设置最小周期 (0.1-1.0)")
        print("  stride <value>  - 设置步长 (0.01-0.5)")
        print("  swing <value>   - 设置摆动高度 (0.01-0.1)")
        print("  show            - 显示当前参数")
        print("  help            - 显示此帮助信息")
        print("  quit            - 退出程序")
        print("  reset           - 重置为默认参数")
        print("\n示例:")
        print("  leg 0.8         - 设置腿长为80%")
        print("  pitch 0.3       - 设置俯仰角为30%")
        print("  period 0.3      - 设置最小周期为0.3")
        print("  stride 0.15     - 设置步长为0.15")
        print("  swing 0.05      - 设置摆动高度为0.05")
        print("========================")

def input_thread(publisher):
    """输入处理线程"""
    while rclpy.ok():
        try:
            user_input = input("> ").strip().lower()
            
            if user_input == 'quit' or user_input == 'exit':
                print("退出程序...")
                break
            elif user_input == 'help':
                publisher.show_help()
            elif user_input == 'show':
                publisher.show_current_params()
            elif user_input == 'reset':
                publisher.current_params = [0.5, 0.5, 0.2, 0.1, 0.04]
                publisher.get_logger().info("参数已重置为默认值")
            elif user_input.startswith('leg '):
                try:
                    value = float(user_input.split()[1])
                    publisher.set_leg_length(value)
                except (IndexError, ValueError):
                    print("错误: 请输入有效的数字 (0.0-1.0)")
            elif user_input.startswith('pitch '):
                try:
                    value = float(user_input.split()[1])
                    publisher.set_pitch_angle(value)
                except (IndexError, ValueError):
                    print("错误: 请输入有效的数字 (0.0-1.0)")
            elif user_input.startswith('period '):
                try:
                    value = float(user_input.split()[1])
                    publisher.set_smallest_period(value)
                except (IndexError, ValueError):
                    print("错误: 请输入有效的数字 (0.1-1.0)")
            elif user_input.startswith('stride '):
                try:
                    value = float(user_input.split()[1])
                    publisher.set_stride(value)
                except (IndexError, ValueError):
                    print("错误: 请输入有效的数字 (0.01-0.5)")
            elif user_input.startswith('swing '):
                try:
                    value = float(user_input.split()[1])
                    publisher.set_z_swing(value)
                except (IndexError, ValueError):
                    print("错误: 请输入有效的数字 (0.01-0.1)")
            elif user_input:
                print("未知命令，输入 'help' 查看可用命令")
                
        except EOFError:
            break
        except KeyboardInterrupt:
            break

def main(args=None):
    rclpy.init(args=args)
    
    publisher = InteractiveParamsPublisher()
    
    # 启动输入处理线程
    input_thread_obj = threading.Thread(target=input_thread, args=(publisher,), daemon=True)
    input_thread_obj.start()
    
    try:
        publisher.show_help()
        publisher.show_current_params()
        
        # 主循环
        while rclpy.ok():
            rclpy.spin_once(publisher, timeout_sec=0.1)
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        publisher.get_logger().info("程序被用户中断")
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 