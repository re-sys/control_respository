#!/usr/bin/env python3
"""
状态机控制器启动脚本
可以直接运行这个脚本来启动重构后的状态机控制器
"""

import sys
import os

# 添加当前目录到Python路径
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

# 使用绝对导入
from state_machine_controller import StateMachineController
from constants import Params
import rclpy

def main():
    rclpy.init()
    controller = StateMachineController(Params())
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("控制器安全停止")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 