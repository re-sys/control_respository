#!/usr/bin/env python3
"""
状态机控制器主入口文件
"""

import rclpy
from .state_machine_controller import StateMachineController
from .constants import Params

def main(args=None):
    rclpy.init(args=args)
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