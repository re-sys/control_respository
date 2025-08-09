#!/usr/bin/env python3
"""
测试StairJumpState的脚本
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# 直接导入模块以避免相对导入问题
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'state_machine_package', 'states'))
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'state_machine_package'))

from stair_jump_state import StairJumpState
from constants import Params
import math

class MockController:
    def __init__(self):
        self.leg_length = 0.20
        self.pitch_angle = math.radians(-90)
        self.get_logger = lambda: MockLogger()
        
    def compute_joint_cmd_from_leg_targets(self, leg_targets):
        # 模拟关节命令计算
        return [0.0] * 9
        
class MockLogger:
    def info(self, msg):
        print(f"INFO: {msg}")

def test_stair_jump_state():
    """测试StairJumpState的基本功能"""
    print("开始测试StairJumpState...")
    
    # 创建模拟控制器
    controller = MockController()
    
    # 创建StairJumpState实例
    stair_jump_state = StairJumpState(controller)
    
    # 测试进入状态
    print("\n1. 测试进入状态...")
    stair_jump_state.enter()
    print(f"   - 轨迹长度: {len(stair_jump_state.trajectory)}")
    print(f"   - 后腿偏置: {stair_jump_state.rear_leg_offset}")
    
    # 测试轨迹生成
    print("\n2. 测试轨迹生成...")
    trajectory = stair_jump_state._generate_stair_jump_trajectory()
    print(f"   - 轨迹总长度: {len(trajectory)}")
    
    # 检查前几个轨迹点
    print("\n3. 检查轨迹点...")
    for i in range(min(5, len(trajectory))):
        leg_data = trajectory[i]
        print(f"   步骤 {i}: 前腿长度={leg_data[0][0]:.3f}, 后腿长度={leg_data[2][0]:.3f}")
    
    # 测试状态更新
    print("\n4. 测试状态更新...")
    stair_jump_state.step_idx = 0
    cmd = stair_jump_state.update()
    print(f"   - 关节命令维度: {len(cmd)}")
    
    # 测试状态完成检查
    print("\n5. 测试状态完成检查...")
    print(f"   - 初始完成状态: {stair_jump_state.is_finished()}")
    stair_jump_state.step_idx = len(trajectory)
    print(f"   - 完成后的状态: {stair_jump_state.is_finished()}")
    
    print("\n测试完成！")

if __name__ == "__main__":
    test_stair_jump_state()
