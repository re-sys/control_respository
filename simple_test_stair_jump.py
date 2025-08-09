#!/usr/bin/env python3
"""
简单的StairJumpState测试脚本
"""

import math
from typing import Dict, Tuple, List

# 模拟基础状态类
class State:
    def __init__(self, controller):
        self.controller = controller
        self.step_idx = 0
        
    def enter(self):
        pass
        
    def update(self):
        pass
        
    def can_transition_to(self, new_state):
        return True
        
    def is_finished(self):
        return False

# 模拟JumpState类
class JumpState(State):
    def __init__(self, controller):
        super().__init__(controller)
        self.step_idx = 0
        
    def enter(self):
        self.step_idx = 0
        self.controller.pitch_angle = math.radians(-110)
        self.front_pitch_angle = math.radians(-110)
        self.rear_pitch_angle = math.radians(-100)
        self.trajectory = self._generate_jump_trajectory()
        self.controller.get_logger().info("进入前跳状态")
        
    def _generate_jump_trajectory(self) -> List[List[Tuple[float, float]]]:
        """ 生成前跳轨迹 """
        trajectory = []
        
        # 定义关键参数
        leg_start_length = self.controller.leg_length
        leg_length_prepare_front = 0.12
        leg_length_prepare_rear = 0.16
        leg_length_extend_front = 0.35
        leg_length_extend_rear = 0.35
        leg_end_length = 0.12
        
        angle_land = 2*math.radians(-100)-self.rear_pitch_angle
        
        # 时间参数
        self.t_prepare = 0.2
        self.t_jump = 0.2
        self.t_short = 0.1
        self.t_land = 0.1
        self.t_recover = 0.5

        # 1. 准备阶段
        for i in range(int(self.t_prepare * 100)):
            progress = i / (self.t_prepare * 100)
            lf = leg_start_length + (leg_length_prepare_front - leg_start_length) * progress
            lr = leg_start_length + (leg_length_prepare_rear - leg_start_length) * progress
            trajectory.append([(lf, self.front_pitch_angle)] * 2 + [(lr, self.rear_pitch_angle)] * 2)
        
        # 2. 起跳阶段
        for i in range(int(self.t_jump * 100)):
            trajectory.append([(leg_length_extend_front, self.front_pitch_angle)] * 2 + 
                              [(leg_length_extend_rear, self.rear_pitch_angle)] * 2)
        
        # 3. 收腿阶段
        for i in range(int(self.t_short * 100)):
            trajectory.append([(leg_length_prepare_front, self.controller.pitch_angle)] * 4)
        
        # 4. 落地阶段
        for i in range(int(self.t_land * 100)):
            progress = i / (self.t_land * 100)
            lf = leg_length_prepare_front + (leg_end_length - leg_length_prepare_front) * progress
            lr = leg_length_prepare_rear + (leg_end_length - leg_length_prepare_rear) * progress
            front_angle = self.front_pitch_angle + (angle_land - self.front_pitch_angle) * progress
            rear_angle = self.rear_pitch_angle + (angle_land - self.rear_pitch_angle) * progress
            trajectory.append([(lf, front_angle)] * 2 + [(lr, rear_angle)] * 2)
            
        # 5. 恢复阶段
        for i in range(int(self.t_recover * 100)):
            progress = i / (self.t_recover * 100)
            l = leg_end_length + (leg_start_length - leg_end_length) * progress
            angle = angle_land + (math.radians(-90) - angle_land) * progress
            trajectory.append([(l, angle)] * 4)
        
        return trajectory
        
    def update(self) -> Dict[int, Tuple[float, float]]:           
        leg_xz = self.trajectory[self.step_idx]
        self.step_idx += 1
        
        # 转换为极坐标
        result = {}
        for leg_id, (l, angle) in enumerate(leg_xz):
            result[leg_id] = (l, angle)
        cmd = self.controller.compute_joint_cmd_from_leg_targets(result)

        return cmd
        
    def can_transition_to(self, new_state) -> bool:
        return self.is_finished() or new_state == "ERROR"  # 只有完成才能切换
        
    def is_finished(self) -> bool:
        return self.step_idx >= len(self.trajectory)

# StairJumpState类
class StairJumpState(JumpState):
    def __init__(self, controller):
        super().__init__(controller)
        self.step_idx = 0
        self.rear_leg_offset = 0.05  # 后腿偏置，可以根据需要调整
        
    def enter(self):
        self.step_idx = 0
        self.controller.pitch_angle = math.radians(-110)
        self.front_pitch_angle = math.radians(-110)
        self.rear_pitch_angle = math.radians(-100)
        self.trajectory = self._generate_stair_jump_trajectory()
        self.controller.get_logger().info("进入台阶跳跃状态")
        
    def _generate_stair_jump_trajectory(self) -> List[List[Tuple[float, float]]]:
        """ 生成台阶跳跃轨迹，后腿添加偏置 """
        trajectory = []
        
        # 定义关键参数
        leg_start_length = self.controller.leg_length
        leg_length_prepare_front = 0.12
        leg_length_prepare_rear = 0.16 + self.rear_leg_offset  # 后腿添加偏置
        leg_length_extend_front = 0.35
        leg_length_extend_rear = 0.35 + self.rear_leg_offset   # 后腿添加偏置
        leg_end_length = 0.12
        
        angle_land = 2*math.radians(-100)-self.rear_pitch_angle
        
        # 时间参数
        self.t_prepare = 0.2
        self.t_jump = 0.2
        self.t_short = 0.1
        self.t_land = 0.1
        self.t_recover = 0.5

        # 1. 准备阶段 - 后腿添加偏置
        for i in range(int(self.t_prepare * 100)):
            progress = i / (self.t_prepare * 100)
            lf = leg_start_length + (leg_length_prepare_front - leg_start_length) * progress
            lr = leg_start_length + (leg_length_prepare_rear - leg_start_length) * progress
            trajectory.append([(lf, self.front_pitch_angle)] * 2 + [(lr, self.rear_pitch_angle)] * 2)
        
        # 2. 起跳阶段 - 后腿添加偏置
        for i in range(int(self.t_jump * 100)):
            trajectory.append([(leg_length_extend_front, self.front_pitch_angle)] * 2 + 
                              [(leg_length_extend_rear, self.rear_pitch_angle)] * 2)
        
        # 3. 收腿阶段 - 后腿添加偏置
        for i in range(int(self.t_short * 100)):
            trajectory.append([(leg_length_prepare_front, self.controller.pitch_angle)] * 2 + 
                              [(leg_length_prepare_rear, self.controller.pitch_angle)] * 2)
        
        # 4. 落地阶段 - 后腿添加偏置
        for i in range(int(self.t_land * 100)):
            progress = i / (self.t_land * 100)
            lf = leg_length_prepare_front + (leg_end_length - leg_length_prepare_front) * progress
            lr = leg_length_prepare_rear + (leg_end_length - leg_length_prepare_rear) * progress
            front_angle = self.front_pitch_angle + (angle_land - self.front_pitch_angle) * progress
            rear_angle = self.rear_pitch_angle + (angle_land - self.rear_pitch_angle) * progress
            trajectory.append([(lf, front_angle)] * 2 + [(lr, rear_angle)] * 2)
            
        # 5. 恢复阶段 - 后腿添加偏置
        for i in range(int(self.t_recover * 100)):
            progress = i / (self.t_recover * 100)
            lf = leg_end_length + (leg_start_length - leg_end_length) * progress
            lr = leg_end_length + (leg_start_length - leg_end_length) * progress
            angle = angle_land + (math.radians(-90) - angle_land) * progress
            trajectory.append([(lf, angle)] * 2 + [(lr, angle)] * 2)
        
        return trajectory

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
    
    # 比较普通跳跃和台阶跳跃的差异
    print("\n6. 比较普通跳跃和台阶跳跃的差异...")
    jump_state = JumpState(controller)
    jump_state.enter()
    
    print(f"   普通跳跃轨迹长度: {len(jump_state.trajectory)}")
    print(f"   台阶跳跃轨迹长度: {len(stair_jump_state.trajectory)}")
    
    # 比较前几个轨迹点的后腿长度
    for i in range(min(3, len(trajectory))):
        jump_leg_data = jump_state.trajectory[i]
        stair_leg_data = stair_jump_state.trajectory[i]
        print(f"   步骤 {i}: 普通跳跃后腿长度={jump_leg_data[2][0]:.3f}, 台阶跳跃后腿长度={stair_leg_data[2][0]:.3f}")
    
    print("\n测试完成！")

if __name__ == "__main__":
    test_stair_jump_state()

