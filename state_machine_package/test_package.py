#!/usr/bin/env python3
"""
状态机包测试脚本
用于验证包的基本功能和导入
"""

import sys
import os

# 添加当前目录到Python路径
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

def test_imports():
    """测试所有模块的导入"""
    print("测试模块导入...")
    
    try:
        from constants import JOINT_NAMES, Leg, Params, StateType
        print("✓ constants.py 导入成功")
        print(f"  - JOINT_NAMES: {len(JOINT_NAMES)} 个关节")
        print(f"  - Leg枚举: FL={Leg.FL}, FR={Leg.FR}, RL={Leg.RL}, RR={Leg.RR}")
        print(f"  - StateType枚举: {[s.value for s in StateType]}")
    except ImportError as e:
        print(f"✗ constants.py 导入失败: {e}")
        return False
    
    try:
        from kinematics import LegKinematics
        print("✓ kinematics.py 导入成功")
        ik = LegKinematics()
        print(f"  - 逆运动学实例创建成功")
    except ImportError as e:
        print(f"✗ kinematics.py 导入失败: {e}")
        return False
    
    try:
        from base_state import State
        print("✓ base_state.py 导入成功")
    except ImportError as e:
        print(f"✗ base_state.py 导入失败: {e}")
        return False
    
    try:
        from states import IdleState, WalkState, FlipState, JumpState, RecoveryState, ErrorState
        print("✓ states 包导入成功")
        print(f"  - 所有状态类导入成功")
    except ImportError as e:
        print(f"✗ states 包导入失败: {e}")
        return False
    
    try:
        from state_machine_controller import StateMachineController
        print("✓ state_machine_controller.py 导入成功")
    except ImportError as e:
        print(f"✗ state_machine_controller.py 导入失败: {e}")
        return False
    
    return True

def test_state_creation():
    """测试状态创建（跳过ROS依赖）"""
    print("\n测试状态创建（跳过ROS依赖）...")
    
    try:
        from constants import Params, StateType
        from states import IdleState, WalkState, FlipState, JumpState, RecoveryState, ErrorState
        
        # 创建一个简单的模拟控制器类
        class MockController:
            def __init__(self, params):
                self.params = params
                self.leg_length = params.leg_length
                self.pitch_angle = params.pitch_angle
                self.cmd_vel = [0.0, 0.0]
                self.back_state = False
                self.flip_io_leg = [False, False]
                self.joint_position = [0.0] * 9
                self.states = {}
            
            def get_logger(self):
                class MockLogger:
                    def info(self, msg):
                        pass
                return MockLogger()
            
            def compute_joint_cmd_from_leg_targets(self, leg_targets):
                return [0.0] * 9
        
        # 创建参数和模拟控制器
        params = Params()
        controller = MockController(params)
        
        # 测试状态创建
        states = {
            'IdleState': IdleState(controller),
            'WalkState': WalkState(controller),
            'FlipState': FlipState(controller),
            'JumpState': JumpState(controller),
            'RecoveryState': RecoveryState(controller),
            'ErrorState': ErrorState(controller)
        }
        
        for name, state in states.items():
            print(f"✓ {name} 创建成功")
            
        print("✓ 所有状态创建成功")
        return True
        
    except Exception as e:
        print(f"✗ 状态创建失败: {e}")
        return False

def test_constants():
    """测试常量定义"""
    print("\n测试常量定义...")
    
    try:
        from constants import JOINT_NAMES, Leg, Params, StateType
        
        # 测试关节名称
        expected_joints = [
            "FL_thigh_joint_i", "FL_thigh_joint_o",
            "FR_thigh_joint_i", "FR_thigh_joint_o",
            "waist_joint",
            "RL_thigh_joint_i", "RL_thigh_joint_o",
            "RR_thigh_joint_i", "RR_thigh_joint_o"
        ]
        
        if JOINT_NAMES == expected_joints:
            print("✓ JOINT_NAMES 定义正确")
        else:
            print("✗ JOINT_NAMES 定义错误")
            return False
        
        # 测试腿枚举
        if Leg.FL == 0 and Leg.FR == 1 and Leg.RL == 2 and Leg.RR == 3:
            print("✓ Leg 枚举定义正确")
        else:
            print("✗ Leg 枚举定义错误")
            return False
        
        # 测试参数类
        params = Params()
        if hasattr(params, 'leg_length') and hasattr(params, 'pitch_angle'):
            print("✓ Params 类定义正确")
        else:
            print("✗ Params 类定义错误")
            return False
        
        # 测试状态类型
        expected_states = ['idle', 'walk', 'flip', 'jump', 'error', 'recovery']
        actual_states = [s.value for s in StateType]
        if actual_states == expected_states:
            print("✓ StateType 枚举定义正确")
        else:
            print("✗ StateType 枚举定义错误")
            return False
            
        return True
        
    except Exception as e:
        print(f"✗ 常量测试失败: {e}")
        return False

def test_kinematics():
    """测试逆运动学功能"""
    print("\n测试逆运动学功能...")
    
    try:
        from kinematics import LegKinematics
        import math
        
        ik = LegKinematics()
        
        # 测试一些基本的逆运动学计算
        result = ik.inverse_kinematics(0.2, 0.0)
        if result is not None:
            print("✓ 逆运动学计算成功")
        else:
            print("✗ 逆运动学计算失败")
            return False
            
        return True
        
    except Exception as e:
        print(f"✗ 逆运动学测试失败: {e}")
        return False

def main():
    """主测试函数"""
    print("=" * 50)
    print("状态机包测试")
    print("=" * 50)
    
    # 运行所有测试
    tests = [
        test_imports,
        test_constants,
        test_kinematics,
        test_state_creation
    ]
    
    passed = 0
    total = len(tests)
    
    for test in tests:
        if test():
            passed += 1
        print()
    
    print("=" * 50)
    print(f"测试结果: {passed}/{total} 通过")
    
    if passed == total:
        print("🎉 所有测试通过！包结构正确。")
        print("\n📝 使用说明:")
        print("1. 在ROS环境中运行: python run_controller.py")
        print("2. 确保ROS话题正确配置")
        print("3. 检查关节状态数据是否正常")
        return True
    else:
        print("❌ 部分测试失败，请检查包结构。")
        return False

if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1) 