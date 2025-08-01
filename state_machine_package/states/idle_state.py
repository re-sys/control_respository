#!/usr/bin/env python3
"""
静止状态文件
"""

from typing import Dict, Tuple
from base_state import State
from constants import StateType, Leg

class IdleState(State):
    def __init__(self, controller: 'StateMachineController'):
        super().__init__(controller)
        
    def enter(self):
        self.controller.get_logger().info("进入静止状态")
        
    def update(self) -> Dict[int, Tuple[float, float]]:
        angle = self.controller.pitch_angle 
        result = {
            Leg.FL: (self.controller.leg_length, angle),
            Leg.FR: (self.controller.leg_length, angle),
            Leg.RL: (self.controller.leg_length, angle),
            Leg.RR: (self.controller.leg_length, angle)
        }
        return self.controller.compute_joint_cmd_from_leg_targets(result)
        
    def can_transition_to(self, new_state: StateType) -> bool:
        return True  # 静止状态可以切换到任何状态
        
    def is_finished(self) -> bool:
        return False  # 静止状态永不完成 