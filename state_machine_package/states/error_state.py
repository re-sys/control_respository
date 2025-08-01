#!/usr/bin/env python3
"""
错误状态文件
"""

import numpy as np
from typing import Dict, Tuple
from base_state import State
from constants import StateType

class ErrorState(State):
    def __init__(self, controller: 'StateMachineController'):
        super().__init__(controller)
        
    def enter(self):
        self.controller.get_logger().info("进入错误状态")
        self.controller.publish_kp_value(0.0)
        self.controller.publish_joint_cmd(np.zeros(9))
        self.controller.back_state = False
        self.controller.flip_io_leg = [False, False]
        
    def update(self) -> Dict[int, Tuple[float, float]]:
        return None
        
    def can_transition_to(self, new_state: 'StateType') -> bool:
        if new_state == StateType.RECOVERY:
            return True
        return False
        
    def is_finished(self) -> bool:
        return False 