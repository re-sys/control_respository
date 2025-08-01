#!/usr/bin/env python3
"""
恢复状态文件
"""

import numpy as np
from typing import Dict, Tuple
from base_state import State
from constants import StateType

class RecoveryState(State):
    def __init__(self, controller: 'StateMachineController'):
        super().__init__(controller)
        self.start_cmd = None
        self.target_cmd = None
        self.steps = 100  # 平滑步数
        self.current_step = 0

    def enter(self):
        self.start_cmd = self.controller.joint_position.copy()
        self.controller.publish_kp_value(20.0)
        # 生成Idle目标关节角度
        self.target_cmd = self.controller.states[StateType.IDLE].update()
        self.current_step = 0
        self.controller.get_logger().info("进入恢复状态，平滑过渡到静止站立")

    def update(self):
        if self.current_step >= self.steps:
            return self.target_cmd
        alpha = self.current_step / self.steps
        cmd = (1 - alpha) * self.start_cmd + alpha * self.target_cmd
        self.current_step += 1
        return cmd

    def can_transition_to(self, new_state: StateType) -> bool:
        return self.is_finished() and new_state == StateType.IDLE

    def is_finished(self) -> bool:
        return self.current_step >= self.steps 