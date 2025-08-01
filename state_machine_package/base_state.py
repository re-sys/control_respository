#!/usr/bin/env python3
"""
状态基类文件
定义所有状态的抽象基类
"""

from abc import ABC, abstractmethod
from typing import Dict, Tuple
from constants import StateType

class State(ABC):
    def __init__(self, controller: 'StateMachineController'):
        self.controller = controller
        
    @abstractmethod
    def enter(self):
        """ 进入状态时的初始化 """
        pass
        
    @abstractmethod
    def update(self) -> Dict[int, Tuple[float, float]]:
        """ 更新状态，返回各腿的目标位置 (leg_id -> (length, angle)) """
        pass
        
    @abstractmethod
    def can_transition_to(self, new_state: StateType) -> bool:
        """ 是否可以切换到新状态 """
        pass
        
    @abstractmethod
    def is_finished(self) -> bool:
        """ 状态是否完成 """
        pass 