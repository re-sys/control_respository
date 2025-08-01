# State Machine Package for ARESdog

这是一个模块化的状态机控制器包，用于控制ARESdog机器人的各种运动状态。

## 包结构

```
state_machine_package/
├── __init__.py                 # 包初始化文件
├── constants.py                # 常量定义（关节名称、腿枚举、参数类等）
├── kinematics.py               # 逆运动学核心
├── base_state.py               # 状态基类
├── state_machine_controller.py # 状态机控制器主类
├── main.py                     # 主入口文件
├── run_controller.py           # 启动脚本（推荐使用）
├── README.md                   # 说明文档
└── states/                     # 状态文件夹
    ├── __init__.py            # 状态包初始化
    ├── idle_state.py          # 静止状态
    ├── walk_state.py          # 对角步态状态
    ├── flip_state.py          # 空翻状态
    ├── jump_state.py          # 前跳状态
    ├── recovery_state.py      # 恢复状态
    └── error_state.py         # 错误状态
```

## 主要组件

### 1. 常量定义 (constants.py)
- `JOINT_NAMES`: 关节名称列表
- `Leg`: 腿的枚举类
- `Params`: 参数配置类
- `StateType`: 状态类型枚举

### 2. 逆运动学 (kinematics.py)
- `LegKinematics`: 腿部运动学计算类

### 3. 状态基类 (base_state.py)
- `State`: 所有状态的抽象基类

### 4. 具体状态实现 (states/)
每个状态都是一个独立的文件：
- `IdleState`: 静止站立状态
- `WalkState`: 对角步态行走状态
- `FlipState`: 空翻动作状态
- `JumpState`: 前跳动作状态
- `RecoveryState`: 恢复状态
- `ErrorState`: 错误处理状态

### 5. 状态机控制器 (state_machine_controller.py)
- `StateMachineController`: 主控制器类，管理状态切换和ROS通信

## 使用方法

### 运行控制器
```bash
# 推荐方式：使用启动脚本
cd state_machine_package
python run_controller.py

# 或者作为模块运行
python -m state_machine_package.main

# 在ROS环境中运行
ros2 run your_package state_machine_controller
```

### 状态切换
通过发布到 `/state` 话题来切换状态：
- `data = 1`: 触发空翻
- `data = 2`: 触发前跳
- `data = 3`: 触发恢复

### 速度控制
通过发布到 `/cmd_vel` 话题来控制行走速度：
- `linear.x`: 前进/后退速度
- `angular.z`: 转向速度

### 参数调整
通过发布到 `/params` 话题来调整参数：
- `data[0]`: 腿长度调整
- `data[1]`: 俯仰角调整
- `data[2]`: 步态周期调整
- `data[3]`: 步幅调整
- `data[4]`: 摆动高度调整

## 设计优势

1. **模块化设计**: 每个状态独立成文件，便于维护和扩展
2. **清晰的层次结构**: 常量、运动学、状态、控制器分离
3. **易于扩展**: 添加新状态只需继承基类并实现相应方法
4. **代码复用**: 通过基类和工具类减少重复代码
5. **类型安全**: 使用类型注解提高代码可读性和安全性

## 扩展新状态

要添加新状态，需要：

1. 在 `states/` 文件夹中创建新的状态文件
2. 继承 `State` 基类并实现所有抽象方法
3. 在 `constants.py` 中添加新的状态类型
4. 在 `state_machine_controller.py` 中注册新状态
5. 在 `states/__init__.py` 中导入新状态类

## 故障排除

### 导入错误
如果遇到导入错误，确保：
1. 在正确的目录下运行脚本
2. 使用 `python run_controller.py` 而不是直接运行其他文件
3. 检查所有依赖是否正确安装

### ROS环境
确保在ROS环境中运行：
```bash
source /opt/ros/humble/setup.bash
source ~/ARESdog_ws/install/setup.bash
```

## 依赖

- ROS2
- rclpy
- numpy
- sensor_msgs
- geometry_msgs
- std_msgs 