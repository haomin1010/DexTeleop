"""
controller package - unified controller nodes

Provides state machine controller nodes with mode switching support:
- TianjiArmControllerNode: Tianji arm controller
- WujiHandControllerNode: Dexterous hand controller

Two control modes:
1. TELEOP mode (default): teleoperation input → kinematic solving → hardware
2. INFERENCE mode: joint_command_in topic → hardware (skips kinematic solving)
   TELEOP publishes joint_command (MuJoCo/rosbag); never subscribes to it.

Module structure:
├── common.py           # Shared utilities (ControlMode, ROS2LoggerAdapter, etc.)
├── wujihand_node.py    # Dexterous hand controller node
└── tianji_arm_node.py  # Tianji arm controller node
"""

from .common import ControlMode, ROS2LoggerAdapter

__all__ = [
    'ControlMode',
    'ROS2LoggerAdapter',
]
