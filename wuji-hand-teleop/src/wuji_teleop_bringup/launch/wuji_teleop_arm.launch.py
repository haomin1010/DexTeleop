"""
Wuji Arm-Only Teleoperation Launch File

Launches arm teleoperation components with configurable input device.

Supported input devices:
  - tracker: HTC Vive Trackers

Usage:
    # Using HTC Vive Trackers
    ros2 launch wuji_teleop_bringup wuji_teleop_arm.launch.py arm_input:=tracker

    # With RViz visualization
    ros2 launch wuji_teleop_bringup wuji_teleop_arm.launch.py arm_input:=tracker enable_rviz:=true
"""

from __future__ import annotations

import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from wuji_teleop_bringup.tf_utils import create_chest_tf_nodes, create_tianji_tf_nodes


def _get_config_path(package: str, config_file: str) -> str:
    """Get the path to a config file in a package's share directory."""
    share_dir = Path(get_package_share_directory(package))
    return str(share_dir / "config" / config_file)


def _get_rviz_path() -> str:
    """Get the path to the RViz config file."""
    share_dir = Path(get_package_share_directory("openvr_input"))
    return str(share_dir / "rviz" / "openvr_visualization.rviz")


def _get_tianji_right_urdf() -> str:
    share_dir = Path(get_package_share_directory("tianji_urdf"))
    return (share_dir / "urdf" / "right.urdf").read_text()


def _get_python_executable() -> str:
    """Use the active environment's Python for Python ROS executables."""
    if os.environ.get("PYTHON_EXECUTABLE"):
        return os.environ["PYTHON_EXECUTABLE"]
    if os.environ.get("CONDA_PREFIX"):
        return str(Path(os.environ["CONDA_PREFIX"]) / "bin" / "python")
    return sys.executable


def generate_launch_description() -> LaunchDescription:
    # ==================== Launch Arguments ====================
    arm_input_arg = DeclareLaunchArgument(
        "arm_input",
        default_value="tracker",
        description="Arm input device: 'tracker' (HTC Vive Trackers)",
    )
    enable_rviz_arg = DeclareLaunchArgument(
        "enable_rviz",
        default_value="false",
        description="Enable RViz visualization",
    )
    openvr_config_arg = DeclareLaunchArgument(
        "openvr_config",
        default_value=_get_config_path("openvr_input", "openvr_input.yaml"),
        description="Path to OpenVR tracker config file",
    )
    controller_config_arg = DeclareLaunchArgument(
        "controller_config",
        default_value=_get_config_path("tianji_output", "tianji_output.yaml"),
        description="Path to Tianji arm controller config file",
    )
    dry_run_arg = DeclareLaunchArgument(
        "dry_run",
        default_value="false",
        description="Print Tianji arm commands without connecting to or driving the robot",
    )
    read_only_arg = DeclareLaunchArgument(
        "read_only",
        default_value="false",
        description="Connect to Tianji feedback but never send robot commands",
    )
    feedback_handshake_arg = DeclareLaunchArgument(
        "feedback_handshake",
        default_value="false",
        description="In read_only mode, send one non-motion SDK command sequence to start feedback",
    )
    sim_viz_arg = DeclareLaunchArgument(
        "sim_viz",
        default_value="false",
        description="Publish RViz markers for tracker/Tianji target debugging",
    )
    enable_tianji_model_arg = DeclareLaunchArgument(
        "enable_tianji_model",
        default_value="false",
        description="Publish Tianji right arm URDF model to RViz",
    )
    enable_mujoco_arg = DeclareLaunchArgument(
        "enable_mujoco",
        default_value="false",
        description="Show Tianji right arm joint states in a MuJoCo window",
    )
    sdk_executor_enable_arg = DeclareLaunchArgument(
        "sdk_executor_enable",
        default_value="false",
        description="Enable separated Tianji SDK executor node (real robot)",
    )
    enable_rviz = LaunchConfiguration("enable_rviz")
    openvr_config = LaunchConfiguration("openvr_config")
    controller_config = LaunchConfiguration("controller_config")
    dry_run = LaunchConfiguration("dry_run")
    read_only = LaunchConfiguration("read_only")
    feedback_handshake = LaunchConfiguration("feedback_handshake")
    sim_viz = LaunchConfiguration("sim_viz")
    enable_tianji_model = LaunchConfiguration("enable_tianji_model")
    enable_mujoco = LaunchConfiguration("enable_mujoco")
    sdk_executor_enable = LaunchConfiguration("sdk_executor_enable")

    return LaunchDescription([
        # Arguments
        arm_input_arg,
        enable_rviz_arg,
        openvr_config_arg,
        controller_config_arg,
        dry_run_arg,
        read_only_arg,
        feedback_handshake_arg,
        sim_viz_arg,
        enable_tianji_model_arg,
        enable_mujoco_arg,
        sdk_executor_enable_arg,

        # ==================== ARM INPUT: Tracker ====================
        Node(
            package="openvr_input",
            executable="openvr_input",
            name="openvr_input",
            output="screen",
            arguments=["-c", openvr_config],
            condition=LaunchConfigurationEquals("arm_input", "tracker"),
        ),

        # ==================== STATIC TF: From Config ====================
        OpaqueFunction(function=lambda ctx: create_chest_tf_nodes() + create_tianji_tf_nodes()),

        # ==================== ARM OUTPUT: Tianji ====================
        # Tianji Arm Controller (TELEOP mode by default)
        # Also publishes state: /tianji_arm/left/joint_state, /tianji_arm/right/joint_state
        Node(
            package="controller",
            executable="tianji_arm_controller",
            name="tianji_arm_controller",
            output="screen",
            emulate_tty=True,
            prefix=[_get_python_executable(), " "],
            arguments=[
                "-c",
                controller_config,
                PythonExpression(["'--dry-run' if '", dry_run, "' == 'true' else ''"]),
                PythonExpression(["'--read-only' if '", read_only, "' == 'true' else ''"]),
                PythonExpression(["'--feedback-handshake' if '", feedback_handshake, "' == 'true' else ''"]),
            ],
        ),
        Node(
            package="controller",
            executable="tianji_tracker_sim_viz",
            name="tianji_tracker_sim_viz",
            output="screen",
            emulate_tty=True,
            condition=IfCondition(sim_viz),
        ),
        Node(
            package="controller",
            executable="tianji_sdk_executor",
            name="tianji_sdk_executor",
            output="screen",
            emulate_tty=True,
            prefix=[_get_python_executable(), " "],
            arguments=[
                "-c",
                controller_config,
            ],
            condition=IfCondition(sdk_executor_enable),
        ),
        Node(
            package="controller",
            executable="tianji_joint_state_bridge",
            name="tianji_joint_state_bridge",
            output="screen",
            emulate_tty=True,
            arguments=[
                "--input-topic",
                "/tianji_arm/right/joint_state",
            ],
            condition=IfCondition(enable_tianji_model),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="tianji_right_robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": _get_tianji_right_urdf()}],
            condition=IfCondition(enable_tianji_model),
        ),
        Node(
            package="controller",
            executable="tianji_mujoco_viewer",
            name="tianji_mujoco_viewer",
            output="screen",
            emulate_tty=True,
            prefix=[_get_python_executable(), " "],
            arguments=[
                "--joint-topic",
                "/tianji_arm/right/joint_command",
                "--initial-joint-topic",
                "/tianji_arm/right/joint_state",
            ],
            additional_env={
                "MUJOCO_GL": "glfw",
                "LIBGL_ALWAYS_SOFTWARE": "1",
                "QT_X11_NO_MITSHM": "1",
            },
            condition=IfCondition(enable_mujoco),
        ),

        # ==================== VISUALIZATION ====================
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", _get_rviz_path()],
            additional_env={
                "LIBGL_ALWAYS_SOFTWARE": "1",
                "QT_X11_NO_MITSHM": "1",
            },
            condition=IfCondition(enable_rviz),
        ),
    ])
