"""
Wuji Full Teleoperation Launch File

Launches all components for hand and arm teleoperation with configurable input devices.

Supported combinations:
  - hand_input: manus (Manus Gloves) or wuji_glove (Wuji Gloves)
  - arm_input: tracker (HTC Vive Trackers)

Usage:
    # Wuji gloves + Vive Trackers
    ros2 launch wuji_teleop_bringup wuji_teleop.launch.py hand_input:=wuji_glove arm_input:=tracker

    # With RViz visualization
    ros2 launch wuji_teleop_bringup wuji_teleop.launch.py hand_input:=wuji_glove arm_input:=tracker enable_rviz:=true
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
from launch_ros.parameter_descriptions import ParameterValue

from wuji_teleop_bringup.hand_defaults import (
    LEFT_HAND_SERIAL, RIGHT_HAND_SERIAL,
    LEFT_HAND_NAME, RIGHT_HAND_NAME,
    DRIVER_PUBLISH_RATE, DRIVER_FILTER_CUTOFF_FREQ, DRIVER_DIAGNOSTICS_RATE,
)
from wuji_teleop_bringup.tf_utils import create_chest_tf_nodes, create_tianji_tf_nodes


def _get_config_path(package: str, config_file: str) -> str:
    """Get the path to a config file in a package's share directory."""
    share_dir = Path(get_package_share_directory(package))
    return str(share_dir / "config" / config_file)


def _get_rviz_path() -> str:
    """Get the path to the RViz config file."""
    share_dir = Path(get_package_share_directory("openvr_input"))
    return str(share_dir / "rviz" / "openvr_visualization.rviz")


def _get_python_executable() -> str:
    """Use the active environment's Python for Python ROS executables."""
    if os.environ.get("PYTHON_EXECUTABLE"):
        return os.environ["PYTHON_EXECUTABLE"]
    if os.environ.get("CONDA_PREFIX"):
        return str(Path(os.environ["CONDA_PREFIX"]) / "bin" / "python")
    return sys.executable


def generate_launch_description() -> LaunchDescription:
    # ==================== Launch Arguments ====================
    hand_input_arg = DeclareLaunchArgument(
        "hand_input",
        default_value="none",
        description="Hand input device: 'none', 'manus', or 'wuji_glove'",
    )
    arm_input_arg = DeclareLaunchArgument(
        "arm_input",
        default_value="tracker",
        description="Arm input device: 'tracker' (HTC Vive Trackers)",
    )
    enable_arm_arg = DeclareLaunchArgument(
        "enable_arm",
        default_value="true",
        description="Enable arm tracker input and Tianji arm controller",
    )
    enable_rviz_arg = DeclareLaunchArgument(
        "enable_rviz",
        default_value="false",
        description="Enable RViz visualization",
    )
    dry_run_arg = DeclareLaunchArgument(
        "dry_run",
        default_value="false",
        description="Print Tianji arm commands without connecting to or driving the robot",
    )
    openvr_config_arg = DeclareLaunchArgument(
        "openvr_config",
        default_value=_get_config_path("openvr_input", "openvr_input.yaml"),
        description="Path to OpenVR tracker config file",
    )
    hand_config_arg = DeclareLaunchArgument(
        "hand_config",
        default_value=_get_config_path("wujihand_output", "wujihand_ik.yaml"),
        description="Path to wujihand_ik config file",
    )

    # ===== wujihandros2 driver parameters =====
    left_serial_arg = DeclareLaunchArgument(
        "left_serial",
        default_value=LEFT_HAND_SERIAL,
        description="Left hand serial number",
    )
    right_serial_arg = DeclareLaunchArgument(
        "right_serial",
        default_value=RIGHT_HAND_SERIAL,
        description="Right hand serial number",
    )
    left_hand_name_arg = DeclareLaunchArgument(
        "left_hand_name",
        default_value=LEFT_HAND_NAME,
        description="Left hand wujihandros2 namespace",
    )
    right_hand_name_arg = DeclareLaunchArgument(
        "right_hand_name",
        default_value=RIGHT_HAND_NAME,
        description="Right hand wujihandros2 namespace",
    )
    left_glove_sn_arg = DeclareLaunchArgument("left_glove_sn", default_value="", description="Left glove serial number")
    right_glove_sn_arg = DeclareLaunchArgument("right_glove_sn", default_value="", description="Right glove serial number")
    left_device_name_arg = DeclareLaunchArgument("left_device_name", default_value="glove_left", description="Left glove device name")
    right_device_name_arg = DeclareLaunchArgument("right_device_name", default_value="glove_right", description="Right glove device name")
    left_retarget_config_arg = DeclareLaunchArgument("left_retarget_config", default_value="", description="Left direct retarget config")
    right_retarget_config_arg = DeclareLaunchArgument("right_retarget_config", default_value="", description="Right direct retarget config")
    include_left_hand_arg = DeclareLaunchArgument("include_left_hand", default_value="true", description="Enable left hand input")
    include_right_hand_arg = DeclareLaunchArgument("include_right_hand", default_value="true", description="Enable right hand input")

    # Get configurations
    hand_input = LaunchConfiguration("hand_input")
    arm_input = LaunchConfiguration("arm_input")
    enable_arm = LaunchConfiguration("enable_arm")
    enable_rviz = LaunchConfiguration("enable_rviz")
    dry_run = LaunchConfiguration("dry_run")
    openvr_config = LaunchConfiguration("openvr_config")
    hand_config = LaunchConfiguration("hand_config")

    # Force serial_number to string type (workaround for ROS2 type inference)
    left_serial_str = ParameterValue(
        LaunchConfiguration("left_serial"), value_type=str
    )
    right_serial_str = ParameterValue(
        LaunchConfiguration("right_serial"), value_type=str
    )

    return LaunchDescription([
        # Arguments
        hand_input_arg,
        arm_input_arg,
        enable_arm_arg,
        enable_rviz_arg,
        dry_run_arg,
        openvr_config_arg,
        hand_config_arg,
        left_serial_arg,
        right_serial_arg,
        left_hand_name_arg,
        right_hand_name_arg,
        left_glove_sn_arg,
        right_glove_sn_arg,
        left_device_name_arg,
        right_device_name_arg,
        left_retarget_config_arg,
        right_retarget_config_arg,
        include_left_hand_arg,
        include_right_hand_arg,

        # ==================== WUJIHANDROS2 DRIVERS ====================
        # Left hand driver (wujihandros2)
        Node(
            package="wujihand_driver",
            executable="wujihand_driver_node",
            name="wujihand_driver",
            namespace=LaunchConfiguration("left_hand_name"),
            parameters=[{
                "serial_number": left_serial_str,
                "publish_rate": DRIVER_PUBLISH_RATE,
                "filter_cutoff_freq": DRIVER_FILTER_CUTOFF_FREQ,
                "diagnostics_rate": DRIVER_DIAGNOSTICS_RATE,
            }],
            output="screen",
            emulate_tty=True,
            condition=LaunchConfigurationEquals("hand_input", "manus"),
        ),
        # Right hand driver (wujihandros2)
        Node(
            package="wujihand_driver",
            executable="wujihand_driver_node",
            name="wujihand_driver",
            namespace=LaunchConfiguration("right_hand_name"),
            parameters=[{
                "serial_number": right_serial_str,
                "publish_rate": DRIVER_PUBLISH_RATE,
                "filter_cutoff_freq": DRIVER_FILTER_CUTOFF_FREQ,
                "diagnostics_rate": DRIVER_DIAGNOSTICS_RATE,
            }],
            output="screen",
            emulate_tty=True,
            condition=LaunchConfigurationEquals("hand_input", "manus"),
        ),

        # ==================== ARM INPUT: Tracker ====================
        # OpenVR Input Node (for tracker arm tracking)
        Node(
            package="openvr_input",
            executable="openvr_input",
            name="openvr_input",
            output="screen",
            arguments=["-c", openvr_config],
            condition=IfCondition(PythonExpression(["'", enable_arm, "' == 'true' and '", arm_input, "' == 'tracker'"])),
        ),

        # ==================== STATIC TF: From Config ====================
        OpaqueFunction(
            function=lambda ctx: create_chest_tf_nodes() + create_tianji_tf_nodes(),
            condition=IfCondition(enable_arm),
        ),

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
            arguments=[PythonExpression(["'--dry-run' if '", dry_run, "' == 'true' else ''"])],
            condition=IfCondition(enable_arm),
        ),

        # ==================== HAND INPUT: Manus ====================
        # Manus ROS2 Driver (USB access via udev rule, no sudo needed)
        Node(
            package="manus_ros2",
            executable="manus_data_publisher",
            name="manus_data_publisher",
            output="screen",
            emulate_tty=True,
            condition=LaunchConfigurationEquals("hand_input", "manus"),
        ),
        # Manus Input Node (convert to MediaPipe format)
        Node(
            package="manus_input_py",
            executable="manus_input",
            name="manus_input",
            output="screen",
            emulate_tty=True,
            condition=LaunchConfigurationEquals("hand_input", "manus"),
        ),
        Node(
            package="wuji_glove_input_py",
            executable="wuji_glove_input",
            name="wuji_glove_input",
            output="screen",
            emulate_tty=True,
            arguments=[
                "--left-glove-sn", LaunchConfiguration("left_glove_sn"),
                "--right-glove-sn", LaunchConfiguration("right_glove_sn"),
                "--left-device-name", LaunchConfiguration("left_device_name"),
                "--right-device-name", LaunchConfiguration("right_device_name"),
                "--include-left-hand", LaunchConfiguration("include_left_hand"),
                "--include-right-hand", LaunchConfiguration("include_right_hand"),
            ],
            condition=LaunchConfigurationEquals("hand_input", "wuji_glove_ros2"),
        ),
        Node(
            package="controller",
            executable="wujihand_direct_controller",
            name="wujihand_direct_left_controller",
            output="screen",
            emulate_tty=True,
            prefix=[_get_python_executable(), " "],
            arguments=[
                "--left-hand-sn", LaunchConfiguration("left_serial"),
                "--left-glove-sn", LaunchConfiguration("left_glove_sn"),
                "--left-device-name", LaunchConfiguration("left_device_name"),
                "--left-retarget-config", LaunchConfiguration("left_retarget_config"),
                "--include-left-hand", "true",
                "--include-right-hand", "false",
            ],
            condition=IfCondition(PythonExpression([
                "'", hand_input, "' == 'wuji_glove' and '",
                LaunchConfiguration("include_left_hand"), "' == 'true'",
            ])),
        ),
        Node(
            package="controller",
            executable="wujihand_direct_controller",
            name="wujihand_direct_right_controller",
            output="screen",
            emulate_tty=True,
            prefix=[_get_python_executable(), " "],
            arguments=[
                "--right-hand-sn", LaunchConfiguration("right_serial"),
                "--right-glove-sn", LaunchConfiguration("right_glove_sn"),
                "--right-device-name", LaunchConfiguration("right_device_name"),
                "--right-retarget-config", LaunchConfiguration("right_retarget_config"),
                "--include-left-hand", "false",
                "--include-right-hand", "true",
            ],
            condition=IfCondition(PythonExpression([
                "'", hand_input, "' == 'wuji_glove' and '",
                LaunchConfiguration("include_right_hand"), "' == 'true'",
            ])),
        ),

        # ==================== HAND OUTPUT: Wuji Hand ====================
        # Wuji Hand Controller
        # Also publishes state: /wuji_hand/left/joint_state, /wuji_hand/right/joint_state
        Node(
            package="controller",
            executable="wujihand_controller",
            name="wujihand_controller",
            output="screen",
            emulate_tty=True,
            arguments=[
                "-c", hand_config,
                "-i", hand_input,
                "--left-hand", LaunchConfiguration("left_hand_name"),
                "--right-hand", LaunchConfiguration("right_hand_name"),
            ],
            condition=LaunchConfigurationEquals("hand_input", "manus"),
        ),

        # ==================== VISUALIZATION ====================
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", _get_rviz_path()],
            condition=IfCondition(enable_rviz),
        ),
    ])
