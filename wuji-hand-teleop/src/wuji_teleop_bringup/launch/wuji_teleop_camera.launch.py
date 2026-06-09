"""
Wuji Teleoperation with Camera Launch File

Launches all teleoperation components plus camera system.

Usage:
    # Basic usage with default camera config
    ros2 launch wuji_teleop_bringup wuji_teleop_camera.launch.py

    # With custom camera config
    ros2 launch wuji_teleop_bringup wuji_teleop_camera.launch.py camera_config:=/path/to/config.yaml

    # Disable camera
    ros2 launch wuji_teleop_bringup wuji_teleop_camera.launch.py enable_camera:=false
"""

from __future__ import annotations

import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from wuji_teleop_bringup.hand_defaults import (
    LEFT_HAND_SERIAL, RIGHT_HAND_SERIAL,
    LEFT_HAND_NAME, RIGHT_HAND_NAME,
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


# Stereo head camera is now integrated via camera_launch.py enable_head parameter
# Integrated via IncludeLaunchDescription



def generate_launch_description() -> LaunchDescription:
    # ==================== Launch Arguments ====================
    hand_input_arg = DeclareLaunchArgument(
        "hand_input",
        default_value="none",
        description="Hand input device: 'none', 'wuji_glove', or 'wuji_glove_ros2'",
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
    sdk_executor_enable_arg = DeclareLaunchArgument(
        "sdk_executor_enable",
        default_value="false",
        description="Enable separated Tianji SDK executor node (real robot)",
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

    # Camera arguments
    enable_camera_arg = DeclareLaunchArgument(
        "enable_camera",
        default_value="true",
        description="Enable camera system",
    )
    camera_config_arg = DeclareLaunchArgument(
        "camera_config",
        default_value=_get_config_path("camera", "camera_config.yaml"),
        description="Path to camera config file",
    )

    # Stereo head camera arguments
    enable_head_arg = DeclareLaunchArgument(
        "enable_head",
        default_value="true",
        description="Enable stereo head camera ROS2 publisher",
    )
    head_device_arg = DeclareLaunchArgument(
        "head_device",
        default_value="/dev/stereo_camera",
        description="Stereo head camera device path",
    )
    head_fps_arg = DeclareLaunchArgument(
        "head_fps",
        default_value="30",
        description="Stereo head camera frame rate",
    )
    head_quality_arg = DeclareLaunchArgument(
        "head_quality",
        default_value="85",
        description="Stereo head camera JPEG quality (1-100)",
    )

    # Get configurations
    hand_input = LaunchConfiguration("hand_input")
    arm_input = LaunchConfiguration("arm_input")
    enable_arm = LaunchConfiguration("enable_arm")
    enable_rviz = LaunchConfiguration("enable_rviz")
    openvr_config = LaunchConfiguration("openvr_config")
    controller_config = LaunchConfiguration("controller_config")
    dry_run = LaunchConfiguration("dry_run")
    read_only = LaunchConfiguration("read_only")
    feedback_handshake = LaunchConfiguration("feedback_handshake")
    sim_viz = LaunchConfiguration("sim_viz")
    sdk_executor_enable = LaunchConfiguration("sdk_executor_enable")
    hand_config = LaunchConfiguration("hand_config")
    enable_camera = LaunchConfiguration("enable_camera")
    camera_config = LaunchConfiguration("camera_config")
    enable_head = LaunchConfiguration("enable_head")
    head_device = LaunchConfiguration("head_device")
    head_fps = LaunchConfiguration("head_fps")
    head_quality = LaunchConfiguration("head_quality")

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
        openvr_config_arg,
        controller_config_arg,
        dry_run_arg,
        read_only_arg,
        feedback_handshake_arg,
        sim_viz_arg,
        sdk_executor_enable_arg,
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
        enable_camera_arg,
        camera_config_arg,
        enable_head_arg,
        head_device_arg,
        head_fps_arg,
        head_quality_arg,

        # ==================== CAMERAS (unified entry: camera_launch.py) ====================
        # Head stereo + wrist D405, enable_pico not passed (SteamVR scheme has no PICO H.264)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("camera"),
                    "launch",
                    "camera_launch.py"
                ])
            ]),
            launch_arguments={
                "config_file": camera_config,
                "enable_head": enable_head,
                "head_device": head_device,
                "head_fps": head_fps,
                "head_quality": head_quality,
                # enable_pico not passed, defaults to false (SteamVR scheme has no PICO H.264 streaming)
            }.items(),
            condition=IfCondition(enable_camera),
        ),

        # ==================== ARM INPUT: Tracker ====================
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
            condition=IfCondition(enable_arm),
        ),
        Node(
            package="controller",
            executable="tianji_tracker_sim_viz",
            name="tianji_tracker_sim_viz",
            output="screen",
            emulate_tty=True,
            condition=IfCondition(PythonExpression(["'", enable_arm, "' == 'true' and '", sim_viz, "' == 'true'"])),
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
            condition=IfCondition(PythonExpression(["'", enable_arm, "' == 'true' and '", sdk_executor_enable, "' == 'true'"])),
        ),

        # ==================== HAND INPUT: Wuji Glove (ROS topic path) ====================
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

        # ==================== HAND OUTPUT: Wuji Hand (ROS topic path) ====================
        Node(
            package="controller",
            executable="wujihand_controller",
            name="wujihand_controller",
            output="screen",
            emulate_tty=True,
            arguments=[
                "-c", hand_config,
                "-i", "wuji_glove",
                "--left-hand", LaunchConfiguration("left_hand_name"),
                "--right-hand", LaunchConfiguration("right_hand_name"),
            ],
            condition=LaunchConfigurationEquals("hand_input", "wuji_glove_ros2"),
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
