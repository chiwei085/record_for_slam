"""
Usage:
  ros2 launch record_for_slam playback.launch.py bag:=slam_xxxxxxxx_yyyyyy
  ros2 launch record_for_slam playback.launch.py bag:=slam_xxxxxxxx_yyyyyy rate:=0.5
  ros2 launch record_for_slam playback.launch.py bag:=slam_xxxxxxxx_yyyyyy enable_rviz:=false
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_PKG_DIR = Path(__file__).resolve().parent.parent
_BAGS_DIR = _PKG_DIR / "bags"
_RVIZ_CFG = _PKG_DIR / "config" / "slam_preview.rviz"


def generate_launch_description():

    bag_arg = DeclareLaunchArgument(
        "bag",
        description="Bag directory name under <pkg>/bags/ (e.g. slam_20260318_033916)",
    )

    rate_arg = DeclareLaunchArgument(
        "rate",
        default_value="1.0",
        description="Playback speed multiplier (0.5 = half speed)",
    )

    loop_arg = DeclareLaunchArgument(
        "loop",
        default_value="false",
        description="Loop playback",
    )

    enable_rviz_arg = DeclareLaunchArgument(
        "enable_rviz",
        default_value="true",
        description="Open RViz2 for preview",
    )

    qos_yaml = PathJoinSubstitution(
        [FindPackageShare("record_for_slam"), "config", "qos_override.yaml"]
    )

    bag_path = PythonExpression(
        [f"r'{_BAGS_DIR}/' + '", LaunchConfiguration("bag"), "'"]
    )

    playback = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            bag_path,
            "--clock",
            "--rate",
            LaunchConfiguration("rate"),
            "--qos-profile-overrides-path",
            qos_yaml,
        ],
        output="screen",
        name="ros2_bag_play",
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("loop"), "' == 'false'"])
        ),
    )

    playback_loop = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            bag_path,
            "--clock",
            "--rate",
            LaunchConfiguration("rate"),
            "--loop",
            "--qos-profile-overrides-path",
            qos_yaml,
        ],
        output="screen",
        name="ros2_bag_play_loop",
        condition=IfCondition(LaunchConfiguration("loop")),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", str(_RVIZ_CFG)],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_rviz")),
    )

    return LaunchDescription(
        [
            bag_arg,
            rate_arg,
            loop_arg,
            enable_rviz_arg,
            LogInfo(msg=["[playback] Playing bag → ", bag_path]),
            rviz_node,
            playback,
            playback_loop,
        ]
    )
