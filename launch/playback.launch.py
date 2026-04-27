"""
Usage:
  ros2 launch record_for_slam playback.launch.py bag:=slam_xxxxxxxx_yyyyyy
  ros2 launch record_for_slam playback.launch.py bag:=slam_xxxxxxxx_yyyyyy rate:=0.5
  ros2 launch record_for_slam playback.launch.py bag:=slam_xxxxxxxx_yyyyyy enable_rviz:=false
  ros2 launch record_for_slam playback.launch.py bag:=slam_xxxxxxxx_yyyyyy image_focus_rviz:=false
  ros2 launch record_for_slam playback.launch.py bag:=slam_xxxxxxxx_yyyyyy camera_profile:=yahboom_astra
"""

from pathlib import Path
import atexit
import sys
import tempfile

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml

_PKG_DIR = Path(__file__).resolve().parent.parent
_BAGS_DIR = _PKG_DIR / "bags"
_RVIZ_CFG = _PKG_DIR / "config" / "slam_preview.rviz"
_RVIZ_IMAGE_FOCUS_CFG = _PKG_DIR / "config" / "slam_preview_image_focus.rviz"
_LAUNCH_DIR = Path(__file__).resolve().parent
if str(_LAUNCH_DIR) not in sys.path:
    sys.path.insert(0, str(_LAUNCH_DIR))

from _camera_profile import (
    build_camera_profile_logs,
    resolve_camera_profile,
)


def _rewrite_rviz_topics(template_path: Path, camera: dict) -> str:
    with template_path.open("r", encoding="utf-8") as handle:
        rviz_config = yaml.safe_load(handle)

    displays = rviz_config.get("Visualization Manager", {}).get("Displays", [])
    for display in displays:
        name = display.get("Name")
        topic = display.get("Topic")
        if not isinstance(topic, dict):
            continue
        if name == "Color Image":
            topic["Value"] = camera["rgb_image_topic"]
        elif name == "Depth Image":
            topic["Value"] = camera["depth_image_topic"]
        elif name == "PointCloud2" and camera["point_cloud_topic"]:
            topic["Value"] = camera["point_cloud_topic"]

    with tempfile.NamedTemporaryFile(
        mode="w",
        prefix=f"{template_path.stem}_",
        suffix=".rviz",
        delete=False,
        encoding="utf-8",
    ) as handle:
        yaml.safe_dump(rviz_config, handle, sort_keys=False)
        atexit.register(lambda path=handle.name: Path(path).unlink(missing_ok=True))
        return handle.name


def _build_rviz_node(context):
    _, _, config_path, camera, warnings, loaded_key_count = resolve_camera_profile(
        context,
        _PKG_DIR,
        LaunchConfiguration("camera_profile"),
        LaunchConfiguration("camera_config"),
    )
    image_focus = (
        context.perform_substitution(LaunchConfiguration("image_focus_rviz")).lower()
        == "true"
    )
    template_path = _RVIZ_IMAGE_FOCUS_CFG if image_focus else _RVIZ_CFG
    rviz_cfg = _rewrite_rviz_topics(template_path, camera)
    actions = build_camera_profile_logs(
        "record_for_slam/playback",
        config_path,
        camera,
        warnings,
        loaded_key_count,
    )
    actions.append(LogInfo(msg=f"[record_for_slam/playback] RViz config: {rviz_cfg}"))
    actions.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_cfg],
            parameters=[{"use_sim_time": True}],
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_rviz")),
        )
    )
    return actions


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

    image_focus_rviz_arg = DeclareLaunchArgument(
        "image_focus_rviz",
        default_value="true",
        description="Use an RViz layout focused on RGB and depth images",
    )

    camera_profile_arg = DeclareLaunchArgument(
        "camera_profile",
        default_value="realsense",
        description="Camera profile name under config/, e.g. realsense or yahboom_astra",
    )

    camera_config_arg = DeclareLaunchArgument(
        "camera_config",
        default_value="",
        description="Absolute path to a camera profile YAML. Overrides camera_profile when set.",
    )

    qos_yaml = PathJoinSubstitution(
        [FindPackageShare("record_for_slam"), "config", "qos_override.yaml"]
    )

    bag_path = PythonExpression(
        [
            "__import__('pathlib').Path('",
            LaunchConfiguration("bag"),
            "').is_absolute() and '",
            LaunchConfiguration("bag"),
            "' or r'",
            str(_BAGS_DIR) + "/",
            "' + '",
            LaunchConfiguration("bag"),
            "'",
        ]
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

    rviz_node = OpaqueFunction(function=_build_rviz_node)

    return LaunchDescription(
        [
            bag_arg,
            rate_arg,
            loop_arg,
            enable_rviz_arg,
            image_focus_rviz_arg,
            camera_profile_arg,
            camera_config_arg,
            LogInfo(msg=["[playback] Playing bag -> ", bag_path]),
            rviz_node,
            playback,
            playback_loop,
        ]
    )
