"""
Launch driver bringup, recorder gating, and bag capture for RTABMap SLAM.

Usage:
  ros2 launch record_for_slam bag_for_slam.launch.py
  ros2 launch record_for_slam bag_for_slam.launch.py bag_prefix:=outdoor_run
  ros2 launch record_for_slam bag_for_slam.launch.py enable_rviz:=false
  ros2 launch record_for_slam bag_for_slam.launch.py enable_pointcloud:=true
  ros2 launch record_for_slam bag_for_slam.launch.py fps:=15
  ros2 launch record_for_slam bag_for_slam.launch.py driver_bringup:=none
  # bags saved to: src/record_for_slam/bags/<prefix>_YYYYMMDD_HHMMSS/

Recorded topics are selected from the active camera profile YAML.
"""

from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_LAUNCH_DIR = Path(__file__).resolve().parent
if str(_LAUNCH_DIR) not in sys.path:
    sys.path.insert(0, str(_LAUNCH_DIR))

from _camera_profile import (
    build_camera_profile_logs,
    resolve_camera_profile,
    resolve_use_point_cloud,
)

# Resolved at import time.
# os.path.realpath (via Path.resolve) follows symlinks created by
# `colcon build --symlink-install`, so this always points to the source tree.
_PKG_DIR = Path(__file__).resolve().parent.parent  # src/record_for_slam/
_BAGS_DIR = _PKG_DIR / "bags"
_RVIZ_CFG = _PKG_DIR / "config" / "slam_preview.rviz"


def _resolve_camera_inputs(context):
    (camera_profile, camera_config, config_path, camera, warnings,
     loaded_key_count) = resolve_camera_profile(
         context,
         _PKG_DIR,
         LaunchConfiguration("camera_profile"),
         LaunchConfiguration("camera_config"),
     )
    use_point_cloud = resolve_use_point_cloud(
        context, LaunchConfiguration("enable_pointcloud"), camera
    )
    return (
        camera_profile,
        camera_config,
        config_path,
        camera,
        warnings,
        loaded_key_count,
        use_point_cloud,
    )


def _build_driver_bringup_actions(context):
    camera_profile, camera_config, _, _, _, _, use_point_cloud = _resolve_camera_inputs(context)
    driver_bringup = context.perform_substitution(LaunchConfiguration("driver_bringup"))
    fps = context.perform_substitution(LaunchConfiguration("fps"))

    if driver_bringup == "none":
        return [LogInfo(msg="[bag_for_slam] driver_bringup=none; expecting an external camera driver")]

    if driver_bringup == "auto":
        if camera_config:
            return [
                LogInfo(
                    msg=(
                        "[bag_for_slam][WARN] camera_config is set, so driver_bringup:=auto falls back to none. "
                        "Start the camera driver yourself or set driver_bringup:=/abs/path/to/driver.launch.py"
                    )
                )
            ]

        bringup_path = _LAUNCH_DIR / f"bringup_{camera_profile}.launch.py"
        if not bringup_path.is_file():
            return [
                LogInfo(
                    msg=(
                        f"[bag_for_slam][WARN] No built-in driver bringup for camera_profile='{camera_profile}'. "
                        "Start the camera driver manually or set driver_bringup:=/abs/path/to/driver.launch.py"
                    )
                )
            ]
    else:
        bringup_path = Path(driver_bringup)
        if not bringup_path.is_file():
            raise FileNotFoundError(f"driver_bringup launch file not found: {bringup_path}")

    return [
        LogInfo(msg=f"[bag_for_slam] Starting driver bringup: {bringup_path}"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup_path)),
            launch_arguments={
                "fps": fps,
                "enable_pointcloud": "true" if use_point_cloud else "false",
            }.items(),
        ),
    ]


def _build_camera_input_actions(context, qos_yaml, bag_output):
    _, _, config_path, camera, warnings, loaded_key_count, use_point_cloud = _resolve_camera_inputs(context)

    record_topics = [
        topic
        for topic in [
            camera["rgb_image_topic"],
            camera["rgb_camera_info_topic"],
            camera["depth_image_topic"],
            camera["depth_camera_info_topic"],
            camera["imu_raw_topic"],
            camera["imu_topic"],
            "/tf",
            "/tf_static",
        ]
        if topic
    ]
    if use_point_cloud and camera["point_cloud_topic"]:
        record_topics.append(camera["point_cloud_topic"])

    actions = build_camera_profile_logs(
        "bag_for_slam",
        config_path,
        camera,
        warnings,
        loaded_key_count,
        use_point_cloud=use_point_cloud,
    )
    actions.append(
        Node(
            package="record_for_slam",
            executable="record_gate",
            name="record_gate",
            output="screen",
            parameters=[{
                "bag_output": bag_output,
                "bag_storage": LaunchConfiguration("storage"),
                "qos_override_path": qos_yaml,
                "record_topics": record_topics,
                "rgb_image_topic": camera["rgb_image_topic"],
                "rgb_camera_info_topic": camera["rgb_camera_info_topic"],
                "depth_image_topic": camera["depth_image_topic"],
                "depth_camera_info_topic": camera["depth_camera_info_topic"],
                "point_cloud_topic": camera["point_cloud_topic"],
                "imu_topic": camera["imu_topic"],
                "use_point_cloud": use_point_cloud,
                "approximate_sync": bool(camera["approximate_sync"]),
                "depth_registered_to_color": bool(camera["depth_registered_to_color"]),
                "require_registered_depth": bool(camera["require_registered_depth"]),
                "require_imu": bool(camera["require_imu"]),
                "qos_profile": camera["qos_profile"],
            }],
        ),
    )
    if camera["imu_raw_topic"] and camera["imu_topic"]:
        actions.append(
            Node(
                package="imu_filter_madgwick",
                executable="imu_filter_madgwick_node",
                name="imu_filter_madgwick",
                output="screen",
                parameters=[
                    {
                        "use_mag": False,
                        "publish_tf": False,
                        "world_frame": "enu",
                        "gain": 0.1,
                        "zeta": 0.0,
                    }
                ],
                remappings=[
                    ("imu/data_raw", camera["imu_raw_topic"]),
                    ("imu/data", camera["imu_topic"]),
                ],
            )
        )
    else:
        actions.append(
            LogInfo(
                msg="[bag_for_slam] IMU filter disabled because imu_raw_topic or imu_topic is empty"
            )
        )
    return actions


# ---------------------------------------------------------------------------
# generate_launch_description
# ---------------------------------------------------------------------------


def generate_launch_description():

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------

    bag_prefix_arg = DeclareLaunchArgument(
        "bag_prefix",
        default_value="slam",
        description="Prefix for the output bag directory (saved to <pkg>/bags/)",
    )

    enable_pointcloud_arg = DeclareLaunchArgument(
        "enable_pointcloud",
        default_value="",
        description="Override use_point_cloud from camera profile: true, false, or empty to follow YAML",
    )

    fps_arg = DeclareLaunchArgument(
        "fps",
        default_value="30",
        description="Camera framerate: 30 or 15 (use 15 if USB bandwidth causes frame drops)",
    )

    enable_rviz_arg = DeclareLaunchArgument(
        "enable_rviz",
        default_value="true",
        description="Open RViz2 for live dataflow preview",
    )

    storage_arg = DeclareLaunchArgument(
        "storage",
        default_value="mcap",
        description="rosbag2 storage plugin: mcap (recommended) or sqlite3",
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

    driver_bringup_arg = DeclareLaunchArgument(
        "driver_bringup",
        default_value="auto",
        description="auto: use bringup_<camera_profile>.launch.py, none: expect external driver, or absolute path to a launch file",
    )

    # ------------------------------------------------------------------
    # RViz2
    # ------------------------------------------------------------------

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", str(_RVIZ_CFG)],
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_rviz")),
    )

    # ------------------------------------------------------------------
    # record_gate — starts the in-process rosbag2 recorder once all sensors are live
    # ------------------------------------------------------------------

    qos_yaml = PathJoinSubstitution(
        [FindPackageShare("record_for_slam"), "config", "qos_override.yaml"]
    )

    bag_output = PythonExpression(
        [
            f"r'{_BAGS_DIR}/' + '",
            LaunchConfiguration("bag_prefix"),
            "' + '_' + __import__('datetime').datetime.now().strftime('%Y%m%d_%H%M%S')",
        ]
    )

    camera_input = OpaqueFunction(
        function=_build_camera_input_actions,
        kwargs={
            "qos_yaml": qos_yaml,
            "bag_output": bag_output,
        },
    )

    driver_input = OpaqueFunction(function=_build_driver_bringup_actions)

    # ------------------------------------------------------------------
    # LaunchDescription
    # ------------------------------------------------------------------

    return LaunchDescription(
        [
            bag_prefix_arg,
            fps_arg,
            enable_pointcloud_arg,
            enable_rviz_arg,
            storage_arg,
            camera_profile_arg,
            camera_config_arg,
            driver_bringup_arg,
            driver_input,
            rviz_node,
            LogInfo(msg="[bag_for_slam] Waiting for all sensors before recording..."),
            camera_input,
        ]
    )
