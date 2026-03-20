"""
Launch RealSense D455 + rosbag2 recorder + RViz2 for RTABMap SLAM.

Starts the realsense2_camera node with settings tuned for RTABMap, opens
RViz2 for live preview, then starts a record_gate node that waits for all
sensor topics to confirm they are actually publishing before starting an
in-process rosbag2 recorder. This avoids recording incomplete data at startup.

Usage:
  ros2 launch record_for_slam bag_for_slam.launch.py
  ros2 launch record_for_slam bag_for_slam.launch.py bag_prefix:=outdoor_run
  ros2 launch record_for_slam bag_for_slam.launch.py enable_rviz:=false
  ros2 launch record_for_slam bag_for_slam.launch.py enable_pointcloud:=true
  ros2 launch record_for_slam bag_for_slam.launch.py fps:=15
  # bags saved to: src/record_for_slam/bags/<prefix>_YYYYMMDD_HHMMSS/

Recorded topics (RTABMap core):
  /camera/color/image_raw
  /camera/color/camera_info
  /camera/aligned_depth_to_color/image_raw   (depth aligned to color frame)
  /camera/aligned_depth_to_color/camera_info
  /camera/imu
  /imu/filtered
  /tf  /tf_static

Optional:
  /camera/depth/color/points   (enable via enable_pointcloud:=true)
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Resolved at import time.
# os.path.realpath (via Path.resolve) follows symlinks created by
# `colcon build --symlink-install`, so this always points to the source tree.
_PKG_DIR = Path(__file__).resolve().parent.parent  # src/record_for_slam/
_BAGS_DIR = _PKG_DIR / "bags"
_RVIZ_CFG = _PKG_DIR / "config" / "slam_preview.rviz"


# ---------------------------------------------------------------------------
# Core topics (always recorded)
# ---------------------------------------------------------------------------

CORE_TOPICS = [
    "/camera/color/image_raw",
    "/camera/color/camera_info",
    "/camera/aligned_depth_to_color/image_raw",
    "/camera/aligned_depth_to_color/camera_info",
    "/camera/imu",
    "/imu/filtered",  # orientation-fused IMU from imu_filter_madgwick
    "/tf",
    "/tf_static",
]

POINTCLOUD_TOPIC = "/camera/depth/color/points"


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
        default_value="false",
        description="Also record /camera/depth/color/points (increases bag size)",
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

    # ------------------------------------------------------------------
    # RealSense camera node (via rs_launch.py)
    # ------------------------------------------------------------------

    rs_launch_path = PathJoinSubstitution(
        [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
    )

    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch_path),
        launch_arguments={
            # -- Stream enables --
            "enable_color": "true",
            "enable_depth": "true",
            "enable_gyro": "true",
            "enable_accel": "true",
            # -- IMU: merge gyro+accel → /camera/imu --
            # 0=None, 1=copy, 2=linear_interpolation (recommended)
            "unite_imu_method": "2",
            # -- Sync color & depth frames (important for RTABMap) --
            "enable_sync": "true",
            # -- Color: 640x480 @ fps (30 or 15 via fps:= arg) --
            "rgb_camera.color_profile": PythonExpression(
                ["'640x480x' + '", LaunchConfiguration("fps"), "'"]
            ),
            "rgb_camera.color_format": "RGB8",
            # -- Depth: 640x480 @ fps --
            "depth_module.depth_profile": PythonExpression(
                ["'640x480x' + '", LaunchConfiguration("fps"), "'"]
            ),
            "depth_module.depth_format": "Z16",
            # -- Align depth to color frame (RTABMap-friendly) --
            "align_depth.enable": "true",
            # -- Point cloud (controlled by launch arg) --
            "pointcloud.enable": LaunchConfiguration("enable_pointcloud"),
            # -- TF --
            # 0 = publish all camera transforms once as static (/tf_static).
            # Dynamic TF (>0) causes "extrapolation into the future" errors
            # during bag playback when RTAB-Map processing lags behind sim time.
            "publish_tf": "true",
            "tf_publish_rate": "0",
            # -- Namespace --
            # namespace='' + name='camera' → node FQN /camera
            # → topics at /camera/color/image_raw  (single-level prefix)
            # namespace='camera' + name='camera' → FQN /camera/camera
            # → topics at /camera/camera/color/... (double-prefix, wrong)
            "camera_namespace": "",
            "camera_name": "camera",
        }.items(),
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
    # IMU orientation filter (Madgwick: gyro+accel → /imu/filtered)
    # D455 raw IMU has no orientation; this adds it for RTABMap gravity-aid.
    # Yaw will drift (no magnetometer) but roll/pitch are accurate.
    # ------------------------------------------------------------------

    imu_filter_node = Node(
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
            ("imu/data_raw", "/camera/imu"),
            ("imu/data", "/imu/filtered"),
        ],
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

    # Two Node instances (with / without point cloud) so that the topic list
    # can be a plain Python literal passed as a parameter.
    record_gate_no_pc = Node(
        package="record_for_slam",
        executable="record_gate",
        name="record_gate",
        output="screen",
        parameters=[{
            "bag_output":        bag_output,
            "bag_storage":       LaunchConfiguration("storage"),
            "qos_override_path": qos_yaml,
            "record_topics":     CORE_TOPICS,
        }],
        condition=UnlessCondition(LaunchConfiguration("enable_pointcloud")),
    )

    record_gate_with_pc = Node(
        package="record_for_slam",
        executable="record_gate",
        name="record_gate",
        output="screen",
        parameters=[{
            "bag_output":        bag_output,
            "bag_storage":       LaunchConfiguration("storage"),
            "qos_override_path": qos_yaml,
            "record_topics":     CORE_TOPICS + [POINTCLOUD_TOPIC],
        }],
        condition=IfCondition(LaunchConfiguration("enable_pointcloud")),
    )

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
            LogInfo(msg="[bag_for_slam] Starting RealSense D455 node..."),
            realsense_node,
            imu_filter_node,
            rviz_node,
            LogInfo(msg="[bag_for_slam] Waiting for all sensors before recording..."),
            record_gate_no_pc,
            record_gate_with_pc,
        ]
    )
