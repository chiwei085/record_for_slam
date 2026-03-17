"""
Launch RealSense D455 + ros2 bag record + RViz2 for RTABMap SLAM.

Starts the realsense2_camera node with settings tuned for RTABMap, opens
RViz2 for live preview, then launches ros2 bag record after a short delay.

Usage:
  ros2 launch record_for_slam bag_for_slam.launch.py
  ros2 launch record_for_slam bag_for_slam.launch.py bag_prefix:=outdoor_run
  ros2 launch record_for_slam bag_for_slam.launch.py enable_rviz:=false
  ros2 launch record_for_slam bag_for_slam.launch.py enable_pointcloud:=true
  # bags saved to: src/record_for_slam/bags/<prefix>_YYYYMMDD_HHMMSS/

Recorded topics (RTABMap core):
  /camera/color/image_raw
  /camera/color/camera_info
  /camera/depth/image_rect_raw
  /camera/depth/camera_info
  /camera/aligned_depth_to_color/image_raw
  /camera/aligned_depth_to_color/camera_info
  /camera/imu
  /tf  /tf_static

Optional:
  /camera/depth/color/points   (enable via enable_pointcloud:=true)
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
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

# Resolved at import time.
# os.path.realpath (via Path.resolve) follows symlinks created by
# `colcon build --symlink-install`, so this always points to the source tree.
_PKG_DIR  = Path(__file__).resolve().parent.parent   # src/record_for_slam/
_BAGS_DIR = _PKG_DIR / 'bags'
_RVIZ_CFG = _PKG_DIR / 'config' / 'slam_preview.rviz'


# ---------------------------------------------------------------------------
# Core topics (always recorded)
# ---------------------------------------------------------------------------

CORE_TOPICS = [
    '/camera/color/image_raw',
    '/camera/color/camera_info',
    '/camera/depth/image_rect_raw',
    '/camera/depth/camera_info',
    '/camera/aligned_depth_to_color/image_raw',
    '/camera/aligned_depth_to_color/camera_info',
    '/camera/imu',
    '/tf',
    '/tf_static',
]

POINTCLOUD_TOPIC = '/camera/depth/color/points'


# ---------------------------------------------------------------------------
# generate_launch_description
# ---------------------------------------------------------------------------

def generate_launch_description():

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------

    bag_prefix_arg = DeclareLaunchArgument(
        'bag_prefix',
        default_value='slam',
        description='Prefix for the output bag directory (saved to <pkg>/bags/)',
    )

    camera_delay_arg = DeclareLaunchArgument(
        'camera_delay',
        default_value='4.0',
        description='Seconds to wait after camera node starts before recording begins',
    )

    enable_pointcloud_arg = DeclareLaunchArgument(
        'enable_pointcloud',
        default_value='false',
        description='Also record /camera/depth/color/points (increases bag size)',
    )

    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Open RViz2 for live dataflow preview',
    )

    storage_arg = DeclareLaunchArgument(
        'storage',
        default_value='mcap',
        description='rosbag2 storage plugin: mcap (recommended) or sqlite3',
    )

    # ------------------------------------------------------------------
    # RealSense camera node (via rs_launch.py)
    # ------------------------------------------------------------------

    rs_launch_path = PathJoinSubstitution([
        FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'
    ])

    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch_path),
        launch_arguments={
            # -- Stream enables --
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_gyro':  'true',
            'enable_accel': 'true',

            # -- IMU: merge gyro+accel → /camera/imu --
            # 0=None, 1=copy, 2=linear_interpolation (recommended)
            'unite_imu_method': '2',

            # -- Sync color & depth frames (important for RTABMap) --
            'enable_sync': 'true',

            # -- Color: 640x480 @ 30 fps --
            'rgb_camera.color_profile': '640x480x30',
            'rgb_camera.color_format':  'RGB8',

            # -- Depth: 640x480 @ 30 fps --
            'depth_module.depth_profile': '640x480x30',
            'depth_module.depth_format':  'Z16',

            # -- Align depth to color frame (RTABMap-friendly) --
            'align_depth.enable': 'true',

            # -- Point cloud (controlled by launch arg) --
            'pointcloud.enable': LaunchConfiguration('enable_pointcloud'),

            # -- TF --
            'publish_tf':      'true',
            'tf_publish_rate': '15.0',  # dynamic TF at 15 Hz

            # -- Namespace --
            # namespace='' + name='camera' → node FQN /camera
            # → topics at /camera/color/image_raw  (single-level prefix)
            # namespace='camera' + name='camera' → FQN /camera/camera
            # → topics at /camera/camera/color/... (double-prefix, wrong)
            'camera_namespace': '',
            'camera_name':      'camera',
        }.items(),
    )

    # ------------------------------------------------------------------
    # RViz2
    # ------------------------------------------------------------------

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(_RVIZ_CFG)],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_rviz')),
    )

    # ------------------------------------------------------------------
    # ros2 bag record (delayed to let the camera node come up)
    # ------------------------------------------------------------------

    qos_yaml = PathJoinSubstitution([
        FindPackageShare('record_for_slam'), 'config', 'qos_override.yaml'
    ])

    bag_output = PythonExpression([
        f"r'{_BAGS_DIR}/' + '",
        LaunchConfiguration('bag_prefix'),
        "' + '_' + __import__('datetime').datetime.now().strftime('%Y%m%d_%H%M%S')",
    ])

    # Two separate ExecuteProcess actions gated by IfCondition because
    # we can't conditionally extend a plain list with LaunchConfiguration
    # at description-generation time.

    record_no_pc = TimerAction(
        period=LaunchConfiguration('camera_delay'),
        actions=[
            LogInfo(msg=['[bag_for_slam] Starting bag record → ', bag_output]),
            ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'record',
                    '--storage', LaunchConfiguration('storage'),
                    '--qos-profile-overrides-path', qos_yaml,
                    '--output', bag_output,
                ] + CORE_TOPICS,
                output='screen',
                name='ros2_bag_record',
                condition=IfCondition(
                    PythonExpression(["'", LaunchConfiguration('enable_pointcloud'), "' == 'false'"])
                ),
            ),
        ],
    )

    record_with_pc = TimerAction(
        period=LaunchConfiguration('camera_delay'),
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'record',
                    '--storage', LaunchConfiguration('storage'),
                    '--qos-profile-overrides-path', qos_yaml,
                    '--output', bag_output,
                ] + CORE_TOPICS + [POINTCLOUD_TOPIC],
                output='screen',
                name='ros2_bag_record_pc',
                condition=IfCondition(
                    PythonExpression(["'", LaunchConfiguration('enable_pointcloud'), "' == 'true'"])
                ),
            ),
        ],
    )

    # ------------------------------------------------------------------
    # LaunchDescription
    # ------------------------------------------------------------------

    return LaunchDescription([
        bag_prefix_arg,
        camera_delay_arg,
        enable_pointcloud_arg,
        enable_rviz_arg,
        storage_arg,

        LogInfo(msg='[bag_for_slam] Starting RealSense D455 node...'),
        realsense_node,
        rviz_node,

        LogInfo(msg=[
            '[bag_for_slam] Camera started. Bag recording will begin in ',
            LaunchConfiguration('camera_delay'),
            ' seconds...',
        ]),

        record_no_pc,
        record_with_pc,
    ])
