"""
Bring up a RealSense RGB-D camera with settings tuned for RTAB-Map bag capture.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fps_arg = DeclareLaunchArgument(
        "fps",
        default_value="30",
        description="Camera framerate: 30 or 15",
    )

    enable_pointcloud_arg = DeclareLaunchArgument(
        "enable_pointcloud",
        default_value="false",
        description="Enable RealSense driver point cloud publication",
    )

    rs_launch_path = PathJoinSubstitution(
        [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
    )

    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch_path),
        launch_arguments={
            "enable_color": "true",
            "enable_depth": "true",
            "enable_gyro": "true",
            "enable_accel": "true",
            # Merge gyro+accel into /camera/imu for downstream IMU filtering.
            "unite_imu_method": "2",
            # Sync color/depth capture on the driver side for bag recording.
            "enable_sync": "true",
            "rgb_camera.color_profile": PythonExpression(
                ["'640x480x' + '", LaunchConfiguration("fps"), "'"]
            ),
            "rgb_camera.color_format": "RGB8",
            "depth_module.depth_profile": PythonExpression(
                ["'640x480x' + '", LaunchConfiguration("fps"), "'"]
            ),
            "depth_module.depth_format": "Z16",
            # RealSense-specific aligned depth output used by the default profile.
            "align_depth.enable": "true",
            "pointcloud.enable": LaunchConfiguration("enable_pointcloud"),
            # Publish camera TFs only once on /tf_static. Dynamic TF during bag replay
            # can trigger "extrapolation into the future" errors when processing lags.
            "publish_tf": "true",
            "tf_publish_rate": "0",
            # namespace='' + name='camera' yields /camera/... topics instead of
            # /camera/camera/... double-prefixing.
            "camera_namespace": "",
            "camera_name": "camera",
        }.items(),
    )

    return LaunchDescription([fps_arg, enable_pointcloud_arg, realsense_node])
