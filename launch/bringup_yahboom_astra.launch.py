"""
Bring up the Yahboom/Orbbec Astra RGB-D camera for RTAB-Map bag capture.

By default this launch expects the driver package share to be named
`astra_camera`, matching the on-robot reference workspace. The validated
ROSMaster R2 `record_for_slam` reference used `astra_pro.launch.xml`.
Override the package or launch filename if your ROS 2 Astra driver uses a
different layout.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
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
        description="Enable Astra driver point cloud publication",
    )

    driver_package_arg = DeclareLaunchArgument(
        "driver_package",
        default_value="astra_camera",
        description="ROS 2 package name that provides the Astra launch file",
    )

    driver_launch_file_arg = DeclareLaunchArgument(
        "driver_launch_file",
        default_value="astra_pro.launch.xml",
        description="Launch file under <driver_package>/launch/",
    )

    astra_launch_path = PathJoinSubstitution(
        [
            FindPackageShare(LaunchConfiguration("driver_package")),
            "launch",
            LaunchConfiguration("driver_launch_file"),
        ]
    )

    astra_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(astra_launch_path),
        launch_arguments={
            "camera_name": "camera",
            "enable_color": "true",
            "enable_depth": "true",
            "enable_ir": "false",
            "enable_point_cloud": LaunchConfiguration("enable_pointcloud"),
            "enable_colored_point_cloud": "false",
            "color_width": "640",
            "color_height": "480",
            "color_fps": LaunchConfiguration("fps"),
            "depth_width": "640",
            "depth_height": "480",
            "depth_fps": LaunchConfiguration("fps"),
            "depth_registration": "false",
            "color_depth_synchronization": "true",
            "use_uvc_camera": "true",
            "uvc_vendor_id": "0x2bc5",
            "uvc_product_id": "0x050f",
            "uvc_camera_format": "mjpeg",
            "publish_tf": "true",
            "tf_publish_rate": "0.0",
        }.items(),
    )

    return LaunchDescription([
        fps_arg,
        enable_pointcloud_arg,
        driver_package_arg,
        driver_launch_file_arg,
        astra_node,
    ])
