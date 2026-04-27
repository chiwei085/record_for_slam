"""
Bring up ROSMASTER R2 base control and joystick teleop for bag recording.

This launch intentionally uses yahboom_joy_R2_v2 instead of Yahboom's stock
yahboom_joy_R2 executable.

The EKF here is part of the robot-control stack copied from the validated
on-robot workspace. It is started to match the real vehicle bringup, not
because bag recording itself requires EKF fusion.

This helper remains as a convenience reference. Generic `record_for_slam`
workflows should prefer `robot_bringup:=/abs/path/to/control.launch.py`
instead of relying on camera-driven auto-selection.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_path("yahboomcar_description")
    model_path = description_share / "urdf" / "yahboomcar_R2.urdf.xacro"
    imu_filter_config = (
        get_package_share_path("yahboomcar_bringup")
        / "param"
        / "imu_filter_param.yaml"
    )

    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="false",
        choices=["true", "false"],
        description="Enable joint_state_publisher_gui",
    )
    model_arg = DeclareLaunchArgument(
        "model",
        default_value=str(model_path),
        description="Absolute path to the robot URDF xacro file",
    )
    pub_odom_tf_arg = DeclareLaunchArgument(
        "pub_odom_tf",
        default_value="false",
        description="Publish odom to base_footprint TF from base_node_R2",
    )
    joy_dev_arg = DeclareLaunchArgument(
        "joy_dev",
        default_value="/dev/input/js0",
        description="Joystick device path",
    )
    deadzone_arg = DeclareLaunchArgument(
        "deadzone",
        default_value="0.2",
        description="joy_node deadzone",
    )
    autorepeat_rate_arg = DeclareLaunchArgument(
        "autorepeat_rate",
        default_value="20.0",
        description="joy_node autorepeat rate in Hz",
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=UnlessCondition(LaunchConfiguration("gui")),
        output="screen",
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("gui")),
        output="screen",
    )

    driver_node = Node(
        package="yahboomcar_bringup",
        executable="Ackman_driver_R2",
        output="screen",
    )

    base_node = Node(
        package="yahboomcar_base_node",
        executable="base_node_R2",
        output="screen",
        parameters=[{
            "pub_odom_tf": LaunchConfiguration("pub_odom_tf"),
            "linear_scale_x": 1.0,
            "linear_scale_y": 1.0,
        }],
    )

    imu_filter_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        output="screen",
        parameters=[str(imu_filter_config)],
    )

    ekf_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            str(Path(get_package_share_path("robot_localization")) / "launch"),
            "/ekf_x1_x3_launch.py",
        ])
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[{
            "dev": LaunchConfiguration("joy_dev"),
            "deadzone": LaunchConfiguration("deadzone"),
            "autorepeat_rate": LaunchConfiguration("autorepeat_rate"),
        }],
    )

    yahboom_joy_node = Node(
        package="yahboomcar_ctrl",
        executable="yahboom_joy_R2_v2",
        name="joy_ctrl",
        output="screen",
    )

    return LaunchDescription([
        gui_arg,
        model_arg,
        pub_odom_tf_arg,
        joy_dev_arg,
        deadzone_arg,
        autorepeat_rate_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        driver_node,
        base_node,
        imu_filter_node,
        ekf_node,
        joy_node,
        yahboom_joy_node,
    ])
