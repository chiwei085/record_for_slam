"""
Offline RTABMap RGB-D mapping from a recorded bag using a configurable camera profile.

Starts RTABMap nodes first, waits for them to initialize, then plays the bag.
The resulting map database is saved alongside the bag as rtabmap.db.

Usage:
  ros2 launch record_for_slam offline_rtabmap.launch.py bag_path:=/abs/path/to/bag_dir
  ros2 launch record_for_slam offline_rtabmap.launch.py bag_path:=/abs/path/to/bag_dir rate:=0.5
  ros2 launch record_for_slam offline_rtabmap.launch.py bag_path:=/abs/path/to/bag_dir delete_db:=false
  ros2 launch record_for_slam offline_rtabmap.launch.py bag_path:=/abs/path/to/bag_dir viz:=true
"""

from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node

_LAUNCH_DIR = Path(__file__).resolve().parent
if str(_LAUNCH_DIR) not in sys.path:
    sys.path.insert(0, str(_LAUNCH_DIR))

from _camera_profile import (
    build_camera_profile_logs,
    build_rgbd_remappings,
    resolve_camera_profile,
)

_PKG_DIR = Path(__file__).resolve().parent.parent


# ---------------------------------------------------------------------------
# RTABMap node factory (needs OpaqueFunction to gate --delete_db_on_start)
# ---------------------------------------------------------------------------


def _launch_rtabmap_nodes(
    context,
    database_path_sub,
    delete_db_sub,
    viz_sub,
    allow_unregistered_depth_sub,
):
    db_path = context.perform_substitution(database_path_sub)
    delete_db = context.perform_substitution(delete_db_sub).lower() == "true"
    viz_enabled = context.perform_substitution(viz_sub).lower() == "true"
    allow_unregistered_depth = (
        context.perform_substitution(allow_unregistered_depth_sub).lower()
        == "true"
    )
    extra_args = ["--delete_db_on_start"] if delete_db else []
    _, _, config_path, camera, warnings, loaded_key_count = resolve_camera_profile(
        context,
        _PKG_DIR,
        LaunchConfiguration("camera_profile"),
        LaunchConfiguration("camera_config"),
    )
    remappings = build_rgbd_remappings(camera)
    use_rgbd_sync = bool(camera["use_rgbd_sync"])
    replay_approx_sync_max_interval = float(camera["replay_approx_sync_max_interval"])
    actions = build_camera_profile_logs(
        "record_for_slam/offline_rtabmap",
        config_path,
        camera,
        warnings,
        loaded_key_count,
        include_replay_params=True,
    )

    requires_registered_depth = bool(camera["require_registered_depth"])
    depth_registered_to_color = bool(camera["depth_registered_to_color"])
    if requires_registered_depth and not depth_registered_to_color:
        if not allow_unregistered_depth:
            raise RuntimeError(
                "Selected camera profile marks depth as not registered to color while offline RTAB-Map "
                "requires aligned RGB-D input. For Astra/Orbbec, verify the driver and enable depth "
                "registration first, then update the camera profile. To bypass this guard for debugging only, "
                "set allow_unregistered_depth:=true."
            )
        actions.append(
            LogInfo(
                msg=(
                    "[record_for_slam/offline_rtabmap][WARN] allow_unregistered_depth=true overrides the "
                    "profile safety check; replay results cannot be used to blame recording quality alone"
                )
            )
        )
    elif not depth_registered_to_color:
        actions.append(
            LogInfo(
                msg=(
                    "[record_for_slam/offline_rtabmap][WARN] "
                    "depth_registered_to_color=false and require_registered_depth=false; replay may still fail "
                    "if downstream RGB-D consumers assume color-depth pixel alignment"
                )
            )
        )

    if use_rgbd_sync:
        actions.append(
            Node(
                package="rtabmap_sync",
                executable="rgbd_sync",
                name="rgbd_sync",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "approx_sync": bool(camera["approximate_sync"]),
                        "approx_sync_max_interval": replay_approx_sync_max_interval,
                        "qos": 2,
                        "qos_camera_info": 2,
                    }
                ],
                remappings=[
                    ("rgb/image", camera["rgb_image_topic"]),
                    ("depth/image", camera["depth_image_topic"]),
                    ("rgb/camera_info", camera["rgb_camera_info_topic"]),
                    ("depth/camera_info", camera["depth_camera_info_topic"]),
                    ("rgbd_image", "rgbd_image"),
                ],
            )
        )

    rgbd_odometry_params = {
        "use_sim_time": True,
        "subscribe_rgb": not use_rgbd_sync,
        "subscribe_depth": not use_rgbd_sync,
        "subscribe_rgbd": use_rgbd_sync,
        "subscribe_imu": bool(camera["require_imu"] and camera["imu_topic"]),
        "wait_imu_to_init": bool(camera["require_imu"] and camera["imu_topic"]),
        "approx_sync": bool(camera["approximate_sync"]),
        "approx_sync_max_interval": replay_approx_sync_max_interval,
        "queue_size": 20,
        "frame_id": "camera_link",
    }
    rgbd_odometry_remappings = [("rgbd_image", "rgbd_image")] if use_rgbd_sync else list(remappings)
    if use_rgbd_sync and camera["require_imu"] and camera["imu_topic"]:
        rgbd_odometry_remappings.append(("imu", camera["imu_topic"]))
    rgbd_odometry = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        name="rgbd_odometry",
        output="screen",
        parameters=[rgbd_odometry_params],
        remappings=rgbd_odometry_remappings,
    )

    rtabmap_params = {
        "use_sim_time": True,
        "subscribe_depth": not use_rgbd_sync,
        "subscribe_rgb": not use_rgbd_sync,
        "subscribe_rgbd": use_rgbd_sync,
        "approx_sync": bool(camera["approximate_sync"]),
        "approx_sync_max_interval": replay_approx_sync_max_interval,
        "frame_id": "camera_link",
        "odom_frame_id": "odom",
        "database_path": db_path,
        "subscribe_imu": bool(camera["require_imu"] and camera["imu_topic"]),
        "queue_size": 20,
        "Mem/UseOdomGravity": "true",
        "RGBD/ForceOdom3DoF": "false",
        "Grid/3D": "true",
        "Grid/DepthDecimation": "2",
        "Grid/RayTracing": "false",
        "Grid/RangeMin": "0.2",
        "Grid/RangeMax": "4.0",
        "Grid/MaxObstacleHeight": "2.0",
        "RGBD/ProximityBySpace": "true",
        "Rtabmap/TimeThr": "0",
    }
    rtabmap_remappings = [("rgbd_image", "rgbd_image")] if use_rgbd_sync else list(remappings)
    if use_rgbd_sync and camera["require_imu"] and camera["imu_topic"]:
        rtabmap_remappings.append(("imu", camera["imu_topic"]))
    rtabmap_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        arguments=extra_args,
        parameters=[rtabmap_params],
        remappings=rtabmap_remappings,
    )

    rtabmap_viz = Node(
        package="rtabmap_viz",
        executable="rtabmap_viz",
        name="rtabmap_viz",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "approx_sync": bool(camera["approximate_sync"]),
                "approx_sync_max_interval": replay_approx_sync_max_interval,
            }
        ],
        remappings=[
            ("rgb/image", camera["rgb_image_topic"]),
            ("rgb/camera_info", camera["rgb_camera_info_topic"]),
            ("depth/image", camera["depth_image_topic"]),
            ("depth/camera_info", camera["depth_camera_info_topic"]),
        ],
    )

    actions.extend([rgbd_odometry, rtabmap_node])
    if viz_enabled:
        actions.append(rtabmap_viz)
    return actions


# ---------------------------------------------------------------------------
# generate_launch_description
# ---------------------------------------------------------------------------


def generate_launch_description():

    bag_path_arg = DeclareLaunchArgument(
        "bag_path",
        description="Absolute path to the bag directory to replay",
    )

    db_path_arg = DeclareLaunchArgument(
        "db_path",
        default_value=PythonExpression(["'", LaunchConfiguration("bag_path"), "/rtabmap.db'"]),
        description="Absolute path to the RTABMap database file",
    )

    rate_arg = DeclareLaunchArgument(
        "rate",
        default_value="1.0",
        description="Bag playback speed multiplier (lower = RTABMap has more time per frame)",
    )

    delete_db_arg = DeclareLaunchArgument(
        "delete_db",
        default_value="true",
        description="Delete existing RTABMap database before start",
    )

    viz_arg = DeclareLaunchArgument(
        "viz",
        default_value="false",
        description="Launch rtabmap_viz GUI",
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

    allow_unregistered_depth_arg = DeclareLaunchArgument(
        "allow_unregistered_depth",
        default_value="false",
        description="Run RTAB-Map even when the profile says depth is not registered to color; debugging only",
    )

    rtabmap_delay_arg = DeclareLaunchArgument(
        "rtabmap_delay",
        default_value="10.0",
        description="Seconds to wait for RTABMap to initialize before playing the bag",
    )

    bag_path = LaunchConfiguration("bag_path")
    database_path = LaunchConfiguration("db_path")

    rtabmap_nodes = OpaqueFunction(
        function=_launch_rtabmap_nodes,
        kwargs={
            "database_path_sub": database_path,
            "delete_db_sub": LaunchConfiguration("delete_db"),
            "viz_sub": LaunchConfiguration("viz"),
            "allow_unregistered_depth_sub": LaunchConfiguration("allow_unregistered_depth"),
        },
    )

    bag_play_proc = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            bag_path,
            "--clock",
            "--rate",
            LaunchConfiguration("rate"),
        ],
        output="screen",
        name="ros2_bag_play",
    )

    bag_play = TimerAction(
        period=LaunchConfiguration("rtabmap_delay"),
        actions=[
            LogInfo(msg=["[record_for_slam/offline_rtabmap] Playing bag -> ", bag_path]),
            bag_play_proc,
        ],
    )

    shutdown_on_bag_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=bag_play_proc,
            on_exit=[
                LogInfo(msg="[record_for_slam/offline_rtabmap] Bag finished; shutting down."),
                Shutdown(),
            ],
        )
    )

    return LaunchDescription(
        [
            bag_path_arg,
            db_path_arg,
            rate_arg,
            delete_db_arg,
            viz_arg,
            camera_profile_arg,
            camera_config_arg,
            allow_unregistered_depth_arg,
            rtabmap_delay_arg,
            LogInfo(msg=["[record_for_slam/offline_rtabmap] Starting RTABMap -> db: ", database_path]),
            rtabmap_nodes,
            LogInfo(
                msg=[
                    "[record_for_slam/offline_rtabmap] Waiting ",
                    LaunchConfiguration("rtabmap_delay"),
                    "s for RTABMap to initialize...",
                ]
            ),
            shutdown_on_bag_exit,
            bag_play,
        ]
    )
