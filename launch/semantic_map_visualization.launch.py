"""
Semantic map visualization launch.

Usage:
  ros2 launch record_for_slam semantic_map_visualization.launch.py \\
    map_cloud_path:=/abs/path/to/own_map_cloud.pcd \\
    semantic_objects_path:=/abs/path/to/semantic_objects.csv \\
    database_path:=/abs/path/to/rtabmap.db

The artifact paths are explicit launch arguments on purpose. This launch file
does not assume a source-tree layout or walk up from the install directory.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("record_for_slam"))
    default_rviz_config = package_share / "config" / "semantic_map_visualization.rviz"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_cloud_path",
                default_value="",
                description="Absolute path to the reconstructed map cloud (.pcd / .ply).",
            ),
            DeclareLaunchArgument(
                "semantic_objects_path",
                default_value="",
                description="Absolute path to semantic_objects.csv.",
            ),
            DeclareLaunchArgument(
                "database_path",
                default_value="",
                description="Absolute path to the RTAB-Map database (.db).",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(default_rviz_config),
                description="Path to RViz config file.",
            ),
            DeclareLaunchArgument(
                "stable_only",
                default_value="true",
                description="Publish only stable semantic objects.",
            ),
            DeclareLaunchArgument(
                "min_seen_count",
                default_value="2",
                description="Minimum seen_count to keep when stable_only is enabled.",
            ),
            DeclareLaunchArgument(
                "selected_node_id",
                default_value="-1",
                description="RTAB-Map node id to highlight as the current camera pose (-1 selects the last pose).",
            ),
            DeclareLaunchArgument(
                "viz_roll_deg",
                default_value="0.0",
                description="Visualization-only roll rotation in degrees.",
            ),
            DeclareLaunchArgument(
                "viz_pitch_deg",
                default_value="0.0",
                description="Visualization-only pitch rotation in degrees.",
            ),
            DeclareLaunchArgument(
                "viz_yaw_deg",
                default_value="0.0",
                description="Visualization-only yaw rotation in degrees.",
            ),
            DeclareLaunchArgument(
                "viz_translation_x_m",
                default_value="0.0",
                description="Visualization-only translation in x (metres).",
            ),
            DeclareLaunchArgument(
                "viz_translation_y_m",
                default_value="0.0",
                description="Visualization-only translation in y (metres).",
            ),
            DeclareLaunchArgument(
                "viz_translation_z_m",
                default_value="1.0",
                description="Visualization-only translation in z (metres).",
            ),
            DeclareLaunchArgument(
                "launch_playback",
                default_value="true",
                description="Launch the frame playback node (RGB/depth images + live trajectory animation).",
            ),
            DeclareLaunchArgument(
                "playback_fps",
                default_value="2.0",
                description="Playback speed in frames per second.",
            ),
            DeclareLaunchArgument(
                "loop_playback",
                default_value="true",
                description="Loop the playback when all frames have been shown.",
            ),
            DeclareLaunchArgument(
                "occupancy_resolution_m",
                default_value="0.05",
                description="Cell size (metres) for the 2-D top-down occupancy grid.",
            ),
            Node(
                package="record_for_slam",
                executable="semantic_map_visualizer_node",
                name="semantic_map_visualizer_node",
                output="screen",
                parameters=[
                    {
                        "map_cloud_path": LaunchConfiguration("map_cloud_path"),
                        "semantic_objects_path": LaunchConfiguration("semantic_objects_path"),
                        "database_path": LaunchConfiguration("database_path"),
                        "frame_id": "map",
                        "stable_only": LaunchConfiguration("stable_only"),
                        "min_seen_count": LaunchConfiguration("min_seen_count"),
                        "selected_node_id": LaunchConfiguration("selected_node_id"),
                        "anchor_scale": 0.12,
                        "label_z_offset": 0.18,
                        "bbox_min_side_m": 0.08,
                        "publish_rate_hz": 1.0,
                        "viz_roll_deg": LaunchConfiguration("viz_roll_deg"),
                        "viz_pitch_deg": LaunchConfiguration("viz_pitch_deg"),
                        "viz_yaw_deg": LaunchConfiguration("viz_yaw_deg"),
                        "viz_translation_x_m": LaunchConfiguration("viz_translation_x_m"),
                        "viz_translation_y_m": LaunchConfiguration("viz_translation_y_m"),
                        "viz_translation_z_m": LaunchConfiguration("viz_translation_z_m"),
                    }
                ],
            ),
            Node(
                package="record_for_slam",
                executable="frame_playback_node",
                name="frame_playback_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("launch_playback")),
                parameters=[
                    {
                        "database_path": LaunchConfiguration("database_path"),
                        "map_cloud_path": LaunchConfiguration("map_cloud_path"),
                        "frame_id": "map",
                        "playback_fps": LaunchConfiguration("playback_fps"),
                        "loop_playback": LaunchConfiguration("loop_playback"),
                        "occupancy_resolution_m": LaunchConfiguration("occupancy_resolution_m"),
                        "viz_roll_deg": LaunchConfiguration("viz_roll_deg"),
                        "viz_pitch_deg": LaunchConfiguration("viz_pitch_deg"),
                        "viz_yaw_deg": LaunchConfiguration("viz_yaw_deg"),
                        "viz_translation_x_m": LaunchConfiguration("viz_translation_x_m"),
                        "viz_translation_y_m": LaunchConfiguration("viz_translation_y_m"),
                        "viz_translation_z_m": LaunchConfiguration("viz_translation_z_m"),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
            ),
        ]
    )
