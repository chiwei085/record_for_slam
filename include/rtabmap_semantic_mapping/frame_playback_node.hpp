#pragma once

#include <Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>

#include "rtabmap_semantic_mapping/types.hpp"

namespace rtabmap_semantic_mapping
{

// FramePlaybackNode plays back keyframes from a RTAB-Map database in
// chronological order, publishing per-frame data on a configurable timer.
//
// Published topics:
//   /playback/rgb              sensor_msgs/Image        — current RGB frame
//   /playback/depth            sensor_msgs/Image        — current depth (TURBO
//   colormap) /playback/camera_markers   visualization_msgs/MarkerArray — pose
//   sphere + heading + frustum /playback/trajectory
//   visualization_msgs/MarkerArray — cumulative trajectory
//   /playback/occupancy_grid   nav_msgs/OccupancyGrid   — 2-D top-down map
//   (published once)
class FramePlaybackNode : public rclcpp::Node
{
public:
    FramePlaybackNode();

private:
    // Timer callback — advances the frame index and publishes.
    void on_playback_tick();

    // Publish data for a single frame.
    void publish_frame(std::size_t index);

    // Build a 2-D occupancy grid by projecting the map point cloud onto XY.
    void build_occupancy_grid(const std::string& cloud_path);

    // Apply the configured visualization transform in-place to all loaded
    // poses.
    void apply_visualization_transform();

    // Compute the Eigen isometry from the viz_* parameters.
    [[nodiscard]] Eigen::Isometry3f visualization_transform() const;

    // Build MarkerArray for the camera pose at the given frame index:
    //   ns="playback_pose"     — sphere at camera origin
    //   ns="playback_heading"  — forward arrow
    //   ns="playback_frustum"  — camera frustum
    [[nodiscard]] visualization_msgs::msg::MarkerArray build_pose_markers(
        std::size_t index) const;

    // Build MarkerArray for the trajectory visited so far (frames
    // 0..up_to_index).
    //   ns="playback_keyframes"  — POINTS (one dot per visited pose)
    //   ns="playback_trajectory" — LINE_STRIP connecting poses
    [[nodiscard]] visualization_msgs::msg::MarkerArray build_trajectory_markers(
        std::size_t up_to_index) const;

    // Convert a cv::Mat to sensor_msgs/Image without cv_bridge.
    // Supports CV_8UC3 (bgr8), CV_8UC1 (mono8), CV_16UC1, CV_32FC1.
    [[nodiscard]] static sensor_msgs::msg::Image mat_to_image_msg(
        const cv::Mat& mat, const std::string& frame_id,
        const rclcpp::Time& stamp);

    // ---- Parameters -------------------------------------------------------
    std::string database_path_;
    std::string map_cloud_path_;
    std::string frame_id_;
    double playback_fps_ = 2.0;
    bool loop_playback_ = true;
    double occupancy_resolution_m_ = 0.05;
    double viz_roll_deg_ = 0.0;
    double viz_pitch_deg_ = 0.0;
    double viz_yaw_deg_ = 0.0;
    double viz_translation_x_m_ = 0.0;
    double viz_translation_y_m_ = 0.0;
    double viz_translation_z_m_ = 0.0;

    // ---- Runtime state ----------------------------------------------------
    std::vector<FrameData> frames_;
    std::size_t current_frame_index_ = 0;
    nav_msgs::msg::OccupancyGrid occupancy_grid_msg_;

    // ---- Publishers -------------------------------------------------------
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        pose_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        trajectory_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_pub_;

    rclcpp::TimerBase::SharedPtr playback_timer_;
};

}  // namespace rtabmap_semantic_mapping
