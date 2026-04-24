#pragma once

#include <Eigen/Geometry>
#include <builtin_interfaces/msg/time.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>

#include "rtabmap_semantic_mapping/types.hpp"

namespace rtabmap_semantic_mapping
{

struct VisualizationTransformParams
{
    double roll_deg = 0.0;
    double pitch_deg = 0.0;
    double yaw_deg = 0.0;
    double translation_x_m = 0.0;
    double translation_y_m = 0.0;
    double translation_z_m = 0.0;
};

struct CameraMarkerStyle
{
    std::string keyframes_ns;
    std::string trajectory_ns;
    std::string pose_ns;
    std::string heading_ns;
    std::string frustum_ns;

    float keyframe_point_size_m = 0.045F;
    float trajectory_line_width_m = 0.03F;
    float pose_scale_m = 0.12F;
    float heading_shaft_diameter_m = 0.05F;
    float heading_head_diameter_m = 0.10F;
    float heading_head_length_m = 0.12F;
    float forward_arrow_length_m = 0.7F;
    float frustum_depth_m = 0.45F;
    float frustum_line_width_m = 0.025F;

    std_msgs::msg::ColorRGBA keyframe_color;
    std_msgs::msg::ColorRGBA trajectory_color;
    std_msgs::msg::ColorRGBA pose_color;
    std_msgs::msg::ColorRGBA heading_color;
    std_msgs::msg::ColorRGBA frustum_color;
};

[[nodiscard]] Eigen::Isometry3f visualization_transform_from_params(
    const VisualizationTransformParams& params);

std::vector<FrameData> load_sorted_frames_from_database(
    const std::string& database_path);

void apply_visualization_transform_to_frames(
    std::vector<FrameData>& frames, const Eigen::Isometry3f& transform);

[[nodiscard]] visualization_msgs::msg::MarkerArray build_camera_pose_markers(
    const FrameData& frame, const std::string& frame_id,
    const builtin_interfaces::msg::Time& stamp, const CameraMarkerStyle& style);

[[nodiscard]] visualization_msgs::msg::MarkerArray
build_camera_trajectory_markers(const std::vector<FrameData>& frames,
                                std::size_t visible_frame_count,
                                const std::string& frame_id,
                                const builtin_interfaces::msg::Time& stamp,
                                const CameraMarkerStyle& style);

void append_selected_camera_markers(
    visualization_msgs::msg::MarkerArray& marker_array, const FrameData& frame,
    const std::string& frame_id, const builtin_interfaces::msg::Time& stamp,
    const CameraMarkerStyle& style);

}  // namespace rtabmap_semantic_mapping
