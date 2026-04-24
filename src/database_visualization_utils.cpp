#include "rtabmap_semantic_mapping/database_visualization_utils.hpp"

#include <geometry_msgs/msg/point.hpp>

#include <algorithm>
#include <array>
#include <cmath>

#include "rtabmap_semantic_mapping/database_reader.hpp"
#include "rtabmap_semantic_mapping/utils.hpp"

namespace rtabmap_semantic_mapping
{
namespace
{

geometry_msgs::msg::Point to_point(const Eigen::Vector3f& vector) {
    geometry_msgs::msg::Point point;
    point.x = vector.x();
    point.y = vector.y();
    point.z = vector.z();
    return point;
}

void initialize_marker(visualization_msgs::msg::Marker& marker,
                       const std::string& frame_id,
                       const builtin_interfaces::msg::Time& stamp,
                       const std::string& ns) {
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
}

bool has_valid_intrinsics(const CameraIntrinsics& intrinsics) {
    return intrinsics.width > 0 && intrinsics.height > 0 &&
           intrinsics.fx > 0.0F && intrinsics.fy > 0.0F;
}

std::array<geometry_msgs::msg::Point, 4> frustum_corners(const FrameData& frame,
                                                         const float depth_m) {
    const auto& intrinsics = frame.intrinsics;
    const auto ray_from_pixel = [&](const float u, const float v) {
        return Eigen::Vector3f((u - intrinsics.cx) * depth_m / intrinsics.fx,
                               (v - intrinsics.cy) * depth_m / intrinsics.fy,
                               depth_m);
    };

    std::array<geometry_msgs::msg::Point, 4> corners;
    const std::array<Eigen::Vector3f, 4> corners_camera = {
        ray_from_pixel(0.0F, 0.0F),
        ray_from_pixel(static_cast<float>(intrinsics.width), 0.0F),
        ray_from_pixel(static_cast<float>(intrinsics.width),
                       static_cast<float>(intrinsics.height)),
        ray_from_pixel(0.0F, static_cast<float>(intrinsics.height)),
    };
    for (std::size_t index = 0; index < corners_camera.size(); ++index) {
        corners[index] = to_point(frame.T_map_camera * corners_camera[index]);
    }
    return corners;
}

void append_frustum_edges(
    visualization_msgs::msg::Marker& marker,
    const geometry_msgs::msg::Point& origin,
    const std::array<geometry_msgs::msg::Point, 4>& corners) {
    const auto append_edge = [&](const geometry_msgs::msg::Point& start,
                                 const geometry_msgs::msg::Point& end) {
        marker.points.push_back(start);
        marker.points.push_back(end);
    };

    for (const auto& corner : corners) {
        append_edge(origin, corner);
    }
    append_edge(corners[0], corners[1]);
    append_edge(corners[1], corners[2]);
    append_edge(corners[2], corners[3]);
    append_edge(corners[3], corners[0]);
}

}  // namespace

Eigen::Isometry3f visualization_transform_from_params(
    const VisualizationTransformParams& params) {
    const float roll_rad = static_cast<float>(params.roll_deg * M_PI / 180.0);
    const float pitch_rad = static_cast<float>(params.pitch_deg * M_PI / 180.0);
    const float yaw_rad = static_cast<float>(params.yaw_deg * M_PI / 180.0);
    return euler_to_isometry3f(
        roll_rad, pitch_rad, yaw_rad,
        Eigen::Vector3f(static_cast<float>(params.translation_x_m),
                        static_cast<float>(params.translation_y_m),
                        static_cast<float>(params.translation_z_m)));
}

std::vector<FrameData> load_sorted_frames_from_database(
    const std::string& database_path) {
    DatabaseReader reader(database_path);
    auto frames = reader.load_frames();
    std::sort(frames.begin(), frames.end(),
              [](const FrameData& lhs, const FrameData& rhs) {
                  if (lhs.stamp_sec == rhs.stamp_sec) {
                      return lhs.node_id < rhs.node_id;
                  }
                  return lhs.stamp_sec < rhs.stamp_sec;
              });
    return frames;
}

void apply_visualization_transform_to_frames(
    std::vector<FrameData>& frames, const Eigen::Isometry3f& transform) {
    for (auto& frame : frames) {
        frame.T_map_base = transform * frame.T_map_base;
        frame.T_map_camera = transform * frame.T_map_camera;
    }
}

visualization_msgs::msg::MarkerArray build_camera_pose_markers(
    const FrameData& frame, const std::string& frame_id,
    const builtin_interfaces::msg::Time& stamp,
    const CameraMarkerStyle& style) {
    visualization_msgs::msg::MarkerArray marker_array;

    const Eigen::Vector3f origin = frame.T_map_camera.translation();
    const Eigen::Matrix3f rotation = frame.T_map_camera.rotation();

    visualization_msgs::msg::Marker pose;
    initialize_marker(pose, frame_id, stamp, style.pose_ns);
    pose.type = visualization_msgs::msg::Marker::SPHERE;
    pose.pose.position = to_point(origin);
    pose.scale.x = style.pose_scale_m;
    pose.scale.y = style.pose_scale_m;
    pose.scale.z = style.pose_scale_m;
    pose.color = style.pose_color;
    marker_array.markers.push_back(std::move(pose));

    visualization_msgs::msg::Marker heading;
    initialize_marker(heading, frame_id, stamp, style.heading_ns);
    heading.type = visualization_msgs::msg::Marker::ARROW;
    heading.scale.x = style.heading_shaft_diameter_m;
    heading.scale.y = style.heading_head_diameter_m;
    heading.scale.z = style.heading_head_length_m;
    heading.color = style.heading_color;
    heading.points.push_back(to_point(origin));
    heading.points.push_back(to_point(
        origin +
        rotation * Eigen::Vector3f(0.0F, 0.0F, style.forward_arrow_length_m)));
    marker_array.markers.push_back(std::move(heading));

    if (has_valid_intrinsics(frame.intrinsics)) {
        visualization_msgs::msg::Marker frustum;
        initialize_marker(frustum, frame_id, stamp, style.frustum_ns);
        frustum.type = visualization_msgs::msg::Marker::LINE_LIST;
        frustum.scale.x = style.frustum_line_width_m;
        frustum.color = style.frustum_color;
        append_frustum_edges(frustum, to_point(origin),
                             frustum_corners(frame, style.frustum_depth_m));
        marker_array.markers.push_back(std::move(frustum));
    }

    return marker_array;
}

visualization_msgs::msg::MarkerArray build_camera_trajectory_markers(
    const std::vector<FrameData>& frames, const std::size_t visible_frame_count,
    const std::string& frame_id, const builtin_interfaces::msg::Time& stamp,
    const CameraMarkerStyle& style) {
    visualization_msgs::msg::MarkerArray marker_array;
    if (frames.empty() || visible_frame_count == 0) {
        return marker_array;
    }

    const std::size_t clamped_count =
        std::min(visible_frame_count, frames.size());

    visualization_msgs::msg::Marker keyframes;
    initialize_marker(keyframes, frame_id, stamp, style.keyframes_ns);
    keyframes.type = visualization_msgs::msg::Marker::POINTS;
    keyframes.scale.x = style.keyframe_point_size_m;
    keyframes.scale.y = style.keyframe_point_size_m;

    visualization_msgs::msg::Marker trajectory;
    initialize_marker(trajectory, frame_id, stamp, style.trajectory_ns);
    trajectory.type = visualization_msgs::msg::Marker::LINE_STRIP;
    trajectory.scale.x = style.trajectory_line_width_m;
    trajectory.color = style.trajectory_color;

    for (std::size_t index = 0; index < clamped_count; ++index) {
        const auto point = to_point(frames[index].T_map_camera.translation());
        keyframes.points.push_back(point);
        keyframes.colors.push_back(style.keyframe_color);
        trajectory.points.push_back(point);
    }

    marker_array.markers.push_back(std::move(keyframes));
    marker_array.markers.push_back(std::move(trajectory));
    return marker_array;
}

void append_selected_camera_markers(
    visualization_msgs::msg::MarkerArray& marker_array, const FrameData& frame,
    const std::string& frame_id, const builtin_interfaces::msg::Time& stamp,
    const CameraMarkerStyle& style) {
    auto pose_markers =
        build_camera_pose_markers(frame, frame_id, stamp, style);
    for (auto& marker : pose_markers.markers) {
        marker_array.markers.push_back(std::move(marker));
    }
}

}  // namespace rtabmap_semantic_mapping
