#include "rtabmap_semantic_mapping/semantic_map_visualizer_node.hpp"

#include <Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/msg/color_rgba.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "rtabmap_semantic_mapping/class_colors.hpp"
#include "rtabmap_semantic_mapping/database_visualization_utils.hpp"
#include "rtabmap_semantic_mapping/utils.hpp"

namespace rtabmap_semantic_mapping
{
namespace
{

std::vector<std::string> split_csv_line(const std::string& line) {
    std::vector<std::string> columns;
    std::stringstream stream(line);
    std::string item;
    while (std::getline(stream, item, ',')) {
        columns.push_back(item);
    }
    return columns;
}

// Marker ID layout: each semantic object occupies a contiguous block of
// kMarkerIdStride IDs, starting at (object_id * kMarkerIdStride).
//   offset 0 → SPHERE anchor
//   offset 1 → TEXT_VIEW_FACING label
//   offset 2 → LINE_LIST bounding box
// kMarkerIdStride must be > the number of marker types used per object (3).
// The static_assert below enforces this at compile time.
constexpr int kMarkerIdStride = 10;
constexpr int kMarkerTypesPerObject = 3;
static_assert(kMarkerIdStride > kMarkerTypesPerObject,
              "kMarkerIdStride must exceed the number of marker types per "
              "object to prevent ID collisions.");

bool ends_with(const std::string& value, const std::string& suffix) {
    return value.size() >= suffix.size() &&
           value.compare(value.size() - suffix.size(), suffix.size(), suffix) ==
               0;
}

std_msgs::msg::ColorRGBA rgba(const float r, const float g, const float b,
                              const float a) {
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

CameraMarkerStyle semantic_map_camera_marker_style() {
    CameraMarkerStyle style;
    style.keyframes_ns = "camera_keyframes";
    style.trajectory_ns = "camera_trajectory";
    style.pose_ns = "camera_selected_pose";
    style.heading_ns = "camera_selected_heading";
    style.frustum_ns = "camera_selected_frustum";
    style.keyframe_point_size_m = 0.045F;
    style.trajectory_line_width_m = 0.035F;
    style.pose_scale_m = 0.12F;
    style.heading_shaft_diameter_m = 0.05F;
    style.heading_head_diameter_m = 0.10F;
    style.heading_head_length_m = 0.12F;
    style.forward_arrow_length_m = 0.7F;
    style.frustum_depth_m = 0.45F;
    style.frustum_line_width_m = 0.025F;
    style.keyframe_color = rgba(0.2F, 0.85F, 1.0F, 0.95F);
    style.trajectory_color = rgba(1.0F, 0.78F, 0.16F, 0.95F);
    style.pose_color = rgba(1.0F, 0.2F, 0.2F, 1.0F);
    style.heading_color = rgba(1.0F, 0.2F, 0.2F, 1.0F);
    style.frustum_color = rgba(1.0F, 0.58F, 0.0F, 0.95F);
    return style;
}

builtin_interfaces::msg::Time to_builtin_time(const rclcpp::Time& time) {
    builtin_interfaces::msg::Time stamp;
    const auto nanoseconds = time.nanoseconds();
    stamp.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
    stamp.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
    return stamp;
}

std::array<geometry_msgs::msg::Point, 8> bbox_corners(
    const Eigen::Vector3f& bbox_min, const Eigen::Vector3f& bbox_max) {
    std::array<geometry_msgs::msg::Point, 8> corners;
    const float min_x = bbox_min.x();
    const float min_y = bbox_min.y();
    const float min_z = bbox_min.z();
    const float max_x = bbox_max.x();
    const float max_y = bbox_max.y();
    const float max_z = bbox_max.z();

    corners[0].x = min_x;
    corners[0].y = min_y;
    corners[0].z = min_z;
    corners[1].x = max_x;
    corners[1].y = min_y;
    corners[1].z = min_z;
    corners[2].x = max_x;
    corners[2].y = max_y;
    corners[2].z = min_z;
    corners[3].x = min_x;
    corners[3].y = max_y;
    corners[3].z = min_z;
    corners[4].x = min_x;
    corners[4].y = min_y;
    corners[4].z = max_z;
    corners[5].x = max_x;
    corners[5].y = min_y;
    corners[5].z = max_z;
    corners[6].x = max_x;
    corners[6].y = max_y;
    corners[6].z = max_z;
    corners[7].x = min_x;
    corners[7].y = max_y;
    corners[7].z = max_z;
    return corners;
}

}  // namespace

SemanticMapVisualizerNode::SemanticMapVisualizerNode()
    : Node("semantic_map_visualizer_node") {
    map_cloud_path_ = declare_parameter<std::string>("map_cloud_path", "");
    semantic_objects_path_ =
        declare_parameter<std::string>("semantic_objects_path", "");
    database_path_ = declare_parameter<std::string>("database_path", "");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    stable_only_ = declare_parameter<bool>("stable_only", true);
    min_seen_count_ = declare_parameter<int>("min_seen_count", 2);
    selected_node_id_ = declare_parameter<int>("selected_node_id", -1);
    anchor_scale_ = declare_parameter<double>("anchor_scale", 0.12);
    label_z_offset_ = declare_parameter<double>("label_z_offset", 0.18);
    bbox_min_side_m_ = declare_parameter<double>("bbox_min_side_m", 0.08);
    // publish_rate_hz <= 0 → publish once only (static data; transient_local
    // handles late subscribers).
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 0.0);
    show_labels_ = declare_parameter<bool>("show_labels", true);
    viz_roll_deg_ = declare_parameter<double>("viz_roll_deg", 0.0);
    viz_pitch_deg_ = declare_parameter<double>("viz_pitch_deg", 0.0);
    viz_yaw_deg_ = declare_parameter<double>("viz_yaw_deg", 0.0);
    viz_translation_x_m_ =
        declare_parameter<double>("viz_translation_x_m", 0.0);
    viz_translation_y_m_ =
        declare_parameter<double>("viz_translation_y_m", 0.0);
    viz_translation_z_m_ =
        declare_parameter<double>("viz_translation_z_m", 0.0);

    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/semantic_map/cloud", qos);
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/semantic_map/markers", qos);
    camera_marker_publisher_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(
            "/semantic_map/camera_markers", qos);

    load_inputs();
    apply_visualization_transform();

    // Build markers once — data is static, no need to rebuild each tick.
    marker_message_ = build_markers();
    camera_marker_message_ = build_camera_markers();

    // Publish immediately so RViz receives data on connect.
    publish_all();

    // Only create a periodic timer if the user explicitly requests it.
    // For static maps, transient_local QoS caches messages for late-joining
    // subscribers, so repeated publishing is wasteful (triggers unnecessary GPU
    // uploads each tick).
    if (publish_rate_hz_ > 0.0) {
        const auto period =
            std::chrono::duration<double>(1.0 / publish_rate_hz_);
        publish_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&SemanticMapVisualizerNode::publish_all, this));
        RCLCPP_INFO(get_logger(), "Periodic republish enabled at %.1f Hz.",
                    publish_rate_hz_);
    }
    else {
        RCLCPP_INFO(get_logger(),
                    "Static publish mode: published once, transient_local "
                    "handles late subscribers.");
    }
}

Eigen::Isometry3f SemanticMapVisualizerNode::visualization_transform() const {
    return visualization_transform_from_params({
        viz_roll_deg_,
        viz_pitch_deg_,
        viz_yaw_deg_,
        viz_translation_x_m_,
        viz_translation_y_m_,
        viz_translation_z_m_,
    });
}

void SemanticMapVisualizerNode::apply_visualization_transform() {
    const Eigen::Isometry3f transform = visualization_transform();
    if (transform.linear().isApprox(Eigen::Matrix3f::Identity(), 1e-6F) &&
        transform.translation().isZero(1e-6F)) {
        return;
    }

    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(cloud_message_, cloud);
    pcl::PointCloud<pcl::PointXYZRGB> transformed_cloud;
    pcl::transformPointCloud(cloud, transformed_cloud, transform.matrix());
    pcl::toROSMsg(transformed_cloud, cloud_message_);
    cloud_message_.header.frame_id = frame_id_;

    for (auto& object : objects_) {
        object.position_map = transform * object.position_map;
        if (object.has_extent) {
            const auto corners =
                bbox_corners(object.bbox_min_map, object.bbox_max_map);
            Eigen::Vector3f transformed_min = Eigen::Vector3f::Constant(
                std::numeric_limits<float>::infinity());
            Eigen::Vector3f transformed_max = Eigen::Vector3f::Constant(
                -std::numeric_limits<float>::infinity());
            for (const auto& corner : corners) {
                const Eigen::Vector3f transformed_corner =
                    transform * Eigen::Vector3f(static_cast<float>(corner.x),
                                                static_cast<float>(corner.y),
                                                static_cast<float>(corner.z));
                transformed_min = transformed_min.cwiseMin(transformed_corner);
                transformed_max = transformed_max.cwiseMax(transformed_corner);
            }
            object.bbox_min_map = transformed_min;
            object.bbox_max_map = transformed_max;
        }
    }

    apply_visualization_transform_to_frames(frames_, transform);

    RCLCPP_INFO(get_logger(),
                "Applied visualization leveling transform roll=%.2f pitch=%.2f "
                "yaw=%.2f deg, translation=(%.2f, %.2f, %.2f)m.",
                viz_roll_deg_, viz_pitch_deg_, viz_yaw_deg_,
                viz_translation_x_m_, viz_translation_y_m_,
                viz_translation_z_m_);
}

void SemanticMapVisualizerNode::load_inputs() {
    if (map_cloud_path_.empty()) {
        throw std::runtime_error("Parameter 'map_cloud_path' is required.");
    }
    if (semantic_objects_path_.empty()) {
        throw std::runtime_error(
            "Parameter 'semantic_objects_path' is required.");
    }

    load_cloud(map_cloud_path_);
    load_objects_from_csv(semantic_objects_path_);
    if (!database_path_.empty()) {
        load_camera_frames(database_path_);
    }
    else {
        RCLCPP_INFO(get_logger(),
                    "Parameter 'database_path' is empty. Camera trajectory "
                    "visualization is disabled.");
    }
}

void SemanticMapVisualizerNode::load_cloud(const std::string& path) {
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    int load_status = -1;
    if (ends_with(path, ".pcd")) {
        load_status = pcl::io::loadPCDFile(path, cloud);
    }
    else if (ends_with(path, ".ply")) {
        load_status = pcl::io::loadPLYFile(path, cloud);
    }
    else {
        throw std::runtime_error("Unsupported map cloud format: " + path);
    }

    if (load_status < 0) {
        throw std::runtime_error("Failed to load map cloud: " + path);
    }

    pcl::toROSMsg(cloud, cloud_message_);
    cloud_message_.header.frame_id = frame_id_;
    RCLCPP_INFO(get_logger(), "Loaded map cloud '%s' with %zu points.",
                path.c_str(), cloud.size());
}

void SemanticMapVisualizerNode::load_objects_from_csv(const std::string& path) {
    std::ifstream stream(path);
    if (!stream.is_open()) {
        throw std::runtime_error("Failed to open semantic objects CSV: " +
                                 path);
    }

    std::string line;
    if (!std::getline(stream, line)) {
        throw std::runtime_error("Semantic objects CSV is empty: " + path);
    }

    std::size_t line_number = 1;
    std::size_t skipped_rows = 0;
    while (std::getline(stream, line)) {
        ++line_number;
        if (line.empty()) {
            continue;
        }
        const auto columns = split_csv_line(line);
        if (columns.size() < 9) {
            RCLCPP_WARN(get_logger(),
                        "Skipping malformed CSV row at line %zu "
                        "(expected >=9 columns, got %zu): %s",
                        line_number, columns.size(), line.c_str());
            ++skipped_rows;
            continue;
        }

        try {
            SemanticObject object;
            object.object_id = std::stoi(columns.at(0));
            object.class_id = std::stoi(columns.at(1));
            object.class_name = columns.at(2);
            object.position_map.x() = std::stof(columns.at(3));
            object.position_map.y() = std::stof(columns.at(4));
            object.position_map.z() = std::stof(columns.at(5));
            object.seen_count = std::stoi(columns.at(6));
            object.max_confidence = std::stof(columns.at(7));

            if (columns.size() >= 16) {
                object.has_extent = std::stoi(columns.at(9)) != 0;
                object.bbox_min_map.x() = std::stof(columns.at(10));
                object.bbox_min_map.y() = std::stof(columns.at(11));
                object.bbox_min_map.z() = std::stof(columns.at(12));
                object.bbox_max_map.x() = std::stof(columns.at(13));
                object.bbox_max_map.y() = std::stof(columns.at(14));
                object.bbox_max_map.z() = std::stof(columns.at(15));
            }
            objects_.push_back(std::move(object));
        }
        catch (const std::exception& e) {
            RCLCPP_WARN(
                get_logger(),
                "Skipping CSV row at line %zu due to parse error (%s): %s",
                line_number, e.what(), line.c_str());
            ++skipped_rows;
        }
    }

    if (skipped_rows > 0) {
        RCLCPP_WARN(get_logger(),
                    "Skipped %zu malformed row(s) while loading '%s'.",
                    skipped_rows, path.c_str());
    }

    RCLCPP_INFO(get_logger(), "Loaded %zu semantic objects from '%s'.",
                objects_.size(), path.c_str());
}

void SemanticMapVisualizerNode::load_camera_frames(const std::string& path) {
    frames_ = load_sorted_frames_from_database(path);

    RCLCPP_INFO(
        get_logger(),
        "Loaded %zu camera frame(s) from '%s' for trajectory visualization.",
        frames_.size(), path.c_str());
}

std::vector<SemanticObject> SemanticMapVisualizerNode::filtered_objects()
    const {
    std::vector<SemanticObject> filtered;
    filtered.reserve(objects_.size());
    for (const auto& object : objects_) {
        if (stable_only_ && object.seen_count < min_seen_count_) {
            continue;
        }
        filtered.push_back(object);
    }
    return filtered;
}

visualization_msgs::msg::MarkerArray SemanticMapVisualizerNode::build_markers()
    const {
    visualization_msgs::msg::MarkerArray marker_array;
    const auto objects = filtered_objects();
    if (objects.empty()) {
        return marker_array;
    }
    const auto stamp = now();

    // --- Batched POINTS marker: one draw call for all anchor spheres. ---
    // Using POINTS instead of N individual SPHERE markers reduces OGRE draw
    // calls from N to 1.
    visualization_msgs::msg::Marker anchors;
    anchors.header.frame_id = frame_id_;
    anchors.header.stamp = stamp;
    anchors.ns = "semantic_anchor";
    anchors.id = 0;
    anchors.type = visualization_msgs::msg::Marker::POINTS;
    anchors.action = visualization_msgs::msg::Marker::ADD;
    anchors.pose.orientation.w = 1.0;
    anchors.scale.x = anchor_scale_;  // billboard width
    anchors.scale.y = anchor_scale_;  // billboard height
    anchors.points.reserve(objects.size());
    anchors.colors.reserve(objects.size());

    // --- Batched LINE_LIST marker: one draw call for all bounding boxes. ---
    // Per-vertex colors via the colors field; replaces N separate LINE_LIST
    // markers.
    visualization_msgs::msg::Marker bboxes;
    bboxes.header.frame_id = frame_id_;
    bboxes.header.stamp = stamp;
    bboxes.ns = "semantic_bbox";
    bboxes.id = 0;
    bboxes.type = visualization_msgs::msg::Marker::LINE_LIST;
    bboxes.action = visualization_msgs::msg::Marker::ADD;
    bboxes.pose.orientation.w = 1.0;
    bboxes.scale.x = 0.03;
    bool any_bbox = false;

    for (const auto& object : objects) {
        const auto [r, g, b] = class_color_normalized(object.class_id);

        // Anchor point
        geometry_msgs::msg::Point pt;
        pt.x = object.position_map.x();
        pt.y = object.position_map.y();
        pt.z = object.position_map.z();
        anchors.points.push_back(pt);

        std_msgs::msg::ColorRGBA color;
        color.r = r;
        color.g = g;
        color.b = b;
        color.a = 0.9F;
        anchors.colors.push_back(color);

        // Optional label (TEXT_VIEW_FACING is recomputed by RViz every frame;
        // disable with show_labels:=false when interactivity is more
        // important).
        if (show_labels_) {
            visualization_msgs::msg::Marker label;
            label.header.frame_id = frame_id_;
            label.header.stamp = stamp;
            label.ns = "semantic_label";
            label.id = object.object_id;
            label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            label.action = visualization_msgs::msg::Marker::ADD;
            label.pose.position.x = object.position_map.x();
            label.pose.position.y = object.position_map.y();
            label.pose.position.z = object.position_map.z() + label_z_offset_;
            label.pose.orientation.w = 1.0;
            label.scale.z = anchor_scale_;
            label.color.r = 1.0F;
            label.color.g = 1.0F;
            label.color.b = 1.0F;
            label.color.a = 1.0F;
            label.text = object.class_name + " [" +
                         std::to_string(object.seen_count) + "]";
            marker_array.markers.push_back(std::move(label));
        }

        // Bbox edges appended to the shared LINE_LIST.
        if (object.has_extent) {
            any_bbox = true;
            const Eigen::Vector3f extent =
                (object.bbox_max_map - object.bbox_min_map)
                    .cwiseMax(Eigen::Vector3f::Constant(
                        static_cast<float>(bbox_min_side_m_)));
            const Eigen::Vector3f half_extent = 0.5F * extent;
            const Eigen::Vector3f center =
                0.5F * (object.bbox_min_map + object.bbox_max_map);
            const Eigen::Vector3f bbox_min = center - half_extent;
            const Eigen::Vector3f bbox_max = center + half_extent;
            const auto corners = bbox_corners(bbox_min, bbox_max);

            std_msgs::msg::ColorRGBA edge_color;
            edge_color.r = r;
            edge_color.g = g;
            edge_color.b = b;
            edge_color.a = 0.95F;

            // Each append_edge call adds 2 points; each point needs a color
            // entry.
            auto append_colored_edge = [&](std::size_t start, std::size_t end) {
                bboxes.points.push_back(corners.at(start));
                bboxes.colors.push_back(edge_color);
                bboxes.points.push_back(corners.at(end));
                bboxes.colors.push_back(edge_color);
            };
            append_colored_edge(0, 1);
            append_colored_edge(1, 2);
            append_colored_edge(2, 3);
            append_colored_edge(3, 0);
            append_colored_edge(4, 5);
            append_colored_edge(5, 6);
            append_colored_edge(6, 7);
            append_colored_edge(7, 4);
            append_colored_edge(0, 4);
            append_colored_edge(1, 5);
            append_colored_edge(2, 6);
            append_colored_edge(3, 7);
        }
    }

    marker_array.markers.push_back(std::move(anchors));
    if (any_bbox) {
        marker_array.markers.push_back(std::move(bboxes));
    }

    return marker_array;
}

int SemanticMapVisualizerNode::resolve_selected_frame_index() const {
    if (frames_.empty()) {
        return -1;
    }
    if (selected_node_id_ >= 0) {
        for (std::size_t i = 0; i < frames_.size(); ++i) {
            if (frames_[i].node_id == selected_node_id_) {
                return static_cast<int>(i);
            }
        }
        RCLCPP_WARN(get_logger(),
                    "selected_node_id=%d not found in database. Falling back "
                    "to the last frame.",
                    selected_node_id_);
    }
    return static_cast<int>(frames_.size()) - 1;
}

visualization_msgs::msg::MarkerArray
SemanticMapVisualizerNode::build_camera_markers() const {
    visualization_msgs::msg::MarkerArray marker_array;
    if (frames_.empty()) {
        return marker_array;
    }

    const auto stamp = now();
    marker_array = build_camera_trajectory_markers(
        frames_, frames_.size(), frame_id_, to_builtin_time(stamp),
        semantic_map_camera_marker_style());

    const int selected_index = resolve_selected_frame_index();
    if (selected_index < 0) {
        return marker_array;
    }

    const FrameData& selected_frame =
        frames_[static_cast<std::size_t>(selected_index)];
    append_selected_camera_markers(marker_array, selected_frame, frame_id_,
                                   to_builtin_time(stamp),
                                   semantic_map_camera_marker_style());

    return marker_array;
}

void SemanticMapVisualizerNode::publish_all() {
    cloud_publisher_->publish(cloud_message_);
    marker_publisher_->publish(marker_message_);
    camera_marker_publisher_->publish(camera_marker_message_);
}

}  // namespace rtabmap_semantic_mapping
