#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>

#include "rtabmap_semantic_mapping/types.hpp"

namespace rtabmap_semantic_mapping
{

class SemanticMapVisualizerNode : public rclcpp::Node
{
public:
    SemanticMapVisualizerNode();

private:
    void load_inputs();
    void load_cloud(const std::string& path);
    void load_objects_from_csv(const std::string& path);
    void load_camera_frames(const std::string& path);
    void publish_all();
    visualization_msgs::msg::MarkerArray build_markers() const;
    visualization_msgs::msg::MarkerArray build_camera_markers() const;
    std::vector<SemanticObject> filtered_objects() const;
    [[nodiscard]] Eigen::Isometry3f visualization_transform() const;
    void apply_visualization_transform();
    [[nodiscard]] int resolve_selected_frame_index() const;

    std::string map_cloud_path_;
    std::string semantic_objects_path_;
    std::string database_path_;
    std::string frame_id_;
    bool stable_only_ = true;
    int min_seen_count_ = 2;
    int selected_node_id_ = -1;
    double anchor_scale_ = 0.12;
    double label_z_offset_ = 0.18;
    double bbox_min_side_m_ = 0.08;
    // publish_rate_hz_ <= 0 means publish once only (recommended for static
    // data).
    double publish_rate_hz_ = 0.0;
    bool show_labels_ = true;
    double viz_roll_deg_ = 0.0;
    double viz_pitch_deg_ = 0.0;
    double viz_yaw_deg_ = 0.0;
    double viz_translation_x_m_ = 0.0;
    double viz_translation_y_m_ = 0.0;
    double viz_translation_z_m_ = 0.0;

    sensor_msgs::msg::PointCloud2 cloud_message_;
    // Pre-built once after loading; republished as-is (data never changes).
    visualization_msgs::msg::MarkerArray marker_message_;
    visualization_msgs::msg::MarkerArray camera_marker_message_;
    std::vector<SemanticObject> objects_;
    std::vector<FrameData> frames_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        cloud_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        camera_marker_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace rtabmap_semantic_mapping
