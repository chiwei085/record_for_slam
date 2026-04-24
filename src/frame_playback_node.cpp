#include "rtabmap_semantic_mapping/frame_playback_node.hpp"

#include <opencv2/imgproc.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>

#include "rtabmap_semantic_mapping/database_visualization_utils.hpp"
#include "rtabmap_semantic_mapping/utils.hpp"

namespace rtabmap_semantic_mapping
{
namespace
{

bool ends_with(const std::string& s, const std::string& suffix) {
    return s.size() >= suffix.size() &&
           s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
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

CameraMarkerStyle playback_camera_marker_style() {
    CameraMarkerStyle style;
    style.keyframes_ns = "playback_keyframes";
    style.trajectory_ns = "playback_trajectory";
    style.pose_ns = "playback_pose";
    style.heading_ns = "playback_heading";
    style.frustum_ns = "playback_frustum";
    style.keyframe_point_size_m = 0.04F;
    style.trajectory_line_width_m = 0.03F;
    style.pose_scale_m = 0.12F;
    style.heading_shaft_diameter_m = 0.05F;
    style.heading_head_diameter_m = 0.10F;
    style.heading_head_length_m = 0.12F;
    style.forward_arrow_length_m = 0.7F;
    style.frustum_depth_m = 0.45F;
    style.frustum_line_width_m = 0.025F;
    style.keyframe_color = rgba(0.2F, 0.9F, 0.55F, 0.85F);
    style.trajectory_color = rgba(0.2F, 1.0F, 0.45F, 0.9F);
    style.pose_color = rgba(1.0F, 0.3F, 0.0F, 1.0F);
    style.heading_color = rgba(1.0F, 0.3F, 0.0F, 1.0F);
    style.frustum_color = rgba(1.0F, 0.65F, 0.0F, 0.9F);
    return style;
}

builtin_interfaces::msg::Time to_builtin_time(const rclcpp::Time& time) {
    builtin_interfaces::msg::Time stamp;
    const auto nanoseconds = time.nanoseconds();
    stamp.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
    stamp.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
    return stamp;
}

}  // namespace

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

FramePlaybackNode::FramePlaybackNode() : Node("frame_playback_node") {
    database_path_ = declare_parameter<std::string>("database_path", "");
    map_cloud_path_ = declare_parameter<std::string>("map_cloud_path", "");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    playback_fps_ = declare_parameter<double>("playback_fps", 2.0);
    loop_playback_ = declare_parameter<bool>("loop_playback", true);
    occupancy_resolution_m_ =
        declare_parameter<double>("occupancy_resolution_m", 0.05);
    viz_roll_deg_ = declare_parameter<double>("viz_roll_deg", 0.0);
    viz_pitch_deg_ = declare_parameter<double>("viz_pitch_deg", 0.0);
    viz_yaw_deg_ = declare_parameter<double>("viz_yaw_deg", 0.0);
    viz_translation_x_m_ =
        declare_parameter<double>("viz_translation_x_m", 0.0);
    viz_translation_y_m_ =
        declare_parameter<double>("viz_translation_y_m", 0.0);
    viz_translation_z_m_ =
        declare_parameter<double>("viz_translation_z_m", 0.0);

    // Publishers — volatile QoS for per-frame data (always fresh)
    const auto volatile_qos =
        rclcpp::QoS(1).best_effort().durability_volatile();
    rgb_pub_ = create_publisher<sensor_msgs::msg::Image>("/playback/rgb",
                                                         volatile_qos);
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>("/playback/depth",
                                                           volatile_qos);
    pose_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/playback/camera_markers", volatile_qos);
    trajectory_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/playback/trajectory", volatile_qos);

    // Transient-local for occupancy grid — published once, cached for late
    // subscribers
    const auto static_qos = rclcpp::QoS(1).reliable().transient_local();
    occupancy_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/playback/occupancy_grid", static_qos);

    // Load data
    if (database_path_.empty()) {
        throw std::runtime_error(
            "Parameter 'database_path' is required for FramePlaybackNode.");
    }

    frames_ = load_sorted_frames_from_database(database_path_);
    if (frames_.empty()) {
        throw std::runtime_error("No valid frames loaded from database: " +
                                 database_path_);
    }

    apply_visualization_transform();
    RCLCPP_INFO(get_logger(), "Loaded %zu frames for playback.",
                frames_.size());

    // Build and publish occupancy grid
    if (!map_cloud_path_.empty()) {
        build_occupancy_grid(map_cloud_path_);
        occupancy_grid_msg_.header.stamp = now();
        occupancy_pub_->publish(occupancy_grid_msg_);
    }
    else {
        RCLCPP_WARN(get_logger(),
                    "'map_cloud_path' not set — occupancy grid disabled.");
    }

    // Start playback timer
    if (playback_fps_ > 0.0) {
        const auto period = std::chrono::duration<double>(1.0 / playback_fps_);
        playback_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&FramePlaybackNode::on_playback_tick, this));
        RCLCPP_INFO(get_logger(), "Playback timer started at %.1f FPS.",
                    playback_fps_);
    }
    else {
        RCLCPP_INFO(
            get_logger(),
            "playback_fps <= 0 — publishing frame 0 once and stopping.");
        publish_frame(0);
    }
}

// ---------------------------------------------------------------------------
// Transform helpers
// ---------------------------------------------------------------------------

Eigen::Isometry3f FramePlaybackNode::visualization_transform() const {
    return visualization_transform_from_params({
        viz_roll_deg_,
        viz_pitch_deg_,
        viz_yaw_deg_,
        viz_translation_x_m_,
        viz_translation_y_m_,
        viz_translation_z_m_,
    });
}

void FramePlaybackNode::apply_visualization_transform() {
    const Eigen::Isometry3f T = visualization_transform();
    if (T.linear().isApprox(Eigen::Matrix3f::Identity(), 1e-6F) &&
        T.translation().isZero(1e-6F)) {
        return;
    }
    apply_visualization_transform_to_frames(frames_, T);
    RCLCPP_INFO(get_logger(),
                "Applied viz transform: roll=%.2f pitch=%.2f yaw=%.2f deg, "
                "t=(%.2f,%.2f,%.2f)m.",
                viz_roll_deg_, viz_pitch_deg_, viz_yaw_deg_,
                viz_translation_x_m_, viz_translation_y_m_,
                viz_translation_z_m_);
}

// ---------------------------------------------------------------------------
// Occupancy grid
// ---------------------------------------------------------------------------

void FramePlaybackNode::build_occupancy_grid(const std::string& cloud_path) {
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    int status = -1;
    if (ends_with(cloud_path, ".pcd")) {
        status = pcl::io::loadPCDFile(cloud_path, cloud);
    }
    else if (ends_with(cloud_path, ".ply")) {
        status = pcl::io::loadPLYFile(cloud_path, cloud);
    }
    else {
        RCLCPP_WARN(get_logger(),
                    "Unsupported cloud format for occupancy grid: %s",
                    cloud_path.c_str());
        return;
    }
    if (status < 0 || cloud.empty()) {
        RCLCPP_WARN(get_logger(), "Failed to load cloud for occupancy grid: %s",
                    cloud_path.c_str());
        return;
    }

    // Apply the same visualization transform to the cloud
    pcl::PointCloud<pcl::PointXYZRGB> transformed;
    pcl::transformPointCloud(cloud, transformed,
                             visualization_transform().matrix());

    // Find Z range to determine the floor level
    float min_z = std::numeric_limits<float>::max();
    for (const auto& pt : transformed) {
        if (std::isfinite(pt.z)) {
            min_z = std::min(min_z, pt.z);
        }
    }
    if (!std::isfinite(min_z)) {
        RCLCPP_WARN(get_logger(),
                    "Cloud has no finite points — skipping occupancy grid.");
        return;
    }

    // Keep points between floor+0.15 m and floor+2.0 m to capture walls/objects
    const float z_low = min_z + 0.15F;
    const float z_high = min_z + 2.00F;

    // XY bounding box of the filtered slice
    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();

    for (const auto& pt : transformed) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) ||
            !std::isfinite(pt.z))
            continue;
        if (pt.z < z_low || pt.z > z_high) continue;
        min_x = std::min(min_x, pt.x);
        max_x = std::max(max_x, pt.x);
        min_y = std::min(min_y, pt.y);
        max_y = std::max(max_y, pt.y);
    }

    if (!std::isfinite(min_x)) {
        RCLCPP_WARN(get_logger(),
                    "No points in height slice — skipping occupancy grid.");
        return;
    }

    const float margin = 0.5F;
    min_x -= margin;
    max_x += margin;
    min_y -= margin;
    max_y += margin;

    const float res = static_cast<float>(occupancy_resolution_m_);
    const int w = static_cast<int>(std::ceil((max_x - min_x) / res));
    const int h = static_cast<int>(std::ceil((max_y - min_y) / res));

    occupancy_grid_msg_.header.frame_id = frame_id_;
    occupancy_grid_msg_.info.resolution = res;
    occupancy_grid_msg_.info.width = static_cast<uint32_t>(w);
    occupancy_grid_msg_.info.height = static_cast<uint32_t>(h);
    occupancy_grid_msg_.info.origin.position.x = min_x;
    occupancy_grid_msg_.info.origin.position.y = min_y;
    occupancy_grid_msg_.info.origin.orientation.w = 1.0;
    occupancy_grid_msg_.data.assign(static_cast<std::size_t>(w * h),
                                    -1);  // unknown

    for (const auto& pt : transformed) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) ||
            !std::isfinite(pt.z))
            continue;
        if (pt.z < z_low || pt.z > z_high) continue;
        const int ix = static_cast<int>((pt.x - min_x) / res);
        const int iy = static_cast<int>((pt.y - min_y) / res);
        if (ix >= 0 && ix < w && iy >= 0 && iy < h) {
            occupancy_grid_msg_.data[static_cast<std::size_t>(iy * w + ix)] =
                100;
        }
    }

    RCLCPP_INFO(get_logger(),
                "Built 2-D occupancy grid: %d x %d cells @ %.3f m/cell (%.1f x "
                "%.1f m).",
                w, h, res, static_cast<double>(max_x - min_x),
                static_cast<double>(max_y - min_y));
}

// ---------------------------------------------------------------------------
// Playback timer
// ---------------------------------------------------------------------------

void FramePlaybackNode::on_playback_tick() {
    if (frames_.empty()) return;
    publish_frame(current_frame_index_);

    ++current_frame_index_;
    if (current_frame_index_ >= frames_.size()) {
        if (loop_playback_) {
            current_frame_index_ = 0;
            RCLCPP_DEBUG(get_logger(), "Playback loop restart.");
        }
        else {
            current_frame_index_ = frames_.size() - 1;
        }
    }
}

// ---------------------------------------------------------------------------
// Per-frame publishing
// ---------------------------------------------------------------------------

void FramePlaybackNode::publish_frame(std::size_t index) {
    const FrameData& frame = frames_[index];
    const rclcpp::Time stamp = now();

    // --- RGB ---
    if (!frame.rgb.empty()) {
        rgb_pub_->publish(mat_to_image_msg(frame.rgb, frame_id_, stamp));
    }

    // --- Depth (Turbo colormap for demo clarity) ---
    if (!frame.depth.empty()) {
        cv::Mat depth_float;
        if (frame.depth.type() == CV_16UC1) {
            // uint16 mm → float32 metres
            frame.depth.convertTo(depth_float, CV_32F, 1.0F / 1000.0F);
        }
        else {
            frame.depth.convertTo(depth_float, CV_32F);
        }

        // Normalize to 8-bit grayscale; invalid pixels remain black.
        cv::Mat valid_mask;
        cv::threshold(depth_float, valid_mask, 0.01, 255.0, cv::THRESH_BINARY);
        valid_mask.convertTo(valid_mask, CV_8U);

        double max_depth = 0.0;
        cv::minMaxLoc(depth_float, nullptr, &max_depth, nullptr, nullptr,
                      valid_mask);
        if (max_depth < 0.01) max_depth = 5.0;  // fallback

        cv::Mat depth_gray;
        depth_float.convertTo(depth_gray, CV_8U, 255.0 / max_depth);
        depth_gray.setTo(0, ~valid_mask);  // black out invalid pixels

        depth_pub_->publish(mat_to_image_msg(depth_gray, frame_id_, stamp));
    }

    // --- Pose markers (sphere + heading + frustum) ---
    auto pose_msg = build_pose_markers(index);
    pose_markers_pub_->publish(pose_msg);

    // --- Cumulative trajectory ---
    auto traj_msg = build_trajectory_markers(index);
    trajectory_pub_->publish(traj_msg);

    RCLCPP_DEBUG(get_logger(),
                 "Published playback frame %zu/%zu (node_id=%d, stamp=%.3f s).",
                 index + 1, frames_.size(), frame.node_id, frame.stamp_sec);
}

// ---------------------------------------------------------------------------
// Marker builders
// ---------------------------------------------------------------------------

visualization_msgs::msg::MarkerArray FramePlaybackNode::build_pose_markers(
    std::size_t index) const {
    return build_camera_pose_markers(frames_[index], frame_id_,
                                     to_builtin_time(now()),
                                     playback_camera_marker_style());
}

visualization_msgs::msg::MarkerArray
FramePlaybackNode::build_trajectory_markers(std::size_t up_to_index) const {
    return build_camera_trajectory_markers(frames_, up_to_index + 1, frame_id_,
                                           to_builtin_time(now()),
                                           playback_camera_marker_style());
}

// ---------------------------------------------------------------------------
// Static helpers
// ---------------------------------------------------------------------------

sensor_msgs::msg::Image FramePlaybackNode::mat_to_image_msg(
    const cv::Mat& mat, const std::string& frame_id,
    const rclcpp::Time& stamp) {
    sensor_msgs::msg::Image msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;
    msg.height = static_cast<uint32_t>(mat.rows);
    msg.width = static_cast<uint32_t>(mat.cols);
    msg.is_bigendian = false;
    msg.step =
        static_cast<uint32_t>(mat.cols) * static_cast<uint32_t>(mat.elemSize());

    switch (mat.type()) {
        case CV_8UC3:
            msg.encoding = "bgr8";
            break;
        case CV_8UC1:
            msg.encoding = "mono8";
            break;
        case CV_16UC1:
            msg.encoding = "16UC1";
            break;
        case CV_32FC1:
            msg.encoding = "32FC1";
            break;
        default:
            msg.encoding = "bgr8";
            break;
    }

    const std::size_t data_size = static_cast<std::size_t>(mat.rows) * msg.step;
    msg.data.resize(data_size);

    if (mat.isContinuous()) {
        std::memcpy(msg.data.data(), mat.data, data_size);
    }
    else {
        for (int r = 0; r < mat.rows; ++r) {
            std::memcpy(
                msg.data.data() + static_cast<std::size_t>(r) * msg.step,
                mat.ptr(r), msg.step);
        }
    }
    return msg;
}

}  // namespace rtabmap_semantic_mapping
