#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>

namespace rtabmap_semantic_mapping
{

inline std::optional<float> convert_depth_to_meters(const cv::Mat& depth,
                                                    const int v, const int u) {
    if (depth.type() == CV_16UC1) {
        const std::uint16_t depth_mm = depth.at<std::uint16_t>(v, u);
        if (depth_mm == 0U) {
            return std::nullopt;
        }
        return static_cast<float>(depth_mm) / 1000.0F;
    }

    if (depth.type() == CV_32FC1) {
        const float depth_m = depth.at<float>(v, u);
        if (!std::isfinite(depth_m) || depth_m <= 0.0F) {
            return std::nullopt;
        }
        return depth_m;
    }

    return std::nullopt;
}

inline cv::Rect centered_sample_roi(const cv::Rect& bbox, const float ratio) {
    const int roi_width = std::max(
        1,
        static_cast<int>(std::round(static_cast<float>(bbox.width) * ratio)));
    const int roi_height = std::max(
        1,
        static_cast<int>(std::round(static_cast<float>(bbox.height) * ratio)));
    const int roi_x = bbox.x + (bbox.width - roi_width) / 2;
    const int roi_y = bbox.y + (bbox.height - roi_height) / 2;
    return cv::Rect(roi_x, roi_y, roi_width, roi_height);
}

inline Eigen::Isometry3f euler_to_isometry3f(
    const float roll_rad, const float pitch_rad, const float yaw_rad,
    const Eigen::Vector3f& translation) {
    Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
    transform.linear() =
        (Eigen::AngleAxisf(yaw_rad, Eigen::Vector3f::UnitZ()) *
         Eigen::AngleAxisf(pitch_rad, Eigen::Vector3f::UnitY()) *
         Eigen::AngleAxisf(roll_rad, Eigen::Vector3f::UnitX()))
            .toRotationMatrix();
    transform.translation() = translation;
    return transform;
}

inline std::string format_vec3(const Eigen::Vector3f& value) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(4) << "(" << value.x() << ", "
           << value.y() << ", " << value.z() << ")";
    return stream.str();
}

inline std::string format_vec2(const Eigen::Vector2i& value) {
    std::ostringstream stream;
    stream << "(" << value.x() << ", " << value.y() << ")";
    return stream.str();
}

inline std::filesystem::path frame_base_path(
    const std::filesystem::path& output_dir, const std::size_t frame_index) {
    std::ostringstream name;
    name << "frame_" << std::setw(4) << std::setfill('0') << frame_index;
    return output_dir / name.str();
}

}  // namespace rtabmap_semantic_mapping
