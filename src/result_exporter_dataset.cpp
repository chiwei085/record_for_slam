#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "rtabmap_semantic_mapping/result_exporter.hpp"
#include "rtabmap_semantic_mapping/utils.hpp"

namespace rtabmap_semantic_mapping
{
namespace
{

std::string cv_type_to_string(const int type) {
    const int depth = type & CV_MAT_DEPTH_MASK;
    const int channels = 1 + (type >> CV_CN_SHIFT);

    std::string depth_string;
    switch (depth) {
        case CV_8U:
            depth_string = "8U";
            break;
        case CV_8S:
            depth_string = "8S";
            break;
        case CV_16U:
            depth_string = "16U";
            break;
        case CV_16S:
            depth_string = "16S";
            break;
        case CV_32S:
            depth_string = "32S";
            break;
        case CV_32F:
            depth_string = "32F";
            break;
        case CV_64F:
            depth_string = "64F";
            break;
        default:
            depth_string = "User";
            break;
    }

    return "CV_" + depth_string + "C" + std::to_string(channels);
}

cv::Mat depth_to_debug_image(const cv::Mat& depth) {
    if (depth.empty()) {
        return {};
    }

    if (depth.type() == CV_16UC1) {
        return depth;
    }

    if (depth.type() == CV_32FC1) {
        cv::Mat depth_mm(depth.rows, depth.cols, CV_16UC1, cv::Scalar(0));
        for (int row = 0; row < depth.rows; ++row) {
            for (int col = 0; col < depth.cols; ++col) {
                const float depth_m = depth.at<float>(row, col);
                if (std::isfinite(depth_m) && depth_m > 0.0F) {
                    const float depth_mm_value = depth_m * 1000.0F;
                    depth_mm.at<std::uint16_t>(row, col) =
                        static_cast<std::uint16_t>(std::min(
                            depth_mm_value,
                            static_cast<float>(
                                std::numeric_limits<std::uint16_t>::max())));
                }
            }
        }
        return depth_mm;
    }

    cv::Mat normalized_depth;
    cv::normalize(depth, normalized_depth, 0, 65535, cv::NORM_MINMAX, CV_16UC1);
    return normalized_depth;
}

std::string format_rotation_matrix(const Eigen::Matrix3f& value) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(4) << "[[" << value(0, 0) << ", "
           << value(0, 1) << ", " << value(0, 2) << "], "
           << "[" << value(1, 0) << ", " << value(1, 1) << ", " << value(1, 2)
           << "], "
           << "[" << value(2, 0) << ", " << value(2, 1) << ", " << value(2, 2)
           << "]]";
    return stream.str();
}

void write_summary(const std::filesystem::path& output_dir,
                   const DatasetSummary& summary,
                   const std::vector<FrameData>& frames) {
    std::ofstream stream(output_dir / "summary.txt");
    if (!stream.is_open()) {
        throw std::runtime_error("Failed to open summary.txt for writing.");
    }

    stream << "total_nodes: " << summary.total_nodes << '\n';
    stream << "optimized_poses: " << summary.optimized_poses << '\n';
    stream << "valid_frames: " << summary.valid_frames << '\n';
    stream << "skipped_frames: " << summary.skipped_frames << '\n';
    stream << "frames_with_rgb: " << summary.frames_with_rgb << '\n';
    stream << "frames_with_depth: " << summary.frames_with_depth << '\n';
    stream << "frames_with_camera_model: " << summary.frames_with_camera_model
           << '\n';
    stream << "exported_frames: " << frames.size() << '\n';
    stream << "skip_reasons:\n";
    for (const auto& [reason, count] : summary.skip_reason_counts) {
        stream << "  " << reason << ": " << count << '\n';
    }
}

void write_frame_metadata(const std::filesystem::path& output_dir,
                          const std::size_t frame_index,
                          const FrameData& frame) {
    std::ofstream stream(frame_base_path(output_dir, frame_index).string() +
                         ".txt");
    if (!stream.is_open()) {
        throw std::runtime_error(
            "Failed to open frame metadata file for writing.");
    }

    stream << "node_id: " << frame.node_id << '\n';
    stream << "stamp_sec: " << frame.stamp_sec << '\n';
    stream << "rgb_size: " << frame.rgb.cols << "x" << frame.rgb.rows << '\n';
    stream << "rgb_type: " << cv_type_to_string(frame.rgb.type()) << '\n';
    stream << "depth_size: " << frame.depth.cols << "x" << frame.depth.rows
           << '\n';
    stream << "depth_type: " << cv_type_to_string(frame.depth.type()) << '\n';
    stream << "intrinsics: "
           << "fx=" << frame.intrinsics.fx << ", "
           << "fy=" << frame.intrinsics.fy << ", "
           << "cx=" << frame.intrinsics.cx << ", "
           << "cy=" << frame.intrinsics.cy << ", "
           << "width=" << frame.intrinsics.width << ", "
           << "height=" << frame.intrinsics.height << '\n';
    stream << "optimized_pose_translation: "
           << frame.T_map_base.translation().x() << ", "
           << frame.T_map_base.translation().y() << ", "
           << frame.T_map_base.translation().z() << '\n';
    stream << "base_to_camera_translation: "
           << frame.T_base_camera.translation().x() << ", "
           << frame.T_base_camera.translation().y() << ", "
           << frame.T_base_camera.translation().z() << '\n';
    stream << "base_to_camera_rotation: "
           << format_rotation_matrix(frame.T_base_camera.rotation()) << '\n';
    stream << "map_to_camera_translation: "
           << frame.T_map_camera.translation().x() << ", "
           << frame.T_map_camera.translation().y() << ", "
           << frame.T_map_camera.translation().z() << '\n';
}

void write_frame_images(const std::filesystem::path& output_dir,
                        const std::size_t frame_index, const FrameData& frame) {
    const std::filesystem::path base = frame_base_path(output_dir, frame_index);
    if (!cv::imwrite(base.string() + "_rgb.png", frame.rgb)) {
        throw std::runtime_error("Failed to write RGB debug image.");
    }

    const cv::Mat depth_debug = depth_to_debug_image(frame.depth);
    if (depth_debug.empty() ||
        !cv::imwrite(base.string() + "_depth.png", depth_debug)) {
        throw std::runtime_error("Failed to write depth debug image.");
    }
}

}  // namespace

ResultExporter::ResultExporter(std::string output_dir)
    : output_dir_(std::move(output_dir)) {
    std::filesystem::create_directories(output_dir_);
}

const std::string& ResultExporter::output_dir() const noexcept {
    return output_dir_;
}

void ResultExporter::export_dataset_debug(
    const DatasetSummary& summary, const std::vector<FrameData>& frames) const {
    const std::filesystem::path output_path(output_dir_);
    std::filesystem::create_directories(output_path);

    write_summary(output_path, summary, frames);

    for (std::size_t index = 0; index < frames.size(); ++index) {
        write_frame_metadata(output_path, index, frames[index]);
        write_frame_images(output_path, index, frames[index]);
    }
}

}  // namespace rtabmap_semantic_mapping
