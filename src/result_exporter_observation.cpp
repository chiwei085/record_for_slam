#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "rtabmap_semantic_mapping/result_exporter.hpp"
#include "rtabmap_semantic_mapping/utils.hpp"

namespace rtabmap_semantic_mapping
{
namespace
{

void write_observation_metadata(
    const std::filesystem::path& output_dir, const FrameData& frame,
    const std::vector<ObjectObservation3D>& observations,
    const std::size_t frame_index) {
    std::ofstream stream(frame_base_path(output_dir, frame_index).string() +
                         "_observations.txt");
    if (!stream.is_open()) {
        throw std::runtime_error(
            "Failed to open observation metadata file for writing.");
    }

    stream << "node_id: " << frame.node_id << '\n';
    stream << "stamp_sec: " << frame.stamp_sec << '\n';
    stream << "observations: " << observations.size() << '\n';
    for (std::size_t index = 0; index < observations.size(); ++index) {
        const auto& observation = observations[index];
        stream << "observation[" << index << "]: "
               << "class_id=" << observation.class_id << ", "
               << "class_name=" << observation.class_name << ", "
               << "confidence=" << observation.confidence << ", "
               << "bbox=(" << observation.bbox.x << ", " << observation.bbox.y
               << ", " << observation.bbox.width << ", "
               << observation.bbox.height << "), "
               << "sampled_depth_m=" << observation.sampled_depth_m << ", "
               << "sample_pixel=" << format_vec2(observation.sample_pixel)
               << ", "
               << "position_camera=" << format_vec3(observation.position_camera)
               << ", "
               << "position_map=" << format_vec3(observation.position_map)
               << '\n';
    }
}

void write_observation_summary(const std::filesystem::path& output_dir,
                               const ProjectionSummary& summary) {
    std::ofstream stream(output_dir / "observations_summary.txt");
    if (!stream.is_open()) {
        throw std::runtime_error(
            "Failed to open observations_summary.txt for writing.");
    }

    stream << "total_detections: " << summary.total_detections << '\n';
    stream << "successful_observations: " << summary.successful_observations
           << '\n';
    stream << "skipped_low_confidence: " << summary.skipped_low_confidence
           << '\n';
    stream << "skipped_invalid_bbox: " << summary.skipped_invalid_bbox << '\n';
    stream << "skipped_no_valid_depth: " << summary.skipped_no_valid_depth
           << '\n';
    stream << "skipped_invalid_intrinsics: "
           << summary.skipped_invalid_intrinsics << '\n';
}

void write_projection_overlay(
    const std::filesystem::path& output_dir, const FrameData& frame,
    const std::vector<Detection2D>& detections,
    const std::vector<ObjectObservation3D>& observations,
    const std::size_t frame_index, const float bbox_center_ratio) {
    cv::Mat overlay = frame.rgb.clone();
    for (const auto& detection : detections) {
        cv::rectangle(overlay, detection.bbox, cv::Scalar(128, 128, 128), 1);
    }

    for (const auto& observation : observations) {
        cv::rectangle(overlay, observation.bbox, cv::Scalar(0, 255, 0), 2);
        const cv::Rect sample_roi =
            centered_sample_roi(observation.bbox, bbox_center_ratio) &
            cv::Rect(0, 0, overlay.cols, overlay.rows);
        if (!sample_roi.empty()) {
            cv::rectangle(overlay, sample_roi, cv::Scalar(0, 255, 255), 1);
        }
        cv::circle(overlay,
                   cv::Point(observation.sample_pixel.x(),
                             observation.sample_pixel.y()),
                   3, cv::Scalar(0, 0, 255), cv::FILLED);

        std::ostringstream label;
        label << observation.class_name << ' ' << std::fixed
              << std::setprecision(2) << observation.confidence
              << " z=" << std::setprecision(2) << observation.sampled_depth_m
              << "m";
        int baseline = 0;
        const cv::Size label_size = cv::getTextSize(
            label.str(), cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        const int text_top =
            std::max(observation.bbox.y, label_size.height + 4);
        const cv::Rect text_background(
            observation.bbox.x, text_top - label_size.height - 4,
            label_size.width + 6, label_size.height + 6);
        cv::rectangle(overlay, text_background, cv::Scalar(0, 255, 0),
                      cv::FILLED);
        cv::putText(overlay, label.str(),
                    cv::Point(observation.bbox.x + 3, text_top - 4),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1,
                    cv::LINE_AA);
    }

    const std::filesystem::path base = frame_base_path(output_dir, frame_index);
    if (!cv::imwrite(base.string() + "_projection_overlay.png", overlay)) {
        throw std::runtime_error("Failed to write projection overlay image.");
    }
}

}  // namespace

void ResultExporter::export_observation_debug(
    const FrameData& frame, const std::vector<Detection2D>& detections,
    const std::vector<ObjectObservation3D>& observations,
    const std::size_t frame_index, const float bbox_center_ratio) const {
    const std::filesystem::path output_path(output_dir_);
    std::filesystem::create_directories(output_path);

    write_observation_metadata(output_path, frame, observations, frame_index);
    write_projection_overlay(output_path, frame, detections, observations,
                             frame_index, bbox_center_ratio);
}

void ResultExporter::export_observation_summary(
    const ProjectionSummary& summary) const {
    const std::filesystem::path output_path(output_dir_);
    std::filesystem::create_directories(output_path);
    write_observation_summary(output_path, summary);
}

}  // namespace rtabmap_semantic_mapping
