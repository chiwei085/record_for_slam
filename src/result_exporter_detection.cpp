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

std::string detection_label(const Detection2D& detection) {
    std::ostringstream stream;
    stream << detection.class_name << ' ' << std::fixed << std::setprecision(2)
           << detection.confidence;
    return stream.str();
}

void write_detection_overlay(const std::filesystem::path& output_dir,
                             const FrameData& frame,
                             const std::vector<Detection2D>& detections,
                             const std::size_t frame_index) {
    cv::Mat overlay = frame.rgb.clone();
    for (const auto& detection : detections) {
        cv::rectangle(overlay, detection.bbox, cv::Scalar(0, 255, 0), 2);
        const std::string label = detection_label(detection);
        int baseline = 0;
        const cv::Size label_size =
            cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        const int text_top = std::max(detection.bbox.y, label_size.height + 4);
        const cv::Rect text_background(
            detection.bbox.x, text_top - label_size.height - 4,
            label_size.width + 6, label_size.height + 6);
        cv::rectangle(overlay, text_background, cv::Scalar(0, 255, 0),
                      cv::FILLED);
        cv::putText(
            overlay, label, cv::Point(detection.bbox.x + 3, text_top - 4),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    }

    const std::filesystem::path base = frame_base_path(output_dir, frame_index);
    if (!cv::imwrite(base.string() + "_overlay.png", overlay)) {
        throw std::runtime_error("Failed to write detection overlay image.");
    }
}

void write_detection_metadata(const std::filesystem::path& output_dir,
                              const FrameData& frame,
                              const std::vector<Detection2D>& detections,
                              const std::size_t frame_index) {
    std::ofstream stream(frame_base_path(output_dir, frame_index).string() +
                         "_detections.txt");
    if (!stream.is_open()) {
        throw std::runtime_error(
            "Failed to open detection metadata file for writing.");
    }

    stream << "node_id: " << frame.node_id << '\n';
    stream << "stamp_sec: " << frame.stamp_sec << '\n';
    stream << "detections: " << detections.size() << '\n';
    for (std::size_t index = 0; index < detections.size(); ++index) {
        const auto& detection = detections[index];
        stream << "detection[" << index << "]: "
               << "class_id=" << detection.class_id << ", "
               << "class_name=" << detection.class_name << ", "
               << "confidence=" << detection.confidence << ", "
               << "bbox=(" << detection.bbox.x << ", " << detection.bbox.y
               << ", " << detection.bbox.width << ", " << detection.bbox.height
               << ")\n";
    }
}

void write_detection_summary(const std::filesystem::path& output_dir,
                             const DetectionSummary& summary) {
    std::ofstream stream(output_dir / "detections_summary.txt");
    if (!stream.is_open()) {
        throw std::runtime_error(
            "Failed to open detections_summary.txt for writing.");
    }

    stream << "total_frames: " << summary.total_frames << '\n';
    stream << "frames_with_detections: " << summary.frames_with_detections
           << '\n';
    stream << "total_detections: " << summary.total_detections << '\n';
    stream << "detections_per_class:\n";
    for (const auto& [class_name, count] : summary.detections_per_class) {
        stream << "  " << class_name << ": " << count << '\n';
    }
}

}  // namespace

void ResultExporter::export_detection_debug(
    const FrameData& frame, const std::vector<Detection2D>& detections,
    const std::size_t frame_index) const {
    const std::filesystem::path output_path(output_dir_);
    std::filesystem::create_directories(output_path);

    write_detection_overlay(output_path, frame, detections, frame_index);
    write_detection_metadata(output_path, frame, detections, frame_index);
}

void ResultExporter::export_detection_summary(
    const DetectionSummary& summary) const {
    const std::filesystem::path output_path(output_dir_);
    std::filesystem::create_directories(output_path);
    write_detection_summary(output_path, summary);
}

}  // namespace rtabmap_semantic_mapping
