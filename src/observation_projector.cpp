#include "rtabmap_semantic_mapping/observation_projector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "rtabmap_semantic_mapping/utils.hpp"

namespace rtabmap_semantic_mapping
{

namespace
{

bool is_valid_depth(float value, float min_depth_m, float max_depth_m) {
    return std::isfinite(value) && value >= min_depth_m && value <= max_depth_m;
}

}  // namespace

ObservationProjector::ObservationProjector(ProjectorConfig config)
    : config_(std::move(config)) {}

const ProjectionSummary& ObservationProjector::summary() const noexcept {
    return summary_;
}

std::vector<ObjectObservation3D> ObservationProjector::project(
    const FrameData& frame, const std::vector<Detection2D>& detections) {
    std::vector<ObjectObservation3D> observations;
    observations.reserve(detections.size());
    summary_.total_detections += detections.size();

    for (const auto& detection : detections) {
        auto observation = project_single(frame, detection);
        if (observation.has_value()) {
            observations.push_back(*observation);
        }
    }

    return observations;
}

std::optional<ObjectObservation3D> ObservationProjector::project_single(
    const FrameData& frame, const Detection2D& detection) {
    if (detection.confidence < config_.min_confidence) {
        ++summary_.skipped_low_confidence;
        return std::nullopt;
    }

    if (detection.bbox.width < config_.min_bbox_size_px ||
        detection.bbox.height < config_.min_bbox_size_px) {
        ++summary_.skipped_invalid_bbox;
        return std::nullopt;
    }

    if (detection.bbox.x >= frame.depth.cols ||
        detection.bbox.y >= frame.depth.rows ||
        detection.bbox.x + detection.bbox.width <= 0 ||
        detection.bbox.y + detection.bbox.height <= 0) {
        ++summary_.skipped_invalid_bbox;
        return std::nullopt;
    }

    if (frame.depth.empty()) {
        ++summary_.skipped_no_valid_depth;
        return std::nullopt;
    }

    if (!has_valid_intrinsics(frame.intrinsics)) {
        ++summary_.skipped_invalid_intrinsics;
        return std::nullopt;
    }

    auto depth_sample = estimate_depth(frame.depth, detection.bbox);
    if (!depth_sample.has_value()) {
        ++summary_.skipped_no_valid_depth;
        return std::nullopt;
    }

    const auto position_camera = pixel_to_camera(
        frame.intrinsics, depth_sample->pixel, depth_sample->depth_m);
    if (!position_camera.has_value()) {
        ++summary_.skipped_invalid_intrinsics;
        return std::nullopt;
    }

    const Eigen::Vector3f position_map =
        frame.T_map_camera * (*position_camera);

    ObjectObservation3D observation;
    observation.node_id = frame.node_id;
    observation.stamp_sec = frame.stamp_sec;
    observation.class_id = detection.class_id;
    observation.class_name = detection.class_name;
    observation.confidence = detection.confidence;
    observation.position_camera = *position_camera;
    observation.position_map = position_map;
    observation.bbox = detection.bbox;
    observation.sampled_depth_m = depth_sample->depth_m;
    observation.sample_pixel = depth_sample->pixel;
    ++summary_.successful_observations;
    return observation;
}

bool ObservationProjector::has_valid_intrinsics(
    const CameraIntrinsics& intrinsics) const noexcept {
    return intrinsics.fx > 0.0F && intrinsics.fy > 0.0F &&
           std::isfinite(intrinsics.cx) && std::isfinite(intrinsics.cy);
}

std::optional<Eigen::Vector3f> ObservationProjector::pixel_to_camera(
    const CameraIntrinsics& intrinsics, const Eigen::Vector2i& pixel,
    const float depth_m) const {
    if (!has_valid_intrinsics(intrinsics) ||
        !is_valid_depth(depth_m, config_.min_depth_m, config_.max_depth_m)) {
        return std::nullopt;
    }

    const float u = static_cast<float>(pixel.x());
    const float v = static_cast<float>(pixel.y());
    const float x = (u - intrinsics.cx) * depth_m / intrinsics.fx;
    const float y = (v - intrinsics.cy) * depth_m / intrinsics.fy;
    return Eigen::Vector3f(x, y, depth_m);
}

std::optional<ObservationProjector::DepthSample>
ObservationProjector::estimate_depth(const cv::Mat& depth,
                                     const cv::Rect& bbox) const {
    const cv::Rect centered_roi = rtabmap_semantic_mapping::centered_sample_roi(
        bbox, config_.bbox_center_ratio);
    const cv::Rect valid_roi =
        centered_roi & cv::Rect(0, 0, depth.cols, depth.rows);

    if (valid_roi.empty()) {
        return std::nullopt;
    }

    std::vector<std::pair<float, Eigen::Vector2i>> samples;
    samples.reserve(static_cast<std::size_t>(valid_roi.area()));

    for (int v = valid_roi.y; v < valid_roi.y + valid_roi.height; ++v) {
        for (int u = valid_roi.x; u < valid_roi.x + valid_roi.width; ++u) {
            const auto depth_value = convert_depth_to_meters(depth, v, u);
            if (!depth_value.has_value()) {
                if (depth.type() != CV_16UC1 && depth.type() != CV_32FC1) {
                    return std::nullopt;
                }
                continue;
            }

            if (is_valid_depth(*depth_value, config_.min_depth_m,
                               config_.max_depth_m)) {
                samples.emplace_back(*depth_value, Eigen::Vector2i(u, v));
            }
            else if (depth.type() != CV_16UC1 && depth.type() != CV_32FC1) {
                return std::nullopt;
            }
        }
    }

    if (samples.empty()) {
        return std::nullopt;
    }

    const auto median_it = samples.begin() + (samples.size() / 2);
    std::nth_element(
        samples.begin(), median_it, samples.end(),
        [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
    return DepthSample{median_it->first, median_it->second};
}

}  // namespace rtabmap_semantic_mapping
