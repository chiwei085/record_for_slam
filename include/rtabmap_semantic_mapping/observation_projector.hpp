#pragma once

#include <optional>
#include <vector>

#include "rtabmap_semantic_mapping/types.hpp"

namespace rtabmap_semantic_mapping
{

class ObservationProjector
{
public:
    ObservationProjector() = default;
    explicit ObservationProjector(ProjectorConfig config);

    [[nodiscard]] std::vector<ObjectObservation3D> project(
        const FrameData& frame, const std::vector<Detection2D>& detections);

    [[nodiscard]] const ProjectionSummary& summary() const noexcept;

private:
    struct DepthSample
    {
        float depth_m = 0.0F;
        Eigen::Vector2i pixel = Eigen::Vector2i::Zero();
    };

    std::optional<ObjectObservation3D> project_single(
        const FrameData& frame, const Detection2D& detection);

    [[nodiscard]] bool has_valid_intrinsics(
        const CameraIntrinsics& intrinsics) const noexcept;
    [[nodiscard]] std::optional<Eigen::Vector3f> pixel_to_camera(
        const CameraIntrinsics& intrinsics, const Eigen::Vector2i& pixel,
        float depth_m) const;
    [[nodiscard]] std::optional<DepthSample> estimate_depth(
        const cv::Mat& depth, const cv::Rect& bbox) const;

    ProjectorConfig config_;
    ProjectionSummary summary_;
};

}  // namespace rtabmap_semantic_mapping
