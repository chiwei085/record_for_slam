#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "rtabmap_semantic_mapping/types.hpp"

namespace rtabmap_semantic_mapping
{

class ResultExporter
{
public:
    explicit ResultExporter(std::string output_dir);

    [[nodiscard]] const std::string& output_dir() const noexcept;

    void export_dataset_debug(const DatasetSummary& summary,
                              const std::vector<FrameData>& frames) const;

    void export_detection_debug(const FrameData& frame,
                                const std::vector<Detection2D>& detections,
                                std::size_t frame_index) const;

    void export_detection_summary(const DetectionSummary& summary) const;

    void export_observation_debug(
        const FrameData& frame, const std::vector<Detection2D>& detections,
        const std::vector<ObjectObservation3D>& observations,
        std::size_t frame_index, float bbox_center_ratio) const;

    void export_observation_summary(const ProjectionSummary& summary) const;

    void export_fusion_debug(const std::vector<SemanticObject>& all_objects,
                             const std::vector<SemanticObject>& kept_objects,
                             const FusionSummary& summary,
                             const std::vector<FusionAssignment>& assignments,
                             int stable_min_seen_count) const;

private:
    std::string output_dir_;
};

}  // namespace rtabmap_semantic_mapping
