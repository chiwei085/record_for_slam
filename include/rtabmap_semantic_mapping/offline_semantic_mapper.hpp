#pragma once

#include <string>
#include <vector>

#include "rtabmap_semantic_mapping/database_reader.hpp"
#include "rtabmap_semantic_mapping/map_cloud_exporter.hpp"
#include "rtabmap_semantic_mapping/observation_fusion.hpp"
#include "rtabmap_semantic_mapping/observation_projector.hpp"
#include "rtabmap_semantic_mapping/result_exporter.hpp"
#include "rtabmap_semantic_mapping/types.hpp"

namespace rtabmap_semantic_mapping
{

// YoloDetector is an internal component used exclusively by
// OfflineSemanticMapper. Its implementation lives in
// offline_semantic_mapper.cpp.
class YoloDetector
{
public:
    explicit YoloDetector(std::string model_path,
                          YoloDetectorConfig config = {});

    [[nodiscard]] std::vector<Detection2D> infer(const cv::Mat& rgb) const;

private:
    std::string model_path_;
    YoloDetectorConfig config_;
};

struct OfflineSemanticMapperResult
{
    DatasetSummary dataset_summary;
    DetectionSummary detection_summary;
    ProjectionSummary projection_summary;
    FusionSummary fusion_summary;
    std::vector<FrameData> frames;
    std::vector<FrameDetections> frame_detections;
    std::vector<std::vector<ObjectObservation3D>> frame_observations;
    std::vector<SemanticObject> semantic_objects;
};

struct OfflineSemanticMapperConfig
{
    std::string database_path;
    std::string model_path;
    std::string output_dir;

    YoloDetectorConfig detector;
    ProjectorConfig projector;
    FusionConfig fusion;
    MapCloudConfig map_cloud;
};

class OfflineSemanticMapper
{
public:
    explicit OfflineSemanticMapper(OfflineSemanticMapperConfig config);

    [[nodiscard]] OfflineSemanticMapperResult run();

private:
    OfflineSemanticMapperConfig config_;
    DatabaseReader database_reader_;
    YoloDetector detector_;
    ObservationProjector projector_;
    ObservationFusion fusion_;
    MapCloudExporter map_cloud_exporter_;
    ResultExporter exporter_;
};

}  // namespace rtabmap_semantic_mapping
