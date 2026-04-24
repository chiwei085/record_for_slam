#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <cstdint>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace rtabmap_semantic_mapping
{

struct CameraIntrinsics
{
    int width = 0;
    int height = 0;

    float fx = 0.0F;
    float fy = 0.0F;
    float cx = 0.0F;
    float cy = 0.0F;
};

struct FrameData
{
    int node_id = -1;
    double stamp_sec = 0.0;

    cv::Mat rgb;
    cv::Mat depth;

    CameraIntrinsics intrinsics;
    Eigen::Isometry3f T_map_base = Eigen::Isometry3f::Identity();
    Eigen::Isometry3f T_base_camera = Eigen::Isometry3f::Identity();
    Eigen::Isometry3f T_map_camera = Eigen::Isometry3f::Identity();
};

struct DatasetSummary
{
    std::size_t total_nodes = 0;
    std::size_t optimized_poses = 0;
    std::size_t valid_frames = 0;
    std::size_t skipped_frames = 0;
    std::size_t frames_with_rgb = 0;
    std::size_t frames_with_depth = 0;
    std::size_t frames_with_camera_model = 0;
    std::map<std::string, std::size_t> skip_reason_counts;
};

struct Detection2D
{
    int class_id = -1;
    std::string class_name;
    float confidence = 0.0F;
    cv::Rect bbox;
};

struct FrameDetections
{
    int node_id = -1;
    double stamp_sec = 0.0;
    std::vector<Detection2D> detections;
};

struct DetectionSummary
{
    std::size_t total_frames = 0;
    std::size_t frames_with_detections = 0;
    std::size_t total_detections = 0;
    std::map<std::string, std::size_t> detections_per_class;
};

struct ObjectObservation3D
{
    int node_id = -1;
    double stamp_sec = 0.0;

    int class_id = -1;
    std::string class_name;
    float confidence = 0.0F;

    cv::Rect bbox;
    float sampled_depth_m = 0.0F;

    Eigen::Vector2i sample_pixel = Eigen::Vector2i::Zero();
    Eigen::Vector3f position_camera = Eigen::Vector3f::Zero();
    Eigen::Vector3f position_map = Eigen::Vector3f::Zero();
};

struct ProjectionSummary
{
    std::size_t total_detections = 0;
    std::size_t successful_observations = 0;
    std::size_t skipped_low_confidence = 0;
    std::size_t skipped_invalid_bbox = 0;
    std::size_t skipped_no_valid_depth = 0;
    std::size_t skipped_invalid_intrinsics = 0;
};

struct YoloDetectorConfig
{
    float confidence_threshold = 0.25F;
    float nms_iou_threshold = 0.45F;
    int max_detections = 300;
    bool class_agnostic_nms = false;
    std::vector<std::string> allowed_labels = {
        "person", "chair",  "couch",        "dining table", "tv",
        "laptop", "bottle", "potted plant", "backpack",     "book"};
};

struct ProjectorConfig
{
    float min_confidence = 0.25F;
    int min_bbox_size_px = 10;
    float min_depth_m = 0.2F;
    float max_depth_m = 8.0F;
    float bbox_center_ratio = 0.3F;
};

struct MapCloudConfig
{
    int pixel_step = 2;
    float min_depth_m = 0.2F;
    float max_depth_m = 8.0F;
    bool use_rgb = true;
    float voxel_leaf_size_m = 0.02F;
};

struct SemanticObject
{
    int object_id = -1;
    int class_id = -1;
    std::string class_name;

    Eigen::Vector3f position_map = Eigen::Vector3f::Zero();

    float max_confidence = 0.0F;
    int seen_count = 0;

    std::vector<int> support_node_ids;
    bool has_extent = false;
    Eigen::Vector3f bbox_min_map = Eigen::Vector3f::Zero();
    Eigen::Vector3f bbox_max_map = Eigen::Vector3f::Zero();
};

struct FusionSummary
{
    std::size_t total_observations = 0;
    std::size_t accepted_observations = 0;
    std::size_t created_objects = 0;
    std::size_t updated_objects = 0;
    std::size_t rejected_observations = 0;
};

struct FusionAssignment
{
    int node_id = -1;
    std::string class_name;
    Eigen::Vector3f observation_position_map = Eigen::Vector3f::Zero();
    std::string action;
    int matched_object_id = -1;
    float distance_to_match_m = -1.0F;
};

struct FusionConfig
{
    float match_distance_m = 0.9F;
    float ema_alpha = 0.3F;
    int min_seen_count_to_keep = 2;
    bool require_same_class = true;
    bool keep_singletons = true;
    std::vector<std::string> allowed_classes;
    std::unordered_map<std::string, float> class_distance_overrides = {
        {"book", 0.7F},
        {"tv", 1.0F},
        {"laptop", 1.2F},
        {"dining table", 1.5F},
    };
};

}  // namespace rtabmap_semantic_mapping
