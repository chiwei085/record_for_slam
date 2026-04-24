#pragma once

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>

#include "rtabmap_semantic_mapping/offline_semantic_mapper.hpp"

namespace rtabmap_semantic_mapping
{

namespace detail
{

inline void load_detector_config(const YAML::Node& node,
                                 YoloDetectorConfig& cfg) {
    if (!node || !node.IsMap()) {
        return;
    }
    if (const auto v = node["confidence_threshold"]) {
        cfg.confidence_threshold = v.as<float>();
    }
    if (const auto v = node["nms_iou_threshold"]) {
        cfg.nms_iou_threshold = v.as<float>();
    }
    if (const auto v = node["max_detections"]) {
        cfg.max_detections = v.as<int>();
    }
    if (const auto v = node["class_agnostic_nms"]) {
        cfg.class_agnostic_nms = v.as<bool>();
    }
    if (const auto v = node["allowed_labels"]) {
        cfg.allowed_labels = v.as<std::vector<std::string>>();
    }
}

inline void load_projector_config(const YAML::Node& node,
                                  ProjectorConfig& cfg) {
    if (!node || !node.IsMap()) {
        return;
    }
    if (const auto v = node["min_confidence"]) {
        cfg.min_confidence = v.as<float>();
    }
    if (const auto v = node["min_bbox_size_px"]) {
        cfg.min_bbox_size_px = v.as<int>();
    }
    if (const auto v = node["min_depth_m"]) {
        cfg.min_depth_m = v.as<float>();
    }
    if (const auto v = node["max_depth_m"]) {
        cfg.max_depth_m = v.as<float>();
    }
    if (const auto v = node["bbox_center_ratio"]) {
        cfg.bbox_center_ratio = v.as<float>();
    }
}

inline void load_fusion_config(const YAML::Node& node, FusionConfig& cfg) {
    if (!node || !node.IsMap()) {
        return;
    }
    if (const auto v = node["match_distance_m"]) {
        cfg.match_distance_m = v.as<float>();
    }
    if (const auto v = node["ema_alpha"]) {
        cfg.ema_alpha = v.as<float>();
    }
    if (const auto v = node["min_seen_count_to_keep"]) {
        cfg.min_seen_count_to_keep = v.as<int>();
    }
    if (const auto v = node["require_same_class"]) {
        cfg.require_same_class = v.as<bool>();
    }
    if (const auto v = node["keep_singletons"]) {
        cfg.keep_singletons = v.as<bool>();
    }
    if (const auto v = node["allowed_classes"]) {
        cfg.allowed_classes = v.as<std::vector<std::string>>();
    }
    if (const auto v = node["class_distance_overrides"]) {
        if (v.IsMap()) {
            cfg.class_distance_overrides.clear();
            for (const auto& pair : v) {
                cfg.class_distance_overrides[pair.first.as<std::string>()] =
                    pair.second.as<float>();
            }
        }
    }
}

inline void load_map_cloud_config(const YAML::Node& node, MapCloudConfig& cfg) {
    if (!node || !node.IsMap()) {
        return;
    }
    if (const auto v = node["pixel_step"]) {
        cfg.pixel_step = v.as<int>();
    }
    if (const auto v = node["min_depth_m"]) {
        cfg.min_depth_m = v.as<float>();
    }
    if (const auto v = node["max_depth_m"]) {
        cfg.max_depth_m = v.as<float>();
    }
    if (const auto v = node["use_rgb"]) {
        cfg.use_rgb = v.as<bool>();
    }
    if (const auto v = node["voxel_leaf_size_m"]) {
        cfg.voxel_leaf_size_m = v.as<float>();
    }
}

}  // namespace detail

// Load sub-configs from a YAML file into config.
// Path fields (database_path, model_path, output_dir) are only overwritten
// when non-empty in the YAML — CLI arguments take precedence over YAML values,
// so callers should apply YAML first, then overwrite with CLI values.
inline void load_config_from_yaml(const std::string& yaml_path,
                                  OfflineSemanticMapperConfig& config) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    }
    catch (const YAML::Exception& e) {
        throw std::runtime_error("Failed to parse YAML config '" + yaml_path +
                                 "': " + e.what());
    }

    if (!root.IsMap()) {
        throw std::runtime_error("YAML config '" + yaml_path +
                                 "' must be a mapping at the top level.");
    }

    if (const auto v = root["database_path"]) {
        const auto s = v.as<std::string>();
        if (!s.empty()) {
            config.database_path = s;
        }
    }
    if (const auto v = root["model_path"]) {
        const auto s = v.as<std::string>();
        if (!s.empty()) {
            config.model_path = s;
        }
    }
    if (const auto v = root["output_dir"]) {
        const auto s = v.as<std::string>();
        if (!s.empty()) {
            config.output_dir = s;
        }
    }

    detail::load_detector_config(root["detector"], config.detector);
    detail::load_projector_config(root["projector"], config.projector);
    detail::load_fusion_config(root["fusion"], config.fusion);
    detail::load_map_cloud_config(root["map_cloud"], config.map_cloud);
}

}  // namespace rtabmap_semantic_mapping
