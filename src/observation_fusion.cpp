#include "rtabmap_semantic_mapping/observation_fusion.hpp"

#include <algorithm>
#include <limits>
#include <utility>

namespace rtabmap_semantic_mapping
{
namespace
{

std::unordered_set<std::string> build_allowed_class_set(
    const std::vector<std::string>& allowed_classes) {
    return std::unordered_set<std::string>(allowed_classes.begin(),
                                           allowed_classes.end());
}

}  // namespace

ObservationFusion::ObservationFusion() : ObservationFusion(FusionConfig{}) {}

ObservationFusion::ObservationFusion(FusionConfig config)
    : config_(std::move(config)),
      allowed_classes_(build_allowed_class_set(config_.allowed_classes)) {}

void ObservationFusion::update(
    const std::vector<ObjectObservation3D>& observations) {
    for (const auto& observation : observations) {
        ++summary_.total_observations;

        if (!is_allowed(observation)) {
            ++summary_.rejected_observations;
            record_assignment(observation, "reject", -1, -1.0F);
            continue;
        }

        ++summary_.accepted_observations;

        float matched_distance = -1.0F;
        const int matched_index = find_match(observation, matched_distance);
        if (matched_index >= 0) {
            auto& object = objects_.at(static_cast<std::size_t>(matched_index));
            update_object(object, observation);
            ++summary_.updated_objects;
            record_assignment(observation, "update", object.object_id,
                              matched_distance);
        }
        else {
            objects_.push_back(create_object(observation));
            ++summary_.created_objects;
            record_assignment(observation, "create", objects_.back().object_id,
                              -1.0F);
        }
    }
}

const std::vector<SemanticObject>& ObservationFusion::objects() const noexcept {
    return objects_;
}

std::vector<SemanticObject> ObservationFusion::filtered_objects() const {
    std::vector<SemanticObject> filtered;
    filtered.reserve(objects_.size());
    const int min_seen_count = std::max(config_.min_seen_count_to_keep, 1);
    for (const auto& object : objects_) {
        // keep_singletons intentionally preserves first-sighting objects even
        // when the configured min_seen_count_to_keep is higher than 1.
        // This means seen_count==1 can be kept while intermediate counts
        // below the threshold are filtered out.
        if (object.seen_count >= min_seen_count ||
            (config_.keep_singletons && object.seen_count == 1)) {
            filtered.push_back(object);
        }
    }
    return filtered;
}

const FusionSummary& ObservationFusion::summary() const noexcept {
    return summary_;
}

const std::vector<FusionAssignment>& ObservationFusion::assignments()
    const noexcept {
    return assignments_;
}

bool ObservationFusion::is_allowed(
    const ObjectObservation3D& observation) const {
    if (allowed_classes_.empty()) {
        return true;
    }

    return allowed_classes_.find(observation.class_name) !=
           allowed_classes_.end();
}

float ObservationFusion::match_distance_for(
    const ObjectObservation3D& observation) const {
    const auto override_it =
        config_.class_distance_overrides.find(observation.class_name);
    if (override_it != config_.class_distance_overrides.end()) {
        return override_it->second;
    }
    return config_.match_distance_m;
}

int ObservationFusion::find_match(const ObjectObservation3D& observation,
                                  float& matched_distance) const {
    int best_index = -1;
    float best_distance = std::numeric_limits<float>::max();
    const float max_distance = match_distance_for(observation);

    for (std::size_t index = 0; index < objects_.size(); ++index) {
        const auto& object = objects_[index];
        if (config_.require_same_class &&
            object.class_id != observation.class_id) {
            continue;
        }

        const float distance =
            (object.position_map - observation.position_map).norm();
        if (distance < max_distance && distance < best_distance) {
            best_distance = distance;
            best_index = static_cast<int>(index);
        }
    }

    matched_distance = (best_index >= 0) ? best_distance : -1.0F;
    return best_index;
}

void ObservationFusion::update_object(SemanticObject& object,
                                      const ObjectObservation3D& observation) {
    object.position_map = (config_.ema_alpha * observation.position_map) +
                          ((1.0F - config_.ema_alpha) * object.position_map);
    object.max_confidence =
        std::max(object.max_confidence, observation.confidence);
    object.seen_count += 1;
    object.has_extent = true;
    object.bbox_min_map =
        object.bbox_min_map.cwiseMin(observation.position_map);
    object.bbox_max_map =
        object.bbox_max_map.cwiseMax(observation.position_map);
    if (std::find(object.support_node_ids.begin(),
                  object.support_node_ids.end(),
                  observation.node_id) == object.support_node_ids.end()) {
        object.support_node_ids.push_back(observation.node_id);
    }
}

SemanticObject ObservationFusion::create_object(
    const ObjectObservation3D& observation) {
    SemanticObject object;
    object.object_id = next_object_id_;
    object.class_id = observation.class_id;
    object.class_name = observation.class_name;
    object.position_map = observation.position_map;
    object.max_confidence = observation.confidence;
    object.seen_count = 1;
    object.support_node_ids.push_back(observation.node_id);
    object.has_extent = true;
    object.bbox_min_map = observation.position_map;
    object.bbox_max_map = observation.position_map;
    ++next_object_id_;
    return object;
}

void ObservationFusion::record_assignment(
    const ObjectObservation3D& observation, std::string action,
    const int matched_object_id, const float distance_to_match_m) {
    assignments_.push_back(FusionAssignment{
        observation.node_id,
        observation.class_name,
        observation.position_map,
        std::move(action),
        matched_object_id,
        distance_to_match_m,
    });
}

}  // namespace rtabmap_semantic_mapping
