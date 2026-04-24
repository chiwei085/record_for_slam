#pragma once

#include <unordered_set>
#include <vector>

#include "rtabmap_semantic_mapping/types.hpp"

namespace rtabmap_semantic_mapping
{

class ObservationFusion
{
public:
    ObservationFusion();
    explicit ObservationFusion(FusionConfig config);

    void update(const std::vector<ObjectObservation3D>& observations);
    [[nodiscard]] const std::vector<SemanticObject>& objects() const noexcept;
    [[nodiscard]] std::vector<SemanticObject> filtered_objects() const;
    [[nodiscard]] const FusionSummary& summary() const noexcept;
    [[nodiscard]] const std::vector<FusionAssignment>& assignments()
        const noexcept;

private:
    [[nodiscard]] bool is_allowed(const ObjectObservation3D& observation) const;
    [[nodiscard]] float match_distance_for(
        const ObjectObservation3D& observation) const;
    [[nodiscard]] int find_match(const ObjectObservation3D& observation,
                                 float& matched_distance) const;
    void update_object(SemanticObject& object,
                       const ObjectObservation3D& observation);
    SemanticObject create_object(const ObjectObservation3D& observation);
    void record_assignment(const ObjectObservation3D& observation,
                           std::string action, int matched_object_id,
                           float distance_to_match_m);

    FusionConfig config_;
    FusionSummary summary_;
    std::vector<SemanticObject> objects_;
    std::vector<FusionAssignment> assignments_;
    std::unordered_set<std::string> allowed_classes_;
    int next_object_id_ = 0;
};

}  // namespace rtabmap_semantic_mapping
