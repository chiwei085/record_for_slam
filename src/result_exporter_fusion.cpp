#include <Eigen/Core>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <numeric>
#include <sstream>
#include <stdexcept>

#include "rtabmap_semantic_mapping/class_colors.hpp"
#include "rtabmap_semantic_mapping/result_exporter.hpp"
#include "rtabmap_semantic_mapping/utils.hpp"

namespace rtabmap_semantic_mapping
{
namespace
{

std::uint32_t pack_rgb(const std::array<std::uint8_t, 3>& color) {
    return (static_cast<std::uint32_t>(color[0]) << 16) |
           (static_cast<std::uint32_t>(color[1]) << 8) |
           static_cast<std::uint32_t>(color[2]);
}

void write_fusion_summary(const std::filesystem::path& output_dir,
                          const std::vector<SemanticObject>& all_objects,
                          const std::vector<SemanticObject>& kept_objects,
                          const FusionSummary& summary) {
    std::map<std::string, std::size_t> per_class_counts;
    for (const auto& object : kept_objects) {
        ++per_class_counts[object.class_name];
    }

    std::ofstream stream(output_dir / "semantic_objects_summary.txt");
    if (!stream.is_open()) {
        throw std::runtime_error(
            "Failed to open semantic_objects_summary.txt for writing.");
    }

    stream << "total_observations: " << summary.total_observations << '\n';
    stream << "accepted_observations: " << summary.accepted_observations
           << '\n';
    stream << "created_objects: " << summary.created_objects << '\n';
    stream << "updated_objects: " << summary.updated_objects << '\n';
    stream << "rejected_observations: " << summary.rejected_observations
           << '\n';
    stream << "all_object_count: " << all_objects.size() << '\n';
    stream << "final_object_count: " << kept_objects.size() << '\n';
    stream << "objects_per_class:\n";
    for (const auto& [class_name, count] : per_class_counts) {
        stream << "  " << class_name << ": " << count << '\n';
    }
}

void write_semantic_objects(const std::filesystem::path& output_dir,
                            const std::vector<SemanticObject>& kept_objects) {
    std::ofstream stream(output_dir / "semantic_objects.txt");
    if (!stream.is_open()) {
        throw std::runtime_error(
            "Failed to open semantic_objects.txt for writing.");
    }

    for (const auto& object : kept_objects) {
        stream << "object_id=" << object.object_id << ", "
               << "class_id=" << object.class_id << ", "
               << "class_name=" << object.class_name << ", "
               << "position_map=" << format_vec3(object.position_map) << ", "
               << "seen_count=" << object.seen_count << ", "
               << "max_confidence=" << object.max_confidence << ", "
               << "support_node_count=" << object.support_node_ids.size()
               << ", "
               << "has_extent=" << object.has_extent << ", "
               << "bbox_min_map=" << format_vec3(object.bbox_min_map) << ", "
               << "bbox_max_map=" << format_vec3(object.bbox_max_map)
               << ", support_node_ids=";
        for (std::size_t index = 0; index < object.support_node_ids.size();
             ++index) {
            stream << object.support_node_ids[index];
            if (index + 1 < object.support_node_ids.size()) {
                stream << ",";
            }
        }
        stream << '\n';
    }
}

void write_semantic_objects_csv(
    const std::filesystem::path& output_dir,
    const std::vector<SemanticObject>& kept_objects) {
    std::ofstream stream(output_dir / "semantic_objects.csv");
    if (!stream.is_open()) {
        throw std::runtime_error(
            "Failed to open semantic_objects.csv for writing.");
    }

    stream << "object_id,class_id,class_name,x,y,z,seen_count,max_confidence,"
              "support_node_count,has_extent,bbox_min_x,bbox_min_y,bbox_min_z,"
              "bbox_max_x,bbox_max_y,bbox_max_z\n";
    for (const auto& object : kept_objects) {
        stream << object.object_id << "," << object.class_id << ","
               << object.class_name << "," << object.position_map.x() << ","
               << object.position_map.y() << "," << object.position_map.z()
               << "," << object.seen_count << "," << object.max_confidence
               << "," << object.support_node_ids.size() << ","
               << object.has_extent << "," << object.bbox_min_map.x() << ","
               << object.bbox_min_map.y() << "," << object.bbox_min_map.z()
               << "," << object.bbox_max_map.x() << ","
               << object.bbox_max_map.y() << "," << object.bbox_max_map.z()
               << '\n';
    }
}

void write_semantic_objects_labels(
    const std::filesystem::path& output_dir,
    const std::vector<SemanticObject>& kept_objects) {
    std::ofstream stream(output_dir / "semantic_objects_labels.txt");
    if (!stream.is_open()) {
        throw std::runtime_error(
            "Failed to open semantic_objects_labels.txt for writing.");
    }

    for (const auto& object : kept_objects) {
        stream << "object_id=" << object.object_id
               << ", class_name=" << object.class_name
               << ", seen_count=" << object.seen_count
               << ", max_confidence=" << object.max_confidence
               << ", position_map=" << format_vec3(object.position_map)
               << ", has_extent=" << object.has_extent
               << ", bbox_min_map=" << format_vec3(object.bbox_min_map)
               << ", bbox_max_map=" << format_vec3(object.bbox_max_map) << '\n';
    }
}

void write_class_colors(const std::filesystem::path& output_dir,
                        const std::vector<SemanticObject>& kept_objects) {
    std::map<std::string, std::array<std::uint8_t, 3>> used_colors;
    for (const auto& object : kept_objects) {
        used_colors.emplace(object.class_name,
                            class_color_rgb(object.class_id));
    }

    std::ofstream stream(output_dir / "class_colors.txt");
    if (!stream.is_open()) {
        throw std::runtime_error(
            "Failed to open class_colors.txt for writing.");
    }

    for (const auto& [class_name, color] : used_colors) {
        stream << class_name << " -> " << static_cast<int>(color[0]) << ","
               << static_cast<int>(color[1]) << ","
               << static_cast<int>(color[2]) << '\n';
    }
}

void write_semantic_objects_pcd(const std::filesystem::path& output_path,
                                const std::vector<SemanticObject>& objects) {
    std::ofstream stream(output_path);
    if (!stream.is_open()) {
        throw std::runtime_error("Failed to open PCD file for writing.");
    }

    stream << "# .PCD v0.7 - Point Cloud Data file format\n";
    stream << "VERSION 0.7\n";
    stream << "FIELDS x y z rgb\n";
    stream << "SIZE 4 4 4 4\n";
    stream << "TYPE F F F U\n";
    stream << "COUNT 1 1 1 1\n";
    stream << "WIDTH " << objects.size() << '\n';
    stream << "HEIGHT 1\n";
    stream << "VIEWPOINT 0 0 0 1 0 0 0\n";
    stream << "POINTS " << objects.size() << '\n';
    stream << "DATA ascii\n";

    for (const auto& object : objects) {
        stream << object.position_map.x() << " " << object.position_map.y()
               << " " << object.position_map.z() << " "
               << pack_rgb(class_color_rgb(object.class_id)) << '\n';
    }
}

void write_class_stats(const std::filesystem::path& output_dir,
                       const std::vector<SemanticObject>& kept_objects,
                       const std::vector<FusionAssignment>& assignments,
                       const int stable_min_seen_count) {
    struct Stats
    {
        std::size_t observation_count = 0;
        std::size_t object_count = 0;
        std::size_t stable_object_count = 0;
        double seen_count_sum = 0.0;
        int max_seen_count = 0;
    };

    std::map<std::string, Stats> stats;
    for (const auto& assignment : assignments) {
        if (assignment.action != "reject") {
            ++stats[assignment.class_name].observation_count;
        }
    }
    for (const auto& object : kept_objects) {
        auto& class_stats = stats[object.class_name];
        ++class_stats.object_count;
        if (object.seen_count >= stable_min_seen_count) {
            ++class_stats.stable_object_count;
        }
        class_stats.seen_count_sum += static_cast<double>(object.seen_count);
        class_stats.max_seen_count =
            std::max(class_stats.max_seen_count, object.seen_count);
    }

    std::ofstream stream(output_dir / "class_stats.txt");
    if (!stream.is_open()) {
        throw std::runtime_error("Failed to open class_stats.txt for writing.");
    }

    for (const auto& [class_name, class_stats] : stats) {
        const double average_seen_count =
            class_stats.object_count > 0
                ? (class_stats.seen_count_sum /
                   static_cast<double>(class_stats.object_count))
                : 0.0;
        stream << "class_name=" << class_name
               << ", observation_count=" << class_stats.observation_count
               << ", final_object_count=" << class_stats.object_count
               << ", stable_object_count=" << class_stats.stable_object_count
               << ", average_seen_count=" << average_seen_count
               << ", max_seen_count=" << class_stats.max_seen_count << '\n';
    }
}

void write_fusion_distance_report(
    const std::filesystem::path& output_dir,
    const std::vector<FusionAssignment>& assignments) {
    std::map<std::string, std::vector<float>> per_class_distances;
    for (const auto& assignment : assignments) {
        if (assignment.action == "update" &&
            assignment.distance_to_match_m >= 0.0F) {
            per_class_distances[assignment.class_name].push_back(
                assignment.distance_to_match_m);
        }
    }

    std::ofstream stream(output_dir / "fusion_distance_report.txt");
    if (!stream.is_open()) {
        throw std::runtime_error(
            "Failed to open fusion_distance_report.txt for writing.");
    }

    stream << "update_matches:\n";
    for (const auto& assignment : assignments) {
        if (assignment.action == "update" &&
            assignment.distance_to_match_m >= 0.0F) {
            stream << "  class_name=" << assignment.class_name
                   << ", matched_object_id=" << assignment.matched_object_id
                   << ", distance_to_match_m=" << assignment.distance_to_match_m
                   << '\n';
        }
    }

    stream << "per_class_distance_stats:\n";
    for (auto& [class_name, distances] : per_class_distances) {
        std::sort(distances.begin(), distances.end());
        const float min_distance = distances.front();
        const float max_distance = distances.back();
        const double mean_distance =
            std::accumulate(distances.begin(), distances.end(), 0.0) /
            static_cast<double>(distances.size());
        const float median_distance = distances[distances.size() / 2];
        stream << "  class_name=" << class_name
               << ", update_count=" << distances.size()
               << ", min_distance=" << min_distance
               << ", median_distance=" << median_distance
               << ", mean_distance=" << mean_distance
               << ", max_distance=" << max_distance << '\n';
    }
}

void write_fusion_assignments(
    const std::filesystem::path& output_dir,
    const std::vector<FusionAssignment>& assignments) {
    std::ofstream stream(output_dir / "fusion_assignments.txt");
    if (!stream.is_open()) {
        throw std::runtime_error(
            "Failed to open fusion_assignments.txt for writing.");
    }

    for (const auto& assignment : assignments) {
        stream << "node_id=" << assignment.node_id << ", "
               << "class_name=" << assignment.class_name << ", "
               << "obs_position_map="
               << format_vec3(assignment.observation_position_map) << ", "
               << "action=" << assignment.action << ", "
               << "matched_object_id=" << assignment.matched_object_id << ", "
               << "distance_to_match_m=" << assignment.distance_to_match_m
               << '\n';
    }
}

void write_semantic_objects_overlay(
    const std::filesystem::path& output_dir,
    const std::vector<SemanticObject>& kept_objects) {
    constexpr int canvas_size_px = 800;
    constexpr int margin_px = 60;
    cv::Mat canvas(canvas_size_px, canvas_size_px, CV_8UC3,
                   cv::Scalar(245, 245, 245));

    cv::line(canvas, cv::Point(margin_px, canvas_size_px - margin_px),
             cv::Point(canvas_size_px - margin_px, canvas_size_px - margin_px),
             cv::Scalar(180, 180, 180), 1);
    cv::line(canvas, cv::Point(margin_px, margin_px),
             cv::Point(margin_px, canvas_size_px - margin_px),
             cv::Scalar(180, 180, 180), 1);

    if (kept_objects.empty()) {
        cv::putText(canvas, "No semantic objects", cv::Point(220, 400),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(30, 30, 30), 2,
                    cv::LINE_AA);
    }
    else {
        float min_x = kept_objects.front().position_map.x();
        float max_x = min_x;
        float min_y = kept_objects.front().position_map.y();
        float max_y = min_y;

        for (const auto& object : kept_objects) {
            min_x = std::min(min_x, object.position_map.x());
            max_x = std::max(max_x, object.position_map.x());
            min_y = std::min(min_y, object.position_map.y());
            max_y = std::max(max_y, object.position_map.y());
        }

        const float range_x = std::max(max_x - min_x, 1.0F);
        const float range_y = std::max(max_y - min_y, 1.0F);
        const float scale_x =
            static_cast<float>(canvas_size_px - (2 * margin_px)) / range_x;
        const float scale_y =
            static_cast<float>(canvas_size_px - (2 * margin_px)) / range_y;

        for (const auto& object : kept_objects) {
            const int x =
                margin_px + static_cast<int>(std::round(
                                (object.position_map.x() - min_x) * scale_x));
            const int y = canvas_size_px - margin_px -
                          static_cast<int>(std::round(
                              (object.position_map.y() - min_y) * scale_y));
            cv::circle(canvas, cv::Point(x, y), 6, cv::Scalar(0, 90, 220),
                       cv::FILLED);

            std::ostringstream label;
            label << object.class_name << " #" << object.object_id
                  << " s=" << object.seen_count;
            cv::putText(canvas, label.str(), cv::Point(x + 8, y - 8),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(20, 20, 20),
                        1, cv::LINE_AA);
        }
    }

    if (!cv::imwrite((output_dir / "semantic_objects_overlay.png").string(),
                     canvas)) {
        throw std::runtime_error(
            "Failed to write semantic_objects_overlay.png.");
    }
}

}  // namespace

void ResultExporter::export_fusion_debug(
    const std::vector<SemanticObject>& all_objects,
    const std::vector<SemanticObject>& kept_objects,
    const FusionSummary& summary,
    const std::vector<FusionAssignment>& assignments,
    const int stable_min_seen_count) const {
    const std::filesystem::path output_path(output_dir_);
    std::filesystem::create_directories(output_path);
    std::vector<SemanticObject> stable_objects;
    stable_objects.reserve(kept_objects.size());
    for (const auto& object : kept_objects) {
        if (object.seen_count >= stable_min_seen_count) {
            stable_objects.push_back(object);
        }
    }

    write_fusion_summary(output_path, all_objects, kept_objects, summary);
    write_semantic_objects(output_path, kept_objects);
    write_semantic_objects_csv(output_path, kept_objects);
    write_semantic_objects_labels(output_path, kept_objects);
    write_fusion_assignments(output_path, assignments);
    write_class_stats(output_path, kept_objects, assignments,
                      stable_min_seen_count);
    write_fusion_distance_report(output_path, assignments);
    write_class_colors(output_path, kept_objects);
    write_semantic_objects_pcd(output_path / "semantic_objects_all.pcd",
                               kept_objects);
    write_semantic_objects_pcd(output_path / "semantic_objects_stable.pcd",
                               stable_objects);
    write_semantic_objects_overlay(output_path, kept_objects);
}

}  // namespace rtabmap_semantic_mapping
