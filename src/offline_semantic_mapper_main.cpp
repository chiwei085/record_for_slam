#include <iostream>
#include <stdexcept>
#include <string>

#include "rtabmap_semantic_mapping/config_loader.hpp"
#include "rtabmap_semantic_mapping/offline_semantic_mapper.hpp"
#include "third_party/argparse/argparse.hpp"

int main(int argc, char** argv) {
    argparse::ArgumentParser program("offline_semantic_mapper_main", "0.1.0",
                                     argparse::default_arguments::help);
    program.add_description(
        "Offline semantic mapping pipeline: reads an RTAB-Map database, runs "
        "YOLO detection, projects observations to 3D, and fuses them into "
        "semantic objects.");

    program.add_argument("--config")
        .metavar("FILE")
        .help(
            "Path to a YAML configuration file. All fields are optional and "
            "overridden by explicit CLI arguments.");

    program.add_argument("--database")
        .metavar("FILE")
        .help("Path to the RTAB-Map database (.db). Overrides config file.");

    program.add_argument("--model").metavar("FILE").help(
        "Path to the YOLO ONNX model. Overrides config file.");

    program.add_argument("--output")
        .metavar("DIR")
        .help("Output directory for debug files. Overrides config file.");

    try {
        program.parse_args(argc, argv);
    }
    catch (const std::exception& err) {
        std::cerr << err.what() << "\n\n" << program;
        return 1;
    }

    try {
        rtabmap_semantic_mapping::OfflineSemanticMapperConfig config;

        // Apply YAML config first (provides defaults for all sub-configs).
        if (const auto yaml_path = program.present("--config")) {
            rtabmap_semantic_mapping::load_config_from_yaml(*yaml_path, config);
        }

        // CLI arguments override YAML values.
        if (const auto v = program.present("--database")) {
            config.database_path = *v;
        }
        if (const auto v = program.present("--model")) {
            config.model_path = *v;
        }
        if (const auto v = program.present("--output")) {
            config.output_dir = *v;
        }

        // Validate required fields.
        if (config.database_path.empty()) {
            throw std::invalid_argument(
                "--database is required (or set 'database_path' in config).");
        }
        if (config.model_path.empty()) {
            throw std::invalid_argument(
                "--model is required (or set 'model_path' in config).");
        }
        if (config.output_dir.empty()) {
            throw std::invalid_argument(
                "--output is required (or set 'output_dir' in config).");
        }

        rtabmap_semantic_mapping::OfflineSemanticMapper mapper(config);
        const auto result = mapper.run();

        std::cout
            << "database: " << config.database_path << '\n'
            << "model: " << config.model_path << '\n'
            << "output: " << config.output_dir << '\n'
            << "total nodes: " << result.dataset_summary.total_nodes << '\n'
            << "optimized poses: " << result.dataset_summary.optimized_poses
            << '\n'
            << "valid frames: " << result.dataset_summary.valid_frames << '\n'
            << "skipped frames: " << result.dataset_summary.skipped_frames
            << '\n'
            << "frames with rgb: " << result.dataset_summary.frames_with_rgb
            << '\n'
            << "frames with depth: " << result.dataset_summary.frames_with_depth
            << '\n'
            << "frames with camera model: "
            << result.dataset_summary.frames_with_camera_model << '\n'
            << "frames with detections: "
            << result.detection_summary.frames_with_detections << '\n'
            << "total detections: " << result.detection_summary.total_detections
            << '\n'
            << "successful observations: "
            << result.projection_summary.successful_observations << '\n'
            << "skipped low confidence: "
            << result.projection_summary.skipped_low_confidence << '\n'
            << "skipped invalid bbox: "
            << result.projection_summary.skipped_invalid_bbox << '\n'
            << "skipped no valid depth: "
            << result.projection_summary.skipped_no_valid_depth << '\n'
            << "skipped invalid intrinsics: "
            << result.projection_summary.skipped_invalid_intrinsics << '\n'
            << "fusion accepted observations: "
            << result.fusion_summary.accepted_observations << '\n'
            << "fusion created objects: "
            << result.fusion_summary.created_objects << '\n'
            << "fusion updated objects: "
            << result.fusion_summary.updated_objects << '\n'
            << "fusion rejected observations: "
            << result.fusion_summary.rejected_observations << '\n'
            << "final semantic objects: " << result.semantic_objects.size()
            << '\n';

        if (!result.dataset_summary.skip_reason_counts.empty()) {
            std::cout << "skip reasons:\n";
            for (const auto& [reason, count] :
                 result.dataset_summary.skip_reason_counts) {
                std::cout << "  " << reason << ": " << count << '\n';
            }
        }

        if (!result.detection_summary.detections_per_class.empty()) {
            std::cout << "detections per class:\n";
            for (const auto& [class_name, count] :
                 result.detection_summary.detections_per_class) {
                std::cout << "  " << class_name << ": " << count << '\n';
            }
        }

        if (!result.frames.empty()) {
            const auto& first_frame = result.frames.front();
            std::cout << "first valid frame node_id: " << first_frame.node_id
                      << '\n'
                      << "first rgb size: " << first_frame.rgb.cols << "x"
                      << first_frame.rgb.rows << '\n'
                      << "first depth size: " << first_frame.depth.cols << "x"
                      << first_frame.depth.rows << '\n'
                      << "first optimized pose translation: "
                      << first_frame.T_map_base.translation().x() << ", "
                      << first_frame.T_map_base.translation().y() << ", "
                      << first_frame.T_map_base.translation().z() << '\n'
                      << "first base->camera translation: "
                      << first_frame.T_base_camera.translation().x() << ", "
                      << first_frame.T_base_camera.translation().y() << ", "
                      << first_frame.T_base_camera.translation().z() << '\n'
                      << "first map->camera translation: "
                      << first_frame.T_map_camera.translation().x() << ", "
                      << first_frame.T_map_camera.translation().y() << ", "
                      << first_frame.T_map_camera.translation().z() << '\n';
        }

        if (!result.frame_detections.empty()) {
            std::cout << "first frame detections: "
                      << result.frame_detections.front().detections.size()
                      << '\n';
        }

        if (!result.frame_observations.empty()) {
            std::cout << "first frame observations: "
                      << result.frame_observations.front().size() << '\n';
        }

        if (!result.semantic_objects.empty()) {
            const auto& first_object = result.semantic_objects.front();
            std::cout << "first semantic object: " << first_object.class_name
                      << " id=" << first_object.object_id
                      << " seen_count=" << first_object.seen_count << '\n';
        }

        return 0;
    }
    catch (const std::exception& error) {
        std::cerr << "offline_semantic_mapper_main failed: " << error.what()
                  << '\n';
        return 1;
    }
}
