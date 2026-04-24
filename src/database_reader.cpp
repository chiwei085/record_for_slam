#include "rtabmap_semantic_mapping/database_reader.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Transform.h>

#include <filesystem>
#include <list>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace rtabmap_semantic_mapping
{
namespace
{

struct DbDriverDeleter
{
    void operator()(rtabmap::DBDriver* driver) const { delete driver; }
};

struct SignatureDeleter
{
    void operator()(rtabmap::Signature* signature) const { delete signature; }
};

using DbDriverPtr = std::unique_ptr<rtabmap::DBDriver, DbDriverDeleter>;
using SignaturePtr = std::unique_ptr<rtabmap::Signature, SignatureDeleter>;

void validate_database_path(const std::string& path) {
    const std::filesystem::path database_path(path);
    if (path.empty()) {
        throw std::invalid_argument("Database path is empty.");
    }
    if (!std::filesystem::exists(database_path)) {
        throw std::runtime_error("Database path does not exist: " + path);
    }
    if (!std::filesystem::is_regular_file(database_path)) {
        throw std::runtime_error("Database path is not a regular file: " +
                                 path);
    }
}

CameraIntrinsics extract_intrinsics_from_camera_model(
    const rtabmap::CameraModel& model) {
    CameraIntrinsics intrinsics;
    intrinsics.width = model.imageWidth();
    intrinsics.height = model.imageHeight();
    intrinsics.fx = static_cast<float>(model.fx());
    intrinsics.fy = static_cast<float>(model.fy());
    intrinsics.cx = static_cast<float>(model.cx());
    intrinsics.cy = static_cast<float>(model.cy());
    return intrinsics;
}

Eigen::Isometry3f to_isometry3f(const rtabmap::Transform& transform) {
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    matrix(0, 0) = transform.r11();
    matrix(0, 1) = transform.r12();
    matrix(0, 2) = transform.r13();
    matrix(0, 3) = transform.x();
    matrix(1, 0) = transform.r21();
    matrix(1, 1) = transform.r22();
    matrix(1, 2) = transform.r23();
    matrix(1, 3) = transform.y();
    matrix(2, 0) = transform.r31();
    matrix(2, 1) = transform.r32();
    matrix(2, 2) = transform.r33();
    matrix(2, 3) = transform.z();

    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
    pose.matrix() = matrix;
    return pose;
}

bool node_has_required_sensor_data(const rtabmap::SensorData& data) {
    return !data.imageRaw().empty() && !data.depthRaw().empty() &&
           !data.cameraModels().empty();
}

std::optional<FrameData> build_frame_data_from_node(
    int node_id, const rtabmap::Transform& optimized_pose,
    const rtabmap::Signature& node, const rtabmap::SensorData& sensor_data) {
    if (optimized_pose.isNull()) {
        return std::nullopt;
    }

    if (!node_has_required_sensor_data(sensor_data)) {
        return std::nullopt;
    }

    const auto& camera_models = sensor_data.cameraModels();
    const rtabmap::CameraModel& camera_model = camera_models.front();
    if (!camera_model.isValidForProjection()) {
        return std::nullopt;
    }

    FrameData frame;
    frame.node_id = node_id;
    frame.stamp_sec =
        node.getStamp() > 0.0 ? node.getStamp() : sensor_data.stamp();
    frame.rgb = sensor_data.imageRaw().clone();
    frame.depth = sensor_data.depthRaw().clone();
    frame.intrinsics = extract_intrinsics_from_camera_model(camera_model);
    frame.T_map_base = to_isometry3f(optimized_pose);
    frame.T_base_camera = to_isometry3f(camera_model.localTransform());
    frame.T_map_camera = frame.T_map_base * frame.T_base_camera;
    return frame;
}

std::string get_skip_reason(const rtabmap::Transform* optimized_pose,
                            const rtabmap::SensorData& sensor_data) {
    if (optimized_pose == nullptr || optimized_pose->isNull()) {
        return "invalid pose";
    }

    if (sensor_data.imageRaw().empty()) {
        return "missing rgb";
    }
    if (sensor_data.depthRaw().empty()) {
        return "missing depth";
    }
    if (sensor_data.cameraModels().empty()) {
        return "missing camera model";
    }
    if (!sensor_data.cameraModels().front().isValidForProjection()) {
        return "invalid camera model";
    }
    // All known checks passed but build_frame_data_from_node still returned
    // nullopt — indicates a logic gap between the two functions.
    return "build_frame_failed (reason unknown)";
}

SignaturePtr load_node_signature(rtabmap::DBDriver& driver, int node_id) {
    SignaturePtr signature(driver.loadSignature(node_id));
    if (!signature) {
        throw std::runtime_error("Failed to load signature for node " +
                                 std::to_string(node_id));
    }
    driver.loadNodeData(signature.get(), true, false, false, false);
    return signature;
}

LoadedDataset load_dataset(const std::string& database_path, bool keep_frames) {
    validate_database_path(database_path);

    DbDriverPtr driver(rtabmap::DBDriver::create());
    if (!driver) {
        throw std::runtime_error("Failed to create RTAB-Map DB driver.");
    }
    if (!driver->openConnection(database_path, false)) {
        throw std::runtime_error("Failed to open RTAB-Map database: " +
                                 database_path);
    }

    LoadedDataset dataset;

    std::set<int> node_ids;
    driver->getAllNodeIds(node_ids, false, false, false);

    std::map<int, rtabmap::Transform> optimized_poses =
        driver->loadOptimizedPoses();

    dataset.summary.total_nodes = node_ids.size();
    dataset.summary.optimized_poses = optimized_poses.size();

    for (const int node_id : node_ids) {
        SignaturePtr signature = load_node_signature(*driver, node_id);

        rtabmap::SensorData sensor_data = signature->sensorData();
        sensor_data.uncompressData();
        if (!sensor_data.imageRaw().empty()) {
            ++dataset.summary.frames_with_rgb;
        }
        if (!sensor_data.depthRaw().empty()) {
            ++dataset.summary.frames_with_depth;
        }
        if (!sensor_data.cameraModels().empty()) {
            ++dataset.summary.frames_with_camera_model;
        }

        const auto pose_it = optimized_poses.find(node_id);
        const rtabmap::Transform* optimized_pose =
            pose_it == optimized_poses.end() ? nullptr : &pose_it->second;

        if (optimized_pose == nullptr || optimized_pose->isNull()) {
            ++dataset.summary.skipped_frames;
            ++dataset.summary.skip_reason_counts["invalid pose"];
            continue;
        }

        std::optional<FrameData> frame = build_frame_data_from_node(
            node_id, *optimized_pose, *signature, sensor_data);
        if (!frame.has_value()) {
            ++dataset.summary.skipped_frames;
            ++dataset.summary.skip_reason_counts[get_skip_reason(optimized_pose,
                                                                 sensor_data)];
            continue;
        }

        ++dataset.summary.valid_frames;
        if (keep_frames) {
            dataset.frames.push_back(std::move(*frame));
        }
    }

    driver->closeConnection(false);
    return dataset;
}

}  // namespace

DatabaseReader::DatabaseReader(std::string database_path)
    : database_path_(std::move(database_path)) {}

const std::string& DatabaseReader::database_path() const noexcept {
    return database_path_;
}

LoadedDataset DatabaseReader::load() {
    return load_dataset(database_path_, true);
}

std::vector<FrameData> DatabaseReader::load_frames() {
    return load().frames;
}

DatasetSummary DatabaseReader::summarize() {
    return load_dataset(database_path_, false).summary;
}

}  // namespace rtabmap_semantic_mapping
