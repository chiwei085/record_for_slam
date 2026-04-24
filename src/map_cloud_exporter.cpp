#include "rtabmap_semantic_mapping/map_cloud_exporter.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <cmath>
#include <cstdint>
#include <stdexcept>

#include "rtabmap_semantic_mapping/utils.hpp"

namespace rtabmap_semantic_mapping
{

MapCloudExporter::MapCloudExporter(MapCloudConfig config) : config_(config) {}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr MapCloudExporter::reconstruct(
    const std::vector<FrameData>& frames) const {
    auto cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
        new pcl::PointCloud<pcl::PointXYZRGB>());

    for (const auto& frame : frames) {
        if (frame.rgb.empty() || frame.depth.empty()) {
            continue;
        }
        if (!intrinsics_valid(frame.intrinsics) ||
            !is_depth_supported(frame.depth)) {
            continue;
        }

        const int row_limit = std::min(frame.depth.rows, frame.rgb.rows);
        const int col_limit = std::min(frame.depth.cols, frame.rgb.cols);
        for (int row = 0; row < row_limit;
             row += std::max(config_.pixel_step, 1)) {
            for (int col = 0; col < col_limit;
                 col += std::max(config_.pixel_step, 1)) {
                const float depth_m = depth_meters_at(frame.depth, row, col);
                if (!std::isfinite(depth_m) || depth_m < config_.min_depth_m ||
                    depth_m > config_.max_depth_m) {
                    continue;
                }

                const float x_camera =
                    (static_cast<float>(col) - frame.intrinsics.cx) * depth_m /
                    frame.intrinsics.fx;
                const float y_camera =
                    (static_cast<float>(row) - frame.intrinsics.cy) * depth_m /
                    frame.intrinsics.fy;
                const Eigen::Vector3f point_map =
                    frame.T_map_camera *
                    Eigen::Vector3f(x_camera, y_camera, depth_m);

                pcl::PointXYZRGB point;
                point.x = point_map.x();
                point.y = point_map.y();
                point.z = point_map.z();

                if (config_.use_rgb) {
                    const cv::Vec3b bgr = frame.rgb.at<cv::Vec3b>(row, col);
                    point.r = bgr[2];
                    point.g = bgr[1];
                    point.b = bgr[0];
                }
                else {
                    point.r = 255;
                    point.g = 255;
                    point.b = 255;
                }

                cloud->points.push_back(point);
            }
        }
    }

    cloud->width = static_cast<std::uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = false;

    if (cloud->empty() || config_.voxel_leaf_size_m <= 0.0F) {
        return cloud;
    }

    auto filtered = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
        new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(config_.voxel_leaf_size_m, config_.voxel_leaf_size_m,
                           config_.voxel_leaf_size_m);
    voxel_grid.filter(*filtered);
    filtered->width = static_cast<std::uint32_t>(filtered->points.size());
    filtered->height = 1;
    filtered->is_dense = false;
    return filtered;
}

void MapCloudExporter::save_pcd(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const std::string& path) const {
    if (!cloud) {
        throw std::runtime_error("Cannot save a null point cloud.");
    }
    if (pcl::io::savePCDFileBinary(path, *cloud) < 0) {
        throw std::runtime_error("Failed to save map cloud PCD: " + path);
    }
}

bool MapCloudExporter::is_depth_supported(const cv::Mat& depth) const {
    return depth.type() == CV_16UC1 || depth.type() == CV_32FC1;
}

bool MapCloudExporter::intrinsics_valid(
    const CameraIntrinsics& intrinsics) const {
    return intrinsics.fx > 0.0F && intrinsics.fy > 0.0F &&
           intrinsics.width > 0 && intrinsics.height > 0;
}

float MapCloudExporter::depth_meters_at(const cv::Mat& depth, const int row,
                                        const int col) const {
    const auto depth_m = convert_depth_to_meters(depth, row, col);
    return depth_m.value_or(0.0F);
}

}  // namespace rtabmap_semantic_mapping
