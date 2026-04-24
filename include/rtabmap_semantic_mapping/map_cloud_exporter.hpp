#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <vector>

#include "rtabmap_semantic_mapping/types.hpp"

namespace rtabmap_semantic_mapping
{

class MapCloudExporter
{
public:
    explicit MapCloudExporter(MapCloudConfig config = {});

    [[nodiscard]] pcl::PointCloud<pcl::PointXYZRGB>::Ptr reconstruct(
        const std::vector<FrameData>& frames) const;

    void save_pcd(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                  const std::string& path) const;

private:
    [[nodiscard]] bool is_depth_supported(const cv::Mat& depth) const;
    [[nodiscard]] bool intrinsics_valid(
        const CameraIntrinsics& intrinsics) const;
    [[nodiscard]] float depth_meters_at(const cv::Mat& depth, int row,
                                        int col) const;

    MapCloudConfig config_;
};

}  // namespace rtabmap_semantic_mapping
