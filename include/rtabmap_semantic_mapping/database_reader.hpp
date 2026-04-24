#pragma once

#include <string>
#include <vector>

#include "rtabmap_semantic_mapping/types.hpp"

namespace rtabmap_semantic_mapping
{

struct LoadedDataset
{
    std::vector<FrameData> frames;
    DatasetSummary summary;
};

class DatabaseReader
{
public:
    explicit DatabaseReader(std::string database_path);

    [[nodiscard]] const std::string& database_path() const noexcept;

    [[nodiscard]] LoadedDataset load();
    [[nodiscard]] std::vector<FrameData> load_frames();
    [[nodiscard]] DatasetSummary summarize();

private:
    std::string database_path_;
};

}  // namespace rtabmap_semantic_mapping
