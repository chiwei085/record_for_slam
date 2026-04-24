#include <rclcpp/rclcpp.hpp>

#include "rtabmap_semantic_mapping/semantic_map_visualizer_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node =
        std::make_shared<rtabmap_semantic_mapping::SemanticMapVisualizerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
