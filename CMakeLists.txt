cmake_minimum_required(VERSION 3.8)
project(record_for_slam)

# Export compile_commands.json to the build directory (used by clangd / IDEs).
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

# ---------------------------------------------------------------------------
# record_gate node
# ---------------------------------------------------------------------------

add_executable(record_gate src/record_gate_node.cpp)
target_compile_features(record_gate PUBLIC cxx_std_17)
ament_target_dependencies(
    record_gate
    rclcpp
    rosbag2_cpp
    rosbag2_storage
    rosbag2_transport
    sensor_msgs
    tf2_msgs
)

install(TARGETS record_gate
    DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Launch files and config
# ---------------------------------------------------------------------------

install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
