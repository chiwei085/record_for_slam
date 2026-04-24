cmake_minimum_required(VERSION 3.8)
project(record_for_slam)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(RTABMap REQUIRED COMPONENTS core)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

option(RECORD_FOR_SLAM_ENABLE_YOLO
  "Enable YOLO inference support when an external ONNX Runtime CMake package is available."
  ON)

if(RECORD_FOR_SLAM_ENABLE_YOLO)
  find_package(onnxruntime CONFIG QUIET)
endif()

find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

if(RECORD_FOR_SLAM_ENABLE_YOLO AND onnxruntime_FOUND)
  # Reuse an externally provided ONNX Runtime config target. We do not rely on
  # YOLO-CPP's vcpkg presets or vendor installation flow from this workspace.
  set(YOLO_CPP_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(YOLO_CPP_BUILD_OPENCV_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(YOLO_CPP_BUILD_DIAGNOSTICS OFF CACHE BOOL "" FORCE)
  set(YOLO_CPP_BUILD_INTEGRATION_TESTS OFF CACHE BOOL "" FORCE)
  set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
  add_subdirectory(include/third_party/YOLO-CPP EXCLUDE_FROM_ALL)
endif()

add_library(semantic_mapping_lib
  src/database_visualization_utils.cpp
  src/database_reader.cpp
  src/map_cloud_exporter.cpp
  src/observation_fusion.cpp
  src/observation_projector.cpp
  src/offline_semantic_mapper.cpp
  src/result_exporter_dataset.cpp
  src/result_exporter_detection.cpp
  src/result_exporter_observation.cpp
  src/result_exporter_fusion.cpp
)

target_compile_features(semantic_mapping_lib PUBLIC cxx_std_17)
target_include_directories(semantic_mapping_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_include_directories(semantic_mapping_lib SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(semantic_mapping_lib
  Eigen3::Eigen
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  rtabmap::core
  yaml-cpp
)
ament_target_dependencies(
  semantic_mapping_lib
  builtin_interfaces
  geometry_msgs
  std_msgs
  visualization_msgs
)

if(RECORD_FOR_SLAM_ENABLE_YOLO AND onnxruntime_FOUND)
  target_compile_features(semantic_mapping_lib PUBLIC cxx_std_20)
  target_compile_definitions(semantic_mapping_lib PRIVATE RTABMAP_SEMANTIC_MAPPING_HAS_YOLO_CPP=1)
  target_link_libraries(semantic_mapping_lib yolo_cpp)
elseif(RECORD_FOR_SLAM_ENABLE_YOLO)
  message(WARNING
    "RECORD_FOR_SLAM_ENABLE_YOLO=ON but onnxruntime was not found. "
    "YOLO inference will be unavailable. "
    "Install an ONNX Runtime release and pass "
    "-Donnxruntime_DIR=<onnxruntime-root>/lib/cmake/onnxruntime, "
    "or set RECORD_FOR_SLAM_ENABLE_YOLO=OFF to suppress this warning.")
else()
  message(STATUS "RECORD_FOR_SLAM_ENABLE_YOLO=OFF. Building without YOLO inference support.")
endif()

function(add_slam_executable name)
  cmake_parse_arguments(SLAM_EXECUTABLE "" "" "SOURCES;LIBS;DEPS;INCLUDE_DIRS" ${ARGN})
  add_executable(${name} ${SLAM_EXECUTABLE_SOURCES})
  target_compile_features(${name} PUBLIC cxx_std_17)
  if(SLAM_EXECUTABLE_INCLUDE_DIRS)
    target_include_directories(${name} PRIVATE ${SLAM_EXECUTABLE_INCLUDE_DIRS})
  endif()
  if(SLAM_EXECUTABLE_LIBS)
    target_link_libraries(${name} ${SLAM_EXECUTABLE_LIBS})
  endif()
  if(SLAM_EXECUTABLE_DEPS)
    ament_target_dependencies(${name} ${SLAM_EXECUTABLE_DEPS})
  endif()
endfunction()

add_slam_executable(
  record_gate
  SOURCES src/record_gate_node.cpp
  LIBS yaml-cpp
  DEPS
    rclcpp
    rosbag2_cpp
    rosbag2_storage
    rosbag2_transport
    sensor_msgs
    tf2_msgs
)

add_slam_executable(
  offline_semantic_mapper_main
  SOURCES src/offline_semantic_mapper_main.cpp
  LIBS semantic_mapping_lib
)

add_slam_executable(
  semantic_map_visualizer_node
  SOURCES
    src/semantic_map_visualizer_node.cpp
    src/semantic_map_visualizer_main.cpp
  LIBS semantic_mapping_lib ${PCL_LIBRARIES}
  DEPS
    rclcpp
    sensor_msgs
    visualization_msgs
    pcl_conversions
  INCLUDE_DIRS ${PCL_INCLUDE_DIRS}
)

add_slam_executable(
  frame_playback_node
  SOURCES
    src/frame_playback_node.cpp
    src/frame_playback_node_main.cpp
  LIBS semantic_mapping_lib ${PCL_LIBRARIES}
  DEPS
    nav_msgs
    rclcpp
    sensor_msgs
    visualization_msgs
    pcl_conversions
  INCLUDE_DIRS ${PCL_INCLUDE_DIRS}
)

install(
  TARGETS
    semantic_mapping_lib
    record_gate
    offline_semantic_mapper_main
    semantic_map_visualizer_node
    frame_playback_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config docs scripts DESTINATION share/${PROJECT_NAME})
set(RECORD_FOR_SLAM_DOC_FILES README.md)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/pipeline.md")
  list(APPEND RECORD_FOR_SLAM_DOC_FILES pipeline.md)
endif()

install(FILES ${RECORD_FOR_SLAM_DOC_FILES} DESTINATION share/${PROJECT_NAME})

ament_package()
