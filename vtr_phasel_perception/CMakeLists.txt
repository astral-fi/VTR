cmake_minimum_required(VERSION 3.0.2)
project(vtr_phase1_perception)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  message_generation
)

find_package(OpenCV 3.4 REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# DBoW2
find_library(DBOW2_LIB DBoW2 PATHS /usr/local/lib)
find_path(DBOW2_INCLUDE DBoW2/DBoW2.h PATHS /usr/local/include)

add_message_files(FILES Keyframe.msg)
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vtr_phase1_perception
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs cv_bridge
    image_transport tf2 tf2_ros std_msgs message_runtime
  DEPENDS OpenCV Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${DBOW2_INCLUDE}
)

# Core library
add_library(vtr_phase1_perception
  src/feature_extractor.cpp
  src/stereo_triangulator.cpp
  src/keyframe_selector.cpp
  src/openvins_bridge.cpp
)

target_link_libraries(vtr_phase1_perception
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${DBOW2_LIB}
)

add_dependencies(vtr_phase1_perception
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Main node
add_executable(perception_node src/perception_node.cpp)
target_link_libraries(perception_node
  vtr_phase1_perception
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${DBOW2_LIB}
)
add_dependencies(perception_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

