cmake_minimum_required(VERSION 3.0.2)
project(vtr_map_manager)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-O3 -Wall)

# ============================================================
# Find ROS Melodic dependencies
# ============================================================
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  image_transport
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_generation
  visualization_msgs
)

# ============================================================
# Find system dependencies
# ============================================================
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization filesystem system)

# DBoW2 - must be installed manually (see README.md)
find_package(DBoW2 QUIET)
if(NOT DBoW2_FOUND)
  # Fallback: assume installed to /usr/local
  set(DBoW2_INCLUDE_DIRS /usr/local/include/DBoW2)
  set(DBoW2_LIBRARIES /usr/local/lib/libDBoW2.so)
  message(WARNING "DBoW2 not found via cmake -- using fallback paths")
endif()

# ============================================================
# Generate custom messages
# ============================================================
add_message_files(
  FILES
  Keyframe.msg
  MapNode.msg
  GoalList.msg
  LoopDetectionResult.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

# ============================================================
# catkin package declaration
# ============================================================
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vtr_map_graph vtr_dbow2_wrapper vtr_keyframe_clusterer vtr_map_expansion
  CATKIN_DEPENDS
    roscpp std_msgs sensor_msgs geometry_msgs nav_msgs
    cv_bridge image_transport tf tf2 tf2_ros
    message_runtime visualization_msgs
  DEPENDS OpenCV Eigen3 Boost
)

# ============================================================
# Include directories
# ============================================================
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${DBoW2_INCLUDE_DIRS}
)

# ============================================================
# Library: vtr_map_graph
#   Core topo-metric graph data structure
# ============================================================
add_library(vtr_map_graph
  src/map_graph.cpp
  src/keyframe_node.cpp
)
target_link_libraries(vtr_map_graph
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${Boost_LIBRARIES}
)
add_dependencies(vtr_map_graph ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ============================================================
# Library: vtr_dbow2_wrapper
#   DBoW2 vocabulary + loop detection
# ============================================================
add_library(vtr_dbow2_wrapper
  src/dbow2_wrapper.cpp
)
target_link_libraries(vtr_dbow2_wrapper
  vtr_map_graph
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${DBoW2_LIBRARIES}
)
add_dependencies(vtr_dbow2_wrapper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ============================================================
# Library: vtr_keyframe_clusterer
#   Offline map redundancy reduction
# ============================================================
add_library(vtr_keyframe_clusterer
  src/keyframe_clusterer.cpp
)
target_link_libraries(vtr_keyframe_clusterer
  vtr_map_graph
  vtr_dbow2_wrapper
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(vtr_keyframe_clusterer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ============================================================
# Library: vtr_map_expansion
#   Live map expansion during repeating phase
# ============================================================
add_library(vtr_map_expansion
  src/map_expansion.cpp
)
target_link_libraries(vtr_map_expansion
  vtr_map_graph
  vtr_dbow2_wrapper
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(vtr_map_expansion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ============================================================
# Node: map_manager_node
#   Main ROS node orchestrating Phase 2
# ============================================================
add_executable(map_manager_node
  src/map_manager_node.cpp
)
target_link_libraries(map_manager_node
  vtr_map_graph
  vtr_dbow2_wrapper
  vtr_keyframe_clusterer
  vtr_map_expansion
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)
add_dependencies(map_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ============================================================
# Node: map_save_load_node
#   Persist / reload the reduced map to disk
# ============================================================
add_executable(map_save_load_node
  src/map_save_load_node.cpp
)
target_link_libraries(map_save_load_node
  vtr_map_graph
  vtr_dbow2_wrapper
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)
add_dependencies(map_save_load_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ============================================================
# Node: map_visualizer_node
#   Publishes RViz markers of the topo-metric graph
# ============================================================
add_executable(map_visualizer_node
  src/map_visualizer_node.cpp
)
target_link_libraries(map_visualizer_node
  vtr_map_graph
  ${catkin_LIBRARIES}
)
add_dependencies(map_visualizer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ============================================================
# Install
# ============================================================
install(TARGETS
  vtr_map_graph vtr_dbow2_wrapper vtr_keyframe_clusterer vtr_map_expansion
  map_manager_node map_save_load_node map_visualizer_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
