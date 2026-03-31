cmake_minimum_required(VERSION 3.0.2)
project(vtr_motion_planner)

## Compile with C++14 (required for Eigen expressions used here)
add_compile_options(-std=c++14 -O2 -Wall)

## Find required catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  laser_geometry
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_generation
)

## Find Eigen3 (ships with ROS Melodic's desktop-full)
find_package(Eigen3 REQUIRED)

## -----------------------------------------------------------------------
## Custom message & service definitions
## -----------------------------------------------------------------------
add_message_files(
  FILES
  GoalList.msg
  TrajectoryCandidate.msg
  MotionPlannerStatus.msg
)

add_service_files(
  FILES
  GetGoalList.srv
  NotifyOdometry.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

## -----------------------------------------------------------------------
## catkin package setup
## -----------------------------------------------------------------------
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vtr_motion_planner_lib
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    laser_geometry
    tf
    tf2
    tf2_ros
    tf2_geometry_msgs
    message_runtime
  DEPENDS EIGEN3
)

## -----------------------------------------------------------------------
## Include directories
## -----------------------------------------------------------------------
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## -----------------------------------------------------------------------
## Core shared library
## -----------------------------------------------------------------------
add_library(vtr_motion_planner_lib
  src/occupancy_grid.cpp
  src/trajectory_generator.cpp
  src/trajectory_scorer.cpp
  src/goal_list_manager.cpp
  src/motion_controller.cpp
)
target_link_libraries(vtr_motion_planner_lib
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
add_dependencies(vtr_motion_planner_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## -----------------------------------------------------------------------
## Main node executable
## -----------------------------------------------------------------------
add_executable(motion_planner_node src/motion_planner_node.cpp)
target_link_libraries(motion_planner_node
  vtr_motion_planner_lib
  ${catkin_LIBRARIES}
)
add_dependencies(motion_planner_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## -----------------------------------------------------------------------
## Install rules
## -----------------------------------------------------------------------
install(TARGETS vtr_motion_planner_lib motion_planner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
