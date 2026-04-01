cmake_minimum_required(VERSION 3.1.3)
project(vtr_relocalization)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# ─── Find catkin packages ────────────────────────────────────────────────────
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  image_transport
  message_filters
  vtr_msgs
)

# ─── OpenCV (ROS Melodic ships with 3.2 / 3.4) ──────────────────────────────
find_package(OpenCV REQUIRED)

# ─── Eigen3 ─────────────────────────────────────────────────────────────────
find_package(Eigen3 REQUIRED)

# ─── DBoW2  (must be installed separately; see README) ──────────────────────
find_package(DBoW2 REQUIRED)

# ─── catkin_package  ────────────────────────────────────────────────────────
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    vtr_pnp_solver
    vtr_goal_manager
    vtr_relocalization_core
  CATKIN_DEPENDS
    roscpp std_msgs geometry_msgs sensor_msgs nav_msgs
    tf2 tf2_ros tf2_geometry_msgs cv_bridge image_transport
    message_filters vtr_msgs
  DEPENDS
    OpenCV Eigen3 DBoW2
)

# ─── Include dirs ────────────────────────────────────────────────────────────
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${DBoW2_INCLUDE_DIRS}
)

# ─── Library: Gauss-Newton PnP solver ────────────────────────────────────────
add_library(vtr_pnp_solver
  src/pnp_solver.cpp
)
target_link_libraries(vtr_pnp_solver
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

# ─── Library: Goal list manager ──────────────────────────────────────────────
add_library(vtr_goal_manager
  src/goal_manager.cpp
)
target_link_libraries(vtr_goal_manager
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

# ─── Library: Core relocalization logic ──────────────────────────────────────
add_library(vtr_relocalization_core
  src/relocalization.cpp
)
target_link_libraries(vtr_relocalization_core
  vtr_pnp_solver
  vtr_goal_manager
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${DBoW2_LIBRARIES}
)

# ─── Executable: ROS node ────────────────────────────────────────────────────
add_executable(vtr_relocalization_node
  src/relocalization_node.cpp
)
target_link_libraries(vtr_relocalization_node
  vtr_relocalization_core
  vtr_pnp_solver
  vtr_goal_manager
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${DBoW2_LIBRARIES}
)

# ─── Install rules ───────────────────────────────────────────────────────────
install(TARGETS
    vtr_pnp_solver
    vtr_goal_manager
    vtr_relocalization_core
    vtr_relocalization_node
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
