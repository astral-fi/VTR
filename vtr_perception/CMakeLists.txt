cmake_minimum_required(VERSION 3.10)
project(vtr_perception)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
# Jetson Nano: ARM Cortex-A57 — tune for in-order execution
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=armv8-a -mtune=cortex-a57")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  message_filters
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV 3.4 REQUIRED)

# ORB-SLAM3 — build separately and point to install dir
set(ORB_SLAM3_DIR $ENV{HOME}/ORB_SLAM3)
find_library(ORBSLAM3_LIB ORB_SLAM3 HINTS ${ORB_SLAM3_DIR}/lib)
find_library(DBOW2_LIB    DBoW2     HINTS ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib)
find_library(G2O_LIB      g2o       HINTS ${ORB_SLAM3_DIR}/Thirdparty/g2o/lib)
find_library(SOPHUS_LIB   Sophus    HINTS /usr/local/lib)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vtr_phase1
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${ORB_SLAM3_DIR}
  ${ORB_SLAM3_DIR}/include
  ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty/DBoW2
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus
)

add_library(vtr_phase1
  src/Phase1Node.cpp
)
target_link_libraries(vtr_phase1
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${ORBSLAM3_LIB}
  ${DBOW2_LIB}
  ${G2O_LIB}
  pthread
)

add_executable(phase1_node src/main.cpp)
target_link_libraries(phase1_node vtr_phase1)

install(TARGETS phase1_node vtr_phase1
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})