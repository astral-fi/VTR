// motion_planner_node.cpp
// ---------------------------------------------------------------------------
// ROS node entry point for VTR Phase 4 – Local Motion Planning & Obstacle Avoidance.
// All parameters can be overridden via the ROS parameter server or a launch file.
// ---------------------------------------------------------------------------

#include <ros/ros.h>
#include "vtr_motion_planner/motion_controller.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vtr_motion_planner");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  ROS_INFO("========================================");
  ROS_INFO("  VTR Phase 4: Motion Planner");
  ROS_INFO("  Local Motion Planning & Obstacle Avoidance");
  ROS_INFO("========================================");

  try {
    vtr_motion_planner::MotionController controller(nh, nh_private);
    controller.run();
  } catch (const std::exception& e) {
    ROS_FATAL("[vtr_motion_planner] Fatal exception: %s", e.what());
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
