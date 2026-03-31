#pragma once
// motion_controller.h
// ---------------------------------------------------------------------------
// Top-level control loop (Algorithm 1 from the implementation guide).
// Runs at 10 Hz.  On each tick it:
//   1. Builds the occupancy grid from the latest laser scan.
//   2. Fetches the current goal list.
//   3. If nearest goal < 0.5 m → direct steering.
//   4. Otherwise → score K arc candidates and publish best ω.
//   5. Safety override: if no feasible trajectory → stop + rotate in place.
// ---------------------------------------------------------------------------

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "vtr_motion_planner/occupancy_grid.h"
#include "vtr_motion_planner/trajectory_generator.h"
#include "vtr_motion_planner/trajectory_scorer.h"
#include "vtr_motion_planner/goal_list_manager.h"

// Generated message/service headers
#include <vtr_motion_planner/GoalList.h>
#include <vtr_motion_planner/MotionPlannerStatus.h>

namespace vtr_motion_planner {

class MotionController {
public:
  // -----------------------------------------------------------------------
  // Configuration
  // -----------------------------------------------------------------------
  struct Config {
    // Topics
    std::string scan_topic        = "/scan";
    std::string cmd_vel_topic     = "/cmd_vel";
    std::string odom_topic        = "/odom";
    std::string goal_list_topic   = "/vtr/goal_list";
    std::string status_topic      = "/vtr/motion_planner/status";

    // Control
    double control_rate_hz        = 10.0;
    double close_range_threshold  = 0.5;   ///< m – switch to direct steering
    double safety_rotate_omega    = 0.3;   ///< rad/s – rotate in place when blocked

    // Trajectory generation
    TrajectoryGenerator::Params traj_params;

    // Occupancy grid
    double grid_width_m           = 6.0;
    double grid_resolution_m      = 0.05;
    double inflation_radius_m     = 0.25;

    // Goal list
    GoalListManager::Params goal_params;
  };

  // -----------------------------------------------------------------------
  // Construction & lifecycle
  // -----------------------------------------------------------------------
  explicit MotionController(ros::NodeHandle& nh,
                            ros::NodeHandle& nh_private,
                            const Config& config = Config());

  /// Block and run the ROS spin loop.
  void run();

private:
  // -----------------------------------------------------------------------
  // ROS callbacks
  // -----------------------------------------------------------------------
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void goalListCallback(const vtr_motion_planner::GoalList::ConstPtr& msg);

  // -----------------------------------------------------------------------
  // Main control step (called by timer at 10 Hz)
  // -----------------------------------------------------------------------
  void controlStep(const ros::TimerEvent& event);

  // -----------------------------------------------------------------------
  // Helpers
  // -----------------------------------------------------------------------
  geometry_msgs::Twist buildStop()   const;
  geometry_msgs::Twist buildRotate() const;
  geometry_msgs::Twist buildCmd(double v, double omega) const;

  bool goalReachable(const geometry_msgs::Point& goal,
                     const OccupancyGrid& grid) const;

  // -----------------------------------------------------------------------
  // Members
  // -----------------------------------------------------------------------
  Config config_;

  ros::NodeHandle& nh_;

  ros::Subscriber scan_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber goal_list_sub_;
  ros::Publisher  cmd_vel_pub_;
  ros::Publisher  status_pub_;
  ros::Timer      control_timer_;

  sensor_msgs::LaserScan::ConstPtr latest_scan_;
  std::mutex scan_mutex_;

  TrajectoryGenerator trajectory_gen_;
  TrajectoryScorer    trajectory_scorer_;
  OccupancyGrid       occ_grid_;
  GoalListManager     goal_manager_;
};

}  // namespace vtr_motion_planner
