// motion_controller.cpp
// ---------------------------------------------------------------------------
// Full implementation of Algorithm 1 (VTR Phase 4).
// ---------------------------------------------------------------------------

#include "vtr_motion_planner/motion_controller.h"
#include <cmath>
#include <mutex>

namespace vtr_motion_planner {

// ---------------------------------------------------------------------------
MotionController::MotionController(ros::NodeHandle& nh,
                                   ros::NodeHandle& nh_private,
                                   const Config& config)
  : config_(config)
  , nh_(nh)
  , trajectory_gen_(config.traj_params)
  , trajectory_scorer_(3)  // score against first 3 goals
  , occ_grid_(config.grid_width_m,
              config.grid_resolution_m,
              config.inflation_radius_m)
  , goal_manager_(config.goal_params)
{
  // -----------------------------------------------------------------------
  // Override config from ROS parameter server if present
  // -----------------------------------------------------------------------
  nh_private.param("scan_topic",          config_.scan_topic,       config_.scan_topic);
  nh_private.param("cmd_vel_topic",       config_.cmd_vel_topic,    config_.cmd_vel_topic);
  nh_private.param("odom_topic",          config_.odom_topic,       config_.odom_topic);
  nh_private.param("goal_list_topic",     config_.goal_list_topic,  config_.goal_list_topic);
  nh_private.param("control_rate_hz",     config_.control_rate_hz,  config_.control_rate_hz);
  nh_private.param("close_range_threshold",
                   config_.close_range_threshold, config_.close_range_threshold);
  nh_private.param("safety_rotate_omega",
                   config_.safety_rotate_omega,   config_.safety_rotate_omega);
  nh_private.param("grid_width_m",        config_.grid_width_m,     config_.grid_width_m);
  nh_private.param("grid_resolution_m",   config_.grid_resolution_m,config_.grid_resolution_m);
  nh_private.param("inflation_radius_m",  config_.inflation_radius_m,
                                          config_.inflation_radius_m);
  nh_private.param("num_trajectory_candidates",
                   config_.traj_params.num_candidates, config_.traj_params.num_candidates);
  nh_private.param("forward_velocity",
                   config_.traj_params.linear_velocity, config_.traj_params.linear_velocity);

  // -----------------------------------------------------------------------
  // Pre-compute trajectory library
  // -----------------------------------------------------------------------
  trajectory_gen_.precompute();
  ROS_INFO("[MotionController] Pre-computed %d trajectory candidates.",
           trajectory_gen_.numCandidates());

  // -----------------------------------------------------------------------
  // ROS pub/sub setup
  // -----------------------------------------------------------------------
  scan_sub_ = nh_.subscribe(config_.scan_topic, 1,
                            &MotionController::scanCallback, this);

  odom_sub_ = nh_.subscribe(config_.odom_topic, 10,
                            &MotionController::odomCallback, this);

  goal_list_sub_ = nh_.subscribe(config_.goal_list_topic, 1,
                                 &MotionController::goalListCallback, this);

  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(config_.cmd_vel_topic, 1);

  status_pub_ = nh_.advertise<vtr_motion_planner::MotionPlannerStatus>(
                  config_.status_topic, 1);

  // -----------------------------------------------------------------------
  // Control timer
  // -----------------------------------------------------------------------
  control_timer_ = nh_.createTimer(
      ros::Duration(1.0 / config_.control_rate_hz),
      &MotionController::controlStep, this);

  ROS_INFO("[MotionController] Initialised. Listening on scan: %s",
           config_.scan_topic.c_str());
}

// ---------------------------------------------------------------------------
void MotionController::run()
{
  ros::spin();
}

// ---------------------------------------------------------------------------
// ROS Callbacks
// ---------------------------------------------------------------------------

void MotionController::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(scan_mutex_);
  latest_scan_ = msg;
}

void MotionController::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  goal_manager_.notifyOdometry(msg->pose.pose);
}

void MotionController::goalListCallback(
    const vtr_motion_planner::GoalList::ConstPtr& msg)
{
  std::vector<geometry_msgs::Point> goals(msg->goals.begin(), msg->goals.end());
  goal_manager_.setGoals(goals);
  ROS_DEBUG("[MotionController] Received %zu goals from Phase 3.", goals.size());
}

// ---------------------------------------------------------------------------
// Main control step — Algorithm 1
// ---------------------------------------------------------------------------
void MotionController::controlStep(const ros::TimerEvent& /*event*/)
{
  // -----------------------------------------------------------------------
  // 1.  Get latest scan (thread-safe copy)
  // -----------------------------------------------------------------------
  sensor_msgs::LaserScan::ConstPtr scan;
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    scan = latest_scan_;
  }

  if (!scan) {
    ROS_WARN_THROTTLE(5.0, "[MotionController] No laser scan received yet. Waiting...");
    cmd_vel_pub_.publish(buildStop());
    return;
  }

  // -----------------------------------------------------------------------
  // 2.  Build occupancy grid (Mo) and inflate (M)
  // -----------------------------------------------------------------------
  occ_grid_.buildFromScan(*scan);

  // -----------------------------------------------------------------------
  // 3.  Get goal list from Phase 3 (via GoalListManager)
  // -----------------------------------------------------------------------
  auto goals = goal_manager_.getGoals();

  // -----------------------------------------------------------------------
  // 4.  Prepare status message
  // -----------------------------------------------------------------------
  vtr_motion_planner::MotionPlannerStatus status;
  status.header.stamp    = ros::Time::now();
  status.header.frame_id = "base_link";
  status.active_goal_count = static_cast<int>(goals.size());

  // -----------------------------------------------------------------------
  // 5.  No goals → safety stop
  // -----------------------------------------------------------------------
  if (goals.empty()) {
    ROS_WARN_THROTTLE(2.0, "[MotionController] No goals available. Stopping.");
    cmd_vel_pub_.publish(buildStop());
    status.mode       = vtr_motion_planner::MotionPlannerStatus::MODE_STOPPED;
    status.cmd_linear = 0.0;
    status.cmd_angular = 0.0;
    status_pub_.publish(status);
    return;
  }

  // -----------------------------------------------------------------------
  // 6.  Distance to nearest goal
  // -----------------------------------------------------------------------
  const auto& g0 = goals[0];
  double dp = std::sqrt(g0.x * g0.x + g0.y * g0.y);
  status.nearest_goal_distance = dp;

  geometry_msgs::Twist cmd;

  if (dp < config_.close_range_threshold) {
    // -------------------------------------------------------------------
    // Close-range direct control (Section 4.4)
    // -------------------------------------------------------------------
    if (goalReachable(g0, occ_grid_)) {
      // Slow down as approaching; steer directly toward goal
      double v     = dp;  // velocity proportional to distance
      double omega = std::atan2(g0.y, g0.x);
      cmd = buildCmd(v, omega);
      status.mode = vtr_motion_planner::MotionPlannerStatus::MODE_DIRECT;
      ROS_DEBUG("[MotionController] Direct steering: v=%.3f ω=%.3f", v, omega);
    } else {
      // Goal blocked – wait
      cmd = buildStop();
      status.mode = vtr_motion_planner::MotionPlannerStatus::MODE_STOPPED;
      ROS_DEBUG("[MotionController] Close goal blocked. Waiting.");
    }
  } else {
    // -------------------------------------------------------------------
    // Trajectory selection (Section 4.3 / Algorithm 1, lines 10-19)
    // -------------------------------------------------------------------
    const auto& arcs = trajectory_gen_.arcs();
    ScoringResult result = trajectory_scorer_.evaluate(arcs, occ_grid_, goals);

    int feasible_count = 0;
    for (bool f : result.feasible) if (f) feasible_count++;
    status.feasible_trajectory_count = feasible_count;

    if (result.best_index >= 0) {
      double omega = arcs[result.best_index].omega;
      double v     = config_.traj_params.linear_velocity;
      cmd = buildCmd(v, omega);
      status.mode                     = vtr_motion_planner::MotionPlannerStatus::MODE_TRAJECTORY;
      status.selected_trajectory_index = result.best_index;
      ROS_DEBUG("[MotionController] Traj %d selected: ω=%.3f score=%.4f",
                result.best_index, omega, result.best_score);
    } else {
      // Safety override: all trajectories blocked → rotate in place
      cmd = buildRotate();
      status.mode = vtr_motion_planner::MotionPlannerStatus::MODE_STOPPED;
      ROS_WARN_THROTTLE(1.0,
          "[MotionController] All %d trajectories blocked! Rotating in place.",
          static_cast<int>(arcs.size()));
    }
  }

  // -----------------------------------------------------------------------
  // 7.  Publish command and status
  // -----------------------------------------------------------------------
  status.cmd_linear  = cmd.linear.x;
  status.cmd_angular = cmd.angular.z;

  cmd_vel_pub_.publish(cmd);
  status_pub_.publish(status);
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

geometry_msgs::Twist MotionController::buildStop() const
{
  geometry_msgs::Twist cmd;
  cmd.linear.x  = 0.0;
  cmd.angular.z = 0.0;
  return cmd;
}

geometry_msgs::Twist MotionController::buildRotate() const
{
  geometry_msgs::Twist cmd;
  cmd.linear.x  = 0.0;
  cmd.angular.z = config_.safety_rotate_omega;
  return cmd;
}

geometry_msgs::Twist MotionController::buildCmd(double v, double omega) const
{
  geometry_msgs::Twist cmd;
  cmd.linear.x  = v;
  cmd.angular.z = omega;
  return cmd;
}

bool MotionController::goalReachable(const geometry_msgs::Point& goal,
                                     const OccupancyGrid& grid) const
{
  // Simple check: sample 5 points along the straight line to the goal
  // and verify none are in an occupied cell.
  const int samples = 5;
  for (int i = 1; i <= samples; ++i) {
    double t  = static_cast<double>(i) / static_cast<double>(samples);
    double px = goal.x * t;
    double py = goal.y * t;
    if (grid.isOccupied(px, py)) {
      return false;
    }
  }
  return true;
}

}  // namespace vtr_motion_planner
