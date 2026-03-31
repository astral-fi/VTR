#pragma once
// goal_list_manager.h
// ---------------------------------------------------------------------------
// Thread-safe goal list cache for Phase 4.
//
// Phase 3 (Relocalization) pushes goal updates via the /vtr/goal_list topic
// or the GetGoalList service.  Phase 4 reads goals each control cycle via
// getGoals().
//
// Between relocalization events, Phase 4 calls notifyOdometry() so the
// goal manager can transform goal positions into the updated robot frame.
// ---------------------------------------------------------------------------

#include <mutex>
#include <vector>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

namespace vtr_motion_planner {

class GoalListManager {
public:
  // -----------------------------------------------------------------------
  // Parameters
  // -----------------------------------------------------------------------
  struct Params {
    double arrival_threshold_m  = 0.4;   ///< Erase goal when within this distance
    double angle_limit_deg      = 60.0;  ///< Erase goal when bearing exceeds ±this
    int    max_goals            = 5;     ///< M from the spec
  };

  explicit GoalListManager(const Params& params = Params());

  // -----------------------------------------------------------------------
  // Phase 3 → Phase 4 interface
  // -----------------------------------------------------------------------

  /// Replace the entire goal list (called when Phase 3 succeeds relocalization).
  void setGoals(const std::vector<geometry_msgs::Point>& goals);

  // -----------------------------------------------------------------------
  // Phase 4 interface
  // -----------------------------------------------------------------------

  /// Returns current goal list (thread-safe copy).
  std::vector<geometry_msgs::Point> getGoals() const;

  /// Update goal positions using the latest odometry delta.
  /// Call every control cycle between relocalization events.
  void notifyOdometry(const geometry_msgs::Pose& current_pose);

  /// Remove goals that have been reached or are out of bearing range.
  void pruneGoals();

  bool hasGoals() const;

private:
  mutable std::mutex mutex_;
  std::vector<geometry_msgs::Point> goals_;
  geometry_msgs::Pose last_pose_;
  bool has_last_pose_ = false;
  Params params_;
};

}  // namespace vtr_motion_planner
