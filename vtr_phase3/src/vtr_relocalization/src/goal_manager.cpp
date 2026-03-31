// ============================================================================
// goal_manager.cpp
//
// Goal List Construction & Management — Section 3.3 of the VTR guide.
//
// Formulae:
//   T_g0 = inv(T_k^i) · T_i^{S(i)}          (goal 0: next cluster node)
//   T_g1 = T_g0 · T_{S(i)}^{S(S(i))}        (goal 1: one hop further)
//   Lg   = [t_g0, t_g1, ..., t_gM]           (translational parts only)
//
// notifyOdometry(T_delta):
//   When the robot advances by T_delta since the last match, each stored
//   goal must be expressed in the NEW robot frame:
//     t_g' = R_delta^T · (t_g - t_delta)
//   This keeps Lg consistent with Phase 4's coordinate expectations.
//
// Pruning rules:
//   - Arrival: erase t_g0 if ‖t_g0‖ < 0.4 m
//   - Angle:   erase t_g0 if |atan2(y,x)| > 60°  (past the robot)
// ============================================================================

#include "vtr_relocalization/goal_manager.h"
#include <ros/ros.h>
#include <cmath>

namespace vtr {

// ─────────────────────────────────────────────────────────────────────────────
GoalManager::GoalManager(const Phase3Config& cfg)
  : cfg_(cfg)
{}

// ─────────────────────────────────────────────────────────────────────────────
void GoalManager::updateGoalList(
    const MapKeyframe& matched_kf,
    const SE3d& T_live_kf,
    const std::vector<MapKeyframe::Ptr>& map_keyframes)
{
  // ── Build the goal transform chain ────────────────────────────────────────
  // T_robot_kf: pose of the matched keyframe in the current robot frame.
  //   = inv(T_live_kf)  (T_live_kf maps kf→live, so its inverse maps live→kf)
  // We want goals expressed in the ROBOT frame, so:
  //   T_g0 = inv(T_k^i) · T_i^{S(i)}

  SE3d T_robot_kf = T_live_kf.inverse();   // inv(T_k^i)

  GoalList new_list;
  new_list.timestamp = ros::Time::now().toSec();

  // Start traversal from the matched keyframe
  const MapKeyframe* current = &matched_kf;
  SE3d cumulative = T_robot_kf;   // cumulative transform from robot frame

  for (int m = 0; m < cfg_.goal_list_length; ++m) {

    // T_i^{S(i)}: transform stored in current keyframe's T_skip field.
    // If skip_idx == -1 there are no more goals (end of map).
    if (current->skip_idx < 0 ||
        current->skip_idx >= static_cast<int>(map_keyframes.size())) {
      ROS_DEBUG("[GoalMgr] Map end reached at goal %d", m);
      break;
    }

    cumulative = cumulative * current->T_skip;
    Eigen::Vector3d t_goal = cumulative.topRightCorner<3, 1>();
    new_list.goals.push_back(t_goal);

    // Advance to next cluster representative
    current = map_keyframes[current->skip_idx].get();
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_list_ = std::move(new_list);
  }

  pruneGoals();

  ROS_INFO("[GoalMgr] Goal list updated: %zu goals",
           goal_list_.goals.size());
}

// ─────────────────────────────────────────────────────────────────────────────
void GoalManager::notifyOdometry(const SE3d& T_delta)
{
  // T_delta: incremental odometry transform since the last call.
  //   T_new_robot_old_robot = T_delta
  //   p_new = R_delta^T · p_old − R_delta^T · t_delta
  //         = R_delta^T · (p_old − t_delta)
  //
  // This matches the convention in the guide: goals are maintained in the
  // *current* robot frame and must be updated whenever the robot moves.

  Eigen::Matrix3d R_delta = T_delta.topLeftCorner<3, 3>();
  Eigen::Vector3d t_delta = T_delta.topRightCorner<3, 1>();

  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& goal : goal_list_.goals) {
    goal = R_delta.transpose() * (goal - t_delta);
  }

  pruneGoals();
}

// ─────────────────────────────────────────────────────────────────────────────
GoalList GoalManager::getGoalList() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return goal_list_;
}

// ─────────────────────────────────────────────────────────────────────────────
void GoalManager::pruneGoals()
{
  // NOTE: caller must NOT hold mutex_ when pruneGoals acquires it internally.
  // Internal callers already hold it (notifyOdometry), so this function
  // is NOT locking here — it is always called from a context that already
  // holds the mutex.  The lock_guard in the public methods ensures safety.

  auto& goals = goal_list_.goals;

  // Continuously remove the front goal while the removal condition holds
  bool keep_pruning = true;
  while (keep_pruning && !goals.empty()) {
    keep_pruning = false;
    const Eigen::Vector3d& g0 = goals.front();

    // ── Arrival check ─────────────────────────────────────────────────────
    double dist = g0.norm();
    if (dist < cfg_.goal_arrival_dist) {
      ROS_INFO("[GoalMgr] Goal arrived (‖t‖=%.3f m), removing.", dist);
      goals.erase(goals.begin());
      keep_pruning = true;
      continue;
    }

    // ── Angle check (2D, x-forward convention) ────────────────────────────
    double angle_deg = std::abs(
        std::atan2(g0.y(), g0.x()) * 180.0 / M_PI);
    if (angle_deg > cfg_.goal_angle_limit) {
      ROS_INFO("[GoalMgr] Goal behind robot (angle=%.1f°), removing.", angle_deg);
      goals.erase(goals.begin());
      keep_pruning = true;
      continue;
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
Eigen::Vector3d GoalManager::transformPoint(const SE3d& T,
                                             const Eigen::Vector3d& p)
{
  Eigen::Vector4d ph;
  ph << p, 1.0;
  return (T * ph).head<3>();
}

// ─────────────────────────────────────────────────────────────────────────────
double GoalManager::angleToGoal(const Eigen::Vector3d& t_goal)
{
  return std::atan2(t_goal.y(), t_goal.x()) * 180.0 / M_PI;
}

} // namespace vtr
