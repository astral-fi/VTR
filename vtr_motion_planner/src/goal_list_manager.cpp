// goal_list_manager.cpp
// ---------------------------------------------------------------------------
// Thread-safe cache for the Phase 3 goal list.
//
// Goal positions are maintained in the robot frame at the time of the last
// successful relocalization.  When Phase 4 calls notifyOdometry(), the
// manager transforms goal positions into the updated robot frame using the
// pose delta from the last odometry update.
// ---------------------------------------------------------------------------

#include "vtr_motion_planner/goal_list_manager.h"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace vtr_motion_planner {

GoalListManager::GoalListManager(const Params& params)
  : params_(params)
{}

// ---------------------------------------------------------------------------
void GoalListManager::setGoals(const std::vector<geometry_msgs::Point>& goals)
{
  std::lock_guard<std::mutex> lock(mutex_);
  goals_ = goals;
  // Truncate to max_goals
  if (static_cast<int>(goals_.size()) > params_.max_goals) {
    goals_.resize(params_.max_goals);
  }
}

// ---------------------------------------------------------------------------
std::vector<geometry_msgs::Point> GoalListManager::getGoals() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return goals_;
}

// ---------------------------------------------------------------------------
bool GoalListManager::hasGoals() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return !goals_.empty();
}

// ---------------------------------------------------------------------------
void GoalListManager::notifyOdometry(const geometry_msgs::Pose& current_pose)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!has_last_pose_) {
    last_pose_     = current_pose;
    has_last_pose_ = true;
    return;
  }

  if (goals_.empty()) {
    last_pose_ = current_pose;
    return;
  }

  // -----------------------------------------------------------------------
  // Compute the incremental pose delta T_prev_curr in 2D (x, y, yaw).
  // Goals are stored in the robot frame at the time of the LAST odometry
  // call.  We need to transform them into the current robot frame.
  //
  // T_world_prev * p_prev = T_world_curr * p_curr
  // => p_curr = T_curr_world * T_world_prev * p_prev
  //           = T_curr_prev * p_prev
  // ---------------------------------------------------------------------------

  // Extract 2D positions and yaw from last and current pose
  double x_prev = last_pose_.position.x;
  double y_prev = last_pose_.position.y;

  double x_curr = current_pose.position.x;
  double y_curr = current_pose.position.y;

  // Extract yaw from quaternions
  auto quatToYaw = [](const geometry_msgs::Quaternion& q) -> double {
    // yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  };

  double yaw_prev = quatToYaw(last_pose_.orientation);
  double yaw_curr = quatToYaw(current_pose.orientation);

  // World-frame translation delta
  double dx_world = x_curr - x_prev;
  double dy_world = y_curr - y_prev;

  // Rotate translation into previous robot frame
  double cos_prev =  std::cos(yaw_prev);
  double sin_prev =  std::sin(yaw_prev);
  double dx_prev  =  cos_prev * dx_world + sin_prev * dy_world;
  double dy_prev  = -sin_prev * dx_world + cos_prev * dy_world;

  // Yaw delta (T_curr relative to T_prev, expressed in prev frame)
  double dyaw = yaw_curr - yaw_prev;

  // Transform each goal from prev robot frame → curr robot frame
  double cos_dyaw =  std::cos(dyaw);
  double sin_dyaw =  std::sin(dyaw);

  for (auto& g : goals_) {
    // Subtract the robot's translation (in prev frame)
    double gx = g.x - dx_prev;
    double gy = g.y - dy_prev;

    // Rotate by -dyaw (inverse yaw change)
    g.x =  cos_dyaw * gx + sin_dyaw * gy;
    g.y = -sin_dyaw * gx + cos_dyaw * gy;
  }

  last_pose_ = current_pose;

  // Prune stale goals after odometry update
  // (done here to keep goals fresh; also called explicitly from controller)
  pruneGoals();
}

// ---------------------------------------------------------------------------
void GoalListManager::pruneGoals()
{
  // NOTE: This may be called with mutex already held (from notifyOdometry),
  // so we do NOT re-lock here.  Callers that hold the lock call this directly;
  // external callers must hold the lock before calling.

  // We use a separate public method that acquires its own lock for external use.
  auto it = goals_.begin();
  while (it != goals_.end()) {
    double dist = std::sqrt(it->x * it->x + it->y * it->y);
    double bearing_deg = std::abs(std::atan2(it->y, it->x)) * (180.0 / M_PI);

    bool arrived      = (dist < params_.arrival_threshold_m);
    bool out_of_range = (bearing_deg > params_.angle_limit_deg);

    if (arrived || out_of_range) {
      it = goals_.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace vtr_motion_planner
