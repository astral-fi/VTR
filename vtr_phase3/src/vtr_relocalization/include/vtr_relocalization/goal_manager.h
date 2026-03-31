#pragma once
// ============================================================================
// vtr_relocalization/goal_manager.h
//
// Goal List Construction & Management (Section 3.3).
//
// On each successful relocalization the manager:
//   1. Computes goal 0 as:  T_g0 = inv(T_k^i) · T_i^{S(i)}
//   2. Propagates M goals along the linked list:
//        T_gm = T_{g(m-1)} · T_{S^m(i)}^{S^{m+1}(i)}
//   3. Extracts the translational parts to form Lg = [t_g0, ..., t_gM].
//
// Between relocalization events:
//   - notifyOdometry() transforms all goals into the new robot frame.
//
// Goal removal:
//   - arrival: ‖robot − t_g0‖ < 0.4 m
//   - angle:   |angle(t_g0)| > 60°  (out of motion range)
// ============================================================================

#include "vtr_relocalization/types.h"
#include <mutex>
#include <ros/ros.h>

namespace vtr {

class GoalManager {
public:
  explicit GoalManager(const Phase3Config& cfg);

  // ── Primary update ───────────────────────────────────────────────────────
  // Rebuild the goal list from the matched keyframe Ki and pose T_k^i.
  // map_keyframes: the full ordered keyframe list (needed for list traversal).
  void updateGoalList(const MapKeyframe& matched_kf,
                      const SE3d& T_live_kf,
                      const std::vector<MapKeyframe::Ptr>& map_keyframes);

  // ── Odometry update ──────────────────────────────────────────────────────
  // Called by Phase 4 at every control cycle with the incremental odometry
  // transform since the last call.  Transforms all stored goals into the
  // updated robot frame.
  void notifyOdometry(const SE3d& T_delta);

  // ── Goal access ──────────────────────────────────────────────────────────
  // Returns a copy of the current goal list (thread-safe).
  GoalList getGoalList() const;

  // Check and apply removal rules (called internally after updates).
  void pruneGoals();

private:
  // Transform a 3D goal position by a SE3 delta.
  static Eigen::Vector3d transformPoint(const SE3d& T, const Eigen::Vector3d& p);

  // Angle between 2D projections of goal and forward axis (degrees).
  static double angleToGoal(const Eigen::Vector3d& t_goal);

  Phase3Config            cfg_;
  GoalList                goal_list_;
  mutable std::mutex      mutex_;
};

} // namespace vtr
