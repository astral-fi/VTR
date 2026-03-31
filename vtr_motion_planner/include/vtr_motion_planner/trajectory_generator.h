#pragma once
// trajectory_generator.h
// ---------------------------------------------------------------------------
// Pre-computes K=21 arc trajectory candidates at node startup.
// Each candidate is a 1-second horizon arc at fixed forward velocity v,
// discretised at dt=0.1 s into 10 waypoints in robot-frame (x forward, y left).
//
// Angular velocities are sampled uniformly in [-0.6, +0.6] rad/s.
// ---------------------------------------------------------------------------

#include <vector>
#include <geometry_msgs/Point.h>

namespace vtr_motion_planner {

/// A single pre-cached arc candidate.
struct TrajectoryArc {
  double omega;                          ///< Angular velocity (rad/s)
  std::vector<geometry_msgs::Point> pts; ///< Waypoints in robot frame
};

class TrajectoryGenerator {
public:
  // -----------------------------------------------------------------------
  // Parameters (all have sensible defaults matching the spec)
  // -----------------------------------------------------------------------
  struct Params {
    int    num_candidates  = 21;    ///< K – number of trajectories
    double omega_min       = -0.6;  ///< rad/s
    double omega_max       =  0.6;  ///< rad/s
    double linear_velocity =  0.3;  ///< m/s (fixed forward speed)
    double horizon_sec     =  1.0;  ///< seconds
    double dt_sec          =  0.1;  ///< discretisation step
  };

  explicit TrajectoryGenerator(const Params& params = Params());

  /// Pre-compute all K arc candidates.  Call once at startup.
  void precompute();

  /// Returns all pre-cached arcs.
  const std::vector<TrajectoryArc>& arcs() const { return arcs_; }

  /// Returns number of candidates K.
  int numCandidates() const { return static_cast<int>(arcs_.size()); }

  const Params& params() const { return params_; }

private:
  Params params_;
  std::vector<TrajectoryArc> arcs_;
};

}  // namespace vtr_motion_planner
