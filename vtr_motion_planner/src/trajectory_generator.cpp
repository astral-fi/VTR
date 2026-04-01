// trajectory_generator.cpp
// ---------------------------------------------------------------------------
// Pre-computes K arc candidates.  Each arc is a unicycle model integration
// from the robot frame origin over a 1-second horizon at dt=0.1 s.
//
// Unicycle kinematics:
//   x_dot = v * cos(theta)
//   y_dot = v * sin(theta)
//   theta_dot = omega
// ---------------------------------------------------------------------------

#include "vtr_motion_planner/trajectory_generator.h"
#include <cmath>
#include <stdexcept>

namespace vtr_motion_planner {

TrajectoryGenerator::TrajectoryGenerator(const Params& params)
  : params_(params)
{}

void TrajectoryGenerator::precompute()
{
  if (params_.num_candidates < 1) {
    throw std::invalid_argument("TrajectoryGenerator: num_candidates must be >= 1");
  }

  arcs_.clear();
  arcs_.reserve(params_.num_candidates);

  const int    K      = params_.num_candidates;
  const double v      = params_.linear_velocity;
  const double dt     = params_.dt_sec;
  const double T      = params_.horizon_sec;
  const int    steps  = static_cast<int>(std::round(T / dt));

  const double L = 0.165;  // JetRacer wheelbase in meters (Ackermann kinematic model)

  // Uniformly sample K steering angles in approx limits [-0.35, +0.35] rad
  double delta_min = -0.35;
  double delta_max =  0.35;

  for (int k = 0; k < K; ++k) {
    double delta;
    if (K == 1) {
      delta = 0.0;
    } else {
      delta = delta_min + 
              static_cast<double>(k) / static_cast<double>(K - 1) *
              (delta_max - delta_min);
    }

    TrajectoryArc arc;
    arc.steering_angle = delta;
    arc.pts.reserve(steps);

    // Integrate Ackermann from (0,0,0) in robot frame
    double x = 0.0, y = 0.0, theta = 0.0;
    for (int s = 0; s < steps; ++s) {
      double dtheta = (v / L) * std::tan(delta) * dt;
      x     += v * std::cos(theta) * dt;
      y     += v * std::sin(theta) * dt;
      theta += dtheta;

      geometry_msgs::Point pt;
      pt.x = x;
      pt.y = y;
      pt.z = 0.0;
      arc.pts.push_back(pt);
    }

    arcs_.push_back(arc);
  }
}

}  // namespace vtr_motion_planner
