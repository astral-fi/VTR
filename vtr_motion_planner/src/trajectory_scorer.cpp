// trajectory_scorer.cpp
// ---------------------------------------------------------------------------
// Implements Algorithm 1 scoring (Section 4.3):
//
//   s_i = (1/3) * sum_{m=0}^{2} ( 1 - (0.005 * theta_deg(endpoint_i, goal_m))^0.5 )
//
// Feasibility:  any waypoint in an inflated obstacle → entire arc rejected.
// ---------------------------------------------------------------------------

#include "vtr_motion_planner/trajectory_scorer.h"
#include <cmath>
#include <limits>

namespace vtr_motion_planner {

TrajectoryScorer::TrajectoryScorer(int goals_to_use)
  : goals_to_use_(goals_to_use)
{}

// ---------------------------------------------------------------------------
ScoringResult TrajectoryScorer::evaluate(
    const std::vector<TrajectoryArc>&        arcs,
    const OccupancyGrid&                     grid,
    const std::vector<geometry_msgs::Point>& goals) const
{
  const int K = static_cast<int>(arcs.size());
  ScoringResult result;
  result.scores.assign(K, 0.0);
  result.feasible.assign(K, false);

  // How many goals are actually available
  const int M = std::min(goals_to_use_, static_cast<int>(goals.size()));

  for (int i = 0; i < K; ++i) {
    const auto& arc = arcs[i];

    // -----------------------------------------------------------------------
    // Step 1: Feasibility check – reject if any waypoint is in obstacle
    // -----------------------------------------------------------------------
    bool feasible = true;
    for (const auto& pt : arc.pts) {
      if (grid.isOccupied(pt.x, pt.y)) {
        feasible = false;
        break;
      }
    }

    if (!feasible) {
      result.feasible[i] = false;
      result.scores[i]   = 0.0;
      continue;
    }

    result.feasible[i] = true;

    // -----------------------------------------------------------------------
    // Step 2: Score against first M goals
    // -----------------------------------------------------------------------
    if (M == 0) {
      // No goals available – all feasible arcs score zero (forward safe motion)
      result.scores[i] = 0.0;
      continue;
    }

    // Terminal point of this arc
    const geometry_msgs::Point& endpoint = arc.pts.back();

    double total_score = 0.0;
    for (int m = 0; m < M; ++m) {
      double theta_deg = angleDegrees(endpoint, goals[m]);
      // Clamp to prevent negative scores when theta > 200 degrees (degenerate)
      double term = 1.0 - std::sqrt(0.005 * theta_deg);
      total_score += std::max(0.0, term);
    }
    result.scores[i] = total_score / static_cast<double>(goals_to_use_);
  }

  // -----------------------------------------------------------------------
  // Find best feasible trajectory
  // -----------------------------------------------------------------------
  double best = -std::numeric_limits<double>::infinity();
  for (int i = 0; i < K; ++i) {
    if (result.feasible[i] && result.scores[i] > best) {
      best = result.scores[i];
      result.best_index = i;
      result.best_score = best;
    }
  }

  return result;
}

// ---------------------------------------------------------------------------
double TrajectoryScorer::angleDegrees(const geometry_msgs::Point& endpoint,
                                      const geometry_msgs::Point& goal)
{
  // Both expressed in the robot frame.  The robot is at origin.
  // Compute angle between the two direction vectors from the origin.
  double ax = endpoint.x, ay = endpoint.y;
  double bx = goal.x,     by = goal.y;

  double dot   = ax * bx + ay * by;
  double mag_a = std::sqrt(ax*ax + ay*ay);
  double mag_b = std::sqrt(bx*bx + by*by);

  if (mag_a < 1e-9 || mag_b < 1e-9) {
    return 0.0;
  }

  double cos_theta = dot / (mag_a * mag_b);
  // Clamp to valid range to guard against floating-point rounding
  cos_theta = std::max(-1.0, std::min(1.0, cos_theta));

  return std::acos(cos_theta) * (180.0 / M_PI);
}

}  // namespace vtr_motion_planner
