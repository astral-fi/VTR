#pragma once
// trajectory_scorer.h
// ---------------------------------------------------------------------------
// Implements the feasibility filter and scoring function from Algorithm 1
// (Section 4.3 of the implementation guide).
//
// Scoring function (per-trajectory):
//   s_i = (1/3) * sum_{m=0}^{2} ( 1 - (0.005 * theta(t_e_i, t_gm))^0.5 )
//
// where theta is the angular deviation in degrees between the trajectory
// endpoint and goal m, measured from the current robot position.
// ---------------------------------------------------------------------------

#include <vector>
#include <geometry_msgs/Point.h>
#include "vtr_motion_planner/occupancy_grid.h"
#include "vtr_motion_planner/trajectory_generator.h"

namespace vtr_motion_planner {

struct ScoringResult {
  int    best_index   = -1;   ///< Index into arcs vector (-1 = none feasible)
  double best_score   = -1.0;
  std::vector<double> scores; ///< Score for each candidate (0 if infeasible)
  std::vector<bool>   feasible;
};

class TrajectoryScorer {
public:
  /// @param goals_to_use  Number of lookahead goals to include in score (spec: 3)
  explicit TrajectoryScorer(int goals_to_use = 3);

  /// Evaluate all trajectory candidates against the occupancy grid and goals.
  /// @param arcs    Pre-cached trajectory candidates
  /// @param grid    Current inflated occupancy grid
  /// @param goals   Goal list in robot frame (at least goals_to_use entries)
  ScoringResult evaluate(const std::vector<TrajectoryArc>& arcs,
                         const OccupancyGrid&              grid,
                         const std::vector<geometry_msgs::Point>& goals) const;

private:
  /// Angular deviation in degrees between vectors (0,0)->a and (0,0)->b.
  static double angleDegrees(const geometry_msgs::Point& endpoint,
                             const geometry_msgs::Point& goal);

  int goals_to_use_;
};

}  // namespace vtr_motion_planner
