// occupancy_grid.cpp
// ---------------------------------------------------------------------------
// Builds a 2D robot-centric occupancy grid from a laser scan and inflates
// obstacles by the robot's footprint radius.
// ---------------------------------------------------------------------------

#include "vtr_motion_planner/occupancy_grid.h"
#include <cmath>
#include <algorithm>

namespace vtr_motion_planner {

// ---------------------------------------------------------------------------
OccupancyGrid::OccupancyGrid(double width_m,
                             double res_m,
                             double inflation_radius_m)
  : width_m_(width_m)
  , resolution_(res_m)
  , half_width_(width_m / 2.0)
  , inflation_radius_m_(inflation_radius_m)
{
  num_cells_       = static_cast<int>(std::ceil(width_m_ / resolution_));
  inflation_cells_ = static_cast<int>(std::ceil(inflation_radius_m_ / resolution_));

  raw_grid_.assign(num_cells_, std::vector<bool>(num_cells_, false));
  inflated_grid_.assign(num_cells_, std::vector<bool>(num_cells_, false));
}

// ---------------------------------------------------------------------------
void OccupancyGrid::buildFromScan(const sensor_msgs::LaserScan& scan)
{
  clearGrids();

  const float angle_min = scan.angle_min;
  const float angle_inc = scan.angle_increment;
  const float range_min = scan.range_min;
  const float range_max = scan.range_max;

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    float r = scan.ranges[i];
    if (!std::isfinite(r) || r < range_min || r > range_max) {
      continue;
    }
    float angle = angle_min + static_cast<float>(i) * angle_inc;
    double x = static_cast<double>(r) * std::cos(angle);
    double y = static_cast<double>(r) * std::sin(angle);
    markRawOccupied(x, y);
  }

  inflateObstacles();
}

// ---------------------------------------------------------------------------
bool OccupancyGrid::isOccupied(double x_m, double y_m) const
{
  int row, col;
  if (!worldToCell(x_m, y_m, row, col)) {
    // Out of grid bounds → treat as occupied (safe fallback)
    return true;
  }
  return inflated_grid_[row][col];
}

// ---------------------------------------------------------------------------
bool OccupancyGrid::cellOccupied(int row, int col) const
{
  if (row < 0 || row >= num_cells_ || col < 0 || col >= num_cells_) {
    return true;  // out of bounds → occupied
  }
  return inflated_grid_[row][col];
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void OccupancyGrid::clearGrids()
{
  for (auto& row : raw_grid_)      std::fill(row.begin(), row.end(), false);
  for (auto& row : inflated_grid_) std::fill(row.begin(), row.end(), false);
}

void OccupancyGrid::markRawOccupied(double x_m, double y_m)
{
  int row, col;
  if (worldToCell(x_m, y_m, row, col)) {
    raw_grid_[row][col] = true;
  }
}

void OccupancyGrid::inflateObstacles()
{
  // For every occupied raw cell, mark a square neighbourhood in the inflated
  // grid.  Using a square approximation is fast and slightly conservative,
  // which is the safe direction for obstacle inflation.
  const int r = inflation_cells_;

  for (int row = 0; row < num_cells_; ++row) {
    for (int col = 0; col < num_cells_; ++col) {
      if (!raw_grid_[row][col]) continue;

      // Expand into neighbourhood
      for (int dr = -r; dr <= r; ++dr) {
        for (int dc = -r; dc <= r; ++dc) {
          // Circular inflation: only mark cells within inflation radius
          double dist = std::sqrt(static_cast<double>(dr*dr + dc*dc)) * resolution_;
          if (dist > inflation_radius_m_) continue;

          int nr = row + dr;
          int nc = col + dc;
          if (nr >= 0 && nr < num_cells_ && nc >= 0 && nc < num_cells_) {
            inflated_grid_[nr][nc] = true;
          }
        }
      }
    }
  }
}

bool OccupancyGrid::worldToCell(double x_m, double y_m,
                                int& row, int& col) const
{
  // Grid convention:
  //   col increases with x (forward)
  //   row increases with y (left)
  // Origin at centre of grid.
  col = static_cast<int>(std::floor((x_m + half_width_) / resolution_));
  row = static_cast<int>(std::floor((y_m + half_width_) / resolution_));

  if (col < 0 || col >= num_cells_ || row < 0 || row >= num_cells_) {
    return false;
  }
  return true;
}

}  // namespace vtr_motion_planner
