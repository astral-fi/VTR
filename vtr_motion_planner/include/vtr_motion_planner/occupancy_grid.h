#pragma once
// occupancy_grid.h
// ---------------------------------------------------------------------------
// Converts a 2D laser scan into a robot-centric binary occupancy grid and
// inflates obstacles by the robot footprint radius (C-space expansion).
//
// Grid origin: robot position.  X = forward, Y = left.
// Cell (row, col) maps to real-world (y, x) = (row - half) * res,
//                                              (col - half) * res
// ---------------------------------------------------------------------------

#include <vector>
#include <cmath>
#include <sensor_msgs/LaserScan.h>

namespace vtr_motion_planner {

class OccupancyGrid {
public:
  // -----------------------------------------------------------------------
  // Construction
  // -----------------------------------------------------------------------
  /// @param width_m   Physical width (and height) of the square grid (m)
  /// @param res_m     Cell resolution (m/cell)
  /// @param inflation_radius_m  Robot radius + safety margin (m)
  OccupancyGrid(double width_m = 6.0,
                double res_m   = 0.05,
                double inflation_radius_m = 0.25);

  // -----------------------------------------------------------------------
  // Public interface
  // -----------------------------------------------------------------------

  /// Build the raw occupancy grid from a laser scan, then inflate obstacles.
  void buildFromScan(const sensor_msgs::LaserScan& scan);

  /// Returns true if the world-frame robot-centric point (x, y) is occupied
  /// (after inflation).
  bool isOccupied(double x_m, double y_m) const;

  /// Returns true if the cell at (row, col) is occupied after inflation.
  bool cellOccupied(int row, int col) const;

  // Accessors
  int    numCells()    const { return num_cells_; }
  double resolution()  const { return resolution_; }
  double halfWidth()   const { return half_width_; }

private:
  // -----------------------------------------------------------------------
  // Internals
  // -----------------------------------------------------------------------
  void clearGrids();
  void markRawOccupied(double x_m, double y_m);
  void inflateObstacles();

  /// Convert world coords (m) to grid index.  Returns false if out of bounds.
  bool worldToCell(double x_m, double y_m, int& row, int& col) const;

  double width_m_;
  double resolution_;
  double half_width_;
  double inflation_radius_m_;
  int    num_cells_;          // grid is num_cells_ x num_cells_
  int    inflation_cells_;    // inflation_radius / resolution (rounded up)

  std::vector<std::vector<bool>> raw_grid_;      // before inflation
  std::vector<std::vector<bool>> inflated_grid_; // after inflation
};

}  // namespace vtr_motion_planner
