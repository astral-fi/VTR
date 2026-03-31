#pragma once
// ============================================================================
// vtr_relocalization/pnp_solver.h
//
// Gauss-Newton iterative PnP solver (Section 3.2 of the implementation guide).
//
// Given a set of 3D–2D correspondences, minimises the sum of squared
// reprojection errors using the Lie-algebra (se3) perturbation model.
//
// Per-point Jacobian:
//   ∂e/∂p' = [[fx/z,  0,    -fx·x/z²],
//              [ 0,   fy/z,  -fy·y/z²]]
//   ∂p'/∂δξ = [I | -p'∧]   (SE3 left perturbation)
//
// Update:  δξ = -(JᵀJ)⁻¹ Jᵀ e
// Convergence: ‖δξ‖ < 1e-6 or max_iter reached.
// ============================================================================

#include "vtr_relocalization/types.h"
#include <vector>

namespace vtr {

class PnPSolver {
public:
  explicit PnPSolver(const Phase3Config& cfg);

  // Estimate the SE(3) transform T_live_kf that maps 3D points expressed in
  // the keyframe camera frame to 2D observations in the live frame.
  //
  // init_pose: initial guess for T_live_kf (identity = robot near teaching view)
  // Returns a PnPResult with converged=false if the solve is rejected.
  PnPResult solve(const std::vector<Correspondence>& correspondences,
                  const SE3d& init_pose = SE3d::Identity()) const;

private:
  // Project a 3D point through the current pose estimate and return (u,v).
  Eigen::Vector2d project(const Eigen::Vector3d& p3d, const SE3d& T) const;

  // Compute 2×6 Jacobian of the reprojection error w.r.t. the se3 perturbation.
  Eigen::Matrix<double, 2, 6> computeJacobian(const Eigen::Vector3d& p_cam) const;

  // Skew-symmetric matrix of a 3-vector (hat operator).
  static Eigen::Matrix3d skew(const Eigen::Vector3d& v);

  // Apply a se3 perturbation δξ (6-vector) to a pose T via left multiplication.
  static SE3d applyPerturbation(const SE3d& T, const Eigen::Matrix<double, 6, 1>& delta);

  // Compute RMS reprojection error and count inliers below the pixel threshold.
  std::pair<double, int> computeInliers(
      const std::vector<Correspondence>& corrs,
      const SE3d& T) const;

  Phase3Config cfg_;
};

} // namespace vtr
