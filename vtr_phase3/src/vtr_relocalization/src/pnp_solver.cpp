// ============================================================================
// pnp_solver.cpp
//
// Gauss-Newton iterative PnP solver implementing Section 3.2 of the VTR
// implementation guide.
//
// Lie-algebra perturbation model (left multiplication):
//   T_new = Exp(δξ) · T_old
//
// Jacobian derivation:
//   e_n = proj(R·p_n + t) − [u_k, v_k]
//   ∂e/∂p' = [[fx/z,  0,    -fx·x/z²],
//              [ 0,   fy/z,  -fy·y/z²]]
//   ∂p'/∂δξ = [I₃ | -p'∧]    (6-column matrix)
//
// Full 2×6 Jacobian per point:  J_n = (∂e/∂p') · (∂p'/∂δξ)
//
// Normal equations:  δξ = -(JᵀJ)⁻¹ Jᵀ e
// ============================================================================

#include "vtr_relocalization/pnp_solver.h"
#include <ros/ros.h>
#include <cmath>

namespace vtr {

// ─────────────────────────────────────────────────────────────────────────────
PnPSolver::PnPSolver(const Phase3Config& cfg)
  : cfg_(cfg)
{}

// ─────────────────────────────────────────────────────────────────────────────
PnPResult PnPSolver::solve(const std::vector<Correspondence>& correspondences,
                            const SE3d& init_pose) const
{
  PnPResult result;
  result.converged = false;

  if (static_cast<int>(correspondences.size()) < cfg_.pnp_min_inliers) {
    ROS_DEBUG("[PnP] Too few correspondences: %zu (need %d)",
              correspondences.size(), cfg_.pnp_min_inliers);
    return result;
  }

  SE3d T = init_pose;   // Current pose estimate, updated each iteration

  for (int iter = 0; iter < cfg_.pnp_max_iter; ++iter) {

    // ── Accumulate JᵀJ and Jᵀe ────────────────────────────────────────────
    Eigen::Matrix<double, 6, 6> JtJ = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> Jte = Eigen::Matrix<double, 6, 1>::Zero();

    for (const auto& corr : correspondences) {
      // Transform 3D point into current camera frame
      Eigen::Vector4d p_h;
      p_h << corr.point3d, 1.0;
      Eigen::Vector3d p_cam = (T * p_h).head<3>();

      if (p_cam.z() <= 0.0) continue;   // behind camera — skip

      // Reprojection error
      Eigen::Vector2d proj = project(corr.point3d, T);
      Eigen::Vector2d e    = proj - corr.pixel2d;

      // 2×6 Jacobian
      Eigen::Matrix<double, 2, 6> J = computeJacobian(p_cam);

      JtJ += J.transpose() * J;
      Jte += J.transpose() * e;
    }

    // ── Solve normal equations ─────────────────────────────────────────────
    // Add a small damping term (Levenberg-Marquardt style) for stability.
    const double lambda = 1e-6;
    JtJ += lambda * Eigen::Matrix<double, 6, 6>::Identity();

    Eigen::Matrix<double, 6, 1> delta =
        -JtJ.ldlt().solve(Jte);

    // ── Apply perturbation ─────────────────────────────────────────────────
    T = applyPerturbation(T, delta);

    // ── Check convergence ──────────────────────────────────────────────────
    if (delta.norm() < cfg_.pnp_convergence_eps) {
      ROS_DEBUG("[PnP] Converged at iteration %d (‖δξ‖=%.2e)", iter, delta.norm());
      break;
    }
  }

  // ── Final quality check ────────────────────────────────────────────────
  auto [rms_err, n_inliers] = computeInliers(correspondences, T);

  result.T_live_kf        = T;
  result.rms_reproj_error = rms_err;
  result.num_inliers      = n_inliers;

  if (rms_err > cfg_.pnp_max_reproj_err) {
    ROS_DEBUG("[PnP] Rejected: RMS reproj %.2f px > %.2f px threshold",
              rms_err, cfg_.pnp_max_reproj_err);
    return result;
  }
  if (n_inliers < cfg_.pnp_min_inliers) {
    ROS_DEBUG("[PnP] Rejected: %d inliers < %d required",
              n_inliers, cfg_.pnp_min_inliers);
    return result;
  }

  result.converged = true;
  ROS_DEBUG("[PnP] Accepted: RMS=%.2f px, inliers=%d", rms_err, n_inliers);
  return result;
}

// ─────────────────────────────────────────────────────────────────────────────
Eigen::Vector2d PnPSolver::project(const Eigen::Vector3d& p3d,
                                    const SE3d& T) const
{
  Eigen::Vector4d p_h;
  p_h << p3d, 1.0;
  Eigen::Vector3d p_cam = (T * p_h).head<3>();

  const double inv_z = 1.0 / p_cam.z();
  return {
    cfg_.camera.fx * p_cam.x() * inv_z + cfg_.camera.cx,
    cfg_.camera.fy * p_cam.y() * inv_z + cfg_.camera.cy
  };
}

// ─────────────────────────────────────────────────────────────────────────────
Eigen::Matrix<double, 2, 6>
PnPSolver::computeJacobian(const Eigen::Vector3d& p_cam) const
{
  const double x = p_cam.x();
  const double y = p_cam.y();
  const double z = p_cam.z();
  const double z2 = z * z;

  const double fx = cfg_.camera.fx;
  const double fy = cfg_.camera.fy;

  // ∂e/∂p' — 2×3
  Eigen::Matrix<double, 2, 3> de_dp;
  de_dp << fx / z,    0.0,  -fx * x / z2,
           0.0,    fy / z,  -fy * y / z2;

  // ∂p'/∂δξ — 3×6  (SE3 left perturbation: [I | -p∧])
  Eigen::Matrix<double, 3, 6> dp_dxi;
  dp_dxi.leftCols<3>()  = Eigen::Matrix3d::Identity();
  dp_dxi.rightCols<3>() = -skew(p_cam);

  return de_dp * dp_dxi;   // 2×6
}

// ─────────────────────────────────────────────────────────────────────────────
Eigen::Matrix3d PnPSolver::skew(const Eigen::Vector3d& v)
{
  Eigen::Matrix3d S;
  S <<     0, -v.z(),  v.y(),
       v.z(),      0, -v.x(),
      -v.y(),  v.x(),      0;
  return S;
}

// ─────────────────────────────────────────────────────────────────────────────
SE3d PnPSolver::applyPerturbation(const SE3d& T,
                                   const Eigen::Matrix<double, 6, 1>& delta)
{
  // δξ = [δt | δφ] in se3 (translation first, rotation second)
  Eigen::Vector3d dt = delta.head<3>();
  Eigen::Vector3d dw = delta.tail<3>();

  double angle = dw.norm();
  Eigen::Matrix3d dR;

  if (angle < 1e-10) {
    dR = Eigen::Matrix3d::Identity() + skew(dw);
  } else {
    // Rodrigues formula
    Eigen::AngleAxisd aa(angle, dw / angle);
    dR = aa.toRotationMatrix();
  }

  // Exp(δξ) as a 4×4 matrix (approximation via Rodrigues)
  SE3d T_delta = SE3d::Identity();
  T_delta.topLeftCorner<3, 3>() = dR;
  T_delta.topRightCorner<3, 1>() = dt;

  return T_delta * T;   // left multiplication
}

// ─────────────────────────────────────────────────────────────────────────────
std::pair<double, int>
PnPSolver::computeInliers(const std::vector<Correspondence>& corrs,
                            const SE3d& T) const
{
  double sum_sq = 0.0;
  int n_inliers = 0;

  for (const auto& corr : corrs) {
    Eigen::Vector2d proj = project(corr.point3d, T);
    double err = (proj - corr.pixel2d).norm();

    if (err < cfg_.pnp_max_reproj_err) {
      sum_sq += err * err;
      ++n_inliers;
    }
  }

  double rms = (n_inliers > 0)
               ? std::sqrt(sum_sq / static_cast<double>(n_inliers))
               : std::numeric_limits<double>::infinity();

  return {rms, n_inliers};
}

} // namespace vtr
