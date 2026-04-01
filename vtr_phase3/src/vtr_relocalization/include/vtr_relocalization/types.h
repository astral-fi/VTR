#pragma once
// ============================================================================
// vtr_relocalization/types.h
//
// Shared data structures for VTR Phase 3:
//   Relocalization & Goal List Management.
//
// These mirror the keyframe and map structures defined in Phases 1–2 and add
// Phase-3-specific types for correspondences, PnP results, and goals.
// ============================================================================

#include <vector>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

namespace vtr {

// ─── SE(3) convenience alias ─────────────────────────────────────────────────
// A 4×4 homogeneous transform stored as double-precision Eigen matrix.
using SE3d = Eigen::Matrix4d;

// ─── 2D feature descriptor point ─────────────────────────────────────────────
struct Feature2D {
  float u;           // pixel column
  float v;           // pixel row
  cv::Mat descriptor;  // 256-bit ORB stored as 1×32 CV_8U row
};

// ─── 3D–2D correspondence ────────────────────────────────────────────────────
// One matched pair produced by frame-to-local-map matching (Section 3.1).
struct Correspondence {
  Eigen::Vector3d point3d;   // 3D point in the matched keyframe's camera frame
  Eigen::Vector2d pixel2d;   // Matching 2D observation in the live frame
};

// ─── PnP result ──────────────────────────────────────────────────────────────
struct PnPResult {
  bool   converged    = false;
  SE3d   T_live_kf;            // T_k^i: transform from keyframe to live frame
  double rms_reproj_error = 0; // RMS reprojection error in pixels
  int    num_inliers      = 0;
};

// ─── Reduced map keyframe (subset of Phase 2 enhanced keyframe) ──────────────
// Phase 3 only needs the fields relevant for retrieval and PnP.
struct MapKeyframe {
  using Ptr = std::shared_ptr<MapKeyframe>;

  int    id        = -1;
  SE3d   T_rel;                          // T_{i-1}^{i}
  SE3d   T_skip;                         // T_{i}^{S(i)} to next cluster node
  int    skip_idx  = -1;                 // index of next cluster representative

  std::vector<Feature2D>       features_2d;   // extended (merged) 2D features
  std::vector<Eigen::Vector3d> features_3d;   // paired 3D points, camera frame
  cv::Mat                      bow_descriptor; // DBoW2 BowVector encoded as Mat
  double                       timestamp = 0;

  // Loop-closure fields (stored but not used by Phase 3 directly)
  SE3d T_loop;
  int  loop_idx = -1;
};

// ─── Goal list ───────────────────────────────────────────────────────────────
// Ordered list of translational goal positions in the current robot frame.
// Updated on every successful relocalization (Section 3.3).
struct GoalList {
  std::vector<Eigen::Vector3d> goals;  // [t_g0, t_g1, ..., t_gM]
  double timestamp = 0.0;              // ROS time of last update (seconds)
};

// ─── Camera intrinsics ───────────────────────────────────────────────────────
struct CameraIntrinsics {
  double fx = 0, fy = 0;
  double cx = 0, cy = 0;

  cv::Mat toCvMat() const {
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    K.at<double>(0, 0) = fx;
    K.at<double>(1, 1) = fy;
    K.at<double>(0, 2) = cx;
    K.at<double>(1, 2) = cy;
    return K;
  }

  // Build 3×4 projection matrix [K | 0]
  Eigen::Matrix<double, 3, 4> projectionMatrix() const {
    Eigen::Matrix<double, 3, 4> P = Eigen::Matrix<double, 3, 4>::Zero();
    P(0, 0) = fx;  P(1, 1) = fy;
    P(0, 2) = cx;  P(1, 2) = cy;  P(2, 2) = 1.0;
    return P;
  }
};

// ─── Phase 3 configuration ───────────────────────────────────────────────────
struct Phase3Config {
  // --- Coarse retrieval ---
  double tau_ret        = 0.012;  // minimum DBoW score to accept a candidate

  // --- ORB window matching ---
  int    search_radius  = 40;     // γ: pixel search window radius
  int    hamming_thresh = 60;     // max Hamming distance for ORB match
  int    grid_rows      = 6;      // image grid rows for spatial filter
  int    grid_cols      = 8;      // image grid cols for spatial filter
  int    max_per_cell   = 3;      // max matches per grid cell

  // --- PnP Gauss-Newton ---
  double pnp_convergence_eps = 1e-6; // ‖δξ‖ < eps → converged
  int    pnp_max_iter        = 20;   // maximum GN iterations
  double pnp_max_reproj_err  = 2.5;  // pixels — reject if RMS above this
  int    pnp_min_inliers     = 12;   // minimum inlier count

  // --- Goal list ---
  int    goal_list_length    = 5;    // M: total goals maintained
  int    goal_score_count    = 3;    // goals used by Phase 4 trajectory scorer
  double goal_arrival_dist   = 0.4;  // metres — erase goal when robot this close
  double goal_angle_limit    = 60.0; // degrees — erase goal[0] if angle exceeds this

  // --- Camera ---
  CameraIntrinsics camera;

  // --- Image dimensions (for grid filter) ---
  int image_width  = 640;
  int image_height = 480;
};

} // namespace vtr
