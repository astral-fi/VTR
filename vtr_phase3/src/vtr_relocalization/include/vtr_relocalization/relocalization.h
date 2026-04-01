#pragma once
// ============================================================================
// vtr_relocalization/relocalization.h
//
// Core relocalization pipeline (Sections 3.1–3.3).
//
// For each live frame:
//   Step 1 — Coarse retrieval:   DBoW2 query against Phase 2 map API.
//   Step 2 — ORB window match:   per-feature search in γ=40 px window.
//   Step 3 — PnP pose estimate:  Gauss-Newton on 3D–2D correspondences.
//   Step 4 — Goal list update:   rebuild via GoalManager on success.
//
// The class is designed to be owned by the ROS node and called from image
// callbacks running in a separate spinner thread.
// ============================================================================

#include "vtr_relocalization/types.h"
#include "vtr_relocalization/pnp_solver.h"
#include "vtr_relocalization/goal_manager.h"

#include <memory>
#include <optional>
#include <opencv2/features2d.hpp>

// DBoW2 — see README for installation
#include <DBoW2/DBoW2.h>

namespace vtr {

// Result of a single relocalization attempt.
struct RelocResult {
  bool             success   = false;
  int              kf_id     = -1;    // matched map keyframe id
  SE3d             T_live_kf;         // pose of live frame in keyframe frame
  double           bow_score = 0.0;
  int              num_corr  = 0;     // correspondences before PnP
  PnPResult        pnp;
};

class Relocalization {
public:
  Relocalization(const Phase3Config& cfg,
                 std::shared_ptr<DBoW2::OrbDatabase> db,
                 std::vector<MapKeyframe::Ptr> map_keyframes);

  // ── Phase 3 API (Section 3.4) ─────────────────────────────────────────────

  // Match live frame; returns matched keyframe index and pose, or empty.
  // Internally updates the goal list on success.
  RelocResult relocalize(const cv::Mat& live_image);

  // Explicitly rebuild goal list (called internally by relocalize on success).
  void updateGoalList(const MapKeyframe& kf, const SE3d& T_live_kf);

  // Return current Lg (thread-safe copy).
  GoalList getGoalList() const;

  // Propagate goals into the updated robot frame (called by Phase 4).
  void notifyOdometry(const SE3d& T_delta);

private:
  // ── Step 1: Coarse retrieval ─────────────────────────────────────────────
  // Returns indices of map keyframes with DBoW score ≥ tau_ret,
  // excluding temporally adjacent frames (within 30 of previous match).
  std::vector<int> coarseRetrieval(const cv::Mat& bow_desc,
                                   int last_match_idx,
                                   DBoW2::QueryResults& raw_results);

  // ── Step 2: ORB window matching ────────────────────────────────────────
  // For each candidate keyframe, extract live features in γ-radius windows
  // around each map feature projection.  Returns correspondence set.
  std::vector<Correspondence> windowMatch(const cv::Mat& live_image,
                                          const MapKeyframe& kf);

  // Apply 8×6 grid filter: keep at most max_per_cell matches per cell.
  std::vector<Correspondence> gridFilter(
      const std::vector<Correspondence>& raw_corrs) const;

  // Extract ORB features from a circular ROI in the live image.
  std::vector<Feature2D> extractInRegion(const cv::Mat& image,
                                          int cx, int cy, int radius);

  // Compute DBoW descriptor for a set of ORB descriptors.
  cv::Mat computeBowDescriptor(const std::vector<Feature2D>& features);

  Phase3Config                           cfg_;
  std::shared_ptr<DBoW2::OrbDatabase>    db_;
  std::vector<MapKeyframe::Ptr>          map_keyframes_;

  PnPSolver                              pnp_solver_;
  GoalManager                            goal_manager_;

  cv::Ptr<cv::ORB>                       orb_extractor_;

  int last_match_idx_ = -1;   // for temporal adjacency rejection
};

} // namespace vtr
