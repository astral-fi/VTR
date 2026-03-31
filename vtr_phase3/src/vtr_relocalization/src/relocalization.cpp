// ============================================================================
// relocalization.cpp
//
// Core Phase 3 pipeline:
//   Step 1: DBoW2 coarse retrieval
//   Step 2: BRIEF window matching + grid filter
//   Step 3: Gauss-Newton PnP
//   Step 4: Goal list update
//
// See Section 3.1–3.3 of the VTR implementation guide.
// ============================================================================

#include "vtr_relocalization/relocalization.h"
#include <ros/ros.h>
#include <opencv2/imgproc.hpp>

// DBoW2 BRIEF database types
using BriefDatabase = DBoW2::TemplatedDatabase<DBoW2::FBrief::TDescriptor,
                                               DBoW2::FBrief>;

namespace vtr {

// ─────────────────────────────────────────────────────────────────────────────
Relocalization::Relocalization(
    const Phase3Config& cfg,
    std::shared_ptr<DBoW2::BriefDatabase> db,
    std::vector<MapKeyframe::Ptr> map_keyframes)
  : cfg_(cfg)
  , db_(std::move(db))
  , map_keyframes_(std::move(map_keyframes))
  , pnp_solver_(cfg)
  , goal_manager_(cfg)
{
  // FAST detector — threshold 20, non-maximum suppression on
  fast_detector_ = cv::FastFeatureDetector::create(20, true);

  // BRIEF extractor — 32 bytes = 256 bits, use_orientation=false (classic BRIEF)
  // Note: cv::xfeatures2d is in opencv_contrib.  For vanilla OpenCV 3 on
  // Melodic, link against opencv3-contrib (see README install instructions).
  brief_extractor_ = cv::xfeatures2d::BriefDescriptorExtractor::create(32);

  ROS_INFO("[Reloc] Initialized with %zu map keyframes.",
           map_keyframes_.size());
}

// ─────────────────────────────────────────────────────────────────────────────
RelocResult Relocalization::relocalize(const cv::Mat& live_image)
{
  RelocResult result;

  // ── Convert to grayscale if needed ────────────────────────────────────────
  cv::Mat gray;
  if (live_image.channels() == 3) {
    cv::cvtColor(live_image, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = live_image;
  }

  // ── Extract FAST+BRIEF for the whole live frame ───────────────────────────
  std::vector<cv::KeyPoint> kps;
  fast_detector_->detect(gray, kps);

  if (kps.empty()) {
    ROS_WARN("[Reloc] No features detected in live frame.");
    return result;
  }

  cv::Mat live_descs;
  brief_extractor_->compute(gray, kps, live_descs);

  // ── Step 1: Coarse DBoW2 retrieval ───────────────────────────────────────
  // Convert live descriptors to DBoW2 format
  std::vector<DBoW2::FBrief::TDescriptor> brief_vec;
  for (int i = 0; i < live_descs.rows; ++i) {
    boost::dynamic_bitset<> bits;
    // FBrief::TDescriptor is a boost::dynamic_bitset<uint64_t>
    // Reconstruct from the CV_8U descriptor row (32 bytes = 256 bits)
    const uchar* row = live_descs.ptr<uchar>(i);
    bits.resize(256);
    for (int b = 0; b < 256; ++b) {
      if (row[b / 8] & (1 << (b % 8))) bits.set(b);
    }
    brief_vec.push_back(bits);
  }

  DBoW2::QueryResults bow_results;
  // Query top-10 candidates from DBoW2
  db_->query(brief_vec, bow_results, 10);

  std::vector<int> candidates = coarseRetrieval(cv::Mat(), last_match_idx_,
                                                 bow_results);
  if (candidates.empty()) {
    ROS_DEBUG("[Reloc] No candidates passed coarse retrieval.");
    return result;
  }

  // ── Step 2+3: Window match + PnP for each candidate ──────────────────────
  for (int idx : candidates) {
    const MapKeyframe& kf = *map_keyframes_[idx];

    // Window matching in live frame
    std::vector<Correspondence> corrs = windowMatch(gray, kf);
    corrs = gridFilter(corrs);

    ROS_DEBUG("[Reloc] Candidate KF%d: %zu correspondences after grid filter",
              kf.id, corrs.size());

    if (static_cast<int>(corrs.size()) < cfg_.pnp_min_inliers) continue;

    // PnP pose estimation
    PnPResult pnp = pnp_solver_.solve(corrs);

    if (!pnp.converged) continue;

    // ── Success ───────────────────────────────────────────────────────────
    result.success   = true;
    result.kf_id     = kf.id;
    result.T_live_kf = pnp.T_live_kf;
    result.num_corr  = static_cast<int>(corrs.size());
    result.pnp       = pnp;

    last_match_idx_ = idx;

    // Update goal list
    updateGoalList(kf, pnp.T_live_kf);

    ROS_INFO("[Reloc] Match KF%d | reproj=%.2f px | inliers=%d",
             kf.id, pnp.rms_reproj_error, pnp.num_inliers);
    break;   // Use first accepted match
  }

  return result;
}

// ─────────────────────────────────────────────────────────────────────────────
void Relocalization::updateGoalList(const MapKeyframe& kf,
                                     const SE3d& T_live_kf)
{
  goal_manager_.updateGoalList(kf, T_live_kf, map_keyframes_);
}

// ─────────────────────────────────────────────────────────────────────────────
GoalList Relocalization::getGoalList() const
{
  return goal_manager_.getGoalList();
}

// ─────────────────────────────────────────────────────────────────────────────
void Relocalization::notifyOdometry(const SE3d& T_delta)
{
  goal_manager_.notifyOdometry(T_delta);
}

// ═════════════════════════════════════════════════════════════════════════════
// Private helpers
// ═════════════════════════════════════════════════════════════════════════════

std::vector<int> Relocalization::coarseRetrieval(
    const cv::Mat& /*bow_desc*/,
    int last_idx,
    DBoW2::QueryResults& raw_results)
{
  std::vector<int> candidates;
  candidates.reserve(raw_results.size());

  for (const auto& r : raw_results) {
    // Score threshold
    if (r.Score < cfg_.tau_ret) continue;

    // Temporal adjacency rejection (within 30 keyframes of last match)
    if (last_idx >= 0 && std::abs(static_cast<int>(r.Id) - last_idx) < 30) {
      continue;
    }

    // Map bounds check
    if (static_cast<size_t>(r.Id) >= map_keyframes_.size()) continue;

    candidates.push_back(static_cast<int>(r.Id));
  }

  return candidates;
}

// ─────────────────────────────────────────────────────────────────────────────
std::vector<Correspondence>
Relocalization::windowMatch(const cv::Mat& live_image, const MapKeyframe& kf)
{
  std::vector<Correspondence> correspondences;
  correspondences.reserve(kf.features_2d.size());

  for (size_t n = 0; n < kf.features_2d.size(); ++n) {
    const Feature2D& map_feat = kf.features_2d[n];

    int cx = static_cast<int>(map_feat.u);
    int cy = static_cast<int>(map_feat.v);

    // Extract FAST+BRIEF features in the search window
    std::vector<Feature2D> live_feats = extractInRegion(live_image, cx, cy,
                                                          cfg_.search_radius);
    if (live_feats.empty()) continue;

    // Find best Hamming match
    int best_dist = cfg_.hamming_thresh + 1;
    int best_idx  = -1;

    for (int i = 0; i < static_cast<int>(live_feats.size()); ++i) {
      // Hamming distance between 32-byte BRIEF descriptors
      int dist = static_cast<int>(
          cv::norm(map_feat.descriptor, live_feats[i].descriptor,
                   cv::NORM_HAMMING));
      if (dist < best_dist) {
        best_dist = dist;
        best_idx  = i;
      }
    }

    if (best_idx < 0) continue;

    Correspondence corr;
    corr.point3d = kf.features_3d[n];
    corr.pixel2d = Eigen::Vector2d(live_feats[best_idx].u,
                                    live_feats[best_idx].v);
    correspondences.push_back(corr);
  }

  return correspondences;
}

// ─────────────────────────────────────────────────────────────────────────────
std::vector<Correspondence>
Relocalization::gridFilter(const std::vector<Correspondence>& raw_corrs) const
{
  // Divide the image into grid_rows × grid_cols cells.
  // Keep at most max_per_cell correspondences per cell.
  const int cell_w = cfg_.image_width  / cfg_.grid_cols;
  const int cell_h = cfg_.image_height / cfg_.grid_rows;

  // cell_count[row * cols + col] = number of matches already accepted
  std::vector<int> cell_count(cfg_.grid_rows * cfg_.grid_cols, 0);
  std::vector<Correspondence> filtered;
  filtered.reserve(raw_corrs.size());

  for (const auto& corr : raw_corrs) {
    int col = static_cast<int>(corr.pixel2d.x()) / std::max(cell_w, 1);
    int row = static_cast<int>(corr.pixel2d.y()) / std::max(cell_h, 1);

    // Clamp to valid range
    col = std::min(col, cfg_.grid_cols - 1);
    row = std::min(row, cfg_.grid_rows - 1);

    int cell_idx = row * cfg_.grid_cols + col;

    if (cell_count[cell_idx] < cfg_.max_per_cell) {
      ++cell_count[cell_idx];
      filtered.push_back(corr);
    }
  }

  return filtered;
}

// ─────────────────────────────────────────────────────────────────────────────
std::vector<Feature2D>
Relocalization::extractInRegion(const cv::Mat& image,
                                  int cx, int cy, int radius)
{
  // Build a circular ROI mask and detect FAST features within it.
  int x0 = std::max(0,                cx - radius);
  int y0 = std::max(0,                cy - radius);
  int x1 = std::min(image.cols - 1,  cx + radius);
  int y1 = std::min(image.rows - 1,  cy + radius);

  if (x1 <= x0 || y1 <= y0) return {};

  cv::Rect roi_rect(x0, y0, x1 - x0, y1 - y0);
  cv::Mat  roi_img = image(roi_rect);

  std::vector<cv::KeyPoint> kps;
  fast_detector_->detect(roi_img, kps);
  if (kps.empty()) return {};

  cv::Mat descs;
  brief_extractor_->compute(roi_img, kps, descs);

  std::vector<Feature2D> feats;
  feats.reserve(kps.size());

  for (int i = 0; i < static_cast<int>(kps.size()); ++i) {
    // Verify point is within circle (not just bounding rectangle)
    float dx = kps[i].pt.x + x0 - cx;
    float dy = kps[i].pt.y + y0 - cy;
    if (dx * dx + dy * dy > static_cast<float>(radius * radius)) continue;

    Feature2D f;
    f.u = kps[i].pt.x + static_cast<float>(x0);   // convert back to full-image coords
    f.v = kps[i].pt.y + static_cast<float>(y0);
    f.descriptor = descs.row(i).clone();
    feats.push_back(f);
  }

  return feats;
}

} // namespace vtr
