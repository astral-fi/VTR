#pragma once

#include <vector>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <DBoW2/DBoW2.h>

namespace vtr {

/**
 * VTR Keyframe struct — exact contract expected by Phases 2, 3, 4.
 *
 * Populated by the ORB-SLAM3 adapter (Phase1Node) and passed downstream
 * via the Phase 1 API (getLatestKeyframe / subscribeKeyframe).
 *
 * IMPORTANT — monocular notes:
 *   features_3d may be sparse early in a session (before sufficient
 *   baseline accumulates). Downstream phases must check
 *   features_3d.size() >= 12 before attempting PnP.
 */
struct Keyframe {
    // --- Core fields (guide §1.5) ---

    // T_{i-1}^{i} : relative SE(3) transform from previous keyframe
    // to this one. 4x4 float64, row-major.
    Eigen::Matrix4d T_rel;

    // 2D features: (u, v) pixel coordinates paired with 256-bit BRIEF
    // descriptors. Index-aligned with features_3d where depth is valid.
    std::vector<cv::KeyPoint>  features_2d;   // u, v, size, angle
    cv::Mat                    descriptors;   // N x 32 CV_8UC1 (256-bit BRIEF)

    // 3D feature positions in left-camera frame.
    // Size may be < features_2d.size() for monocular (uninitialized pts).
    std::vector<cv::Point3f>   features_3d;

    // Index map: features_3d[i] corresponds to features_2d[feat3d_idx[i]]
    // Required because monocular frames may have 2D-only features.
    std::vector<int>           feat3d_idx;

    // DBoW2 bag-of-words vector for place recognition (Phase 2 loop det.)
    DBoW2::BowVector           descriptor;

    // Grayscale image — kept for map expansion (Phase 2 §2.4)
    cv::Mat                    image;         // CV_8UC1

    // Seconds since epoch (ROS time converted to double)
    double                     timestamp;

    // Unique sequential ID assigned by Phase1Node (0-indexed)
    int                        id;

    // --- Validity helpers ---
    bool has3D()    const { return features_3d.size() >= 12; }
    bool isValid()  const { return !image.empty() && features_2d.size() > 0; }
};

} // namespace vtr