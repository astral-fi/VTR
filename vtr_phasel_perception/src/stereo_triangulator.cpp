#include "vtr_phase1_perception/stereo_triangulator.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <ros/ros.h>

namespace vtr_phase1 {

StereoTriangulator::StereoTriangulator(const StereoCalibration& cal)
    : cal_(cal)
{
    // Pre-compute remap tables for faster rectification
    cv::initUndistortRectifyMap(
        cal_.K_left, cal_.D_left, cal_.R_rect, cal_.P_left,
        cv::Size(640, 480),   // update to actual resolution
        CV_32FC1, map_l1_, map_l2_);

    // For right camera — minimal remap (assume already rectified in ideal case)
    cv::Mat R_right = cv::Mat::eye(3, 3, CV_64F);
    cv::initUndistortRectifyMap(
        cal_.K_right, cal_.D_right, R_right, cal_.P_right,
        cv::Size(640, 480),
        CV_32FC1, map_r1_, map_r2_);
}

void StereoTriangulator::rectify(const cv::Mat& left_raw,
                                 const cv::Mat& right_raw,
                                 cv::Mat& left_rect,
                                 cv::Mat& right_rect)
{
    cv::remap(left_raw,  left_rect,  map_l1_, map_l2_, cv::INTER_LINEAR);
    cv::remap(right_raw, right_rect, map_r1_, map_r2_, cv::INTER_LINEAR);
}

bool StereoTriangulator::isDepthValid(float depth) const
{
    return (depth > 0.1f && depth < 15.0f);
}

void StereoTriangulator::triangulate(
        const cv::Mat& left_rect,
        const cv::Mat& right_rect,
        const std::vector<cv::KeyPoint>& left_kps,
        const cv::Mat& left_descs,
        std::vector<Feature2D>& features_2d_out,
        std::vector<Feature3D>& features_3d_out)
{
    features_2d_out.clear();
    features_3d_out.clear();

    if (left_kps.empty()) return;

    // Use a horizontal strip search for stereo matching (epipolar constraint)
    const int STRIP_HEIGHT = 3;   // ±3 pixels vertical tolerance
    const float MAX_DISP   = 128.0f;

    // Compute disparity via BM or use descriptor matching along epipolar lines
    // For simplicity here: descriptor match along horizontal strip
    for (size_t i = 0; i < left_kps.size(); ++i) {
        const cv::KeyPoint& kp_l = left_kps[i];
        cv::Mat desc_l = left_descs.row(static_cast<int>(i));

        float best_dist = 100.0f;
        float best_x_r  = -1.0f;

        // Search in right image along epipolar line (same row ±strip)
        for (float x_r = kp_l.pt.x - MAX_DISP; x_r < kp_l.pt.x; x_r += 1.0f) {
            if (x_r < 0) continue;
            // Sample BRISK descriptor at candidate right location
            // In practice, detect right-image keypoints first and match;
            // here we do a lightweight patch-based search.
            // --- Simplified: use sparse optical flow for right match ---
        }

        // Practical approach for Melodic: use OpenCV's StereoSGBM
        // or pre-detect right-image keypoints and match descriptors.
        // Below is the descriptor-matching branch:

        // (This is intentionally a skeleton for the epipolar search;
        //  see perception_node.cpp for the complete integrated approach.)

        float disparity = kp_l.pt.x - best_x_r;
        if (best_x_r < 0 || disparity < 1.0f) continue;  // min disparity guard

        float depth = (cal_.focal_length * cal_.baseline) / disparity;
        if (!isDepthValid(depth)) continue;

        float X = (kp_l.pt.x - cal_.cx) * depth / cal_.focal_length;
        float Y = (kp_l.pt.y - cal_.cy) * depth / cal_.focal_length;
        float Z = depth;

        Feature2D f2;
        f2.u = kp_l.pt.x;
        f2.v = kp_l.pt.y;
        f2.descriptor = desc_l.clone();

        Feature3D f3;
        f3.x = X; f3.y = Y; f3.z = Z;

        features_2d_out.push_back(f2);
        features_3d_out.push_back(f3);
    }
}

} // namespace vtr_phase1
