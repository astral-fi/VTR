#pragma once
#include <opencv2/core.hpp>
#include <vector>
#include "vtr_phase1_perception/keyframe.hpp"

namespace vtr_phase1 {

struct StereoCalibration {
    cv::Mat K_left;       // 3x3 intrinsics
    cv::Mat K_right;
    cv::Mat D_left;       // distortion coefficients
    cv::Mat D_right;
    cv::Mat R_rect;       // rectification rotation
    cv::Mat P_left;       // 3x4 projection after rectification
    cv::Mat P_right;
    cv::Mat Q;            // disparity-to-depth matrix (from stereoRectify)
    float baseline;       // metres, positive value
    float focal_length;   // pixels (from rectified K)
    float cx, cy;
};

class StereoTriangulator {
public:
    explicit StereoTriangulator(const StereoCalibration& cal);

    // Rectify a stereo pair
    void rectify(const cv::Mat& left_raw, const cv::Mat& right_raw,
                 cv::Mat& left_rect, cv::Mat& right_rect);

    // Match left keypoints to right image and triangulate valid 3D points.
    // Output vectors are 1:1 aligned (some entries may be invalid — check valid[]).
    void triangulate(const cv::Mat& left_rect,
                     const cv::Mat& right_rect,
                     const std::vector<cv::KeyPoint>& left_kps,
                     const cv::Mat& left_descs,
                     std::vector<Feature2D>& features_2d_out,
                     std::vector<Feature3D>& features_3d_out);

private:
    StereoCalibration cal_;
    cv::Mat map_l1_, map_l2_;   // remap tables for left
    cv::Mat map_r1_, map_r2_;   // remap tables for right

    bool isDepthValid(float depth) const;
};

} // namespace vtr_phase1
