#pragma once
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <vector>
#include "vtr_phase1_perception/keyframe.hpp"

namespace vtr_phase1 {

struct FASTConfig {
    int threshold       = 25;      // FAST corner threshold
    int nms_radius      = 7;       // NMS suppression radius in pixels
    int max_features    = 600;     // hard cap per frame
    int target_features = 400;     // desired count
};

struct BRIEFConfig {
    int descriptor_bytes = 32;     // 256 bits
    float gaussian_sigma = 2.0f;   // pre-blur sigma
    int hamming_threshold = 60;    // max bits for a valid match
};

class FeatureExtractor {
public:
    FeatureExtractor(const FASTConfig& fast_cfg, const BRIEFConfig& brief_cfg);

    // Extract FAST corners + BRIEF descriptors from a grayscale image
    void extract(const cv::Mat& gray,
                 std::vector<cv::KeyPoint>& keypoints,
                 cv::Mat& descriptors);

    // Apply NMS: remove keypoints within nms_radius of a stronger one
    void applyNMS(std::vector<cv::KeyPoint>& keypoints, int radius);

private:
    FASTConfig  fast_cfg_;
    BRIEFConfig brief_cfg_;
    cv::Ptr<cv::FastFeatureDetector>  detector_;
    cv::Ptr<cv::DescriptorExtractor>  extractor_;
};

} // namespace vtr_phase1

