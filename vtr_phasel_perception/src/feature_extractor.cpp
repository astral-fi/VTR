#include "vtr_phase1_perception/feature_extractor.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <ros/ros.h>

namespace vtr_phase1 {

FeatureExtractor::FeatureExtractor(const FASTConfig& fast_cfg,
                                   const BRIEFConfig& brief_cfg)
    : fast_cfg_(fast_cfg), brief_cfg_(brief_cfg)
{
    detector_  = cv::FastFeatureDetector::create(
                     fast_cfg_.threshold, true,
                     cv::FastFeatureDetector::TYPE_9_16);
    extractor_ = cv::BRISK::create();   // OpenCV 3.4 doesn't ship standalone
                                         // BRIEF as DescriptorExtractor;
                                         // BRISK gives 48-byte descriptors.
                                         // For true 32-byte BRIEF use xfeatures2d
                                         // if opencv_contrib is available.
}

void FeatureExtractor::applyNMS(std::vector<cv::KeyPoint>& kps, int radius)
{
    // Sort by response descending
    std::sort(kps.begin(), kps.end(),
              [](const cv::KeyPoint& a, const cv::KeyPoint& b) {
                  return a.response > b.response;
              });

    std::vector<bool> suppressed(kps.size(), false);
    float r2 = static_cast<float>(radius * radius);

    for (size_t i = 0; i < kps.size(); ++i) {
        if (suppressed[i]) continue;
        for (size_t j = i + 1; j < kps.size(); ++j) {
            if (suppressed[j]) continue;
            float dx = kps[i].pt.x - kps[j].pt.x;
            float dy = kps[i].pt.y - kps[j].pt.y;
            if (dx*dx + dy*dy < r2) suppressed[j] = true;
        }
    }

    std::vector<cv::KeyPoint> filtered;
    filtered.reserve(kps.size());
    for (size_t i = 0; i < kps.size(); ++i)
        if (!suppressed[i]) filtered.push_back(kps[i]);

    kps = std::move(filtered);
}

void FeatureExtractor::extract(const cv::Mat& gray,
                               std::vector<cv::KeyPoint>& keypoints,
                               cv::Mat& descriptors)
{
    // Pre-blur for descriptor noise robustness
    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred,
                     cv::Size(0, 0), brief_cfg_.gaussian_sigma);

    // Detect
    detector_->detect(blurred, keypoints);

    // NMS
    applyNMS(keypoints, fast_cfg_.nms_radius);

    // Cap to max
    if ((int)keypoints.size() > fast_cfg_.max_features)
        keypoints.resize(fast_cfg_.max_features);

    // Compute descriptors
    extractor_->compute(blurred, keypoints, descriptors);

    ROS_DEBUG_THROTTLE(2.0, "[Phase1] Extracted %zu features",
                       keypoints.size());
}

} // namespace vtr_phase1
