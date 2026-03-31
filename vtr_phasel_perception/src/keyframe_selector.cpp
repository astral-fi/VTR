#include "vtr_phase1_perception/keyframe_selector.hpp"
#include <cmath>

namespace vtr_phase1 {

KeyframeSelector::KeyframeSelector(const KeyframePolicy& policy)
    : policy_(policy) {}

bool KeyframeSelector::shouldEmitKeyframe(const Eigen::Matrix4d& T,
                                          int tracked_features)
{
    frame_count_++;

    // 1. Fixed-rate trigger
    if (frame_count_ >= policy_.frame_interval) return true;

    // 2. Large translation
    float tx = static_cast<float>(T(0,3));
    float ty = static_cast<float>(T(1,3));
    float tz = static_cast<float>(T(2,3));
    float translation = std::sqrt(tx*tx + ty*ty + tz*tz);
    if (translation > policy_.translation_threshold) return true;

    // 3. Large rotation — extract angle from rotation matrix
    Eigen::Matrix3d R = T.block<3,3>(0,0);
    double trace = R(0,0) + R(1,1) + R(2,2);
    double cos_angle = std::max(-1.0, std::min(1.0, (trace - 1.0) / 2.0));
    double angle_deg = std::acos(cos_angle) * (180.0 / M_PI);
    if (angle_deg > policy_.rotation_threshold_deg) return true;

    // 4. Feature-track drop
    if (tracked_features < policy_.min_tracked_features) return true;

    return false;
}

void KeyframeSelector::reset()
{
    frame_count_ = 0;
}

} // namespace vtr_phase1
