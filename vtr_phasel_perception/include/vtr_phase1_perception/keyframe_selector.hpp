#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace vtr_phase1 {

struct KeyframePolicy {
    int   frame_interval         = 5;      // emit keyframe every N frames
    float translation_threshold  = 0.30f;  // metres
    float rotation_threshold_deg = 10.0f;  // degrees
    int   min_tracked_features   = 80;     // force KF on feature drop
};

class KeyframeSelector {
public:
    explicit KeyframeSelector(const KeyframePolicy& policy);

    // Call each frame. Returns true if a new keyframe should be emitted.
    // accumulated_T: cumulative relative pose since last keyframe
    // tracked_features: number of features tracked from last KF
    bool shouldEmitKeyframe(const Eigen::Matrix4d& accumulated_T,
                            int tracked_features);

    // Reset after a keyframe is emitted
    void reset();

private:
    KeyframePolicy policy_;
    int frame_count_ = 0;
};

} // namespace vtr_phase1
