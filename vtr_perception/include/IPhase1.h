#pragma once

#include "vtr_perception/include/Keyframe.h"
#include <functional>
#include <Eigen/Dense>

namespace vtr {

/**
 * IPhase1 — Abstract API contract that Phase 1 exposes to Phases 2, 3, 4.
 * Exactly matches §1.6 of the VTR Implementation Guide.
 *
 * Phase1Node (ORB-SLAM3 adapter) implements this interface.
 * Phases 2/3/4 depend only on this interface — never on ORB-SLAM3 directly.
 */
class IPhase1 {
public:
    virtual ~IPhase1() = default;

    // Returns the most recently emitted keyframe.
    // Returns nullptr if no keyframe has been emitted yet.
    virtual const Keyframe* getLatestKeyframe() const = 0;

    // Register a callback invoked synchronously on each new keyframe.
    // cb receives a const reference valid only during the call.
    using KeyframeCb = std::function<void(const Keyframe&)>;
    virtual void subscribeKeyframe(KeyframeCb cb) = 0;

    // Returns current robot odometry pose as SE(3) 4x4.
    // Used by Phase 4 for goal-frame updates between relocalization hits.
    virtual Eigen::Matrix4d getOdometryPose() const = 0;

    // Accumulates relative transforms from keyframe i to keyframe j.
    // Traverses the stored chain: T_{i}^{i+1} * ... * T_{j-1}^{j}
    // Returns identity if i == j. Throws std::out_of_range if invalid ids.
    virtual Eigen::Matrix4d getRelativePose(int i, int j) const = 0;
};

} // namespace vtr