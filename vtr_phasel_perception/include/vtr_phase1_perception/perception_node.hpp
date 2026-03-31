#pragma once
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <DBoW2/DBoW2.h>

#include "vtr_phase1_perception/keyframe.hpp"
#include "vtr_phase1_perception/feature_extractor.hpp"
#include "vtr_phase1_perception/stereo_triangulator.hpp"
#include "vtr_phase1_perception/keyframe_selector.hpp"
#include "vtr_phase1_perception/openvins_bridge.hpp"
#include "vtr_phase1_perception/Keyframe.h"

#include <mutex>
#include <functional>

namespace vtr_phase1 {

using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, sensor_msgs::Image>;

class PerceptionNode {
public:
    explicit PerceptionNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

    // Phase 1 API -----------------------------------------------------------
    Keyframe getLatestKeyframe();
    void subscribeKeyframe(std::function<void(const Keyframe&)> cb);
    Eigen::Matrix4d getOdometryPose();
    Eigen::Matrix4d getRelativePose(uint32_t from_id, uint32_t to_id);
    // -----------------------------------------------------------------------

private:
    void stereoCallback(const sensor_msgs::ImageConstPtr& left_msg,
                        const sensor_msgs::ImageConstPtr& right_msg);

    void buildAndPublishKeyframe(const cv::Mat& left_gray,
                                 const ros::Time& stamp,
                                 std::vector<Feature2D>& features_2d,
                                 std::vector<Feature3D>& features_3d,
                                 const Eigen::Matrix4d& T_rel);

    ros::NodeHandle nh_, nh_private_;
    ros::Publisher  keyframe_pub_;

    // Message-filters stereo sync
    message_filters::Subscriber<sensor_msgs::Image> left_sub_, right_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    std::unique_ptr<FeatureExtractor>   feature_extractor_;
    std::unique_ptr<StereoTriangulator> triangulator_;
    std::unique_ptr<KeyframeSelector>   keyframe_selector_;
    std::unique_ptr<OpenVINSBridge>     openvins_bridge_;

    // DBoW2 vocabulary (loaded from file)
    std::shared_ptr<DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor,
                                               DBoW2::FORB>> vocabulary_;

    // Keyframe store (Phase 1 keeps a rolling window for getRelativePose)
    std::deque<Keyframe> keyframe_history_;
    static constexpr size_t MAX_HISTORY = 500;
    mutable std::mutex history_mutex_;

    // Callbacks registered via subscribeKeyframe()
    std::vector<std::function<void(const Keyframe&)>> subscribers_;
    mutable std::mutex subscriber_mutex_;

    // Accumulated relative pose chain for getRelativePose()
    std::vector<Eigen::Matrix4d> accumulated_poses_;  // T_0^i for each KF i

    uint32_t keyframe_id_counter_ = 0;
    int      frame_counter_       = 0;
};

} // namespace vtr_phase1
