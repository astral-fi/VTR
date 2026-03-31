#include "vtr_phase1_perception/openvins_bridge.hpp"
#include <ros/ros.h>

namespace vtr_phase1 {

OpenVINSBridge::OpenVINSBridge(ros::NodeHandle& nh,
                               const std::string& pose_topic)
{
    last_pose_world_    = Eigen::Matrix4d::Identity();
    current_pose_world_ = Eigen::Matrix4d::Identity();

    pose_sub_ = nh.subscribe(pose_topic, 10,
                              &OpenVINSBridge::poseCallback, this);
    ROS_INFO("[Phase1] OpenVINS bridge subscribing to: %s",
             pose_topic.c_str());
}

void OpenVINSBridge::poseCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // Build 4x4 from quaternion + position
    Eigen::Quaterniond q(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);
    Eigen::Matrix3d R = q.toRotationMatrix();

    current_pose_world_ = Eigen::Matrix4d::Identity();
    current_pose_world_.block<3,3>(0,0) = R;
    current_pose_world_(0,3) = msg->pose.pose.position.x;
    current_pose_world_(1,3) = msg->pose.pose.position.y;
    current_pose_world_(2,3) = msg->pose.pose.position.z;

    pose_received_ = true;
}

Eigen::Matrix4d OpenVINSBridge::getRelativePose()
{
    std::lock_guard<std::mutex> lock(mutex_);
    // T_prev^current = T_world^prev_inv * T_world^current
    Eigen::Matrix4d T_rel =
        last_pose_world_.inverse() * current_pose_world_;
    last_pose_world_ = current_pose_world_;
    return T_rel;
}

Eigen::Matrix4d OpenVINSBridge::getAccumulatedPose()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return current_pose_world_;
}

int OpenVINSBridge::getTrackedFeatureCount() const
{
    // OpenVINS publishes feature count on a separate topic;
    // extend this class to subscribe to it if needed.
    return -1;
}

} // namespace vtr_phase1
