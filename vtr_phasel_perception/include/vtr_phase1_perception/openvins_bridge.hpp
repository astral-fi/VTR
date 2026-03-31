#pragma once
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mutex>
#include <deque>

namespace vtr_phase1 {

// Lightweight wrapper: subscribes to OpenVINS output topics and
// exposes relative pose queries. OpenVINS itself runs as a separate node.
class OpenVINSBridge {
public:
    OpenVINSBridge(ros::NodeHandle& nh, const std::string& pose_topic);

    // Returns T_{prev}^{current}: relative SE(3) since last call to getPose()
    Eigen::Matrix4d getRelativePose();

    // Returns the accumulated pose since system start (for debugging)
    Eigen::Matrix4d getAccumulatedPose();

    // Returns number of tracked features reported by OpenVINS (if available)
    int getTrackedFeatureCount() const;

    bool isReady() const { return pose_received_; }

private:
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    ros::Subscriber pose_sub_;
    mutable std::mutex mutex_;

    Eigen::Matrix4d last_pose_world_;   // T_world^{cam} at last query
    Eigen::Matrix4d current_pose_world_;
    bool pose_received_ = false;
};

} // namespace vtr_phase1
