#include "vtr_perception/include/IPhase1.h"
#include "vtr_perception/include/Keyframe.h"

// ORB-SLAM3 headers
#include <System.h>
#include <KeyFrame.h>
#include <MapPoint.h>
#include <ORBextractor.h>

// ROS Melodic
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

// DBoW2 (ORB vocabulary — same vocab ORB-SLAM3 uses internally)
#include <memory>
#include <mutex>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>

#include <deque>
#include <atomic>

namespace vtr {

class Phase1Node : public IPhase1 {
public:
    Phase1Node(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
    {
        // --- Load parameters ---
        std::string vocab_path, settings_path;
        nh_priv.param<std::string>("orb_vocab",     vocab_path,     "");
        nh_priv.param<std::string>("orb_settings",  settings_path,  "");
        nh_priv.param<int>        ("keyframe_every", kf_every_,     5);
        nh_priv.param<double>     ("kf_trans_thresh", kf_t_thresh_, 0.3);
        nh_priv.param<double>     ("kf_rot_thresh",   kf_r_thresh_, 0.1745); // 10 deg

        ROS_ASSERT_MSG(!vocab_path.empty(),    "orb_vocab path must be set");
        ROS_ASSERT_MSG(!settings_path.empty(), "orb_settings path must be set");

        // --- Init ORB-SLAM3 monocular-inertial ---
        // MONOCULAR_IMU uses the IMU tightly coupled with vision.
        // Disable ORB-SLAM3's own loop-closure and global BA —
        // the VTR map layer handles loop correction at the graph level.
        slam_ = std::make_unique<ORB_SLAM3::System>(
            vocab_path, settings_path,
            ORB_SLAM3::System::IMU_MONOCULAR,
            /*bUseViewer=*/false
        );

        // Load same ORB vocabulary for our DBoW2 queries in Phase 2.
        // ORB-SLAM3 uses ORBvoc.txt (ORB-based, not BRIEF).
        // Phase 2 must use ORB vocab — see CHANGE NOTE in Phase 2 section.
        vocab_ = std::make_shared<OrbVocabulary>();
        ROS_INFO("Loading DBoW2 vocabulary...");
        vocab_->loadFromTextFile(vocab_path);
        ROS_INFO("Vocabulary loaded.");

        // --- IMU buffer ---
        imu_sub_ = nh.subscribe("/imu/data", 400,
                                &Phase1Node::imuCallback, this);

        // --- Camera subscriber (single — monocular) ---
        img_sub_ = nh.subscribe("/camera/image_raw", 30,
                                &Phase1Node::imageCallback, this);

        // --- Odometry Publisher (for Phase 4) ---
        odom_pub_ = nh.advertise<nav_msgs::Odometry>("/vtr/odometry", 10);

        ROS_INFO("[Phase1] ORB-SLAM3 adapter ready.");
    }

    // ------------------------------------------------------------------ //
    //  IPhase1 interface                                                   //
    // ------------------------------------------------------------------ //

    const Keyframe* getLatestKeyframe() const override {
        std::lock_guard<std::mutex> lk(kf_mutex_);
        if (keyframes_.empty()) return nullptr;
        return &keyframes_.back();
    }

    void subscribeKeyframe(KeyframeCb cb) override {
        std::lock_guard<std::mutex> lk(cb_mutex_);
        callbacks_.push_back(cb);
    }

    Eigen::Matrix4d getOdometryPose() const override {
        std::lock_guard<std::mutex> lk(pose_mutex_);
        return current_odom_pose_;
    }

    Eigen::Matrix4d getRelativePose(int i, int j) const override {
        std::lock_guard<std::mutex> lk(kf_mutex_);
        ROS_ASSERT(i >= 0 && j >= 0 && j >= i);
        if (i == j) return Eigen::Matrix4d::Identity();

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        for (int k = i; k < j; ++k) {
            // keyframes_ is indexed from 0 by insertion order
            if (k >= (int)keyframes_.size()) break;
            T = T * keyframes_[k].T_rel;
        }
        return T;
    }

private:
    // ------------------------------------------------------------------ //
    //  IMU callback — buffer measurements for ORB-SLAM3 pre-integration   //
    // ------------------------------------------------------------------ //
    void imuCallback(const sensor_msgs::ImuConstPtr& msg)
    {
        std::lock_guard<std::mutex> lk(imu_mutex_);
        ORB_SLAM3::IMU::Point pt(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z,
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z,
            msg->header.stamp.toSec()
        );
        imu_buf_.push_back(pt);

        // Keep only last 2 seconds to bound memory
        double cutoff = msg->header.stamp.toSec() - 2.0;
        while (!imu_buf_.empty() && imu_buf_.front().t < cutoff)
            imu_buf_.pop_front();
    }

    // ------------------------------------------------------------------ //
    //  Image callback — main processing loop                               //
    // ------------------------------------------------------------------ //
    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImageConstPtr cv_img;
        try {
            cv_img = cv_bridge::toCvShare(msg, "mono8");
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge: %s", e.what());
            return;
        }

        double t_img = msg->header.stamp.toSec();

        // Collect IMU measurements up to this image timestamp
        std::vector<ORB_SLAM3::IMU::Point> imu_meas;
        {
            std::lock_guard<std::mutex> lk(imu_mutex_);
            for (auto& pt : imu_buf_)
                if (pt.t <= t_img)
                    imu_meas.push_back(pt);
        }

        if (imu_meas.empty()) {
            ROS_WARN_THROTTLE(1.0, "[Phase1] No IMU data yet — skipping frame.");
            return;
        }

        // --- Feed to ORB-SLAM3 ---
        Sophus::SE3f T_cam_world = slam_->TrackMonocular(
            cv_img->image, t_img, imu_meas);

        // ORB-SLAM3 returns identity / invalid during init — skip
        if (slam_->GetTrackingState() != ORB_SLAM3::Tracking::OK)
            return;

        // Update odometry pose
        Eigen::Matrix4d T_odom;
        {
            std::lock_guard<std::mutex> lk(pose_mutex_);
            current_odom_pose_ = sophusToEigen(T_cam_world);
            T_odom = current_odom_pose_;
        }

        // Publish high-frequency V-SLAM Odometry
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = msg->header.stamp;
        odom_msg.header.frame_id = "map";
        odom_msg.child_frame_id = "camera_link";
        
        Eigen::Matrix3d R = T_odom.block<3,3>(0,0);
        Eigen::Quaterniond q(R);
        odom_msg.pose.pose.position.x = T_odom(0,3);
        odom_msg.pose.pose.position.y = T_odom(1,3);
        odom_msg.pose.pose.position.z = T_odom(2,3);
        odom_msg.pose.pose.orientation.w = q.w();
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_pub_.publish(odom_msg);

        frame_counter_++;

        // --- Keyframe selection (guide §1.4) ---
        bool force_kf = false;

        // Motion-based trigger
        Eigen::Matrix4d T_cur = sophusToEigen(T_cam_world);
        if (!keyframes_.empty()) {
            Eigen::Matrix4d T_last = last_kf_pose_;
            Eigen::Matrix4d T_delta = T_last.inverse() * T_cur;
            double dt = T_delta.block<3,1>(0,3).norm();
            double dr = Eigen::AngleAxisd(
                Eigen::Matrix3d(T_delta.block<3,3>(0,0))).angle();
            if (dt > kf_t_thresh_ || dr > kf_r_thresh_)
                force_kf = true;
        }

        bool emit_kf = force_kf || (frame_counter_ % kf_every_ == 0);
        if (!emit_kf) return;

        // --- Build Keyframe struct ---
        Keyframe kf = buildKeyframe(cv_img->image, t_img, T_cam_world);
        if (!kf.isValid()) return;

        {
            std::lock_guard<std::mutex> lk(kf_mutex_);
            kf.id = (int)keyframes_.size();

            // Compute T_rel = T_{i-1}^{i}
            if (!keyframes_.empty()) {
                Eigen::Matrix4d T_prev = sophusToEigen(last_kf_T_);
                kf.T_rel = T_prev.inverse() * sophusToEigen(T_cam_world);
            } else {
                kf.T_rel = Eigen::Matrix4d::Identity();
            }
            keyframes_.push_back(kf);
        }

        last_kf_pose_ = T_cur;
        last_kf_T_    = T_cam_world;

        // Fire callbacks (Phase 2 addKeyframe lives here)
        fireCallbacks(kf);

        ROS_DEBUG("[Phase1] Emitted keyframe %d | 3D pts: %zu | 2D pts: %zu",
                  kf.id, kf.features_3d.size(), kf.features_2d.size());
    }

    // ------------------------------------------------------------------ //
    //  Build VTR Keyframe from ORB-SLAM3 internals                         //
    // ------------------------------------------------------------------ //
    Keyframe buildKeyframe(const cv::Mat& img, double t,
                           const Sophus::SE3f& T_cw)
    {
        Keyframe kf;
        kf.timestamp = t;
        kf.image     = img.clone();  // CV_8UC1 as required

        // Get the latest ORB-SLAM3 keyframe to extract features
        ORB_SLAM3::KeyFrame* slam_kf = slam_->GetCurrentKeyFrame();
        if (!slam_kf) return kf;

        // --- 2D features + descriptors ---
        // ORB-SLAM3 uses ORB descriptors (32-byte = 256-bit), not BRIEF.
        // Phase 2/3 must use ORB hamming distance — see CHANGE NOTES.
        const std::vector<cv::KeyPoint>& kps = slam_kf->mvKeysUn;
        cv::Mat descs = slam_kf->mDescriptors.clone(); // N x 32 CV_8UC1

        kf.features_2d  = kps;
        kf.descriptors  = descs;

        // --- 3D features from map points ---
        const std::vector<ORB_SLAM3::MapPoint*>& mps = slam_kf->GetMapPointMatches();
        for (int i = 0; i < (int)mps.size(); ++i) {
            if (!mps[i] || mps[i]->isBad()) continue;

            // Map point world position → camera frame
            Eigen::Vector3f pw = mps[i]->GetWorldPos();
            Eigen::Vector3f pc = T_cw.rotationMatrix() * pw + T_cw.translation();

            // Guide §1.3: discard depth > 15 m or behind camera
            if (pc.z() <= 0.0f || pc.z() > 15.0f) continue;

            kf.features_3d.push_back({pc.x(), pc.y(), pc.z()});
            kf.feat3d_idx.push_back(i);  // maps back to features_2d[i]
        }

        // --- DBoW2 bag-of-words vector ---
        // ORB-SLAM3 computes this internally; extract directly.
        kf.descriptor = slam_kf->mBowVec;

        return kf;
    }

    // ------------------------------------------------------------------ //
    //  Helpers                                                             //
    // ------------------------------------------------------------------ //
    static Eigen::Matrix4d sophusToEigen(const Sophus::SE3f& T)
    {
        Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
        M.block<3,3>(0,0) = T.rotationMatrix().cast<double>();
        M.block<3,1>(0,3) = T.translation().cast<double>();
        return M;
    }

    void fireCallbacks(const Keyframe& kf)
    {
        std::lock_guard<std::mutex> lk(cb_mutex_);
        for (auto& cb : callbacks_) cb(kf);
    }

    // ------------------------------------------------------------------ //
    //  Members                                                             //
    // ------------------------------------------------------------------ //
    std::unique_ptr<ORB_SLAM3::System>  slam_;
    std::shared_ptr<OrbVocabulary>      vocab_;

    ros::Subscriber img_sub_;
    ros::Subscriber imu_sub_;
    ros::Publisher  odom_pub_;

    // IMU ring buffer
    mutable std::mutex          imu_mutex_;
    std::deque<ORB_SLAM3::IMU::Point> imu_buf_;

    // Keyframe store
    mutable std::mutex          kf_mutex_;
    std::vector<Keyframe>       keyframes_;

    // Odometry pose (latest)
    mutable std::mutex          pose_mutex_;
    Eigen::Matrix4d             current_odom_pose_ = Eigen::Matrix4d::Identity();

    // Callbacks
    mutable std::mutex          cb_mutex_;
    std::vector<KeyframeCb>     callbacks_;

    // Keyframe selection state
    int                         frame_counter_ = 0;
    int                         kf_every_;
    double                      kf_t_thresh_;
    double                      kf_r_thresh_;
    Eigen::Matrix4d             last_kf_pose_ = Eigen::Matrix4d::Identity();
    Sophus::SE3f                last_kf_T_;
};
} // namespace vtr

#include <vtr_map_manager/Keyframe.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "phase1_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    vtr::Phase1Node node(nh, nh_priv);

    ros::Publisher kf_pub = nh.advertise<vtr_map_manager::Keyframe>("/vtr/keyframe", 10);

    node.subscribeKeyframe([&](const vtr::Keyframe& kf) {
        vtr_map_manager::Keyframe msg;
        msg.header.stamp = ros::Time(kf.timestamp);
        msg.header.frame_id = "camera_link";

        for (int i=0; i<16; ++i) msg.T_rel.push_back(kf.T_rel(i/4, i%4));

        for (const auto& kp : kf.features_2d) {
            msg.features_u.push_back(kp.pt.x);
            msg.features_v.push_back(kp.pt.y);
        }

        if (!kf.descriptors.empty()) {
            for (int i=0; i<kf.descriptors.rows; ++i) {
                for (int j=0; j<kf.descriptors.cols; ++j) {
                    msg.features_descriptors.push_back(kf.descriptors.at<uint8_t>(i, j));
                }
            }
        }

        for (const auto& pt3 : kf.features_3d) {
            msg.features_x.push_back(pt3.x);
            msg.features_y.push_back(pt3.y);
            msg.features_z.push_back(pt3.z);
        }

        for (int idx : kf.feat3d_idx) {
            msg.feat3d_idx.push_back(idx);
        }

        for (const auto& w : kf.descriptor) {
            msg.bow_word_ids.push_back(w.first);
            msg.bow_word_weights.push_back(w.second);
        }

        kf_pub.publish(msg);
    });

    ros::spin();
    return 0;
}