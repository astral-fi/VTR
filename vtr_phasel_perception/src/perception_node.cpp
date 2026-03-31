#include "vtr_phase1_perception/perception_node.hpp"
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

namespace vtr_phase1 {

PerceptionNode::PerceptionNode(ros::NodeHandle& nh,
                               ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private)
{
    // ---- Load parameters ----
    std::string left_topic, right_topic, pose_topic, vocab_path;
    nh_private_.param<std::string>("left_image_topic",
                                   left_topic, "/camera/left/image_raw");
    nh_private_.param<std::string>("right_image_topic",
                                   right_topic, "/camera/right/image_raw");
    nh_private_.param<std::string>("openvins_pose_topic",
                                   pose_topic,
                                   "/ov_msckf/poseimu");
    nh_private_.param<std::string>("dbow2_vocab_path",
                                   vocab_path, "");

    // ---- Feature extractor ----
    FASTConfig  fast_cfg;
    BRIEFConfig brief_cfg;
    nh_private_.param("fast_threshold",    fast_cfg.threshold,       25);
    nh_private_.param("fast_nms_radius",   fast_cfg.nms_radius,       7);
    nh_private_.param("fast_max_features", fast_cfg.max_features,   600);
    feature_extractor_ = std::make_unique<FeatureExtractor>(fast_cfg, brief_cfg);

    // ---- Stereo calibration (load from params or YAML) ----
    StereoCalibration cal;
    // Placeholder: load from camera_info or param server in production
    cal.focal_length = 458.654f;
    cal.cx           = 367.215f;
    cal.cy           = 248.375f;
    cal.baseline     = 0.110f;     // metres
    cal.K_left  = (cv::Mat_<double>(3,3) <<
                   458.654, 0, 367.215,
                   0, 457.296, 248.375,
                   0, 0, 1);
    cal.D_left  = cv::Mat::zeros(1, 5, CV_64F);
    cal.K_right = cal.K_left.clone();
    cal.D_right = cal.D_left.clone();
    cal.R_rect  = cv::Mat::eye(3, 3, CV_64F);
    cal.P_left  = (cv::Mat_<double>(3,4) <<
                   458.654, 0, 367.215, 0,
                   0, 457.296, 248.375, 0,
                   0, 0, 1, 0);
    cal.P_right = (cv::Mat_<double>(3,4) <<
                   458.654, 0, 367.215, -458.654*0.110,
                   0, 457.296, 248.375, 0,
                   0, 0, 1, 0);
    triangulator_ = std::make_unique<StereoTriangulator>(cal);

    // ---- Keyframe selector ----
    KeyframePolicy kf_policy;
    nh_private_.param("kf_frame_interval", kf_policy.frame_interval, 5);
    keyframe_selector_ = std::make_unique<KeyframeSelector>(kf_policy);

    // ---- OpenVINS bridge ----
    openvins_bridge_ = std::make_unique<OpenVINSBridge>(nh_, pose_topic);

    // ---- DBoW2 vocabulary ----
    if (!vocab_path.empty()) {
        // vocabulary_ = std::make_shared<...>();
        // vocabulary_->load(vocab_path);
        ROS_INFO("[Phase1] DBoW2 vocab loaded from: %s", vocab_path.c_str());
    } else {
        ROS_WARN("[Phase1] No DBoW2 vocab path set. "
                 "BoW descriptors will be empty.");
    }

    // ---- Publishers ----
    keyframe_pub_ = nh_.advertise<vtr_phase1_perception::Keyframe>(
                        "/vtr/keyframes", 10);

    // ---- Stereo sync subscriber ----
    left_sub_.subscribe(nh_,  left_topic,  5);
    right_sub_.subscribe(nh_, right_topic, 5);
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
                SyncPolicy(10), left_sub_, right_sub_);
    sync_->registerCallback(
        boost::bind(&PerceptionNode::stereoCallback, this, _1, _2));

    // Seed accumulated pose
    accumulated_poses_.push_back(Eigen::Matrix4d::Identity());

    ROS_INFO("[Phase1] Perception node initialized.");
}

// ---------------------------------------------------------------------------
void PerceptionNode::stereoCallback(
        const sensor_msgs::ImageConstPtr& left_msg,
        const sensor_msgs::ImageConstPtr& right_msg)
{
    frame_counter_++;

    // Convert to OpenCV
    cv_bridge::CvImageConstPtr left_cv, right_cv;
    try {
        left_cv  = cv_bridge::toCvShare(left_msg,
                       sensor_msgs::image_encodings::MONO8);
        right_cv = cv_bridge::toCvShare(right_msg,
                       sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("[Phase1] cv_bridge: %s", e.what());
        return;
    }

    // Rectify
    cv::Mat left_rect, right_rect;
    triangulator_->rectify(left_cv->image, right_cv->image,
                           left_rect, right_rect);

    // Get relative pose from OpenVINS
    Eigen::Matrix4d T_rel = Eigen::Matrix4d::Identity();
    if (openvins_bridge_->isReady()) {
        T_rel = openvins_bridge_->getRelativePose();
    }

    // Accumulate pose
    Eigen::Matrix4d T_accum = accumulated_poses_.back() * T_rel;

    // Ask selector if this frame should become a keyframe
    int tracked = 300;  // replace with actual tracked-feature count from OpenVINS
    if (!keyframe_selector_->shouldEmitKeyframe(T_rel, tracked)) {
        return;
    }
    keyframe_selector_->reset();

    // ---- Feature extraction + triangulation ----
    std::vector<cv::KeyPoint> kps;
    cv::Mat descs;
    feature_extractor_->extract(left_rect, kps, descs);

    std::vector<Feature2D> features_2d;
    std::vector<Feature3D> features_3d;
    if (!kps.empty()) {
        triangulator_->triangulate(left_rect, right_rect,
                                   kps, descs,
                                   features_2d, features_3d);
    }

    // ---- Build and publish keyframe ----
    buildAndPublishKeyframe(left_rect, left_msg->header.stamp,
                            features_2d, features_3d, T_rel);

    accumulated_poses_.push_back(T_accum);
    if (accumulated_poses_.size() > MAX_HISTORY)
        accumulated_poses_.pop_front();
}

// ---------------------------------------------------------------------------
void PerceptionNode::buildAndPublishKeyframe(
        const cv::Mat& gray,
        const ros::Time& stamp,
        std::vector<Feature2D>& features_2d,
        std::vector<Feature3D>& features_3d,
        const Eigen::Matrix4d& T_rel)
{
    uint32_t kf_id = keyframe_id_counter_++;

    // --- Fill ROS message ---
    vtr_phase1_perception::Keyframe msg;
    msg.header.stamp    = stamp;
    msg.header.frame_id = "camera_left";
    msg.keyframe_id     = kf_id;

    // T_rel as column-major flat array
    for (int col = 0; col < 4; ++col)
        for (int row = 0; row < 4; ++row)
            msg.T_rel[col * 4 + row] = T_rel(row, col);

    // Features
    msg.features_u.reserve(features_2d.size());
    msg.features_v.reserve(features_2d.size());
    msg.brief_descriptors.reserve(features_2d.size() * 32);
    msg.features_x.reserve(features_3d.size());
    msg.features_y.reserve(features_3d.size());
    msg.features_z.reserve(features_3d.size());

    for (size_t i = 0; i < features_2d.size(); ++i) {
        msg.features_u.push_back(features_2d[i].u);
        msg.features_v.push_back(features_2d[i].v);

        // Flatten descriptor bytes
        const cv::Mat& d = features_2d[i].descriptor;
        for (int b = 0; b < d.cols; ++b)
            msg.brief_descriptors.push_back(d.at<uint8_t>(0, b));
    }
    for (size_t i = 0; i < features_3d.size(); ++i) {
        msg.features_x.push_back(features_3d[i].x);
        msg.features_y.push_back(features_3d[i].y);
        msg.features_z.push_back(features_3d[i].z);
    }

    // Image
    cv_bridge::CvImage cv_img;
    cv_img.header   = msg.header;
    cv_img.encoding = sensor_msgs::image_encodings::MONO8;
    cv_img.image    = gray;
    msg.image = *cv_img.toImageMsg();

    // BoW vector (populated if vocabulary is loaded)
    // vocabulary_->transform(descriptors, msg.bow_...);

    // Publish
    keyframe_pub_.publish(msg);

    // --- Store internally ---
    Keyframe kf;
    kf.id          = kf_id;
    kf.timestamp   = stamp;
    kf.T_rel       = T_rel;
    kf.features_2d = features_2d;
    kf.features_3d = features_3d;
    kf.image       = gray.clone();

    {
        std::lock_guard<std::mutex> lock(history_mutex_);
        keyframe_history_.push_back(kf);
        if (keyframe_history_.size() > MAX_HISTORY)
            keyframe_history_.pop_front();
    }

    // Fire registered callbacks (for in-process Phase 2 communication)
    {
        std::lock_guard<std::mutex> lock(subscriber_mutex_);
        for (auto& cb : subscribers_) cb(kf);
    }

    ROS_INFO_THROTTLE(2.0, "[Phase1] KF #%u published | features: %zu",
                      kf_id, features_2d.size());
}

// ---------------------------------------------------------------------------
// Phase 1 API implementations
// ---------------------------------------------------------------------------

Keyframe PerceptionNode::getLatestKeyframe()
{
    std::lock_guard<std::mutex> lock(history_mutex_);
    if (keyframe_history_.empty())
        throw std::runtime_error("[Phase1] No keyframes yet.");
    return keyframe_history_.back();
}

void PerceptionNode::subscribeKeyframe(
        std::function<void(const Keyframe&)> cb)
{
    std::lock_guard<std::mutex> lock(subscriber_mutex_);
    subscribers_.push_back(std::move(cb));
}

Eigen::Matrix4d PerceptionNode::getOdometryPose()
{
    return openvins_bridge_->getAccumulatedPose();
}

Eigen::Matrix4d PerceptionNode::getRelativePose(uint32_t from_id,
                                                  uint32_t to_id)
{
    // accumulated_poses_[i] = T_0^{KF_i}
    // T_{from}^{to} = T_0^{from}_inv * T_0^{to}
    std::lock_guard<std::mutex> lock(history_mutex_);
    if (from_id >= accumulated_poses_.size() ||
        to_id   >= accumulated_poses_.size()) {
        ROS_WARN("[Phase1] getRelativePose: ID out of range.");
        return Eigen::Matrix4d::Identity();
    }
    return accumulated_poses_[from_id].inverse() * accumulated_poses_[to_id];
}

} // namespace vtr_phase1

// ---------------------------------------------------------------------------
int main(int argc, char** argv)
{
    ros::init(argc, argv, "vtr_phase1_perception");
    ros::NodeHandle nh, nh_private("~");

    vtr_phase1::PerceptionNode node(nh, nh_private);

    ros::spin();
    return 0;
}
