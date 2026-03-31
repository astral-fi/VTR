// ============================================================================
// relocalization_node.cpp
//
// ROS Melodic node for VTR Phase 3: Relocalization & Goal List Management.
//
// Subscriptions:
//   /camera/image_raw          (sensor_msgs/Image)     — live grayscale frames
//   /vtr/map_keyframes         (vtr_msgs/MapGraph)     — Phase 2 reduced map
//   /vtr/odometry              (nav_msgs/Odometry)     — Phase 4 feedback
//
// Publications:
//   /vtr/goal_list             (vtr_msgs/GoalList)     — Phase 4 input
//   /vtr/reloc_status          (std_msgs/String)       — debug info
//
// Service (Phase 3 API):
//   /vtr/get_goal_list         (vtr_msgs/GetGoalList)
//   /vtr/notify_odometry       (vtr_msgs/NotifyOdometry)
//
// The node loads:
//   - DBoW2 vocabulary from ~vocabulary_path parameter
//   - Pre-built map keyframes from a serialised file (~map_path parameter)
//   - Camera intrinsics from ~camera_fx/fy/cx/cy parameters
// ============================================================================

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cv_bridge/cv_bridge.h>

#include "vtr_relocalization/relocalization.h"
#include "vtr_relocalization/types.h"

#include <DBoW2/DBoW2.h>
#include <memory>
#include <mutex>
#include <sstream>

namespace vtr {

// ─────────────────────────────────────────────────────────────────────────────
class RelocalizationNode {
public:
  RelocalizationNode() : nh_("~")
  {
    loadParameters();
    loadMapAndVocabulary();
    setupROS();
    ROS_INFO("[RelocNode] Phase 3 node ready.");
  }

  void spin() { ros::spin(); }

private:
  // ─── Parameters ────────────────────────────────────────────────────────────
  void loadParameters()
  {
    cfg_.camera.fx = nh_.param("camera_fx", 458.654);
    cfg_.camera.fy = nh_.param("camera_fy", 457.296);
    cfg_.camera.cx = nh_.param("camera_cx", 367.215);
    cfg_.camera.cy = nh_.param("camera_cy", 248.375);

    cfg_.image_width  = nh_.param("image_width",  752);
    cfg_.image_height = nh_.param("image_height", 480);

    cfg_.tau_ret             = nh_.param("tau_ret",             0.012);
    cfg_.search_radius       = nh_.param("search_radius",       40);
    cfg_.hamming_thresh      = nh_.param("hamming_thresh",      60);
    cfg_.pnp_max_reproj_err  = nh_.param("pnp_max_reproj_err",  2.5);
    cfg_.pnp_min_inliers     = nh_.param("pnp_min_inliers",     12);
    cfg_.pnp_max_iter        = nh_.param("pnp_max_iter",        20);
    cfg_.goal_list_length    = nh_.param("goal_list_length",    5);
    cfg_.goal_arrival_dist   = nh_.param("goal_arrival_dist",   0.4);
    cfg_.goal_angle_limit    = nh_.param("goal_angle_limit",    60.0);

    vocabulary_path_ = nh_.param<std::string>("vocabulary_path", "");
    map_path_        = nh_.param<std::string>("map_path",        "");

    if (vocabulary_path_.empty()) {
      ROS_FATAL("[RelocNode] ~vocabulary_path parameter is required!");
      ros::shutdown();
    }
  }

  // ─── Map / vocabulary loading ───────────────────────────────────────────────
  void loadMapAndVocabulary()
  {
    // Load DBoW2 BRIEF vocabulary
    ROS_INFO("[RelocNode] Loading DBoW2 vocabulary from: %s",
             vocabulary_path_.c_str());
    auto vocab = std::make_shared<DBoW2::BriefVocabulary>(vocabulary_path_);
    auto db    = std::make_shared<DBoW2::BriefDatabase>(*vocab, false, 0);

    // Load map keyframes
    // In a full system, Phase 2 publishes the map over a ROS topic or writes
    // it to a file.  Here we support both:
    //   a) Load from file (map_path_ set)
    //   b) Wait for topic /vtr/map_keyframes (implemented via subscriber below)
    std::vector<MapKeyframe::Ptr> keyframes;

    if (!map_path_.empty()) {
      keyframes = loadKeyframesFromFile(map_path_, db);
      ROS_INFO("[RelocNode] Loaded %zu keyframes from file.", keyframes.size());
    } else {
      ROS_INFO("[RelocNode] Waiting for map keyframes on /vtr/map_keyframes ...");
      // Map will be received via the map subscriber; defer reloc init.
    }

    db_       = db;
    if (!keyframes.empty()) {
      reloc_ = std::make_unique<Relocalization>(cfg_, db_, keyframes);
    }
  }

  // Stub: real implementation reads a serialised binary/YAML file written by
  // Phase 2.  Replace with your actual deserialization code.
  std::vector<MapKeyframe::Ptr> loadKeyframesFromFile(
      const std::string& path,
      std::shared_ptr<DBoW2::BriefDatabase>& db)
  {
    (void)path; (void)db;
    ROS_WARN("[RelocNode] loadKeyframesFromFile: stub — integrate Phase 2 serializer.");
    return {};
  }

  // ─── ROS setup ─────────────────────────────────────────────────────────────
  void setupROS()
  {
    // Subscribers
    image_sub_ = nh_.subscribe<sensor_msgs::Image>(
        "/camera/image_raw", 5,
        &RelocalizationNode::imageCallback, this);

    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        "/vtr/odometry", 10,
        &RelocalizationNode::odometryCallback, this);

    // Publishers
    goal_pub_ = nh_.advertise<geometry_msgs::PoseArray>(
        "/vtr/goal_list", 5);

    status_pub_ = nh_.advertise<std_msgs::String>(
        "/vtr/reloc_status", 10);
  }

  // ─── Image callback (main relocalization loop) ───────────────────────────
  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    if (!reloc_) {
      ROS_WARN_THROTTLE(5.0, "[RelocNode] Relocalization not initialized yet.");
      return;
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, "mono8");
    } catch (const cv_bridge::Exception& e) {
      ROS_ERROR("[RelocNode] cv_bridge: %s", e.what());
      return;
    }

    RelocResult result = reloc_->relocalize(cv_ptr->image);

    // Publish status
    std_msgs::String status;
    std::ostringstream oss;
    if (result.success) {
      oss << "OK KF=" << result.kf_id
          << " reproj=" << result.pnp.rms_reproj_error
          << "px inliers=" << result.pnp.num_inliers;
    } else {
      oss << "FAIL";
    }
    status.data = oss.str();
    status_pub_.publish(status);

    // Publish goal list
    publishGoalList(msg->header.stamp);
  }

  // ─── Odometry callback ────────────────────────────────────────────────────
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    if (!reloc_) return;

    // Compute incremental odometry delta since last call
    // Extract current pose from odometry message
    const auto& pos = msg->pose.pose.position;
    const auto& ori = msg->pose.pose.orientation;

    Eigen::Quaterniond q(ori.w, ori.x, ori.y, ori.z);
    Eigen::Vector3d    t(pos.x, pos.y, pos.z);

    SE3d T_current = SE3d::Identity();
    T_current.topLeftCorner<3, 3>()  = q.toRotationMatrix();
    T_current.topRightCorner<3, 1>() = t;

    if (has_prev_odom_) {
      SE3d T_delta = T_prev_odom_.inverse() * T_current;
      reloc_->notifyOdometry(T_delta);
    }

    T_prev_odom_  = T_current;
    has_prev_odom_ = true;
  }

  // ─── Goal list publisher ──────────────────────────────────────────────────
  void publishGoalList(const ros::Time& stamp)
  {
    if (!reloc_) return;

    GoalList gl = reloc_->getGoalList();

    // Publish as PoseArray (positions only — orientation unused by Phase 4)
    geometry_msgs::PoseArray pa;
    pa.header.stamp    = stamp;
    pa.header.frame_id = "base_link";   // goals in robot frame

    for (const auto& goal : gl.goals) {
      geometry_msgs::Pose p;
      p.position.x = goal.x();
      p.position.y = goal.y();
      p.position.z = goal.z();
      p.orientation.w = 1.0;
      pa.poses.push_back(p);
    }

    goal_pub_.publish(pa);
  }

  // ─── Members ───────────────────────────────────────────────────────────────
  ros::NodeHandle nh_;

  Phase3Config  cfg_;
  std::string   vocabulary_path_;
  std::string   map_path_;

  std::shared_ptr<DBoW2::BriefDatabase> db_;
  std::unique_ptr<Relocalization>       reloc_;

  ros::Subscriber image_sub_;
  ros::Subscriber odom_sub_;

  ros::Publisher  goal_pub_;
  ros::Publisher  status_pub_;

  SE3d T_prev_odom_  = SE3d::Identity();
  bool has_prev_odom_ = false;
};

} // namespace vtr

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
  ros::init(argc, argv, "vtr_relocalization_node");
  vtr::RelocalizationNode node;
  node.spin();
  return 0;
}
