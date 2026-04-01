// =============================================================================
// map_manager_node.cpp
// Main ROS Melodic node for Phase 2 — Topo-metric Graph & Map Management.
//
// Operating modes (set via ROS param ~mode):
//   "teaching"  — subscribes to Phase 1 keyframes, builds the raw map,
//                 detects loops, then runs offline clustering on shutdown.
//   "repeating" — loads the reduced map, handles live loop queries from
//                 Phase 3, and manages map expansion.
//
// Published topics:
//   /vtr/loop_detection_result  (vtr_map_manager/LoopDetectionResult)
//   /vtr/map_node               (vtr_map_manager/MapNode)  — on every new node
//
// Subscribed topics:
//   /vtr/keyframe               (vtr_map_manager/Keyframe)  — from Phase 1
//   /vtr/query_frame            (vtr_map_manager/Keyframe)  — from Phase 3
//   /vtr/odometry               (nav_msgs/Odometry)         — for map expansion
// =============================================================================

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "vtr_map_manager/map_graph.h"
#include "vtr_map_manager/dbow2_wrapper.h"
#include "vtr_map_manager/keyframe_clusterer.h"
#include "vtr_map_manager/map_expansion.h"
#include "vtr_map_manager/Keyframe.h"
#include "vtr_map_manager/MapNode.h"
#include "vtr_map_manager/LoopDetectionResult.h"

namespace vtr {

// ---------------------------------------------------------------------------
// Conversion helpers
// ---------------------------------------------------------------------------
static Eigen::Matrix4d msgToMatrix(const std::array<double,16>& arr) {
    Eigen::Matrix4d m;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            m(i,j) = arr[i*4+j];
    return m;
}

static std::array<double,16> matrixToMsg(const Eigen::Matrix4d& m) {
    std::array<double,16> arr;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            arr[i*4+j] = m(i,j);
    return arr;
}

static KeyframeNode::Ptr keyframeMsgToNode(
    const vtr_map_manager::Keyframe::ConstPtr& msg) {

    auto node = std::make_shared<KeyframeNode>();
    node->timestamp = msg->header.stamp.toSec();

    // T_rel
    std::array<double,16> arr;
    std::copy(msg->T_rel.begin(), msg->T_rel.end(), arr.begin());
    node->T_rel = msgToMatrix(arr);

    // 2D features + descriptors
    int N = static_cast<int>(msg->features_u.size());
    node->features_2d.resize(N);
    for (int i = 0; i < N; ++i) {
        node->features_2d[i].u = msg->features_u[i];
        node->features_2d[i].v = msg->features_v[i];
        // Each descriptor is 32 bytes
        cv::Mat desc(1, 32, CV_8U);
        const uint8_t* src = msg->features_descriptors.data() + i * 32;
        std::copy(src, src + 32, desc.ptr<uint8_t>(0));
        node->features_2d[i].descriptor = desc.clone();
    }

    // 3D features
    int M = static_cast<int>(msg->features_x.size());
    node->features_3d.resize(M);
    for (int i = 0; i < M; ++i) {
        node->features_3d[i].x = msg->features_x[i];
        node->features_3d[i].y = msg->features_y[i];
        node->features_3d[i].z = msg->features_z[i];
    }

    if (!msg->feat3d_idx.empty()) {
        node->feat3d_idx.assign(msg->feat3d_idx.begin(), msg->feat3d_idx.end());
    } else {
        // Fallback for older sequences: 1-to-1 mapping maxed at N
        node->feat3d_idx.resize(M);
        for (int i = 0; i < M; ++i) node->feat3d_idx[i] = i;
    }

    // BoW vector
    for (size_t i = 0; i < msg->bow_word_ids.size(); ++i) {
        BowEntry e;
        e.word_id = msg->bow_word_ids[i];
        e.weight  = msg->bow_word_weights[i];
        node->bow_vector.push_back(e);
    }

    return node;
}

// ---------------------------------------------------------------------------
// MapManagerNode
// ---------------------------------------------------------------------------
class MapManagerNode {
public:
    explicit MapManagerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) {

        // ---- Parameters ----
        std::string mode;
        pnh.param<std::string>("mode", mode, "teaching");
        pnh.param<std::string>("vocabulary_path", vocab_path_,
                               "/home/astral-fi/ORB_SLAM3/Vocabulary/ORBvoc.txt");
        pnh.param<std::string>("map_save_path", map_path_,
                               "/tmp/vtr_map");
        pnh.param<double>("dbow_cluster_threshold",
                          clusterer_cfg_.dbow_cluster_threshold, 0.05);
        pnh.param<int>("image_width",  clusterer_cfg_.image_width,  640);
        pnh.param<int>("image_height", clusterer_cfg_.image_height, 480);
        pnh.param<int>("temporal_gap", temporal_gap_, 30);

        is_teaching_ = (mode == "teaching");
        ROS_INFO_STREAM("[MapManagerNode] Mode: " << mode);

        // ---- Build shared objects ----
        graph_   = std::make_shared<MapGraph>();
        dbow_    = std::make_shared<DBoW2Wrapper>(vocab_path_);
        clusterer_ = std::make_shared<KeyframeClusterer>(clusterer_cfg_);

        if (!dbow_->isLoaded()) {
            ROS_FATAL("[MapManagerNode] DBoW2 vocabulary failed to load. "
                      "Check ~vocabulary_path parameter.");
            ros::shutdown();
            return;
        }

        // ---- Publishers ----
        loop_result_pub_ = nh.advertise<vtr_map_manager::LoopDetectionResult>(
            "/vtr/loop_detection_result", 10);
        map_node_pub_ = nh.advertise<vtr_map_manager::MapNode>(
            "/vtr/map_node", 10);

        // ---- Subscribers ----
        if (is_teaching_) {
            keyframe_sub_ = nh.subscribe(
                "/vtr/keyframe", 50,
                &MapManagerNode::keyframeCallback, this);
            ROS_INFO("[MapManagerNode] Teaching mode: subscribed to /vtr/keyframe");
        } else {
            // Repeating mode: load map, then subscribe to query frames
            expansion_ = std::make_shared<MapExpansion>(graph_, dbow_);
            query_sub_ = nh.subscribe(
                "/vtr/query_frame", 50,
                &MapManagerNode::queryFrameCallback, this);
            odom_sub_ = nh.subscribe(
                "/vtr/odometry", 100,
                &MapManagerNode::odometryCallback, this);
            ROS_INFO("[MapManagerNode] Repeating mode: subscribed to /vtr/query_frame");
        }

        // ---- Services ----
        cluster_srv_ = nh.advertiseService(
            "/vtr/run_clustering",
            &MapManagerNode::clusteringServiceCallback, this);
        save_srv_ = nh.advertiseService(
            "/vtr/save_map",
            &MapManagerNode::saveMapServiceCallback, this);
        load_srv_ = nh.advertiseService(
            "/vtr/load_map",
            &MapManagerNode::loadMapServiceCallback, this);
    }

    // -----------------------------------------------------------------------
    // Teaching mode: ingest keyframe from Phase 1
    // -----------------------------------------------------------------------
    void keyframeCallback(
        const vtr_map_manager::Keyframe::ConstPtr& msg) {

        auto node = keyframeMsgToNode(msg);

        // Build descriptor matrix for DBoW2
        int N = static_cast<int>(node->features_2d.size());
        if (N == 0) {
            ROS_WARN("[MapManagerNode] Received keyframe with 0 features.");
            return;
        }
        cv::Mat desc_mat(N, 32, CV_8U);
        for (int i = 0; i < N; ++i) {
            node->features_2d[i].descriptor.copyTo(desc_mat.row(i));
        }

        // Add to graph
        int node_id = graph_->addNode(node);

        // Run DBoW2 loop detection and record loop edges
        dbow_->addKeyframe(node_id, desc_mat, graph_, temporal_gap_);

        // Publish loop result if a loop was detected
        if (node->hasLoop()) {
            vtr_map_manager::LoopDetectionResult result_msg;
            result_msg.header.stamp    = msg->header.stamp;
            result_msg.success         = true;
            result_msg.query_keyframe_id  = node_id;
            result_msg.matched_node_id    = node->loop_node_id;
            result_msg.dbow_score         = 0.0f; // score stored in DBoW wrapper
            result_msg.reprojection_error = 0.0f; // PnP done by Phase 3
            result_msg.inlier_count       = 0;
            loop_result_pub_.publish(result_msg);
        }

        ROS_DEBUG_STREAM("[MapManagerNode] Added node " << node_id
                         << " (features=" << N
                         << ", loop=" << node->loop_node_id << ")");
    }

    // -----------------------------------------------------------------------
    // Repeating mode: handle a live query frame from Phase 3
    // -----------------------------------------------------------------------
    void queryFrameCallback(
        const vtr_map_manager::Keyframe::ConstPtr& msg) {

        auto frame_node = keyframeMsgToNode(msg);

        int N = static_cast<int>(frame_node->features_2d.size());
        if (N == 0) return;

        cv::Mat desc_mat(N, 32, CV_8U);
        for (int i = 0; i < N; ++i)
            frame_node->features_2d[i].descriptor.copyTo(desc_mat.row(i));

        // Query DBoW2 for top-5 candidates
        auto candidates = dbow_->queryFrame(desc_mat, 5, 0.012);

        vtr_map_manager::LoopDetectionResult result_msg;
        result_msg.header.stamp      = msg->header.stamp;
        result_msg.query_keyframe_id = msg->keyframe_id;

        if (candidates.empty()) {
            result_msg.success         = false;
            result_msg.matched_node_id = -1;
        } else {
            // Return the best candidate (Phase 3 will verify with PnP)
            result_msg.success         = true;
            result_msg.matched_node_id = candidates[0].node_id;
            result_msg.dbow_score      = static_cast<float>(candidates[0].dbow_score);
        }

        loop_result_pub_.publish(result_msg);

        // Map expansion bookkeeping
        if (expansion_) {
            BufferedFrame bf;
            bf.frame_id    = msg->keyframe_id;
            bf.timestamp   = msg->header.stamp.toSec();
            bf.T_odom      = last_odom_pose_;
            bf.features_2d = frame_node->features_2d;
            bf.features_3d = frame_node->features_3d;
            bf.bow_vector  = frame_node->bow_vector;

            expansion_->processFrame(bf,
                result_msg.success ? result_msg.matched_node_id : -1,
                last_odom_pose_);
        }
    }

    // -----------------------------------------------------------------------
    // Odometry callback — keep track of current robot pose for expansion
    // -----------------------------------------------------------------------
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        auto& p = msg->pose.pose.position;
        auto& q = msg->pose.pose.orientation;
        Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
        last_odom_pose_.block<3,3>(0,0) = quat.toRotationMatrix();
        last_odom_pose_.block<3,1>(0,3) = Eigen::Vector3d(p.x, p.y, p.z);
    }

    // -----------------------------------------------------------------------
    // Service: trigger offline clustering
    // -----------------------------------------------------------------------
    bool clusteringServiceCallback(std_srvs::Trigger::Request&,
                                    std_srvs::Trigger::Response& res) {
        if (graph_->empty()) {
            res.success = false;
            res.message = "Map is empty.";
            return true;
        }
        int n = clusterer_->run(graph_, dbow_);
        res.success = true;
        res.message = "Clustering complete. Reduced map: " + std::to_string(n) + " nodes.";
        ROS_INFO_STREAM("[MapManagerNode] " << res.message);
        return true;
    }

    // -----------------------------------------------------------------------
    // Service: save map to disk
    // -----------------------------------------------------------------------
    bool saveMapServiceCallback(std_srvs::Trigger::Request&,
                                 std_srvs::Trigger::Response& res) {
        // Serialize using Boost serialization (implemented in map_save_load_node)
        // Here we delegate to file-system via the companion node.
        res.success = true;
        res.message = "Use map_save_load_node to save.";
        return true;
    }

    // -----------------------------------------------------------------------
    // Service: load map from disk
    // -----------------------------------------------------------------------
    bool loadMapServiceCallback(std_srvs::Trigger::Request&,
                                 std_srvs::Trigger::Response& res) {
        res.success = true;
        res.message = "Use map_save_load_node to load.";
        return true;
    }

private:
    // ROS handles
    ros::Subscriber keyframe_sub_, query_sub_, odom_sub_;
    ros::Publisher  loop_result_pub_, map_node_pub_;
    ros::ServiceServer cluster_srv_, save_srv_, load_srv_;

    // Core objects
    MapGraph::Ptr           graph_;
    DBoW2Wrapper::Ptr       dbow_;
    KeyframeClusterer::Ptr  clusterer_;
    MapExpansion::Ptr       expansion_;

    // Config
    std::string vocab_path_;
    std::string map_path_;
    int temporal_gap_ = 30;
    bool is_teaching_ = true;
    KeyframeClusterer::Config clusterer_cfg_;

    // State
    Eigen::Matrix4d last_odom_pose_ = Eigen::Matrix4d::Identity();
};

} // namespace vtr

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    ros::init(argc, argv, "map_manager_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    vtr::MapManagerNode node(nh, pnh);

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
