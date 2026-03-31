// =============================================================================
// map_save_load_node.cpp
// Saves the reduced topo-metric map to disk and reloads it.
//
// File layout under <map_path>/:
//   map.bin        -- serialized MapGraph nodes (Boost binary archive)
//   dbow_db.bin    -- serialized DBoW2 database (DBoW2 native format)
//   images/        -- one PNG per map node: node_<id>.png
//
// Save workflow (call after teaching + clustering are complete):
//   rosservice call /vtr/save_map  (triggers saveMap() via service)
//
// Load workflow (start before map_manager_node in repeating mode):
//   rosrun vtr_map_manager map_save_load_node _op:=load _map_path:=/path/map
// After loading, publishes all MapNode messages on /vtr/map_node and stays up.
// =============================================================================

#include <ros/ros.h>
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/string.hpp>

#include "vtr_map_manager/map_graph.h"
#include "vtr_map_manager/dbow2_wrapper.h"
#include "vtr_map_manager/MapNode.h"

namespace fs = boost::filesystem;

// ---------------------------------------------------------------------------
// Boost serialization for Eigen::Matrix4d
// ---------------------------------------------------------------------------
namespace boost { namespace serialization {
template<class Archive>
void serialize(Archive& ar, Eigen::Matrix4d& m, unsigned int) {
    for (int i = 0; i < 16; ++i) ar & m.data()[i];
}
template<class Archive>
void serialize(Archive& ar, vtr::Feature2D& f, unsigned int) {
    ar & f.u & f.v;
    std::vector<uint8_t> buf;
    if (Archive::is_saving::value && !f.descriptor.empty())
        buf.assign(f.descriptor.datastart, f.descriptor.dataend);
    ar & buf;
    if (Archive::is_loading::value) {
        f.descriptor = cv::Mat(1, 32, CV_8U);
        if (!buf.empty())
            std::copy(buf.begin(), buf.end(), f.descriptor.ptr<uint8_t>(0));
    }
}
template<class Archive>
void serialize(Archive& ar, vtr::Feature3D& f, unsigned int) {
    ar & f.x & f.y & f.z;
}
template<class Archive>
void serialize(Archive& ar, vtr::BowEntry& e, unsigned int) {
    ar & e.word_id & e.weight;
}
template<class Archive>
void serialize(Archive& ar, vtr::KeyframeNode& n, unsigned int) {
    ar & n.node_id & n.timestamp;
    ar & n.T_rel & n.T_to_next;
    ar & n.has_skip;
    ar & n.T_loop & n.loop_node_id;
    ar & n.prev_node_id & n.next_node_id;
    ar & n.features_2d & n.features_3d & n.bow_vector;
}
}} // namespace boost::serialization

// ---------------------------------------------------------------------------
// saveMapToDisk
// ---------------------------------------------------------------------------
bool saveMapToDisk(const vtr::MapGraph::Ptr& graph,
                   const vtr::DBoW2Wrapper::Ptr& dbow,
                   const std::string& map_path) {
    fs::path root(map_path);
    fs::create_directories(root / "images");

    ROS_INFO_STREAM("[MapSaveLoad] Saving " << graph->size()
                    << " nodes to " << map_path);

    // Serialize nodes
    {
        std::ofstream ofs((root / "map.bin").string(),
                          std::ios::binary | std::ios::trunc);
        if (!ofs) { ROS_ERROR("[MapSaveLoad] Cannot open map.bin for write."); return false; }
        boost::archive::binary_oarchive oa(ofs);
        auto ids = graph->getAllNodeIds();
        int sz = static_cast<int>(ids.size());
        oa & sz;
        for (int id : ids) {
            auto node = graph->getNode(id);
            if (!node) continue;
            oa & *node;
            if (!node->image.empty()) {
                std::string img = (root / "images" /
                    ("node_" + std::to_string(id) + ".png")).string();
                cv::imwrite(img, node->image);
            }
        }
    }

    // Serialize DBoW2 database
    dbow->saveDatabase((root / "dbow_db.bin").string());
    ROS_INFO("[MapSaveLoad] Save complete.");
    return true;
}

// ---------------------------------------------------------------------------
// loadMapFromDisk
// ---------------------------------------------------------------------------
bool loadMapFromDisk(vtr::MapGraph::Ptr& graph,
                     vtr::DBoW2Wrapper::Ptr& dbow,
                     const std::string& map_path,
                     ros::Publisher& pub) {
    fs::path root(map_path);
    if (!fs::exists(root / "map.bin")) {
        ROS_ERROR_STREAM("[MapSaveLoad] map.bin not found in " << map_path);
        return false;
    }

    std::vector<vtr::KeyframeNode::Ptr> nodes;
    {
        std::ifstream ifs((root / "map.bin").string(), std::ios::binary);
        if (!ifs) { ROS_ERROR("[MapSaveLoad] Cannot open map.bin for read."); return false; }
        boost::archive::binary_iarchive ia(ifs);
        int sz = 0; ia & sz;
        for (int i = 0; i < sz; ++i) {
            auto n = std::make_shared<vtr::KeyframeNode>();
            ia & *n;
            std::string img = (root / "images" /
                ("node_" + std::to_string(n->node_id) + ".png")).string();
            if (fs::exists(img)) n->image = cv::imread(img, cv::IMREAD_GRAYSCALE);
            nodes.push_back(n);
        }
    }

    graph->applyClusteredMap(nodes);

    std::string dbow_path = (root / "dbow_db.bin").string();
    if (fs::exists(dbow_path)) dbow->loadDatabase(dbow_path);
    else ROS_WARN("[MapSaveLoad] dbow_db.bin not found — queries will fail.");

    ROS_INFO_STREAM("[MapSaveLoad] Loaded " << graph->size() << " nodes. Publishing...");

    ros::Duration(0.5).sleep();
    for (auto& node : nodes) {
        vtr_map_manager::MapNode msg;
        msg.header.stamp = ros::Time::now();
        msg.node_id      = node->node_id;
        msg.prev_node_id = node->prev_node_id;
        msg.next_node_id = node->next_node_id;
        msg.loop_node_id = node->loop_node_id;
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c) {
                msg.T_to_prev[r*4+c] = node->T_rel(r, c);
                msg.T_to_next[r*4+c] = node->T_to_next(r, c);
                msg.T_to_loop[r*4+c] = node->T_loop(r, c);
            }
        for (auto& f : node->features_2d) {
            msg.features_u.push_back(f.u);
            msg.features_v.push_back(f.v);
            if (!f.descriptor.empty())
                for (int b = 0; b < 32; ++b)
                    msg.features_descriptors.push_back(f.descriptor.at<uint8_t>(0, b));
        }
        for (auto& f : node->features_3d) {
            msg.features_x.push_back(f.x);
            msg.features_y.push_back(f.y);
            msg.features_z.push_back(f.z);
        }
        for (auto& e : node->bow_vector) {
            msg.bow_word_ids.push_back(e.word_id);
            msg.bow_word_weights.push_back(e.weight);
        }
        pub.publish(msg);
        ros::Duration(0.002).sleep();
    }
    ROS_INFO("[MapSaveLoad] All map nodes published.");
    return true;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    ros::init(argc, argv, "map_save_load_node");
    ros::NodeHandle nh, pnh("~");

    std::string op, map_path, vocab_path;
    pnh.param<std::string>("op",               op,         "load");
    pnh.param<std::string>("map_path",         map_path,   "/tmp/vtr_map");
    pnh.param<std::string>("vocabulary_path",  vocab_path,
                           "/opt/vtr/vocab/brief_k10L6.voc.gz");

    auto graph = std::make_shared<vtr::MapGraph>();
    auto dbow  = std::make_shared<vtr::DBoW2Wrapper>(vocab_path);
    auto pub   = nh.advertise<vtr_map_manager::MapNode>("/vtr/map_node", 200, true);

    if (op == "load") {
        if (!loadMapFromDisk(graph, dbow, map_path, pub)) return 1;
    } else if (op == "save") {
        ROS_WARN("[MapSaveLoad] Save op: graph is empty here. "
                 "Trigger save via /vtr/save_map service on map_manager_node.");
    } else {
        ROS_ERROR_STREAM("[MapSaveLoad] Unknown op: " << op);
        return 1;
    }

    ros::spin();
    return 0;
}
