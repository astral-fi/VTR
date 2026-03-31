// =============================================================================
// map_visualizer_node.cpp
// Publishes visualization_msgs/MarkerArray to RViz showing:
//   - Map nodes as spheres (green = normal, yellow = loop node)
//   - Temporal edges as blue lines
//   - Loop edges as red lines
//   - Expansion frames as small orange dots
// =============================================================================

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Core>
#include <unordered_map>

#include "vtr_map_manager/MapNode.h"

namespace vtr {

class MapVisualizerNode {
public:
    MapVisualizerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
        pnh.param<std::string>("frame_id", frame_id_, "map");
        pnh.param<double>("publish_rate", publish_rate_, 2.0);

        marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
            "/vtr/map_markers", 1, /*latch=*/true);

        map_node_sub_ = nh.subscribe(
            "/vtr/map_node", 200,
            &MapVisualizerNode::mapNodeCallback, this);

        timer_ = nh.createTimer(
            ros::Duration(1.0 / publish_rate_),
            &MapVisualizerNode::publishCallback, this);

        ROS_INFO("[MapVisualizerNode] Initialized.");
    }

private:
    // Store received nodes for visualization
    struct NodeViz {
        int node_id, prev_id, next_id, loop_id;
        Eigen::Vector3d position;  // accumulated translation from head
    };

    void mapNodeCallback(const vtr_map_manager::MapNode::ConstPtr& msg) {
        NodeViz viz;
        viz.node_id = msg->node_id;
        viz.prev_id = msg->prev_node_id;
        viz.next_id = msg->next_node_id;
        viz.loop_id = msg->loop_node_id;

        // Extract translation from T_to_prev (first 3 elements of 4th column)
        viz.position = Eigen::Vector3d(
            msg->T_to_prev[3],
            msg->T_to_prev[7],
            msg->T_to_prev[11]);

        nodes_[msg->node_id] = viz;
    }

    void publishCallback(const ros::TimerEvent&) {
        if (nodes_.empty()) return;

        // Accumulate absolute positions by walking linked list from head
        // Find head (prev_id == -1)
        int head_id = -1;
        for (auto& kv : nodes_) {
            if (kv.second.prev_id == -1) { head_id = kv.first; break; }
        }
        if (head_id == -1) return;

        // Walk list and accumulate translations
        std::unordered_map<int, Eigen::Vector3d> abs_pos;
        Eigen::Vector3d cur_pos(0,0,0);
        int cur = head_id;
        while (cur != -1) {
            auto it = nodes_.find(cur);
            if (it == nodes_.end()) break;
            abs_pos[cur] = cur_pos;
            int nxt = it->second.next_id;
            if (nxt != -1) {
                auto nxt_it = nodes_.find(nxt);
                if (nxt_it != nodes_.end())
                    cur_pos += nxt_it->second.position;
            }
            cur = nxt;
        }

        visualization_msgs::MarkerArray ma;
        ros::Time now = ros::Time::now();

        // --- Node spheres ---
        {
            visualization_msgs::Marker m;
            m.header.frame_id = frame_id_;
            m.header.stamp    = now;
            m.ns              = "nodes";
            m.type            = visualization_msgs::Marker::SPHERE_LIST;
            m.action          = visualization_msgs::Marker::ADD;
            m.id              = 0;
            m.scale.x = m.scale.y = m.scale.z = 0.12;
            m.color.a = 1.0; m.color.r = 0.1; m.color.g = 0.8; m.color.b = 0.2;
            m.pose.orientation.w = 1.0;
            for (auto& kv : abs_pos) {
                geometry_msgs::Point p;
                p.x = kv.second.x(); p.y = kv.second.y(); p.z = kv.second.z();
                m.points.push_back(p);
            }
            ma.markers.push_back(m);
        }

        // --- Temporal edges (blue lines) ---
        {
            visualization_msgs::Marker m;
            m.header.frame_id = frame_id_;
            m.header.stamp    = now;
            m.ns              = "temporal_edges";
            m.type            = visualization_msgs::Marker::LINE_LIST;
            m.action          = visualization_msgs::Marker::ADD;
            m.id              = 1;
            m.scale.x         = 0.02;
            m.color.a = 0.8; m.color.r = 0.1; m.color.g = 0.3; m.color.b = 0.9;
            m.pose.orientation.w = 1.0;
            for (auto& kv : nodes_) {
                int nxt = kv.second.next_id;
                if (nxt == -1) continue;
                auto it_a = abs_pos.find(kv.first);
                auto it_b = abs_pos.find(nxt);
                if (it_a == abs_pos.end() || it_b == abs_pos.end()) continue;
                geometry_msgs::Point pa, pb;
                pa.x=it_a->second.x(); pa.y=it_a->second.y(); pa.z=it_a->second.z();
                pb.x=it_b->second.x(); pb.y=it_b->second.y(); pb.z=it_b->second.z();
                m.points.push_back(pa); m.points.push_back(pb);
            }
            ma.markers.push_back(m);
        }

        // --- Loop edges (red lines) ---
        {
            visualization_msgs::Marker m;
            m.header.frame_id = frame_id_;
            m.header.stamp    = now;
            m.ns              = "loop_edges";
            m.type            = visualization_msgs::Marker::LINE_LIST;
            m.action          = visualization_msgs::Marker::ADD;
            m.id              = 2;
            m.scale.x         = 0.025;
            m.color.a = 0.9; m.color.r = 0.9; m.color.g = 0.1; m.color.b = 0.1;
            m.pose.orientation.w = 1.0;
            for (auto& kv : nodes_) {
                int lp = kv.second.loop_id;
                if (lp == -1) continue;
                auto it_a = abs_pos.find(kv.first);
                auto it_b = abs_pos.find(lp);
                if (it_a == abs_pos.end() || it_b == abs_pos.end()) continue;
                geometry_msgs::Point pa, pb;
                pa.x=it_a->second.x(); pa.y=it_a->second.y(); pa.z=it_a->second.z();
                pb.x=it_b->second.x(); pb.y=it_b->second.y(); pb.z=it_b->second.z();
                m.points.push_back(pa); m.points.push_back(pb);
            }
            ma.markers.push_back(m);
        }

        // --- Node ID text labels ---
        {
            int label_id = 10;
            for (auto& kv : abs_pos) {
                visualization_msgs::Marker m;
                m.header.frame_id = frame_id_;
                m.header.stamp    = now;
                m.ns              = "node_labels";
                m.id              = label_id++;
                m.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
                m.action          = visualization_msgs::Marker::ADD;
                m.pose.position.x = kv.second.x();
                m.pose.position.y = kv.second.y();
                m.pose.position.z = kv.second.z() + 0.15;
                m.pose.orientation.w = 1.0;
                m.scale.z         = 0.08;
                m.color.a = 1.0; m.color.r = 1.0; m.color.g = 1.0; m.color.b = 1.0;
                m.text            = std::to_string(kv.first);
                ma.markers.push_back(m);
            }
        }

        marker_pub_.publish(ma);
    }

    ros::Subscriber map_node_sub_;
    ros::Publisher  marker_pub_;
    ros::Timer      timer_;

    std::string frame_id_;
    double      publish_rate_;

    std::unordered_map<int, NodeViz> nodes_;
};

} // namespace vtr

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_visualizer_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    vtr::MapVisualizerNode node(nh, pnh);
    ros::spin();
    return 0;
}
