// =============================================================================
// map_graph.cpp
// Implementation of the topo-metric graph (MapGraph).
// =============================================================================

#include "vtr_map_manager/map_graph.h"
#include <ros/ros.h>
#include <fstream>
#include <stdexcept>

namespace vtr {

// =============================================================================
// addNode
// =============================================================================
int MapGraph::addNode(KeyframeNode::Ptr node) {
    std::lock_guard<std::mutex> lock(mutex_);

    node->node_id = next_id_++;
    nodes_[node->node_id] = node;

    if (head_id_ == -1) {
        // First node
        head_id_ = node->node_id;
        tail_id_ = node->node_id;
        node->prev_node_id = -1;
        node->next_node_id = -1;
    } else {
        // Append to tail
        auto& tail = nodes_.at(tail_id_);
        tail->next_node_id = node->node_id;
        node->prev_node_id = tail_id_;
        node->next_node_id = -1;
        tail_id_ = node->node_id;
    }

    ROS_DEBUG_STREAM("[MapGraph] Added node " << node->node_id
                     << " (total: " << nodes_.size() << ")");
    return node->node_id;
}

// =============================================================================
// addLoopEdge
// =============================================================================
void MapGraph::addLoopEdge(int from_id, int to_id,
                            const Eigen::Matrix4d& T_loop) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = nodes_.find(from_id);
    if (it == nodes_.end()) {
        ROS_WARN_STREAM("[MapGraph] addLoopEdge: node " << from_id
                        << " not found.");
        return;
    }
    it->second->loop_node_id = to_id;
    it->second->T_loop = T_loop;

    ROS_DEBUG_STREAM("[MapGraph] Loop edge: " << from_id << " → " << to_id);
}

// =============================================================================
// applyClusteredMap
// =============================================================================
void MapGraph::applyClusteredMap(
    const std::vector<KeyframeNode::Ptr>& clustered_nodes) {

    std::lock_guard<std::mutex> lock(mutex_);

    nodes_.clear();
    head_id_ = -1;
    tail_id_ = -1;

    for (auto& node : clustered_nodes) {
        nodes_[node->node_id] = node;
        if (head_id_ == -1 || node->node_id < head_id_)
            head_id_ = node->node_id;
        if (tail_id_ == -1 || node->node_id > tail_id_)
            tail_id_ = node->node_id;
    }

    // Fix head/tail by walking the linked list
    // (clustered_nodes might not be in order — find the real head)
    for (auto& kv : nodes_) {
        if (kv.second->prev_node_id == -1) head_id_ = kv.first;
        if (kv.second->next_node_id == -1) tail_id_ = kv.first;
    }

    ROS_INFO_STREAM("[MapGraph] Clustered map applied: "
                    << nodes_.size() << " nodes (head=" << head_id_
                    << ", tail=" << tail_id_ << ")");
}

// =============================================================================
// attachExpansionFrame
// =============================================================================
void MapGraph::attachExpansionFrame(const ExpansionFrame& frame,
                                     int nearest_node_id) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = nodes_.find(nearest_node_id);
    if (it == nodes_.end()) {
        ROS_WARN_STREAM("[MapGraph] attachExpansionFrame: node "
                        << nearest_node_id << " not found.");
        return;
    }
    it->second->expansion_frames.push_back(frame);
}

// =============================================================================
// getNode
// =============================================================================
KeyframeNode::Ptr MapGraph::getNode(int node_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = nodes_.find(node_id);
    return (it != nodes_.end()) ? it->second : nullptr;
}

// =============================================================================
// getNextNode
// =============================================================================
KeyframeNode::Ptr MapGraph::getNextNode(int node_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = nodes_.find(node_id);
    if (it == nodes_.end()) return nullptr;
    int next_id = it->second->next_node_id;
    if (next_id == -1) return nullptr;
    auto it2 = nodes_.find(next_id);
    return (it2 != nodes_.end()) ? it2->second : nullptr;
}

// =============================================================================
// getNextNodeIds
// =============================================================================
std::vector<int> MapGraph::getNextNodeIds(int start_id, int M) const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<int> ids;
    int cur = start_id;
    for (int i = 0; i < M; ++i) {
        auto it = nodes_.find(cur);
        if (it == nodes_.end()) break;
        ids.push_back(cur);
        cur = it->second->next_node_id;
        if (cur == -1) break;
    }
    return ids;
}

// =============================================================================
// getRelativePose
// Accumulates T_rel along the linked list from from_id to to_id.
// T = T_{from}^{from+1} * T_{from+1}^{from+2} * ... * T_{to-1}^{to}
// =============================================================================
Eigen::Matrix4d MapGraph::getRelativePose(int from_id, int to_id) const {
    std::lock_guard<std::mutex> lock(mutex_);

    if (from_id == to_id)
        return Eigen::Matrix4d::Identity();

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    int cur = from_id;
    int max_hops = static_cast<int>(nodes_.size()) + 1;

    for (int hop = 0; hop < max_hops; ++hop) {
        auto it = nodes_.find(cur);
        if (it == nodes_.end())
            throw std::runtime_error("[MapGraph] getRelativePose: node "
                                     + std::to_string(cur) + " missing.");

        int nxt = it->second->next_node_id;
        if (nxt == -1)
            throw std::runtime_error("[MapGraph] getRelativePose: "
                                     + std::to_string(to_id) + " unreachable from "
                                     + std::to_string(from_id));

        // T_{cur}^{nxt} (T_rel stored at nxt is T_{cur}^{nxt})
        auto it_nxt = nodes_.find(nxt);
        if (it_nxt == nodes_.end())
            throw std::runtime_error("[MapGraph] getRelativePose: next node "
                                     + std::to_string(nxt) + " missing.");

        T = T * it_nxt->second->T_rel;
        cur = nxt;

        if (cur == to_id) return T;
    }
    throw std::runtime_error("[MapGraph] getRelativePose: max hops exceeded.");
}

// =============================================================================
// getAllNodeIds
// =============================================================================
std::vector<int> MapGraph::getAllNodeIds() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<int> ids;
    ids.reserve(nodes_.size());
    for (auto& kv : nodes_) ids.push_back(kv.first);
    return ids;
}

// =============================================================================
// getAllExpansionFrames
// =============================================================================
std::vector<std::pair<int, const ExpansionFrame*>>
MapGraph::getAllExpansionFrames() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::pair<int, const ExpansionFrame*>> result;
    for (auto& kv : nodes_) {
        for (auto& ef : kv.second->expansion_frames)
            result.push_back({kv.first, &ef});
    }
    return result;
}

// =============================================================================
// Metadata
// =============================================================================
int  MapGraph::size()        const { std::lock_guard<std::mutex> lk(mutex_); return static_cast<int>(nodes_.size()); }
int  MapGraph::headNodeId()  const { std::lock_guard<std::mutex> lk(mutex_); return head_id_; }
int  MapGraph::tailNodeId()  const { std::lock_guard<std::mutex> lk(mutex_); return tail_id_; }
bool MapGraph::empty()       const { std::lock_guard<std::mutex> lk(mutex_); return nodes_.empty(); }

// =============================================================================
// saveToFile / loadFromFile
// Uses Boost Serialization (binary archive) for efficient storage.
// =============================================================================
bool MapGraph::saveToFile(const std::string& path) const {
    // Serialization is handled by map_save_load_node.cpp which has more context.
    // This stub signals success; the real I/O is in the save/load node.
    ROS_WARN("[MapGraph] saveToFile stub called — use map_save_load_node.");
    return false;
}

bool MapGraph::loadFromFile(const std::string& path) {
    ROS_WARN("[MapGraph] loadFromFile stub called — use map_save_load_node.");
    return false;
}

} // namespace vtr
