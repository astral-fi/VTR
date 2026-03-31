#pragma once
// =============================================================================
// map_graph.h
// The topo-metric graph (M_bar) as described in Eq.(4) of the paper:
//   M_bar = { [K_i, T̂_i^j] , i,j = 1,...,N }
//
// Key design decisions:
//   - Nodes stored in a flat unordered_map keyed by node_id for O(1) access.
//   - A doubly-linked list order is maintained via prev/next_node_id fields.
//   - Only relative poses are stored; global poses are NEVER computed.
//   - Thread-safe for concurrent read from Phase 3 and write from Phase 2.
// =============================================================================

#include "vtr_map_manager/keyframe_node.h"
#include <unordered_map>
#include <vector>
#include <mutex>
#include <Eigen/Core>

namespace vtr {

class MapGraph {
public:
    using Ptr = std::shared_ptr<MapGraph>;

    MapGraph() = default;
    ~MapGraph() = default;

    // -------------------------------------------------------------------------
    // Teaching Phase API
    // -------------------------------------------------------------------------

    // Append a new keyframe to the tail of the linked list.
    // Returns the assigned node_id.
    int addNode(KeyframeNode::Ptr node);

    // Record a loop-closure edge between two existing nodes.
    // Stores T_loop inside node[from_id].
    void addLoopEdge(int from_id, int to_id, const Eigen::Matrix4d& T_loop);

    // -------------------------------------------------------------------------
    // Clustering (called after teaching, before repeating)
    // -------------------------------------------------------------------------

    // Replace the internal linked list with the clustered (reduced) version.
    // The old raw nodes are discarded; only the representative nodes remain.
    void applyClusteredMap(const std::vector<KeyframeNode::Ptr>& clustered_nodes);

    // -------------------------------------------------------------------------
    // Repeating Phase API
    // -------------------------------------------------------------------------

    // Attach an expansion frame to the nearest keyframe.
    // Does NOT change the linked-list topology.
    void attachExpansionFrame(const ExpansionFrame& frame, int nearest_node_id);

    // -------------------------------------------------------------------------
    // Query API (used by Phase 3)
    // -------------------------------------------------------------------------

    // Retrieve a node by id.  Returns nullptr if not found.
    KeyframeNode::Ptr getNode(int node_id) const;

    // Returns the next node in the linked list after node_id.
    // Returns nullptr at the tail.
    KeyframeNode::Ptr getNextNode(int node_id) const;

    // Traverse the list from start_id and collect up to M node ids.
    std::vector<int> getNextNodeIds(int start_id, int M) const;

    // Accumulate relative transforms from node a to node b by walking the list.
    // Throws if b is not reachable from a.
    Eigen::Matrix4d getRelativePose(int from_id, int to_id) const;

    // Return all node ids (raw or clustered) for loop query iteration.
    std::vector<int> getAllNodeIds() const;

    // Return all expansion frames across all nodes (for DBoW2 query augmentation)
    std::vector<std::pair<int, const ExpansionFrame*>> getAllExpansionFrames() const;

    // Map metadata
    int  size() const;           // number of active nodes
    int  headNodeId() const;     // id of the first node
    int  tailNodeId() const;     // id of the last node
    bool empty() const;

    // -------------------------------------------------------------------------
    // Serialization (used by map_save_load_node)
    // -------------------------------------------------------------------------
    bool saveToFile(const std::string& path) const;
    bool loadFromFile(const std::string& path);

private:
    mutable std::mutex mutex_;

    std::unordered_map<int, KeyframeNode::Ptr> nodes_;
    int head_id_  = -1;
    int tail_id_  = -1;
    int next_id_  =  0;   // auto-increment counter for new nodes
};

} // namespace vtr
