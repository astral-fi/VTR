// =============================================================================
// map_expansion.cpp
// Live map expansion during the repeating phase — Section III-C-2.
// =============================================================================

#include "vtr_map_manager/map_expansion.h"
#include <ros/ros.h>
#include <cmath>

namespace vtr {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
MapExpansion::MapExpansion(MapGraph::Ptr graph, DBoW2Wrapper::Ptr dbow)
    : graph_(graph), dbow_(dbow) {}

// ---------------------------------------------------------------------------
// processFrame
// ---------------------------------------------------------------------------
void MapExpansion::processFrame(const BufferedFrame& frame,
                                 int matched_node_id,
                                 const Eigen::Matrix4d& T_odom_current) {

    if (matched_node_id < 0) {
        // No match this frame — buffer it for possible expansion later
        pending_frames_.push_back(frame);
        return;
    }

    // ---- Successful match ----
    if (last_matched_node_id_ < 0) {
        // This is the very first match — just record it and clear any buffer
        last_matched_node_id_   = matched_node_id;
        T_odom_at_last_match_   = T_odom_current;
        pending_frames_.clear();
        return;
    }

    // We have two consecutive successful matches:
    //   I_k  matched K_i (= last_matched_node_id_)
    //   I_{k+t} matched K_j (= matched_node_id)
    //
    // Process all frames buffered between them as expansion candidates.
    int Ki = last_matched_node_id_;
    int Kj = matched_node_id;

    ROS_DEBUG_STREAM("[MapExpansion] Processing " << pending_frames_.size()
                     << " buffered frames between nodes "
                     << Ki << " and " << Kj);

    for (auto& bf : pending_frames_) {
        // Compute odometry offset of this frame from the last match
        // T_last_match^{frame} = inv(T_odom_at_last_match_) * T_odom_frame
        Eigen::Matrix4d T_from_last = T_odom_at_last_match_.inverse() * bf.T_odom;

        // Find which map node is spatially nearest to this frame's pose
        int nearest = findNearestNode(T_from_last, Ki);
        if (nearest < 0) continue;

        // Build an ExpansionFrame and attach it to the map
        ExpansionFrame ef;
        ef.frame_id          = next_frame_id_++;
        ef.nearest_node_id   = nearest;

        // T from the nearest node's frame to this expansion frame
        // (approximate using odometry chain)
        auto nearest_node = graph_->getNode(nearest);
        if (!nearest_node) continue;

        ef.T_node_to_frame = T_from_last;  // simplified: use odom offset
        ef.features_2d     = bf.features_2d;
        ef.features_3d     = bf.features_3d;
        ef.bow_vector      = bf.bow_vector;
        ef.image           = bf.image;
        ef.timestamp       = bf.timestamp;

        graph_->attachExpansionFrame(ef, nearest);
        total_expansions_++;

        ROS_DEBUG_STREAM("[MapExpansion] Frame " << ef.frame_id
                         << " attached to node " << nearest);
    }

    // Reset state for next interval
    pending_frames_.clear();
    last_matched_node_id_ = matched_node_id;
    T_odom_at_last_match_ = T_odom_current;
}

// ---------------------------------------------------------------------------
// findNearestNode
// Walk the linked list starting from search_start_node_id and find the
// node whose accumulated pose is closest (in Euclidean distance) to the
// query transform T_from_last_match.
// ---------------------------------------------------------------------------
int MapExpansion::findNearestNode(const Eigen::Matrix4d& T_from_last,
                                   int search_start_node_id) const {

    Eigen::Vector3d query_t = T_from_last.block<3,1>(0,3);

    int best_id   = -1;
    double best_d = std::numeric_limits<double>::max();

    // Walk up to 50 nodes from the search start
    int cur = search_start_node_id;
    Eigen::Matrix4d T_accum = Eigen::Matrix4d::Identity();

    for (int hop = 0; hop < 50; ++hop) {
        auto node = graph_->getNode(cur);
        if (!node) break;

        // Position of this node relative to the last-match node (in odom frame)
        Eigen::Vector3d node_t = T_accum.block<3,1>(0,3);
        double dist = (query_t - node_t).norm();

        if (dist < best_d) {
            best_d  = dist;
            best_id = cur;
        }

        int nxt = node->next_node_id;
        if (nxt == -1) break;

        auto nxt_node = graph_->getNode(nxt);
        if (!nxt_node) break;

        T_accum = T_accum * nxt_node->T_rel;
        cur = nxt;
    }

    return best_id;
}

} // namespace vtr
