#pragma once
// =============================================================================
// map_expansion.h
// Live map expansion during the repeating phase — Section III-C-2, Eq.(14).
//
// Triggered when two consecutive successful loop matches occur:
//   I_k matches K_i  AND  I_{k+t} matches K_j.
//
// Each live frame I between I_k and I_{k+t} that failed to find a loop is:
//   1. Located on the map via odometry dead-reckoning.
//   2. Attached to its nearest keyframe as an ExpansionFrame.
//   3. Included in subsequent DBoW2 queries (not as a keyframe node).
//
// Crucially, expansion does NOT:
//   - Insert new nodes into the linked list topology.
//   - Create new goal management responsibilities.
//   - Trigger re-clustering.
// =============================================================================

#include "vtr_map_manager/map_graph.h"
#include "vtr_map_manager/dbow2_wrapper.h"
#include "vtr_map_manager/keyframe_node.h"
#include <Eigen/Core>
#include <vector>
#include <memory>

namespace vtr {

// A live frame buffered between two successful matches.
struct BufferedFrame {
    int    frame_id;
    double timestamp;
    Eigen::Matrix4d T_odom;       // odometry pose at this frame
    std::vector<Feature2D> features_2d;
    std::vector<Feature3D> features_3d;
    std::vector<BowEntry>  bow_vector;
    cv::Mat image;
};

class MapExpansion {
public:
    using Ptr = std::shared_ptr<MapExpansion>;

    explicit MapExpansion(MapGraph::Ptr graph, DBoW2Wrapper::Ptr dbow);

    // -------------------------------------------------------------------------
    // Called every frame during repeating phase.
    //
    // If the current live frame matched successfully (matched_node_id >= 0),
    // this call checks whether the previous successful match is known and
    // processes all buffered frames between them.
    //
    // If the frame did NOT match (matched_node_id == -1), it is buffered.
    //
    // Parameters:
    //   frame             -- the current live frame (may have matched or not)
    //   matched_node_id   -- node id of the match (-1 if no match this frame)
    //   T_odom_current    -- current odometry pose (world frame)
    // -------------------------------------------------------------------------
    void processFrame(const BufferedFrame& frame,
                      int matched_node_id,
                      const Eigen::Matrix4d& T_odom_current);

    // Returns total number of expansion frames attached so far.
    int expansionCount() const { return total_expansions_; }

private:
    MapGraph::Ptr    graph_;
    DBoW2Wrapper::Ptr dbow_;

    // State between consecutive matches
    int    last_matched_node_id_    = -1;
    Eigen::Matrix4d T_odom_at_last_match_ = Eigen::Matrix4d::Identity();
    std::vector<BufferedFrame> pending_frames_;
    int next_frame_id_ = 0;
    int total_expansions_ = 0;

    // Find the map node whose position (via accumulated T_rel from head)
    // is nearest to the given odometry offset from the last match.
    int findNearestNode(const Eigen::Matrix4d& T_from_last_match,
                        int search_start_node_id) const;
};

} // namespace vtr
