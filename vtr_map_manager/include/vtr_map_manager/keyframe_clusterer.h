#pragma once
// =============================================================================
// keyframe_clusterer.h
// Offline keyframe clustering (map redundancy reduction) — Section III-B-3.
//
// Algorithm (from the paper and implementation guide):
//   1. Start at K0.  i = 0.
//   2. Compute DBoW2 similarity between Ki and Ki+1, Ki+2, …
//      Stop when score < τ_cluster.  Last similar KF = K_{S(i)}.
//   3. For each erased KF Ki+s: transform its 3D features into Ki's frame
//      using the accumulated T_rel chain, append to Ki.features_3d / 2d.
//      Store T_{i}^{S(i)} in Ki.
//   4. Remove KFs between Ki and K_{S(i)}.  Connect Ki → K_{S(i)}.
//   5. i = S(i).  Repeat from step 2.
//
// Input:  raw MapGraph (from teaching phase)
// Output: reduced MapGraph (in-place replacement via graph->applyClusteredMap)
//
// Acceptance criteria (from implementation guide):
//   - Map size after clustering < 40% of raw keyframe count on 50 m route
//   - Enhanced keyframe feature count >= 1.5x original after clustering
// =============================================================================

#include "vtr_map_manager/map_graph.h"
#include "vtr_map_manager/dbow2_wrapper.h"
#include <memory>

namespace vtr {

class KeyframeClusterer {
public:
    using Ptr = std::shared_ptr<KeyframeClusterer>;

    struct Config {
        // DBoW2 similarity threshold for clustering (τ_cluster in guide)
        double dbow_cluster_threshold = 0.05;

        // Minimum number of features retained per enhanced node
        int min_features_per_node = 200;

        // Maximum features per enhanced node (cap to bound PnP cost)
        int max_features_per_node = 800;

        // Grid filter: divide image into grid_cols x grid_rows cells,
        // keep at most max_per_cell features per cell after clustering.
        int grid_cols       = 8;
        int grid_rows       = 6;
        int max_per_cell    = 5;

        // Image resolution (needed for grid filter)
        int image_width  = 640;
        int image_height = 480;
    };

    explicit KeyframeClusterer(const Config& cfg = Config());

    // -------------------------------------------------------------------------
    // Run offline clustering on the raw teaching map.
    //
    // This mutates the graph by calling graph->applyClusteredMap().
    // It also rebuilds the DBoW2 database with the reduced node set.
    //
    // Returns the number of nodes in the reduced map.
    // -------------------------------------------------------------------------
    int run(MapGraph::Ptr graph, DBoW2Wrapper::Ptr dbow);

private:
    Config cfg_;

    // Transform 3D features from src_node's camera frame into dst_node's
    // frame using the accumulated relative pose chain.
    void mergeFeatures(KeyframeNode::Ptr dst,
                       KeyframeNode::Ptr src,
                       const Eigen::Matrix4d& T_dst_from_src);

    // Apply grid filter to limit feature density after merging.
    void gridFilter(KeyframeNode::Ptr node);

    // Accumulate T_rel along the chain from start_id to end_id (exclusive).
    Eigen::Matrix4d accumulatePose(const MapGraph::Ptr& graph,
                                   int start_id, int end_id) const;
};

} // namespace vtr
