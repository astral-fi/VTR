// =============================================================================
// keyframe_clusterer.cpp
// Offline keyframe clustering — Section III-B-3 and Eq.(7) in the paper.
//
// The algorithm walks the raw temporal chain.  For each representative K_i
// it greedily absorbs all following keyframes whose DBoW2 similarity exceeds
// tau_cluster, projecting their 3D features into K_i's coordinate frame and
// appending them to K_i's feature lists.
// =============================================================================

#include "vtr_map_manager/keyframe_clusterer.h"
#include <ros/ros.h>
#include <Eigen/SVD>
#include <unordered_set>

namespace vtr {

// ---------------------------------------------------------------------------
// Helper: extract descriptors as Nx32 Mat from a node's feature list
// ---------------------------------------------------------------------------
static cv::Mat nodeDescriptorMat(const KeyframeNode::Ptr& node) {
    if (node->features_2d.empty()) return cv::Mat();
    int N = static_cast<int>(node->features_2d.size());
    cv::Mat mat(N, 32, CV_8U);
    for (int i = 0; i < N; ++i) {
        node->features_2d[i].descriptor.copyTo(mat.row(i));
    }
    return mat;
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
KeyframeClusterer::KeyframeClusterer(const Config& cfg) : cfg_(cfg) {}

// ---------------------------------------------------------------------------
// accumulatePose
// Walk the linked list from start_id up to (but NOT including) end_id,
// multiplying relative transforms.  T is expressed as:
//   T_{start}^{end} = T_rel[start+1] * T_rel[start+2] * ... * T_rel[end]
// ---------------------------------------------------------------------------
Eigen::Matrix4d KeyframeClusterer::accumulatePose(
    const MapGraph::Ptr& graph, int start_id, int end_id) const {

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    int cur = start_id;

    while (cur != end_id) {
        auto node = graph->getNode(cur);
        if (!node) break;

        int nxt = node->next_node_id;
        if (nxt == -1) break;

        auto node_nxt = graph->getNode(nxt);
        if (!node_nxt) break;

        // T_rel stored in nxt is T_{cur}^{nxt}
        T = T * node_nxt->T_rel;
        cur = nxt;
    }
    return T;
}

// ---------------------------------------------------------------------------
// mergeFeatures
// Project src_node's 3D features into dst_node's camera frame and append
// them (along with the paired 2D projections) to dst_node.
//
// T_dst_from_src transforms a point in src's frame into dst's frame.
// ---------------------------------------------------------------------------
void KeyframeClusterer::mergeFeatures(KeyframeNode::Ptr dst,
                                       KeyframeNode::Ptr src,
                                       const Eigen::Matrix4d& T_dst_from_src) {
    Eigen::Matrix3d R = T_dst_from_src.block<3,3>(0,0);
    Eigen::Vector3d t = T_dst_from_src.block<3,1>(0,3);

    for (size_t i = 0; i < src->features_3d.size(); ++i) {
        // Transform 3D point into dst's frame
        Eigen::Vector3d p_src(src->features_3d[i].x,
                               src->features_3d[i].y,
                               src->features_3d[i].z);
        Eigen::Vector3d p_dst = R * p_src + t;

        // Only keep points in front of the camera and within reasonable depth
        if (p_dst.z() <= 0.1 || p_dst.z() > 15.0) continue;

        Feature3D f3;
        f3.x = static_cast<float>(p_dst.x());
        f3.y = static_cast<float>(p_dst.y());
        f3.z = static_cast<float>(p_dst.z());
        dst->features_3d.push_back(f3);

        // Keep the 2D feature and descriptor from the source
        // (useful for additional matching diversity)
        dst->features_2d.push_back(src->features_2d[i]);
    }
}

// ---------------------------------------------------------------------------
// gridFilter
// Divide the image into a grid and keep at most max_per_cell features per
// cell.  This prevents PnP from degrading due to concentrated features.
// ---------------------------------------------------------------------------
void KeyframeClusterer::gridFilter(KeyframeNode::Ptr node) {
    if (node->features_2d.size() <= static_cast<size_t>(cfg_.min_features_per_node))
        return;

    int cell_w = cfg_.image_width  / cfg_.grid_cols;
    int cell_h = cfg_.image_height / cfg_.grid_rows;

    // Count per cell
    std::vector<std::vector<int>> cell_indices(
        cfg_.grid_cols * cfg_.grid_rows);

    for (int i = 0; i < static_cast<int>(node->features_2d.size()); ++i) {
        float u = node->features_2d[i].u;
        float v = node->features_2d[i].v;
        int col = std::min(static_cast<int>(u / cell_w), cfg_.grid_cols - 1);
        int row = std::min(static_cast<int>(v / cell_h), cfg_.grid_rows - 1);
        int cell = row * cfg_.grid_cols + col;
        if (static_cast<int>(cell_indices[cell].size()) < cfg_.max_per_cell) {
            cell_indices[cell].push_back(i);
        }
    }

    // Rebuild feature lists with only the filtered indices
    std::vector<Feature2D> f2;
    std::vector<Feature3D> f3;
    for (auto& cell : cell_indices) {
        for (int idx : cell) {
            f2.push_back(node->features_2d[idx]);
            f3.push_back(node->features_3d[idx]);
        }
    }

    node->features_2d = std::move(f2);
    node->features_3d = std::move(f3);
}

// ---------------------------------------------------------------------------
// run (the main clustering algorithm)
// ---------------------------------------------------------------------------
int KeyframeClusterer::run(MapGraph::Ptr graph, DBoW2Wrapper::Ptr dbow) {
    if (graph->empty()) {
        ROS_WARN("[KeyframeClusterer] Graph is empty — nothing to cluster.");
        return 0;
    }

    int raw_size = graph->size();
    ROS_INFO_STREAM("[KeyframeClusterer] Starting clustering on "
                    << raw_size << " raw keyframes...");

    std::vector<KeyframeNode::Ptr> clustered;
    std::unordered_set<int> erased;

    // Walk the linked list from the head
    int cur_id = graph->headNodeId();

    while (cur_id != -1) {
        auto cur_node = graph->getNode(cur_id);
        if (!cur_node || erased.count(cur_id)) {
            break;
        }

        cv::Mat cur_desc = nodeDescriptorMat(cur_node);

        // Step 2: find the last similar keyframe K_{S(i)}
        int scan_id   = cur_node->next_node_id;
        int last_sim  = cur_id;  // initialize S(i) = i (no merging yet)

        while (scan_id != -1) {
            auto scan_node = graph->getNode(scan_id);
            if (!scan_node) break;

            cv::Mat scan_desc = nodeDescriptorMat(scan_node);
            if (scan_desc.empty()) {
                scan_id = scan_node->next_node_id;
                continue;
            }

            // Compute DBoW2 similarity
            auto candidates = dbow->queryFrame(scan_desc, 1, 0.0);
            double score = 0.0;
            for (auto& c : candidates) {
                if (c.node_id == cur_id) { score = c.dbow_score; break; }
            }

            if (score < cfg_.dbow_cluster_threshold) break;

            last_sim = scan_id;
            scan_id  = scan_node->next_node_id;
        }

        int skip_id = (last_sim == cur_id)
                      ? cur_node->next_node_id  // no merging; move one step
                      : last_sim;

        // Step 3: merge erased keyframes Ki+1 … K_{S(i)-1} into cur_node
        if (skip_id != -1 && skip_id != cur_node->next_node_id) {
            // Accumulate the transform from cur to each erased node
            int merge_id = cur_node->next_node_id;
            while (merge_id != skip_id && merge_id != -1) {
                auto merge_node = graph->getNode(merge_id);
                if (!merge_node) break;

                // T_{cur}^{merge}: accumulated from cur to merge
                Eigen::Matrix4d T_cur_to_merge =
                    accumulatePose(graph, cur_id, merge_id);

                // Project merge_node's features into cur_node's frame
                mergeFeatures(cur_node, merge_node, T_cur_to_merge.inverse());

                erased.insert(merge_id);
                merge_id = merge_node->next_node_id;
            }

            // Store the skip transform T_{cur}^{S(i)}
            cur_node->T_to_next = accumulatePose(graph, cur_id, skip_id);
            cur_node->has_skip  = true;
        }

        // Update linked-list pointers for the clustered chain
        cur_node->next_node_id = skip_id;
        if (skip_id != -1) {
            auto skip_node = graph->getNode(skip_id);
            if (skip_node) skip_node->prev_node_id = cur_id;
        }

        // Apply grid filter to limit feature density
        gridFilter(cur_node);

        clustered.push_back(cur_node);
        cur_id = skip_id;
    }

    int reduced_size = static_cast<int>(clustered.size());
    double reduction = 100.0 * (1.0 - static_cast<double>(reduced_size) / raw_size);

    ROS_INFO_STREAM("[KeyframeClusterer] Clustering complete. "
                    << raw_size << " → " << reduced_size
                    << " nodes (" << std::fixed << std::setprecision(1)
                    << reduction << "% reduction)");

    // Rebuild the graph and DBoW2 database with the clustered nodes
    graph->applyClusteredMap(clustered);

    dbow->clearDatabase();
    for (auto& node : clustered) {
        cv::Mat desc = nodeDescriptorMat(node);
        if (!desc.empty()) {
            dbow->addKeyframe(node->node_id, desc, graph, /*temporal_gap=*/0);
        }
    }

    ROS_INFO_STREAM("[KeyframeClusterer] DBoW2 database rebuilt with "
                    << dbow->databaseSize() << " entries.");

    return reduced_size;
}

} // namespace vtr
