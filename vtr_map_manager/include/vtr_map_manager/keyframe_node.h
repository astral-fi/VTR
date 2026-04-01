#pragma once
// =============================================================================
// keyframe_node.h
// Core data structures for the Phase 2 topo-metric graph.
//
// Mirrors the paper's keyframe definition:
//   Ki = {T_{i-1}^i, U_i, P_i, I_i}                        [Eq. 5]
//   Ki (with loop) = {T_{i-1}^i, U_i, P_i, I_i, T_{L(i)}^i, L(i)}  [Eq. 6]
//   Ki (enhanced) = {T_{i-1}^i, U_bar_i, P_bar_i, I_i,
//                    T_{L(i)}^i, L(i), T_{S(i)}^i, S(i), {K}}       [Eq. 7,14]
// =============================================================================

#include <vector>
#include <memory>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

namespace vtr {

// ---------------------------------------------------------------------------
// Feature2D: one 2D feature point with its BRIEF descriptor.
// ---------------------------------------------------------------------------
struct Feature2D {
    float u, v;              // pixel coordinates
    cv::Mat descriptor;      // 1 x 32 CV_8U (256-bit BRIEF)
};

// ---------------------------------------------------------------------------
// Feature3D: one 3D point in the camera frame of its keyframe.
// ---------------------------------------------------------------------------
struct Feature3D {
    float x, y, z;           // metres, in camera coordinates of the keyframe
};

// ---------------------------------------------------------------------------
// BowEntry: one (word_id, weight) pair from a DBoW2 BowVector.
// We store it plain to avoid pulling DBoW2 headers throughout the codebase.
// ---------------------------------------------------------------------------
struct BowEntry {
    uint32_t word_id;
    double   weight;
};

// ---------------------------------------------------------------------------
// ExpansionFrame: a novel frame attached to a map node during the repeating
// phase.  It does NOT become a keyframe; it carries no goal responsibility.
// See Section III-C-2 and Eq.(14) in the paper.
// ---------------------------------------------------------------------------
struct ExpansionFrame {
    int      frame_id;          // global unique ID
    int      nearest_node_id;   // the node this is attached to
    Eigen::Matrix4d T_node_to_frame;  // T from node coords to this frame
    std::vector<Feature2D> features_2d;
    std::vector<Feature3D> features_3d;
    std::vector<int>       feat3d_idx;
    std::vector<BowEntry>  bow_vector;
    cv::Mat  image;             // grayscale
    double   timestamp;
};

// ---------------------------------------------------------------------------
// KeyframeNode: one node in the topo-metric graph.
// ---------------------------------------------------------------------------
struct KeyframeNode {
    using Ptr = std::shared_ptr<KeyframeNode>;

    // ---- Identity -----------------------------------------------------------
    int    node_id   = -1;    // index in the graph (assigned on insertion)
    double timestamp =  0.0;  // seconds since epoch

    // ---- Relative poses (SE3 as 4x4 matrix) ---------------------------------
    // T_rel: T_{i-1}^{i}  relative to the PREVIOUS node in the temporal chain
    Eigen::Matrix4d T_rel = Eigen::Matrix4d::Identity();

    // T_to_next: T_{i}^{S(i)}  skip transform to next cluster representative.
    // Populated after keyframe clustering (Section III-B-3).
    Eigen::Matrix4d T_to_next = Eigen::Matrix4d::Identity();
    bool            has_skip   = false;

    // T_loop: T_{L(i)}^{i}  transform to the loop-closure partner.
    // Populated when a loop is detected (Section III-B-2).
    Eigen::Matrix4d T_loop = Eigen::Matrix4d::Identity();
    int             loop_node_id = -1;   // -1 = no loop

    // ---- Linked list pointers -----------------------------------------------
    int prev_node_id = -1;   // -1 = head of list
    int next_node_id = -1;   // -1 = tail of list (points to K_{S(i)} after clustering)

    // ---- Visual observations ------------------------------------------------
    // U_bar: 2D feature pixels (extended after clustering to include merged KFs)
    std::vector<Feature2D> features_2d;

    // P_bar: 3D positions in this node's camera frame (parallel to features_2d)
    std::vector<Feature3D> features_3d;

    // Index map mapping features_3d to their corresponding features_2d index
    std::vector<int> feat3d_idx;

    // DBoW2 bag-of-words vector for fast retrieval
    std::vector<BowEntry>  bow_vector;

    // Grayscale image (for map expansion and debug visualisation)
    cv::Mat image;   // CV_8UC1

    // ---- Map expansion (Eq. 14) ---------------------------------------------
    // Low-level frames attached to this node during the repeating phase.
    std::vector<ExpansionFrame> expansion_frames;

    // ---- Helpers ------------------------------------------------------------
    bool isHead() const { return prev_node_id == -1; }
    bool isTail() const { return next_node_id == -1; }
    bool hasLoop() const { return loop_node_id != -1; }
};

} // namespace vtr
