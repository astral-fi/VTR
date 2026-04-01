// =============================================================================
// dbow2_wrapper.cpp
// DBoW2 integration for Phase 2 loop detection.
//
// Build requirements:
//   - DBoW2 built with BRIEF support (see README for build instructions)
//   - A pre-trained BRIEF vocabulary (brief_k10L6.voc.gz recommended)
// =============================================================================

#include "vtr_map_manager/dbow2_wrapper.h"
#include <ros/ros.h>

// DBoW2 headers — available after building DBoW2 from source
#include <DBoW2/DBoW2.h>          // OrbVocabulary, OrbDatabase
#include <DBoW2/FBrief.h>         // BRIEF feature functions
#include <DBoW2/QueryResults.h>   // QueryResults, Result

namespace vtr {

// ---------------------------------------------------------------------------
// Pimpl struct
// ---------------------------------------------------------------------------
struct DBoW2Wrapper::Impl {
    std::unique_ptr<DBoW2::OrbVocabulary> vocab;
    std::unique_ptr<DBoW2::OrbDatabase>   db;

    // node_id -> DBoW2 entry id mapping
    std::unordered_map<int, DBoW2::EntryId> node_to_entry;
    std::unordered_map<DBoW2::EntryId, int> entry_to_node;
    int next_entry = 0;
};

// ---------------------------------------------------------------------------
// Helper: convert Nx32 CV_8U Mat to DBoW2 feature format (vector of bitsets)
// ---------------------------------------------------------------------------
static std::vector<DBoW2::FBrief::TDescriptor>
matToDescriptors(const cv::Mat& mat) {
    std::vector<DBoW2::FBrief::TDescriptor> descs;
    descs.reserve(mat.rows);
    for (int i = 0; i < mat.rows; ++i) {
        // DBoW2 BRIEF uses bitset<256> internally via boost or std
        // FBrief::TDescriptor is a cv::Mat row (32 bytes)
        descs.push_back(mat.row(i));
    }
    return descs;
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
DBoW2Wrapper::DBoW2Wrapper(const std::string& vocabulary_path)
    : vocab_path_(vocabulary_path), impl_(std::make_unique<Impl>()) {

    ROS_INFO_STREAM("[DBoW2Wrapper] Loading vocabulary from: " << vocabulary_path);

    impl_->vocab = std::make_unique<DBoW2::OrbVocabulary>(vocabulary_path);

    if (impl_->vocab->empty()) {
        ROS_ERROR_STREAM("[DBoW2Wrapper] Failed to load vocabulary from "
                         << vocabulary_path
                         << ". Check the path and ensure the file exists.");
        vocab_loaded_ = false;
        return;
    }

    // Create an empty database backed by the loaded vocabulary.
    // direct_index_levels=4 enables fast geometric checks inside DBoW2.
    impl_->db = std::make_unique<DBoW2::OrbDatabase>(
        *impl_->vocab, /*use_di=*/true, /*di_levels=*/4);

    vocab_loaded_ = true;
    ROS_INFO_STREAM("[DBoW2Wrapper] Vocabulary loaded. Words: "
                    << impl_->vocab->size());
}

DBoW2Wrapper::~DBoW2Wrapper() = default;

// ---------------------------------------------------------------------------
// addKeyframe (teaching phase)
// ---------------------------------------------------------------------------
void DBoW2Wrapper::addKeyframe(int node_id,
                                const cv::Mat& descriptors,
                                MapGraph::Ptr graph,
                                int temporal_gap) {
    if (!vocab_loaded_) {
        ROS_WARN_THROTTLE(5.0, "[DBoW2Wrapper] Vocabulary not loaded — skipping addKeyframe.");
        return;
    }
    if (descriptors.empty()) {
        ROS_WARN_STREAM("[DBoW2Wrapper] addKeyframe: empty descriptors for node " << node_id);
        return;
    }

    auto feature_vec = matToDescriptors(descriptors);

    // --- Query BEFORE adding (so we don't match against ourselves) ---
    DBoW2::QueryResults results;
    impl_->db->query(feature_vec, results, /*max_results=*/10);

    // Filter and record loop edges
    for (auto& r : results) {
        if (r.Score < kScoreThresholdTeaching) continue;

        auto it = impl_->entry_to_node.find(r.Id);
        if (it == impl_->entry_to_node.end()) continue;

        int candidate_node_id = it->second;

        // Skip temporally adjacent frames (avoids matching near-neighbors)
        if (std::abs(candidate_node_id - node_id) < temporal_gap) continue;

        // Record the loop edge in the MapGraph.
        // Note: the actual relative pose T_loop is computed by Phase 3 (PnP).
        // Here we store an identity placeholder; Phase 3 will overwrite it.
        ROS_DEBUG_STREAM("[DBoW2Wrapper] Loop candidate: node " << node_id
                         << " ↔ node " << candidate_node_id
                         << " (score=" << r.Score << ")");
        graph->addLoopEdge(node_id, candidate_node_id,
                           Eigen::Matrix4d::Identity());
        break; // Take the best candidate only during teaching
    }

    // --- Add to database ---
    DBoW2::EntryId eid = impl_->db->add(feature_vec);
    impl_->node_to_entry[node_id] = eid;
    impl_->entry_to_node[eid] = node_id;
}

// ---------------------------------------------------------------------------
// queryFrame (repeating phase)
// ---------------------------------------------------------------------------
std::vector<LoopCandidate> DBoW2Wrapper::queryFrame(
    const cv::Mat& descriptors, int top_n, double min_score) const {

    std::vector<LoopCandidate> candidates;

    if (!vocab_loaded_ || descriptors.empty()) return candidates;

    auto feature_vec = matToDescriptors(descriptors);

    DBoW2::QueryResults results;
    impl_->db->query(feature_vec, results, top_n * 2); // fetch extra to filter

    for (auto& r : results) {
        if (r.Score < min_score) continue;

        auto it = impl_->entry_to_node.find(r.Id);
        if (it == impl_->entry_to_node.end()) continue;

        candidates.push_back({it->second, r.Score});

        if (static_cast<int>(candidates.size()) >= top_n) break;
    }

    return candidates;
}

// ---------------------------------------------------------------------------
// computeBowVector
// ---------------------------------------------------------------------------
std::vector<BowEntry> DBoW2Wrapper::computeBowVector(
    const cv::Mat& descriptors) const {

    std::vector<BowEntry> bow;
    if (!vocab_loaded_ || descriptors.empty()) return bow;

    auto feature_vec = matToDescriptors(descriptors);

    DBoW2::BowVector bv;
    impl_->vocab->transform(feature_vec, bv);

    bow.reserve(bv.size());
    for (auto& kv : bv) {
        BowEntry e;
        e.word_id = static_cast<uint32_t>(kv.first);
        e.weight  = kv.second;
        bow.push_back(e);
    }
    return bow;
}

// ---------------------------------------------------------------------------
// clearDatabase
// ---------------------------------------------------------------------------
void DBoW2Wrapper::clearDatabase() {
    if (!vocab_loaded_) return;
    impl_->db = std::make_unique<DBoW2::OrbDatabase>(
        *impl_->vocab, true, 4);
    impl_->node_to_entry.clear();
    impl_->entry_to_node.clear();
    impl_->next_entry = 0;
    ROS_INFO("[DBoW2Wrapper] Database cleared.");
}

int DBoW2Wrapper::databaseSize() const {
    if (!vocab_loaded_) return 0;
    return static_cast<int>(impl_->db->size());
}

bool DBoW2Wrapper::saveDatabase(const std::string& path) const {
    if (!vocab_loaded_) return false;
    impl_->db->save(path);
    ROS_INFO_STREAM("[DBoW2Wrapper] Database saved to " << path);
    return true;
}

bool DBoW2Wrapper::loadDatabase(const std::string& path) {
    if (!vocab_loaded_) return false;
    impl_->db->load(path);
    ROS_INFO_STREAM("[DBoW2Wrapper] Database loaded from " << path);
    return true;
}

} // namespace vtr
