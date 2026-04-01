#pragma once
// =============================================================================
// dbow2_wrapper.h
// DBoW2-based loop detection for Phase 2 (Section III-B-2 in the paper).
//
// Two responsibilities:
//   1. Teaching phase: call addKeyframe() for each incoming keyframe.
//      Internally queries the database and records loop edges in MapGraph.
//   2. Repeating phase: call queryFrame() to find the best matching map node
//      for a live camera frame (coarse retrieval step for Phase 3).
//
// The geometric verification (PnP) is handled by Phase 3 (relocalization).
// This class returns the top-N DBoW candidates with their similarity scores.
// =============================================================================

#include "vtr_map_manager/keyframe_node.h"
#include "vtr_map_manager/map_graph.h"
#include <string>
#include <vector>
#include <memory>

// Forward-declare DBoW2 types to keep headers clean.
// The .cpp includes DBoW2 directly.
namespace DBoW2 {
    class OrbVocabulary;
    class OrbDatabase;
}

namespace vtr {

struct LoopCandidate {
    int    node_id;       // candidate map node id
    double dbow_score;    // similarity score from DBoW2 (higher = more similar)
};

class DBoW2Wrapper {
public:
    using Ptr = std::shared_ptr<DBoW2Wrapper>;

    // -------------------------------------------------------------------------
    // Construction
    // -------------------------------------------------------------------------

    // Load a pre-trained BRIEF vocabulary from disk.
    // vocabulary_path: path to the .voc.gz file (see install instructions).
    explicit DBoW2Wrapper(const std::string& vocabulary_path);
    ~DBoW2Wrapper();

    bool isLoaded() const { return vocab_loaded_; }

    // -------------------------------------------------------------------------
    // Teaching Phase
    // -------------------------------------------------------------------------

    // Add a keyframe's descriptors to the DBoW2 database.
    // Returns the DBoW2 entry_id (== node_id in the map graph).
    // Also detects loops and records them in the provided MapGraph.
    //
    // Parameters:
    //   node_id      -- MapGraph node id for this keyframe
    //   descriptors  -- Nx32 CV_8U BRIEF descriptor matrix (N features)
    //   graph        -- the MapGraph to write loop edges into
    //   temporal_gap -- reject candidates within this many recent nodes
    //                   (avoids matching adjacent frames; default 30)
    void addKeyframe(int node_id,
                     const cv::Mat& descriptors,
                     MapGraph::Ptr graph,
                     int temporal_gap = 30);

    // -------------------------------------------------------------------------
    // Repeating Phase
    // -------------------------------------------------------------------------

    // Query the database for the top-N candidates matching the live frame.
    // Used by Phase 3 as the coarse retrieval step.
    //
    // Parameters:
    //   descriptors  -- Nx32 CV_8U BRIEF descriptor matrix of live frame
    //   top_n        -- number of candidates to return (default 5)
    //   min_score    -- minimum DBoW score to accept (default 0.012)
    //
    // Returns: sorted list (best score first) of LoopCandidate structs.
    std::vector<LoopCandidate> queryFrame(const cv::Mat& descriptors,
                                          int top_n     = 5,
                                          double min_score = 0.012) const;

    // -------------------------------------------------------------------------
    // Compute bow vector for a set of BRIEF descriptors (utility)
    // -------------------------------------------------------------------------
    std::vector<BowEntry> computeBowVector(const cv::Mat& descriptors) const;

    // -------------------------------------------------------------------------
    // Database management
    // -------------------------------------------------------------------------
    void clearDatabase();
    int  databaseSize() const;

    // Save / load the populated database (not the vocabulary)
    bool saveDatabase(const std::string& path) const;
    bool loadDatabase(const std::string& path);

private:
    std::string vocab_path_;
    bool vocab_loaded_ = false;

    // DBoW2 objects (pimpl to avoid header dependency)
    struct Impl;
    std::unique_ptr<Impl> impl_;

    // Thresholds (from paper / implementation guide)
    static constexpr double kScoreThresholdTeaching  = 0.015;
    static constexpr double kScoreThresholdRepeating = 0.012;
};

} // namespace vtr
