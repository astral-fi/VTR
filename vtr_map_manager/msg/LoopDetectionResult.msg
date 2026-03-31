# LoopDetectionResult.msg
# Result of a single DBoW2 + PnP loop query (Phase 2 → Phase 3).

std_msgs/Header header

bool   success             # True if a verified loop was found
int32  query_keyframe_id   # ID of the live frame that was queried
int32  matched_node_id     # ID of the matched map node (-1 if none)
float64[16] T_match        # T_{matched}^{query}: relative pose from query to match
float32 dbow_score         # Raw DBoW2 similarity score
float32 reprojection_error # PnP reprojection error in pixels
int32   inlier_count       # Number of PnP inliers
