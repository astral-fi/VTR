# GoalList.msg
# Published by Phase 3 (relocalization), consumed by Phase 4 (motion planning).
# Contains an ordered list of 3D goal positions in the current robot frame.

std_msgs/Header header   # stamp = time of last successful relocalization

# Ordered goal positions in robot body frame (x forward, y left, z up)
# g[0] = t_g0 (immediate next goal), g[M-1] = furthest look-ahead goal
geometry_msgs/Point[] goals

# ID of the map node that produced this goal list (for debugging)
int32 matched_node_id

# Reprojection error of the PnP match that generated this list (pixels, RMS)
float32 match_reprojection_error

# Number of PnP inliers for the match
int32 match_inlier_count
