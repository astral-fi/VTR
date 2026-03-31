# MapNode.msg
# Represents one node in the reduced topo-metric graph after keyframe clustering.
# Corresponds to the enhanced Ki from Eq.(7) in the paper.

std_msgs/Header header

int32  node_id          # Index in the reduced map linked list
int32  prev_node_id     # Previous node in temporal chain (-1 if head)
int32  next_node_id     # Next node = K_{S(i)}, the cluster representative (-1 if tail)
int32  loop_node_id     # Loop-closure partner (-1 if none)

# Relative transforms (4x4 SE(3), row-major float64)
float64[16] T_to_prev      # T_{i-1}^{i}
float64[16] T_to_next      # T_{i}^{S(i)} (skip transform to next cluster node)
float64[16] T_to_loop      # T_{L(i)}^{i} (loop transform, zeros if no loop)

# Extended feature set after merging neighboring keyframes (U_bar, P_bar)
float32[]   features_u
float32[]   features_v
uint8[]     features_descriptors   # N * 32 bytes
float32[]   features_x
float32[]   features_y
float32[]   features_z

# DBoW2 descriptor for this enhanced node
uint32[]    bow_word_ids
float64[]   bow_word_weights

# Original grayscale image from the representative keyframe
sensor_msgs/Image image

# List of expansion frame IDs attached to this node (from repeating phase)
int32[]     expansion_frame_ids
