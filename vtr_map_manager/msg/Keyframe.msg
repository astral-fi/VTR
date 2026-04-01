# Keyframe.msg
# Carries the full keyframe data from Phase 1 into Phase 2.
# Mirrors the Ki struct: Ki = {T_rel, features_2d, features_3d, descriptor, image}

std_msgs/Header header

# Unique keyframe index (assigned by Phase 1 in sequential order)
int32 keyframe_id

# Relative SE(3) transform from K_{i-1} to K_i as a 4x4 row-major float64 array
float64[16] T_rel

# 2D feature pixel coordinates and ORB descriptor (256-bit = 32 bytes each)
# Packed as (u, v) pairs; descriptor bytes follow in features_descriptors
float32[]   features_u
float32[]   features_v
uint8[]     features_descriptors   # N * 32 bytes, row-major

# 3D feature positions in camera coordinates (paired 1-to-1 with 2D features)
float32[]   features_x
float32[]   features_y
float32[]   features_z

# Index map: maps each 3D point to its corresponding 2D feature / descriptor index
int32[]     feat3d_idx

# DBoW2 bag-of-words vector: word_id and word_weight pairs
uint32[]    bow_word_ids
float64[]   bow_word_weights

# Grayscale image (for map expansion and debugging)
sensor_msgs/Image image
