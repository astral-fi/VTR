# Phase 1 → Phase 2 keyframe contract
std_msgs/Header header

# Relative transform T_{i-1}^{i} as a 4x4 column-major float64 array
float64[16] T_rel

# 2D features: u, v pixel coords (paired with features_3d)
float32[] features_u
float32[] features_v

# BRIEF descriptors: each is 32 bytes = 256 bits; packed flat
# Length = num_features * 32 bytes
uint8[] brief_descriptors

# 3D features in left camera frame (paired with features_2d)
float32[] features_x
float32[] features_y
float32[] features_z

# DBoW2 bag-of-words vector (word_id, weight pairs, flat-packed)
uint32[] bow_word_ids
float64[] bow_word_weights

# Grayscale image
sensor_msgs/Image image

# Keyframe index
uint32 keyframe_id
