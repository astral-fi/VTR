# GoalList.msg
# Ordered list of 3D goal positions in the current robot frame.
# Published by Phase 3 (Relocalization) and consumed by Phase 4 (Motion Planner).

std_msgs/Header header

# Ordered goal positions (robot-frame, metres).
# Index 0 is the immediate next goal; indices 1..N are lookahead goals.
geometry_msgs/Point[] goals

# Frame in which the goals are expressed (should be "base_link" or "odom")
string frame_id
