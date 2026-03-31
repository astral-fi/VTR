# TrajectoryCandidate.msg
# Represents a single pre-cached arc trajectory candidate.

# Angular velocity (rad/s) for this candidate
float64 angular_velocity

# Linear forward velocity (m/s)
float64 linear_velocity

# Score assigned by the scoring function (higher = better)
float64 score

# Whether this candidate passed the feasibility (collision) check
bool feasible

# 2D waypoints of the arc in robot frame (x forward, y left)
geometry_msgs/Point[] waypoints
