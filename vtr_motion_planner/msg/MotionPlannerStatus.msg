# MotionPlannerStatus.msg
# Diagnostic status published by the motion planner each control cycle.

std_msgs/Header header

# Current operating mode
uint8 MODE_TRAJECTORY = 0   # Normal arc-selection mode
uint8 MODE_DIRECT     = 1   # Close-range direct steering (< 0.5 m)
uint8 MODE_STOPPED    = 2   # All trajectories blocked; rotating in place
uint8 mode

# Selected trajectory index (valid only in MODE_TRAJECTORY)
int32 selected_trajectory_index

# Number of feasible trajectory candidates this cycle
int32 feasible_trajectory_count

# Distance to nearest goal (metres)
float64 nearest_goal_distance

# Number of active goals in current goal list
int32 active_goal_count

# Selected linear and angular velocity commands
float64 cmd_linear
float64 cmd_angular
