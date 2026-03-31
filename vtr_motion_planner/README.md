# VTR Phase 4 — Local Motion Planning & Obstacle Avoidance

ROS Melodic C++ package implementing **Phase 4** of the Visual Teach-and-Repeat (VTR) navigation system.

> Based on: Wang et al., *"Robust Visual Teach-and-Repeat Navigation with Flexible Topo-metric Graph Map Representation"*, arXiv:2510.09089v1, 2025.

---

## Overview

This package runs the real-time control loop.  It:

1. Ingests **2D laser scan** data (`/scan`)
2. Reads an ordered **goal list** from Phase 3 (`/vtr/goal_list`)
3. Builds a **robot-centric occupancy grid** and inflates obstacles by the robot footprint
4. Pre-caches **K=21 arc trajectory candidates** at startup
5. **Scores** each feasible arc using angular alignment to the first 3 goals
6. Publishes **`/cmd_vel`** (linear + angular velocity) at 10 Hz

---

## System Requirements

| Dependency | Version | Notes |
|---|---|---|
| ROS | **Melodic** | Ubuntu 18.04 |
| CMake | ≥ 3.0.2 | Ships with Melodic |
| Eigen3 | ≥ 3.3 | `sudo apt install libeigen3-dev` |
| roscpp / std_msgs / sensor_msgs / geometry_msgs / nav_msgs | Melodic | ROS desktop-full |
| laser_geometry | Melodic | `sudo apt install ros-melodic-laser-geometry` |
| tf / tf2 / tf2_ros / tf2_geometry_msgs | Melodic | Ships with desktop-full |

### Install all dependencies at once

```bash
sudo apt update
sudo apt install -y \
    ros-melodic-desktop-full \
    ros-melodic-laser-geometry \
    ros-melodic-tf2-geometry-msgs \
    libeigen3-dev
```

---

## Build

```bash
# 1. Create / navigate to your catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 2. Copy (or symlink) this package
cp -r /path/to/vtr_motion_planner .

# 3. Build
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release

# 4. Source
source devel/setup.bash
```

> **Tip:** Use `catkin_make -j4` on resource-constrained hardware to limit parallel jobs.

---

## Running

### Standalone (with mock Phase 3 goals)

```bash
# Terminal 1 – launch the planner
roslaunch vtr_motion_planner vtr_motion_planner.launch

# Terminal 2 – publish fake goals (straight-line pattern)
rosrun vtr_motion_planner mock_phase3.py

# Terminal 3 – replay a bag or run a simulator that publishes /scan and /odom
rosbag play your_lidar_bag.bag
```

### With a real robot

```bash
roslaunch vtr_motion_planner vtr_motion_planner.launch \
    scan_topic:=/base_scan \
    cmd_vel_topic:=/mobile_base/commands/velocity \
    forward_velocity:=0.25
```

---

## Parameters

All parameters can be set in `config/planner_params.yaml` or passed as launch arguments.

| Parameter | Default | Description |
|---|---|---|
| `scan_topic` | `/scan` | Input laser scan topic |
| `cmd_vel_topic` | `/cmd_vel` | Output velocity command topic |
| `odom_topic` | `/odom` | Odometry topic (for goal-frame updates) |
| `goal_list_topic` | `/vtr/goal_list` | Goal list from Phase 3 |
| `control_rate_hz` | `10.0` | Control loop frequency (Hz) |
| `close_range_threshold` | `0.5` | Switch to direct steering below this distance (m) |
| `safety_rotate_omega` | `0.3` | Rotate-in-place speed when all arcs blocked (rad/s) |
| `num_trajectory_candidates` | `21` | K – number of pre-cached arcs |
| `forward_velocity` | `0.3` | Fixed forward speed during arc selection (m/s) |
| `omega_min` / `omega_max` | `-0.6` / `+0.6` | Angular velocity sweep range (rad/s) |
| `grid_width_m` | `6.0` | Occupancy grid physical size (m) |
| `grid_resolution_m` | `0.05` | Cell size (m) |
| `inflation_radius_m` | `0.25` | Robot radius + safety margin (m) |
| `arrival_threshold_m` | `0.4` | Remove goal when within this distance (m) |
| `angle_limit_deg` | `60.0` | Remove goal when bearing exceeds ±this (deg) |
| `max_goals` | `5` | Maximum goals held in list |

---

## Published & Subscribed Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | **Subscribe** | 2D LiDAR input |
| `/odom` | `nav_msgs/Odometry` | **Subscribe** | Odometry for goal-frame updates |
| `/vtr/goal_list` | `vtr_motion_planner/GoalList` | **Subscribe** | Ordered goals from Phase 3 |
| `/cmd_vel` | `geometry_msgs/Twist` | **Publish** | Linear + angular velocity commands |
| `/vtr/motion_planner/status` | `vtr_motion_planner/MotionPlannerStatus` | **Publish** | Diagnostic status per cycle |

---

## Custom Messages

### `GoalList.msg`
Ordered list of 3D goal positions in the robot frame, published by Phase 3.

```
std_msgs/Header header
geometry_msgs/Point[] goals
string frame_id
```

### `MotionPlannerStatus.msg`
Per-cycle diagnostic: current mode, selected trajectory, goal distance.

```
uint8 MODE_TRAJECTORY = 0
uint8 MODE_DIRECT     = 1
uint8 MODE_STOPPED    = 2
uint8  mode
int32  selected_trajectory_index
int32  feasible_trajectory_count
float64 nearest_goal_distance
int32  active_goal_count
float64 cmd_linear
float64 cmd_angular
```

---

## Algorithm (Phase 4, Algorithm 1 from the paper)

```
INPUT:  P (laser scan),  Lg (goal list from Phase 3)
OUTPUT: v (m/s),  ω (rad/s)

1.  T  ← pre-cached K=21 arc candidates
2.  Mo ← BuildOccupancyGrid(P)
3.  M  ← InflateObstacles(Mo, radius=0.25 m)
4.  Lg ← getGoalList()
5.  dp ← dist(Lg[0])

6.  IF dp < 0.5 m:
7.      IF goal reachable: v = dp;  ω = atan2(y_goal, x_goal)
8.      ELSE:              v = 0;   ω = 0

9.  ELSE:
10.     FOR i in 0..K-1:
11.         score[i] = 0
12.         FOR m in 0..2:
13.             IF feasible(M, T[i]):
14.                 θ = angle(endpoint(T[i]), Lg[m])  [degrees]
15.                 score[i] += 1 − sqrt(0.005 · θ)
16.             ELSE: score[i] += 0
17.     mI ← argmax(score)
18.     v = 0.3;  ω = T[mI].omega

19. IF no feasible trajectory: v = 0;  ω = 0.3  (rotate in place)
20. RETURN v, ω
```

---

## Testing

### Unit tests (no ROS required)

```bash
python scripts/test_scoring.py
```

Validates: arc generation, omega range, endpoint distance, straight/left/right goal tracking, obstacle avoidance, all-blocked safety behaviour.

### Integration test with Phase 3 mock

```bash
# Terminal 1
roscore

# Terminal 2
roslaunch vtr_motion_planner vtr_motion_planner.launch

# Terminal 3 – pick a pattern: straight | curved | zigzag
rosrun vtr_motion_planner mock_phase3.py _pattern:=curved _rate:=2.0

# Terminal 4 – inspect commands
rostopic echo /cmd_vel
rostopic echo /vtr/motion_planner/status
```

### Acceptance criteria (from spec)

| Criterion | Target |
|---|---|
| Control loop rate | 10 Hz, jitter < 50 ms |
| Obstacle avoidance | No collision with 0.5 m obstacle |
| Goal tracking endpoint error | < 0.5 m on straight route |
| All-blocked behaviour | Publish v=0, ω=0.3 (rotate in place) |
| Phase 3 goal list integration | Updated within same control cycle |

---

## Package Structure

```
vtr_motion_planner/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── planner_params.yaml        # All tunable parameters
├── launch/
│   └── vtr_motion_planner.launch  # Main launch file
├── msg/
│   ├── GoalList.msg
│   ├── TrajectoryCandidate.msg
│   └── MotionPlannerStatus.msg
├── srv/
│   ├── GetGoalList.srv
│   └── NotifyOdometry.srv
├── include/vtr_motion_planner/
│   ├── occupancy_grid.h           # 2D laser → binary grid + inflation
│   ├── trajectory_generator.h     # Pre-cached arc library
│   ├── trajectory_scorer.h        # Feasibility filter + scoring function
│   ├── goal_list_manager.h        # Thread-safe goal cache + odometry update
│   └── motion_controller.h        # Top-level Algorithm 1 controller
└── src/
    ├── occupancy_grid.cpp
    ├── trajectory_generator.cpp
    ├── trajectory_scorer.cpp
    ├── goal_list_manager.cpp
    ├── motion_controller.cpp
    └── motion_planner_node.cpp    # ROS node entry point
```

---

## Integration with Other VTR Phases

| Interface | Direction | Topic / Service |
|---|---|---|
| Phase 3 → Phase 4 | Goal list | `/vtr/goal_list` (GoalList.msg) |
| Phase 4 → Phase 3 | Odometry notification | `notifyOdometry()` via `/vtr/notify_odometry` service |
| Phase 4 → Robot | Velocity commands | `/cmd_vel` (geometry_msgs/Twist) |

Phase 4 has **no dependency on Phase 1 or Phase 2** – it only consumes the goal list from Phase 3 and raw laser data.
