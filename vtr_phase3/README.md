# VTR Phase 3 — Relocalization & Goal List Management

ROS Melodic (Ubuntu 18.04) implementation of **Phase 3** of the Visual Teach-and-Repeat system described in:

> Wang et al., *"Robust Visual Teach-and-Repeat Navigation with Flexible Topo-metric Graph Map Representation"*, arXiv:2510.09089v1, 2025.

---

## Architecture Overview

```
Live Camera Frame
       │
       ▼
[Step 1] DBoW2 Coarse Retrieval   τ_ret = 0.012
       │  top-N candidates, temporal adjacency filtered
       ▼
[Step 2] BRIEF Window Matching    γ = 40 px, Hamming ≤ 60 bits
       │  3D–2D correspondence set, 8×6 grid filter (≤3 per cell)
       ▼
[Step 3] Gauss-Newton PnP         ≤20 iterations, ε = 1e-6
       │  T_k^i, inliers ≥ 12, RMS reproj ≤ 2.5 px
       ▼
[Step 4] Goal List (M=5 goals)    → Phase 4 /vtr/goal_list
```

---

## 1. Prerequisites

### 1.1 ROS Melodic Base

```bash
sudo apt-get install ros-melodic-desktop-full
```

### 1.2 Eigen 3

Ships with ROS Melodic; verify with:
```bash
apt list --installed 2>/dev/null | grep libeigen3
# if missing:
sudo apt-get install libeigen3-dev
```

### 1.3 OpenCV 3 with contrib (required for `cv::xfeatures2d::BriefDescriptorExtractor`)

ROS Melodic ships with `opencv3` but **without** the contrib modules.  
Install the contrib package:

```bash
sudo apt-get install python-opencv   # ensures 3.x is default
sudo apt-get install libopencv-contrib-dev   # provides xfeatures2d
```

If you need to build from source:
```bash
# OpenCV 3.4.x with contrib
git clone --branch 3.4 https://github.com/opencv/opencv.git
git clone --branch 3.4 https://github.com/opencv/opencv_contrib.git
mkdir opencv/build && cd opencv/build
cmake .. \
  -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF
make -j$(nproc) && sudo make install
```

> **Note:** After installing, update the CMakeLists `find_package(OpenCV 3 REQUIRED)` to point to
> the correct prefix if needed: `set(OpenCV_DIR /usr/local/lib/cmake/opencv3)`.

### 1.4 Boost (for DBoW2 bitset)

Usually pre-installed; check:
```bash
sudo apt-get install libboost-all-dev
```

### 1.5 DBoW2

DBoW2 is the place-recognition library. The BRIEF variant (FBrief) is required.

```bash
# Clone and build
git clone https://github.com/dorian3d/DBoW2.git
cd DBoW2
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install
sudo ldconfig
```

After install, verify `DBoW2Config.cmake` exists:
```bash
find /usr/local -name "DBoW2Config.cmake"
```

If it is not generated, set the path manually in CMakeLists.txt:
```cmake
set(DBoW2_DIR /usr/local/lib/cmake/DBoW2)
```

#### DBoW2 Vocabulary

Download a pre-trained BRIEF vocabulary (k=10, L=6, 10-ary, 6 levels):

```bash
# Option A: ORB-SLAM2 community brief vocabulary
wget https://github.com/raulmur/ORB_SLAM2/raw/master/Vocabulary/ORBvoc.txt.tar.gz
tar -xzf ORBvoc.txt.tar.gz   # contains ORBvoc.txt (text format)
```

Or generate your own from a dataset:
```cpp
// Use DBoW2's training utility with your BRIEF descriptors
DBoW2::BriefVocabulary voc(10, 6);  // k=10, L=6
voc.create(training_descriptors);
voc.save("brief_k10L6.voc.gz");
```

Place the vocabulary at:
```
vtr_relocalization/config/brief_k10L6.voc.gz
```

---

## 2. Workspace Setup

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
# Clone or copy this package
cp -r vtr_phase3/src/vtr_relocalization .

# Also need the vtr_msgs package (message definitions shared across phases)
# Clone from your Phase 1/2 repository or create:
# See Section 5 below for required message definitions.

cd ~/catkin_ws
source /opt/ros/melodic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

---

## 3. Required vtr_msgs Definitions

Create a `vtr_msgs` package with these message files:

### MapGraph.msg (published by Phase 2)
```
# Serialised reduced map keyframe graph
std_msgs/Header header
vtr_msgs/MapKeyframe[] keyframes
```

### MapKeyframe.msg
```
int32 id
float64[16] T_rel      # 4x4 SE3 row-major
float64[16] T_skip     # T_{i}^{S(i)}
int32 skip_idx
float64[] features_2d  # interleaved (u, v) pairs
float64[] features_3d  # interleaved (x, y, z) triples
uint8[] bow_descriptor
float64 timestamp
```

### GoalList.msg (published by Phase 3)
```
std_msgs/Header header
geometry_msgs/Point[] goals   # 3D goal positions in robot frame
```

---

## 4. Running the Node

```bash
# With a pre-built map file from Phase 2:
roslaunch vtr_relocalization relocalization.launch \
  vocabulary_path:=/path/to/brief_k10L6.voc.gz \
  map_path:=/path/to/phase2_map.bin \
  camera_topic:=/stereo/left/image_raw

# With EuRoC MAV dataset (rosbag):
rosbag play MH_01_easy.bag --clock
roslaunch vtr_relocalization relocalization.launch \
  vocabulary_path:=$(rospack find vtr_relocalization)/config/brief_k10L6.voc.gz \
  camera_topic:=/cam0/image_raw \
  camera_fx:=458.654 camera_fy:=457.296 \
  camera_cx:=367.215 camera_cy:=248.375
```

---

## 5. Published Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/vtr/goal_list` | `geometry_msgs/PoseArray` | ~10 Hz | Lg: ordered 3D goal positions in robot frame |
| `/vtr/reloc_status` | `std_msgs/String` | ~10 Hz | Debug string: match KF id, reproj error, inliers |

## 6. Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | Live grayscale frames (remappable) |
| `/vtr/odometry` | `nav_msgs/Odometry` | Phase 4 odometry for goal frame updates |

---

## 7. Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `vocabulary_path` | — | **Required.** DBoW2 vocabulary file path |
| `map_path` | — | Phase 2 serialised map file (empty = wait for topic) |
| `tau_ret` | 0.012 | DBoW2 minimum score for candidate acceptance |
| `search_radius` | 40 | BRIEF window match search radius (pixels) |
| `hamming_thresh` | 60 | Maximum Hamming distance for BRIEF match |
| `pnp_max_reproj_err` | 2.5 | PnP rejection threshold (pixels RMS) |
| `pnp_min_inliers` | 12 | Minimum PnP inliers for acceptance |
| `pnp_max_iter` | 20 | Gauss-Newton maximum iterations |
| `goal_list_length` | 5 | M: number of goals maintained |
| `goal_arrival_dist` | 0.4 | Goal removal distance (metres) |
| `goal_angle_limit` | 60.0 | Goal removal angle limit (degrees) |
| `camera_fx/fy/cx/cy` | EuRoC | Camera intrinsics |

---

## 8. Phase 3 Acceptance Criteria

Per the implementation guide:

- ✅ PnP convergence rate **> 90%** when DBoW retrieval succeeds
- ✅ Goal list length: always **3–5 goals** when map coverage is adequate
- ✅ Reprojection error on accepted matches: **< 2.5 px** (RMS)
- ✅ Goal list update latency: **< 20 ms** per relocalization cycle
- ✅ No goal-list corruption when robot deviates **> 1 m** from teaching route

---

## 9. Integration with Other Phases

```
Phase 2 (Map Mgmt)  →  map file / topic  →  Phase 3 (this node)
Phase 3             →  /vtr/goal_list    →  Phase 4 (Motion Planner)
Phase 4             →  /vtr/odometry     →  Phase 3 (notifyOdometry)
```

Phase 4 must call `notifyOdometry` every control cycle (10 Hz) by publishing
to `/vtr/odometry`.  This keeps the goal list expressed in the current robot
frame, preventing drift between relocalization events.

---

## 10. Troubleshooting

**DBoW2 not found at cmake time:**
```cmake
set(DBoW2_DIR /usr/local/lib/cmake/DBoW2)
find_package(DBoW2 REQUIRED)
```

**`cv::xfeatures2d` not found:**  
Install `libopencv-contrib-dev` or rebuild OpenCV with `OPENCV_EXTRA_MODULES_PATH`.

**PnP convergence < 90%:**  
- Check `tau_ret` — try lowering to 0.008 to include more candidates.  
- Verify Phase 2 map clustering produced ≥1.5× feature density.  
- Ensure camera intrinsics match your sensor precisely.

**Goal list always empty:**  
- Check DBoW2 vocabulary matches the descriptor type used in Phase 1 (BRIEF-256).  
- Verify map keyframes have non-empty `features_3d` after Phase 2 clustering.
