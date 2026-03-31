# VTR Phase 2 — Topo-metric Graph & Map Management

**ROS Melodic | Ubuntu 18.04**

Implementation of Phase 2 from:
> Wang et al., *"Robust Visual Teach-and-Repeat Navigation with Flexible Topo-metric Graph Map Representation"*, arXiv:2510.09089v1, 2025.

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Prerequisites](#2-prerequisites)
3. [Dependency Installation](#3-dependency-installation)
4. [Workspace Setup and Build](#4-workspace-setup-and-build)
5. [Phase 2 Node Reference](#5-phase-2-node-reference)
6. [Teaching Workflow](#6-teaching-workflow)
7. [Repeating Workflow](#7-repeating-workflow)
8. [Topic and Service Reference](#8-topic-and-service-reference)
9. [Parameter Reference](#9-parameter-reference)
10. [Integration Contracts with Phase 1 and Phase 3](#10-integration-contracts-with-phase-1-and-phase-3)
11. [Acceptance Criteria](#11-acceptance-criteria)
12. [Troubleshooting](#12-troubleshooting)

---

## 1. System Overview

Phase 2 owns the **topo-metric map**. The map stores only relative poses between adjacent nodes — never global coordinates — making it resilient to odometry drift (paper Eq. 4).

```
Phase 1 (OpenVINS)  --/vtr/keyframe-->  Phase 2 (this package)
                                             |
                            /vtr/loop_detection_result
                            /vtr/map_node
                                             |
                                        Phase 3 (Relocalization)
                                             |
                                        Phase 4 (Motion Planning)
```

Phase 2 responsibilities:

| Task | Phase |
|---|---|
| Ingest keyframes, build raw graph, detect loops | Teaching |
| Offline keyframe clustering (redundancy reduction) | Post-teaching |
| Save reduced map to disk | Post-teaching |
| Load map at startup | Repeating |
| Answer live DBoW2 loop queries from Phase 3 | Repeating |
| Expand map with novel frames | Repeating |

---

## 2. Prerequisites

| Component | Version |
|---|---|
| Ubuntu | 18.04 LTS |
| ROS | Melodic Morenia |
| OpenCV | 3.x (ROS Melodic default) |
| Eigen3 | >= 3.3 |
| Boost | >= 1.65 (serialization, filesystem) |
| DBoW2 | Built from source with BRIEF support |

---

## 3. Dependency Installation

### 3.1 ROS Melodic system packages

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-melodic-desktop-full \
  ros-melodic-cv-bridge \
  ros-melodic-image-transport \
  ros-melodic-tf \
  ros-melodic-tf2 \
  ros-melodic-tf2-ros \
  ros-melodic-tf2-geometry-msgs \
  ros-melodic-nav-msgs \
  ros-melodic-visualization-msgs \
  cmake build-essential \
  libeigen3-dev \
  libboost-all-dev \
  libopencv-dev
```

### 3.2 DBoW2 from source (required — no apt package)

DBoW2 must be compiled with BRIEF descriptor support. The standard apt package does not include it.

```bash
cd ~
git clone https://github.com/dorian3d/DBoW2.git
cd DBoW2
mkdir build && cd build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local

make -j$(nproc)
sudo make install
sudo ldconfig
```

Verify the installation:

```bash
ls /usr/local/include/DBoW2/FBrief.h   # must exist
ls /usr/local/lib/libDBoW2.so           # must exist
```

If `FBrief.h` is missing from the DBoW2 repo, copy it from ORB-SLAM2:

```bash
cd ~/DBoW2/DBoW2
wget https://raw.githubusercontent.com/raulmur/ORB_SLAM2/master/Thirdparty/DBoW2/DBoW2/FBrief.cpp
wget https://raw.githubusercontent.com/raulmur/ORB_SLAM2/master/Thirdparty/DBoW2/DBoW2/FBrief.h
# Rebuild after adding these files
```

### 3.3 DBoW2 vocabulary file

A pre-trained BRIEF vocabulary is required at runtime.

```bash
sudo mkdir -p /opt/vtr/vocab
cd /opt/vtr/vocab

# Option A: generate a small demo vocabulary using DBoW2's built-in demo
# (good for testing; train on your own images for production)
cd ~/DBoW2/build
./demo
sudo cp brief_k10L6.voc.gz /opt/vtr/vocab/

# Option B: use the ORB-SLAM2 vocabulary (larger, better for diverse scenes)
# Download from: https://github.com/raulmur/ORB_SLAM2/tree/master/Vocabulary
# ORBvoc.txt works with DBoW2 in text mode — update vocabulary_path accordingly.
```

Set the vocabulary path in every launch file:
```xml
<param name="vocabulary_path" value="/opt/vtr/vocab/brief_k10L6.voc.gz" />
```

---

## 4. Workspace Setup and Build

```bash
# Source ROS Melodic
source /opt/ros/melodic/setup.bash

# Create workspace (skip if you already have one)
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Copy this package
cp -r /path/to/vtr_map_manager .

# Build
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release

# If CMake cannot find DBoW2 automatically, pass the paths explicitly:
catkin_make -DCMAKE_BUILD_TYPE=Release \
  -DDBoW2_INCLUDE_DIRS=/usr/local/include \
  -DDBoW2_LIBRARIES=/usr/local/lib/libDBoW2.so

# Source the overlay
source devel/setup.bash
# Add to .bashrc for persistent sourcing:
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

Verify the build:

```bash
rospack find vtr_map_manager
# Expected output: /home/<user>/catkin_ws/src/vtr_map_manager
```

---

## 5. Phase 2 Node Reference

| Node binary | Purpose |
|---|---|
| `map_manager_node` | Main orchestrator: teaching or repeating mode |
| `map_save_load_node` | Serializes the reduced map to disk; loads on startup |
| `map_visualizer_node` | Publishes RViz `MarkerArray` of the graph |

---

## 6. Teaching Workflow

### Step 1: Start Phase 2 (teaching mode)

```bash
roslaunch vtr_map_manager teaching.launch \
  vocabulary_path:=/opt/vtr/vocab/brief_k10L6.voc.gz \
  map_name:=office_route_01 \
  image_width:=640 \
  image_height:=480
```

### Step 2: Start Phase 1

Phase 1 must publish `vtr_map_manager/Keyframe` on `/vtr/keyframe`. If using the companion Phase 1 package:

```bash
roslaunch vtr_perception openvins_keyframe.launch
```

### Step 3: Drive the teaching route

Teleoperate the robot through the complete teaching route. Monitor in RViz:

```bash
rviz
# Add: MarkerArray -> /vtr/map_markers (shows nodes and loop edges)
rostopic echo /vtr/loop_detection_result  # watch for detected loops
```

### Step 4: Run offline keyframe clustering

After the robot completes the teaching route:

```bash
rosservice call /vtr/run_clustering "{}"
```

Expected response:
```
success: True
message: "Clustering complete. Reduced map: 84 nodes."
```

The reduced count should be less than 40% of the raw keyframe count. If not, increase `dbow_cluster_threshold` (see §9).

### Step 5: Save the map

```bash
rosservice call /vtr/save_map "{}"
```

The map is written to `map_save_path`. Verify:

```bash
ls ~/vtr_maps/teaching_office_route_01/
# map.bin    dbow_db.bin    images/
```

### Step 6: Shut down

```bash
rosnode kill /map_manager_node /map_visualizer_node
```

---

## 7. Repeating Workflow

### Step 1: Start Phase 2 (repeating mode)

```bash
roslaunch vtr_map_manager repeating.launch \
  map_path:=$HOME/vtr_maps/teaching_office_route_01 \
  vocabulary_path:=/opt/vtr/vocab/brief_k10L6.voc.gz
```

This starts three nodes in sequence:
1. `map_save_load_node` loads `map.bin` and publishes all `MapNode` messages
2. `map_manager_node` starts (2 s delay) and begins answering query frames
3. `map_visualizer_node` publishes markers at 1 Hz

### Step 2: Start Phase 1, Phase 3, and Phase 4

```bash
roslaunch vtr_perception openvins_keyframe.launch
roslaunch vtr_relocalization relocalize.launch
roslaunch vtr_motion_planning local_planner.launch
```

### Step 3: Position robot and start navigating

Place the robot at approximately the start of the teaching route. Phase 3 will begin issuing query frames to `/vtr/query_frame`; Phase 2 will return DBoW2 candidates on `/vtr/loop_detection_result`.

Monitor:

```bash
rostopic hz /vtr/loop_detection_result   # should be ~10 Hz
rostopic echo /vtr/loop_detection_result | grep success  # watch match rate
```

---

## 8. Topic and Service Reference

### Published Topics

| Topic | Message Type | Node | Description |
|---|---|---|---|
| `/vtr/loop_detection_result` | `LoopDetectionResult` | `map_manager_node` | DBoW2 result for every keyframe (teaching) or query (repeating) |
| `/vtr/map_node` | `MapNode` | `map_save_load_node` | One message per map node, published at load time |
| `/vtr/map_markers` | `visualization_msgs/MarkerArray` | `map_visualizer_node` | RViz graph at 1 Hz |

### Subscribed Topics

| Topic | Message Type | Node | Description |
|---|---|---|---|
| `/vtr/keyframe` | `Keyframe` | `map_manager_node` (teaching) | New keyframe from Phase 1 |
| `/vtr/query_frame` | `Keyframe` | `map_manager_node` (repeating) | Live frame from Phase 3 |
| `/vtr/odometry` | `nav_msgs/Odometry` | `map_manager_node` (repeating) | Robot odometry for map expansion |

### Services

| Service | Type | Description |
|---|---|---|
| `/vtr/run_clustering` | `std_srvs/Trigger` | Run offline keyframe clustering after teaching |
| `/vtr/save_map` | `std_srvs/Trigger` | Save clustered map to disk |
| `/vtr/load_map` | `std_srvs/Trigger` | Debug: trigger map reload |

---

## 9. Parameter Reference

All parameters scoped under `map_manager_node` namespace (`~`).

| Parameter | Default | Description |
|---|---|---|
| `~mode` | `"teaching"` | Operating mode: `"teaching"` or `"repeating"` |
| `~vocabulary_path` | `/opt/vtr/vocab/brief_k10L6.voc.gz` | Path to the DBoW2 BRIEF vocabulary |
| `~map_save_path` | `/tmp/vtr_map` | Root directory for map files |
| `~temporal_gap` | `30` | Reject DBoW candidates within this many recent nodes |
| `~dbow_cluster_threshold` | `0.05` | Similarity cutoff for keyframe clustering (τ_cluster) |
| `~dbow_score_threshold_teaching` | `0.015` | Minimum DBoW score during teaching (τ_bow) |
| `~dbow_score_threshold_repeating` | `0.012` | Minimum DBoW score for live queries (τ_ret) |
| `~dbow_top_n_candidates` | `5` | Candidates returned per live query |
| `~min_features_per_node` | `200` | Minimum features after clustering |
| `~max_features_per_node` | `800` | Maximum features after clustering |
| `~grid_filter_cols` | `8` | Grid filter columns (feature distribution) |
| `~grid_filter_rows` | `6` | Grid filter rows |
| `~grid_filter_max_per_cell` | `5` | Max features retained per grid cell |
| `~image_width` | `640` | Camera image width in pixels |
| `~image_height` | `480` | Camera image height in pixels |

---

## 10. Integration Contracts with Phase 1 and Phase 3

### Phase 1 publishes to `/vtr/keyframe` (Keyframe.msg)

```
keyframe_id           int32       sequential index from Phase 1
T_rel                 float64[16] row-major 4x4 SE(3): T_{i-1}^{i}
features_u, features_v float32[]  pixel coords (N features)
features_descriptors  uint8[]     N x 32 bytes (BRIEF 256-bit descriptors)
features_x/y/z        float32[]   3D positions in camera frame (parallel to 2D)
bow_word_ids/weights              DBoW2 BowVector
image                 sensor_msgs/Image  grayscale CV_8UC1
```

### Phase 2 publishes to `/vtr/loop_detection_result` (LoopDetectionResult.msg)

```
matched_node_id       int32       map node ID of best DBoW candidate (-1 = none)
dbow_score            float32     similarity score
success               bool        true if a candidate above threshold was found
query_keyframe_id     int32       echoes the incoming keyframe_id
```

Phase 3 must then perform its own BRIEF window matching and Gauss-Newton PnP on the candidate node's extended feature set (`U_bar`, `P_bar` from the MapNode message).

### Phase 2 publishes to `/vtr/map_node` (MapNode.msg)

Contains the full enhanced keyframe data: features, descriptors, transforms. Phase 3 caches these at startup to avoid repeated lookups.

---

## 11. Acceptance Criteria

Criteria from the implementation guide (Section: Phase 2 Acceptance Criteria):

| Criterion | Target | Verification |
|---|---|---|
| Map size after clustering | < 40% of raw count | Check log output from `/vtr/run_clustering` |
| Loop detection recall (clean revisit) | > 85% | Run repeating same day; count `success=true` messages |
| Enhanced feature count | >= 1.5x original | Logged per-node by `keyframe_clusterer.cpp` |
| `queryLoop()` latency | < 30 ms per call | `rostopic delay /vtr/loop_detection_result` |
| Map expansion: topology intact | Novel frames attach without breaking traversal | Check expansion_frame_ids in MapNode msgs |

---

## 12. Troubleshooting

**DBoW2 vocabulary not loading**

```
[MapManagerNode] DBoW2 vocabulary failed to load.
```

Check that the file exists and is readable. Some builds require the uncompressed `.txt` format:

```bash
cd /opt/vtr/vocab
gunzip brief_k10L6.voc.gz
# Then update vocabulary_path to point to brief_k10L6.voc (no .gz)
```

**No loop candidates detected during teaching**

Lower `temporal_gap` for short routes (< 10 m), or reduce `dbow_score_threshold_teaching` to `0.010`. Confirm features are non-zero: `rostopic echo /vtr/keyframe -n 1 | grep features_u`.

**Clustering reduces map by less than 60%**

The scene has high visual diversity between adjacent frames. Increase `dbow_cluster_threshold` to `0.08`. Also check that Phase 1 is not producing extremely sparse descriptors (< 100 features per keyframe).

**`catkin_make` cannot find DBoW2**

```
Could not find a package configuration file provided by "DBoW2"
```

DBoW2 does not always install a CMake config file. Pass explicit paths:

```bash
catkin_make \
  -DDBoW2_INCLUDE_DIRS=/usr/local/include \
  -DDBoW2_LIBRARIES=/usr/local/lib/libDBoW2.so
```

**map_save_load_node cannot find map.bin**

Confirm the clustering and save steps both completed before shutdown. The save operation writes three artefacts: `map.bin`, `dbow_db.bin`, and `images/`. If only some exist, re-run clustering and save.

**RViz shows no markers**

Add a `MarkerArray` display in RViz and set the topic to `/vtr/map_markers`. Set the Fixed Frame to `odom`. If no markers appear after 2 s, check that `map_visualizer_node` is running and has received at least one `MapNode` message.
