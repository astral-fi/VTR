#!/usr/bin/env bash
# =============================================================================
# install_dependencies.sh
# Complete dependency installation script for Phase 2 (vtr_map_manager)
# Target: Ubuntu 18.04 + ROS Melodic
#
# Run as: bash install_dependencies.sh
# =============================================================================

set -e  # exit on error
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "============================================================"
echo " VTR Phase 2 — Dependency Installer"
echo " Target: Ubuntu 18.04 / ROS Melodic"
echo "============================================================"

# ------------------------------------------------------------
# 1. ROS Melodic base
# ------------------------------------------------------------
echo ""
echo "[1/7] Installing ROS Melodic packages..."

sudo apt-get update -qq

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
    ros-melodic-std-srvs \
    python-catkin-tools \
    python-rosdep

echo "    [OK] ROS Melodic packages installed."

# ------------------------------------------------------------
# 2. System libraries
# ------------------------------------------------------------
echo ""
echo "[2/7] Installing system libraries (OpenCV, Eigen, Boost)..."

sudo apt-get install -y \
    libopencv-dev \
    libopencv-contrib-dev \
    libeigen3-dev \
    libboost-all-dev \
    cmake \
    git \
    build-essential \
    wget \
    unzip

echo "    [OK] System libraries installed."

# ------------------------------------------------------------
# 3. DBoW2
# ------------------------------------------------------------
echo ""
echo "[3/7] Building and installing DBoW2..."

DBOW2_DIR="/tmp/DBoW2_build"
DBOW2_INSTALL="/usr/local"

if [ ! -f "${DBOW2_INSTALL}/lib/libDBoW2.so" ]; then
    mkdir -p "${DBOW2_DIR}"
    cd "${DBOW2_DIR}"

    # Clone DBoW2
    if [ ! -d "DBoW2" ]; then
        git clone https://github.com/dorian3d/DBoW2.git
    fi

    cd DBoW2
    mkdir -p build && cd build
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="${DBOW2_INSTALL}"
    make -j$(nproc)
    sudo make install

    echo "    [OK] DBoW2 installed to ${DBOW2_INSTALL}"
else
    echo "    [SKIP] DBoW2 already installed at ${DBOW2_INSTALL}/lib/libDBoW2.so"
fi

# DBoW2 cmake config (if not created by make install, create manually)
DBOW2_CMAKE_DIR="/usr/local/lib/cmake/DBoW2"
if [ ! -f "${DBOW2_CMAKE_DIR}/DBoW2Config.cmake" ]; then
    sudo mkdir -p "${DBOW2_CMAKE_DIR}"
    cat <<EOF | sudo tee "${DBOW2_CMAKE_DIR}/DBoW2Config.cmake" > /dev/null
# DBoW2Config.cmake (manually generated)
set(DBoW2_FOUND TRUE)
set(DBoW2_INCLUDE_DIRS /usr/local/include/DBoW2)
set(DBoW2_LIBRARIES /usr/local/lib/libDBoW2.so)
EOF
    echo "    [OK] DBoW2 cmake config written."
fi

# ------------------------------------------------------------
# 4. DLoopDetector (optional, for enhanced loop verification)
# ------------------------------------------------------------
echo ""
echo "[4/7] Building DLoopDetector (optional)..."

DLOOP_DIR="/tmp/DLoopDetector_build"
if [ ! -d "/usr/local/include/DLoopDetector" ]; then
    mkdir -p "${DLOOP_DIR}" && cd "${DLOOP_DIR}"
    if [ ! -d "DLoopDetector" ]; then
        git clone https://github.com/dorian3d/DLoopDetector.git
    fi
    cd DLoopDetector
    mkdir -p build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
    make -j$(nproc)
    sudo make install
    echo "    [OK] DLoopDetector installed."
else
    echo "    [SKIP] DLoopDetector already installed."
fi

# ------------------------------------------------------------
# 5. ORB DBoW2 Vocabulary
# ------------------------------------------------------------
echo ""
echo "[5/7] Verifying ORB vocabulary for DBoW2..."

VOCAB_FILE="${HOME}/ORB_SLAM3/Vocabulary/ORBvoc.txt"

if [ -f "${VOCAB_FILE}" ]; then
    echo "    [OK] Found vocabulary at ${VOCAB_FILE}"
else
    echo "    [WARNING] Vocabulary not found at ${VOCAB_FILE}"
    echo "    Please ensure ORB-SLAM3 is installed in your home directory,"
    echo "    or update the launch files to point to its installed location."
fi

# ------------------------------------------------------------
# 6. OpenVINS (Phase 1 dependency — needed for Phase 2 integration)
# ------------------------------------------------------------
echo ""
echo "[6/7] Checking OpenVINS..."

OPENVINS_WS="${HOME}/catkin_ws_openvins"
if [ ! -d "${OPENVINS_WS}/src/open_vins" ]; then
    echo "    OpenVINS not found at ${OPENVINS_WS}."
    echo "    To install OpenVINS (Phase 1):"
    echo "      mkdir -p ${OPENVINS_WS}/src && cd ${OPENVINS_WS}/src"
    echo "      git clone https://github.com/rpng/open_vins.git"
    echo "      cd ${OPENVINS_WS} && catkin build"
    echo ""
    echo "    Phase 2 does NOT require OpenVINS at build time."
    echo "    It only needs the /vtr/keyframe topic at runtime."
else
    echo "    [OK] OpenVINS found at ${OPENVINS_WS}"
fi

# ------------------------------------------------------------
# 7. rosdep and workspace setup reminder
# ------------------------------------------------------------
echo ""
echo "[7/7] Running rosdep for remaining ROS dependencies..."

source /opt/ros/melodic/setup.bash

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update --quiet

echo ""
echo "============================================================"
echo " Installation complete!"
echo "============================================================"
echo ""
echo "NEXT STEPS:"
echo ""
echo "  1. Create or reuse a catkin workspace:"
echo "       mkdir -p ~/catkin_ws/src"
echo "       cd ~/catkin_ws/src"
echo ""
echo "  2. Copy or symlink this package:"
echo "       ln -s $(readlink -f ${SCRIPT_DIR}) ~/catkin_ws/src/vtr_map_manager"
echo ""
echo "  3. Build:"
echo "       cd ~/catkin_ws"
echo "       source /opt/ros/melodic/setup.bash"
echo "       catkin build vtr_map_manager"
echo ""
echo "  4. Source the workspace:"
echo "       source ~/catkin_ws/devel/setup.bash"
echo ""
echo "  5. Teaching phase:"
echo "       roslaunch vtr_map_manager vtr_teaching.launch"
echo ""
echo "  6. After teaching, run offline clustering:"
echo "       rosservice call /vtr/run_clustering"
echo ""
echo "  7. Save the map:"
echo "       rosservice call /vtr/save_map_full"
echo ""
echo "  8. Repeating phase:"
echo "       roslaunch vtr_map_manager vtr_repeating.launch"
echo ""
echo "  Vocabulary path: ${VOCAB_FILE}"
echo ""
