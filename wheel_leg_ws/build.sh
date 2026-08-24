#!/bin/bash
# ============================================================
# Build & Deploy Script for wheel_leg_ws on RDK X5
#
# Prerequisites:
#   sudo apt install -y ros-humble-joy
#   sudo apt install -y tros-humble-mipi-cam
#   sudo apt install -y ros-humble-cv-bridge
#   sudo apt install -y ros-humble-tf2-ros
# ============================================================

set -e

WS_DIR="$HOME/wheel_leg_ws"

echo "========================================"
echo "  Wheel-Leg Robot Build Script"
echo "========================================"

# Step 1: Install dependencies
echo "[1/4] Installing ROS2 dependencies..."
sudo apt update
sudo apt install -y \
  ros-humble-joy \
  ros-humble-cv-bridge \
  ros-humble-tf2-ros \
  ros-humble-nav-msgs \
  tros-humble-mipi-cam

# Step 2: Source TROS environment
echo "[2/4] Sourcing TROS environment..."
source /opt/tros/humble/setup.bash

# Step 3: Build workspace
echo "[3/4] Building workspace..."
cd "$WS_DIR"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Step 4: Source workspace
echo "[4/4] Sourcing workspace..."
source "$WS_DIR/install/setup.bash"

echo ""
echo "========================================"
echo "  Build Complete!"
echo ""
echo "  To launch the full system:"
echo "    source /opt/tros/humble/setup.bash"
echo "    source ~/wheel_leg_ws/install/setup.bash"
echo "    ros2 launch wheel_leg_bringup wheel_leg_bringup.launch.py"
echo ""
echo "  For yellow tape, add: color:=yellow"
echo "  For USB serial, add: serial_port:=/dev/ttyUSB0"
echo "========================================"
