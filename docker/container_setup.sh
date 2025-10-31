#!/bin/bash
# Setup script to run inside Docker container
# This installs dependencies, builds workspace, and prepares for launch

set -e

echo "========================================"
echo "UAV Simulation - Container Setup"
echo "========================================"
echo ""

# Navigate to workspace
cd /workspace

# Check if we're in the right directory
if [ ! -f "README.md" ]; then
    echo "ERROR: Not in workspace directory!"
    exit 1
fi

echo "[1/5] Updating package lists..."
apt-get update > /dev/null 2>&1

echo "[2/5] Installing ROS dependencies..."
apt-get install -y \
    python3-pip \
    python3-opencv \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-vision-opencv \
    ros-noetic-hector-quadrotor \
    ros-noetic-hector-quadrotor-gazebo \
    ros-noetic-hector-quadrotor-description \
    ros-noetic-hector-quadrotor-teleop \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    > /dev/null 2>&1

echo "[3/5] Installing Python dependencies..."
pip3 install --upgrade pip > /dev/null 2>&1
pip3 install -q -r requirements.txt

echo "[4/5] Setting file permissions..."
chmod +x scripts/*.sh scripts/*.py 2>/dev/null || true
find src -name "*.py" -type f -exec chmod +x {} \; 2>/dev/null || true

echo "[5/5] Building ROS workspace..."
source /opt/ros/noetic/setup.bash

# Clean build
if [ -d "build" ]; then
    echo "  Cleaning previous build..."
    rm -rf build/ devel/
fi

echo "  Running catkin_make..."
catkin_make

# Source workspace
source devel/setup.bash

echo ""
echo "========================================"
echo "Setup Complete!"
echo "========================================"
echo ""
echo "Workspace built successfully!"
echo ""
echo "To launch the simulation:"
echo "  source devel/setup.bash"
echo "  roslaunch uav_simulation full_mission.launch"
echo ""
echo "To test GUI:"
echo "  xclock"
echo ""
echo "To verify messages:"
echo "  rosmsg list | grep uav_msgs"
echo ""
echo "========================================"
echo ""

