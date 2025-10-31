#!/bin/bash
# Setup script for UAV Target Detection and Engagement System

set -e  # Exit on error

echo "========================================="
echo "UAV System Setup Script"
echo "========================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if running on Ubuntu
if [ ! -f /etc/lsb-release ]; then
    echo -e "${RED}Error: This script is designed for Ubuntu${NC}"
    exit 1
fi

# Check Ubuntu version
source /etc/lsb-release
if [ "$DISTRIB_RELEASE" != "20.04" ]; then
    echo -e "${YELLOW}Warning: This script is tested on Ubuntu 20.04. You have $DISTRIB_RELEASE${NC}"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo -e "${GREEN}Step 1: Updating system packages${NC}"
sudo apt-get update

echo -e "${GREEN}Step 2: Installing ROS Noetic (if not already installed)${NC}"
if ! command -v roscore &> /dev/null; then
    echo "ROS not found. Installing ROS Noetic..."
    
    # Setup sources.list
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    
    # Setup keys
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    
    # Update and install
    sudo apt-get update
    sudo apt-get install -y ros-noetic-desktop-full
    
    # Initialize rosdep
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init
    fi
    rosdep update
    
    # Setup environment
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source /opt/ros/noetic/setup.bash
    
    # Install build tools
    sudo apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    
    echo -e "${GREEN}ROS Noetic installed successfully${NC}"
else
    echo "ROS already installed"
fi

echo -e "${GREEN}Step 3: Installing ROS packages${NC}"
sudo apt-get install -y \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-hector-quadrotor \
    ros-noetic-hector-quadrotor-gazebo \
    ros-noetic-hector-quadrotor-description \
    ros-noetic-hector-quadrotor-teleop \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-vision-opencv \
    ros-noetic-tf \
    ros-noetic-rqt-image-view

echo -e "${GREEN}Step 4: Installing Python dependencies${NC}"
sudo apt-get install -y \
    python3-pip \
    python3-opencv \
    python3-numpy \
    python3-scipy

# Install Python packages
pip3 install --upgrade pip
pip3 install -r requirements.txt

echo -e "${GREEN}Step 5: Building workspace${NC}"
cd "$(dirname "$0")/.."

# Source ROS
source /opt/ros/noetic/setup.bash

# Build with catkin_make
if command -v catkin_make &> /dev/null; then
    catkin_make
    echo -e "${GREEN}Workspace built with catkin_make${NC}"
elif command -v catkin &> /dev/null; then
    catkin build
    echo -e "${GREEN}Workspace built with catkin build${NC}"
else
    echo -e "${RED}Error: No catkin build tool found${NC}"
    exit 1
fi

echo -e "${GREEN}Step 6: Setting up environment${NC}"
WORKSPACE_DIR="$(pwd)"
SETUP_FILE="$WORKSPACE_DIR/devel/setup.bash"

if [ -f "$SETUP_FILE" ]; then
    # Add to bashrc if not already there
    if ! grep -q "$SETUP_FILE" ~/.bashrc; then
        echo "source $SETUP_FILE" >> ~/.bashrc
        echo -e "${GREEN}Added workspace to ~/.bashrc${NC}"
    fi
    
    # Source for current session
    source "$SETUP_FILE"
else
    echo -e "${RED}Error: Setup file not found at $SETUP_FILE${NC}"
    exit 1
fi

echo -e "${GREEN}Step 7: Downloading YOLOv5 model (optional)${NC}"
read -p "Download YOLOv5 model now? This will take a few minutes. (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    python3 -c "import torch; torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)"
    echo -e "${GREEN}YOLOv5 model downloaded${NC}"
fi

echo ""
echo "========================================="
echo -e "${GREEN}Setup Complete!${NC}"
echo "========================================="
echo ""
echo "To use the system:"
echo "1. Source the workspace: source devel/setup.bash"
echo "2. Launch the simulation: roslaunch uav_simulation full_mission.launch"
echo ""
echo "For more information, see README.md"
echo ""

