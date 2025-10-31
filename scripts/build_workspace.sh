#!/bin/bash
# Build workspace script

set -e

echo "========================================="
echo "Building UAV Workspace"
echo "========================================="

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Get workspace directory
WORKSPACE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$WORKSPACE_DIR"

echo -e "${GREEN}Workspace: $WORKSPACE_DIR${NC}"

# Source ROS
if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
    echo -e "${GREEN}Sourced ROS Noetic${NC}"
else
    echo -e "${RED}Error: ROS Noetic not found${NC}"
    exit 1
fi

# Clean build (optional)
if [ "$1" == "clean" ]; then
    echo -e "${YELLOW}Cleaning workspace...${NC}"
    rm -rf build/ devel/ install/
    echo -e "${GREEN}Clean complete${NC}"
fi

# Build
echo -e "${GREEN}Building workspace...${NC}"

if command -v catkin_make &> /dev/null; then
    catkin_make
    BUILD_RESULT=$?
elif command -v catkin &> /dev/null; then
    catkin build
    BUILD_RESULT=$?
else
    echo -e "${RED}Error: No catkin build tool found${NC}"
    exit 1
fi

if [ $BUILD_RESULT -eq 0 ]; then
    echo ""
    echo "========================================="
    echo -e "${GREEN}Build Successful!${NC}"
    echo "========================================="
    echo ""
    echo "To use the workspace, run:"
    echo "  source devel/setup.bash"
    echo ""
else
    echo ""
    echo "========================================="
    echo -e "${RED}Build Failed!${NC}"
    echo "========================================="
    exit 1
fi

