#!/bin/bash
# Run automated tests

set -e

echo "========================================="
echo "UAV System Test Suite"
echo "========================================="

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Get workspace directory
WORKSPACE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$WORKSPACE_DIR"

# Source workspace
if [ -f devel/setup.bash ]; then
    source devel/setup.bash
else
    echo -e "${RED}Error: Workspace not built. Run build_workspace.sh first${NC}"
    exit 1
fi

# Test results
TESTS_PASSED=0
TESTS_FAILED=0
TESTS_WARNED=0

# Test function
run_test() {
    local test_name="$1"
    local test_command="$2"
    
    echo ""
    echo -e "${YELLOW}Running: $test_name${NC}"
    
    if eval "$test_command"; then
        echo -e "${GREEN}[PASS]${NC} $test_name"
        ((TESTS_PASSED++))
        return 0
    else
        echo -e "${RED}[FAIL]${NC} $test_name"
        ((TESTS_FAILED++))
        return 1
    fi
}

# Test 1: Check ROS installation
run_test "ROS Installation" "rosversion -d | grep -q noetic"

# Test 2: Check package dependencies
run_test "Package Dependencies" "rosdep check --from-paths src --ignore-src -r -y"

# Test 3: Check Python dependencies
run_test "Python Dependencies" "python3 -c 'import cv2, torch, numpy, scipy, sklearn, filterpy'"

# Test 4: Check message generation
run_test "Message Generation" "rosmsg show uav_msgs/Detection"

# Test 5: Check service generation
run_test "Service Generation" "rossrv show uav_msgs/EngageTarget"

# Test 6: Check launch files exist
run_test "Launch Files" "test -f src/uav_simulation/launch/full_mission.launch"

# Test 7: Check Python nodes are executable
run_test "Node Executables" "test -x src/uav_vision/nodes/object_detector.py"

# Test 8: Check configuration files
run_test "Config Files" "test -f config/flight_params.yaml && test -f config/vision_params.yaml"

# Test 9: Check models exist
run_test "Gazebo Models" "test -f src/uav_simulation/models/tank/model.sdf"

# Test 10: Check RViz config
run_test "RViz Config" "test -f src/uav_simulation/config/mission.rviz"

echo ""
echo "========================================="
echo "Test Summary"
echo "========================================="
echo -e "${GREEN}Passed: $TESTS_PASSED${NC}"
echo -e "${RED}Failed: $TESTS_FAILED${NC}"
echo -e "${YELLOW}Warned: $TESTS_WARNED${NC}"
echo "========================================="

if [ $TESTS_FAILED -eq 0 ]; then
    echo -e "${GREEN}All tests passed!${NC}"
    exit 0
else
    echo -e "${RED}Some tests failed!${NC}"
    exit 1
fi

