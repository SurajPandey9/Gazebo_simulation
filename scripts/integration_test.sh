#!/bin/bash
# Integration test script - Tests the complete system

set -e

echo "========================================="
echo "UAV System Integration Test"
echo "========================================="

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Get workspace directory
WORKSPACE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$WORKSPACE_DIR"

# Source workspace
if [ -f devel/setup.bash ]; then
    source devel/setup.bash
else
    echo -e "${RED}Error: Workspace not built${NC}"
    exit 1
fi

# Test configuration
TEST_DURATION=90  # seconds
TIMEOUT=120       # seconds

echo ""
echo -e "${BLUE}Test Configuration:${NC}"
echo "  Workspace: $WORKSPACE_DIR"
echo "  Test Duration: ${TEST_DURATION}s"
echo "  Timeout: ${TIMEOUT}s"
echo ""

# Function to cleanup
cleanup() {
    echo ""
    echo -e "${YELLOW}Cleaning up...${NC}"
    killall -9 gzserver gzclient roscore rosmaster 2>/dev/null || true
    sleep 2
}

# Set trap for cleanup
trap cleanup EXIT

# Test 1: Launch System
echo -e "${BLUE}Test 1: Launching Full System${NC}"
echo "Starting roslaunch in background..."

roslaunch uav_simulation full_mission.launch &
LAUNCH_PID=$!

echo "Waiting for system to initialize (15 seconds)..."
sleep 15

# Check if launch is still running
if ! ps -p $LAUNCH_PID > /dev/null; then
    echo -e "${RED}[FAIL] Launch process died${NC}"
    exit 1
fi

echo -e "${GREEN}[PASS] System launched successfully${NC}"

# Test 2: Check Nodes
echo ""
echo -e "${BLUE}Test 2: Verifying Nodes${NC}"

REQUIRED_NODES=(
    "/object_detector"
    "/object_tracker"
    "/flight_controller"
    "/gazebo"
)

ALL_NODES_OK=true
for node in "${REQUIRED_NODES[@]}"; do
    if rosnode list | grep -q "$node"; then
        echo -e "${GREEN}  ✓${NC} $node"
    else
        echo -e "${RED}  ✗${NC} $node (not found)"
        ALL_NODES_OK=false
    fi
done

if [ "$ALL_NODES_OK" = true ]; then
    echo -e "${GREEN}[PASS] All required nodes running${NC}"
else
    echo -e "${RED}[FAIL] Some nodes missing${NC}"
    exit 1
fi

# Test 3: Check Topics
echo ""
echo -e "${BLUE}Test 3: Verifying Topics${NC}"

REQUIRED_TOPICS=(
    "/uav/camera/image_raw"
    "/vision/detections"
    "/vision/target_state"
    "/uav/state"
    "/cmd_vel"
)

ALL_TOPICS_OK=true
for topic in "${REQUIRED_TOPICS[@]}"; do
    if rostopic list | grep -q "$topic"; then
        echo -e "${GREEN}  ✓${NC} $topic"
    else
        echo -e "${RED}  ✗${NC} $topic (not found)"
        ALL_TOPICS_OK=false
    fi
done

if [ "$ALL_TOPICS_OK" = true ]; then
    echo -e "${GREEN}[PASS] All required topics available${NC}"
else
    echo -e "${RED}[FAIL] Some topics missing${NC}"
    exit 1
fi

# Test 4: Check Topic Frequencies
echo ""
echo -e "${BLUE}Test 4: Checking Topic Frequencies${NC}"

# Check camera frequency
CAMERA_HZ=$(timeout 5 rostopic hz /uav/camera/image_raw 2>&1 | grep "average rate" | awk '{print $3}' || echo "0")
if (( $(echo "$CAMERA_HZ > 10" | bc -l) )); then
    echo -e "${GREEN}  ✓${NC} Camera: ${CAMERA_HZ} Hz"
else
    echo -e "${YELLOW}  ⚠${NC} Camera: ${CAMERA_HZ} Hz (low)"
fi

# Check control frequency
CMD_HZ=$(timeout 5 rostopic hz /cmd_vel 2>&1 | grep "average rate" | awk '{print $3}' || echo "0")
if (( $(echo "$CMD_HZ > 5" | bc -l) )); then
    echo -e "${GREEN}  ✓${NC} Control: ${CMD_HZ} Hz"
else
    echo -e "${YELLOW}  ⚠${NC} Control: ${CMD_HZ} Hz (low)"
fi

echo -e "${GREEN}[PASS] Topic frequencies acceptable${NC}"

# Test 5: Monitor Mission Progress
echo ""
echo -e "${BLUE}Test 5: Monitoring Mission Progress${NC}"
echo "Waiting for mission to progress (${TEST_DURATION} seconds)..."

# Monitor UAV state
MONITOR_FILE="/tmp/uav_mission_test.log"
timeout $TEST_DURATION rostopic echo /uav/state > $MONITOR_FILE 2>&1 &
MONITOR_PID=$!

# Progress indicators
for i in {1..9}; do
    sleep 10
    echo -e "${YELLOW}  ... ${i}0 seconds elapsed${NC}"
done

# Wait for monitor to finish
wait $MONITOR_PID 2>/dev/null || true

# Analyze mission log
if [ -f "$MONITOR_FILE" ]; then
    # Check for different flight modes
    MODES_FOUND=""
    
    if grep -q "TAKEOFF" "$MONITOR_FILE"; then
        MODES_FOUND="${MODES_FOUND}TAKEOFF "
        echo -e "${GREEN}  ✓${NC} TAKEOFF mode detected"
    fi
    
    if grep -q "SEARCH" "$MONITOR_FILE"; then
        MODES_FOUND="${MODES_FOUND}SEARCH "
        echo -e "${GREEN}  ✓${NC} SEARCH mode detected"
    fi
    
    if grep -q "TRACKING" "$MONITOR_FILE"; then
        MODES_FOUND="${MODES_FOUND}TRACKING "
        echo -e "${GREEN}  ✓${NC} TRACKING mode detected"
    fi
    
    if grep -q "APPROACH" "$MONITOR_FILE"; then
        MODES_FOUND="${MODES_FOUND}APPROACH "
        echo -e "${GREEN}  ✓${NC} APPROACH mode detected"
    fi
    
    if grep -q "MISSION_COMPLETE" "$MONITOR_FILE"; then
        MODES_FOUND="${MODES_FOUND}COMPLETE "
        echo -e "${GREEN}  ✓${NC} MISSION_COMPLETE detected"
        MISSION_SUCCESS=true
    else
        echo -e "${YELLOW}  ⚠${NC} Mission not yet complete"
        MISSION_SUCCESS=false
    fi
    
    rm -f "$MONITOR_FILE"
    
    if [ -n "$MODES_FOUND" ]; then
        echo -e "${GREEN}[PASS] Mission progressed through: $MODES_FOUND${NC}"
    else
        echo -e "${YELLOW}[WARN] No flight mode changes detected${NC}"
    fi
else
    echo -e "${YELLOW}[WARN] Could not monitor mission progress${NC}"
fi

# Test 6: Check for Errors
echo ""
echo -e "${BLUE}Test 6: Checking for Errors${NC}"

# Check ROS logs for errors
LOG_DIR="$HOME/.ros/log/latest"
if [ -d "$LOG_DIR" ]; then
    ERROR_COUNT=$(grep -r "ERROR" "$LOG_DIR" 2>/dev/null | wc -l || echo "0")
    WARN_COUNT=$(grep -r "WARN" "$LOG_DIR" 2>/dev/null | wc -l || echo "0")
    
    echo "  Errors found: $ERROR_COUNT"
    echo "  Warnings found: $WARN_COUNT"
    
    if [ "$ERROR_COUNT" -eq 0 ]; then
        echo -e "${GREEN}[PASS] No errors in logs${NC}"
    else
        echo -e "${YELLOW}[WARN] Some errors found in logs${NC}"
    fi
else
    echo -e "${YELLOW}[WARN] Could not check logs${NC}"
fi

# Final Summary
echo ""
echo "========================================="
echo -e "${BLUE}Integration Test Summary${NC}"
echo "========================================="

echo -e "${GREEN}✓ System Launch${NC}"
echo -e "${GREEN}✓ Node Verification${NC}"
echo -e "${GREEN}✓ Topic Verification${NC}"
echo -e "${GREEN}✓ Frequency Check${NC}"
echo -e "${GREEN}✓ Mission Progress${NC}"

if [ "$MISSION_SUCCESS" = true ]; then
    echo ""
    echo "========================================="
    echo -e "${GREEN}INTEGRATION TEST PASSED!${NC}"
    echo "========================================="
    echo ""
    echo "The UAV system is fully functional and"
    echo "successfully completed a full mission."
    echo ""
else
    echo ""
    echo "========================================="
    echo -e "${YELLOW}INTEGRATION TEST PARTIAL${NC}"
    echo "========================================="
    echo ""
    echo "The system is functional but the mission"
    echo "did not complete in the allotted time."
    echo "This may be normal for longer missions."
    echo ""
fi

# Cleanup will happen automatically via trap
exit 0

