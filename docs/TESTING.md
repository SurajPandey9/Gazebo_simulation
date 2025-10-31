# Testing Guide

Comprehensive testing procedures for the UAV Target Detection and Engagement System.

## Table of Contents
1. [Unit Testing](#unit-testing)
2. [Integration Testing](#integration-testing)
3. [System Testing](#system-testing)
4. [Performance Testing](#performance-testing)
5. [Edge Case Testing](#edge-case-testing)

---

## Unit Testing

### Vision System Tests

#### Test 1: Object Detection Accuracy

**Objective:** Verify detection accuracy at various altitudes

**Procedure:**
```bash
# Launch Gazebo with tank
roslaunch uav_simulation battlefield.launch

# Manually position UAV at different altitudes
# Use teleop or Gazebo GUI

# Monitor detections
rostopic echo /vision/detections
```

**Expected Results:**
- 5m altitude: >95% detection rate
- 10m altitude: >90% detection rate
- 15m altitude: >80% detection rate

**Pass Criteria:** Detection confidence >0.5 at 10m altitude

---

#### Test 2: Tracking Continuity

**Objective:** Verify track ID remains consistent

**Procedure:**
```bash
# Launch full system
roslaunch uav_simulation full_mission.launch

# Monitor track IDs
rostopic echo /vision/target_state | grep track_id
```

**Expected Results:**
- Track ID should not change once assigned
- No ID switches during continuous tracking

**Pass Criteria:** Zero ID switches during 30-second tracking period

---

#### Test 3: Re-Identification

**Objective:** Test re-ID after temporary loss

**Procedure:**
1. Start mission
2. Wait for target detection
3. Manually move tank behind building (Gazebo GUI)
4. Wait 5 seconds
5. Move tank back into view
6. Check if same track ID is assigned

**Expected Results:**
- Track marked as "lost" during occlusion
- Same track ID restored after reappearance
- `is_reidentified` flag set to true

**Pass Criteria:** >80% re-ID success rate

---

### Flight Control Tests

#### Test 4: Takeoff Accuracy

**Objective:** Verify UAV reaches target altitude

**Procedure:**
```bash
# Launch flight controller
roslaunch uav_control flight_controller.launch

# Monitor altitude
rostopic echo /uav/state | grep 'position.z'
```

**Expected Results:**
- Reaches 10m ± 0.5m
- Smooth ascent without oscillations
- Time to altitude: 8-12 seconds

**Pass Criteria:** Final altitude within tolerance

---

#### Test 5: Search Pattern Coverage

**Objective:** Verify search pattern covers designated area

**Procedure:**
```bash
# Record UAV path
rosbag record /uav/ground_truth/pose -O search_pattern.bag

# Launch mission
roslaunch uav_simulation full_mission.launch

# After search phase, stop recording
# Ctrl+C on rosbag

# Analyze coverage
python3 scripts/analyze_coverage.py search_pattern.bag
```

**Expected Results:**
- Covers 100m x 100m area
- No repeated areas
- Systematic pattern

**Pass Criteria:** >90% area coverage

---

#### Test 6: Visual Servoing Accuracy

**Objective:** Verify target centering in image

**Procedure:**
```bash
# Monitor target position in image
rostopic echo /vision/target_state | grep bbox

# Calculate center
# center_x = bbox_x + bbox_width/2
# center_y = bbox_y + bbox_height/2

# Compare to image center (320, 240)
```

**Expected Results:**
- Target centered within ±50 pixels
- Smooth convergence
- No oscillations

**Pass Criteria:** Centering error <50 pixels during approach

---

## Integration Testing

### Test 7: Vision-Control Integration

**Objective:** Verify vision system controls flight correctly

**Test Cases:**

**7a. Detection triggers tracking mode**
```bash
# Start mission
# Verify mode changes from SEARCH to TRACKING when target detected
rostopic echo /uav/state | grep flight_mode
```

**7b. Lost target triggers search**
```bash
# During tracking, remove tank from simulation
rosservice call /gazebo/delete_model '{model_name: tank}'

# Verify UAV returns to search or hover
```

**7c. Re-identified target resumes tracking**
```bash
# After loss, respawn tank
# Verify tracking resumes with same ID
```

**Pass Criteria:** All mode transitions occur correctly

---

### Test 8: End-to-End Mission

**Objective:** Complete mission from start to finish

**Procedure:**
```bash
# Launch full mission
roslaunch uav_simulation full_mission.launch

# Monitor progress
rostopic echo /uav/state
```

**Expected Sequence:**
1. IDLE → TAKEOFF (0-10s)
2. TAKEOFF → SEARCH (10-20s)
3. SEARCH → TRACKING (20-40s)
4. TRACKING → APPROACH (40-50s)
5. APPROACH → ENGAGE (50-55s)
6. ENGAGE → COMPLETE (55-60s)

**Pass Criteria:** Mission completes successfully in <90 seconds

---

## System Testing

### Test 9: Multiple Runs Consistency

**Objective:** Verify consistent performance across runs

**Procedure:**
```bash
# Run mission 10 times
for i in {1..10}; do
    echo "Run $i"
    roslaunch uav_simulation full_mission.launch &
    sleep 90
    killall -9 gzserver gzclient roscore
    sleep 5
done
```

**Metrics to Record:**
- Mission success rate
- Time to detection
- Time to engagement
- Total mission time

**Expected Results:**
- 100% success rate
- Time variance <20%

**Pass Criteria:** >90% success rate, consistent timing

---

### Test 10: Stress Testing

**Objective:** Test system under load

**Procedure:**
```bash
# Reduce available resources
# Limit CPU cores
taskset -c 0,1 roslaunch uav_simulation full_mission.launch

# Monitor performance
top -p $(pgrep -f flight_controller)
```

**Expected Results:**
- System remains functional
- Graceful degradation if needed
- No crashes

**Pass Criteria:** Mission completes even with limited resources

---

## Performance Testing

### Test 11: Detection Latency

**Objective:** Measure detection processing time

**Procedure:**
```bash
# Add timing to detector node
# Modify object_detector.py to log timestamps

# Run mission and analyze logs
grep "Detection time" ~/.ros/log/latest/*.log
```

**Expected Results:**
- YOLOv5: <50ms per frame
- Fallback: <20ms per frame

**Pass Criteria:** Maintains 20+ FPS

---

### Test 12: Control Loop Frequency

**Objective:** Verify control loop runs at target frequency

**Procedure:**
```bash
# Monitor control commands
rostopic hz /cmd_vel
```

**Expected Results:**
- Frequency: 20 Hz ± 2 Hz
- No dropped messages

**Pass Criteria:** Consistent 20 Hz operation

---

### Test 13: Memory Usage

**Objective:** Verify no memory leaks

**Procedure:**
```bash
# Monitor memory over time
while true; do
    ps aux | grep -E 'detector|tracker|flight_controller' | awk '{print $6}'
    sleep 10
done > memory_log.txt

# Run for 10 minutes
# Analyze for increasing trend
```

**Expected Results:**
- Stable memory usage
- No continuous growth

**Pass Criteria:** <10% memory increase over 10 minutes

---

## Edge Case Testing

### Test 14: Target Occlusion

**Objective:** Handle temporary occlusions

**Scenarios:**

**14a. Brief occlusion (1-2 seconds)**
```bash
# During tracking, pause tank movement
# Move building in front of tank
# Wait 2 seconds
# Remove building
```

**Expected:** Track maintained, no ID switch

**14b. Extended occlusion (5-10 seconds)**
```bash
# Similar to above, longer duration
```

**Expected:** Track lost, then re-identified

**Pass Criteria:** Correct behavior in both cases

---

### Test 15: Target Leaving Frame

**Objective:** Handle target exiting camera view

**Procedure:**
```bash
# During tracking, manually move tank outside camera FOV
# Wait 5 seconds
# Move tank back into view
```

**Expected Results:**
- Track marked as lost
- Re-identification when target returns
- Same track ID assigned

**Pass Criteria:** Successful re-ID >80% of time

---

### Test 16: Multiple Similar Objects

**Objective:** Distinguish between similar targets

**Procedure:**
```bash
# Spawn two tanks
rosrun gazebo_ros spawn_model -file $(rospack find uav_simulation)/models/tank/model.sdf -sdf -model tank2 -x 20 -y 15 -z 0.3

# Verify correct target is tracked
```

**Expected Results:**
- Both tanks detected
- Correct tank tracked (first detected)
- No ID confusion

**Pass Criteria:** Maintains correct target identity

---

### Test 17: Poor Lighting Conditions

**Objective:** Test under varying lighting

**Procedure:**
```bash
# Modify world file to change lighting
# Edit battlefield.world:
<ambient>0.2 0.2 0.2 1</ambient>  # Darker

# Run mission
```

**Expected Results:**
- Detection still functional
- May require lower confidence threshold

**Pass Criteria:** >70% detection rate in poor lighting

---

### Test 18: Fast-Moving Target

**Objective:** Track moving target

**Procedure:**
```bash
# Add velocity to tank model
# Or use Gazebo GUI to move tank during mission
```

**Expected Results:**
- Tracking maintains lock
- Kalman filter predicts motion
- Visual servoing compensates

**Pass Criteria:** Maintains track at <3 m/s target velocity

---

### Test 19: System Recovery

**Objective:** Recover from node failures

**Procedure:**
```bash
# Kill detector node during mission
rosnode kill /object_detector

# Wait 5 seconds

# Restart detector
rosrun uav_vision object_detector.py
```

**Expected Results:**
- System detects failure
- Graceful degradation
- Recovery when node restarts

**Pass Criteria:** Mission continues after recovery

---

### Test 20: Boundary Conditions

**Objective:** Test at operational limits

**Test Cases:**

**20a. Maximum altitude**
```bash
# Set takeoff_altitude to 20m
# Verify detection still works
```

**20b. Minimum altitude**
```bash
# Set engagement_altitude to 0.5m
# Verify safe landing
```

**20c. Maximum search area**
```bash
# Increase search pattern size
# Verify coverage and battery life
```

**Pass Criteria:** System operates safely at all limits

---

## Automated Test Suite

### Running All Tests

```bash
# Make test script executable
chmod +x scripts/run_tests.sh

# Run all tests
./scripts/run_tests.sh

# View results
cat test_results.txt
```

### Test Results Format

```
========================================
UAV System Test Results
========================================
Date: 2024-01-15 14:30:00

Unit Tests:
  [PASS] Test 1: Object Detection Accuracy
  [PASS] Test 2: Tracking Continuity
  [PASS] Test 3: Re-Identification
  [PASS] Test 4: Takeoff Accuracy
  [PASS] Test 5: Search Pattern Coverage
  [PASS] Test 6: Visual Servoing Accuracy

Integration Tests:
  [PASS] Test 7: Vision-Control Integration
  [PASS] Test 8: End-to-End Mission

System Tests:
  [PASS] Test 9: Multiple Runs Consistency
  [PASS] Test 10: Stress Testing

Performance Tests:
  [PASS] Test 11: Detection Latency
  [PASS] Test 12: Control Loop Frequency
  [PASS] Test 13: Memory Usage

Edge Case Tests:
  [PASS] Test 14: Target Occlusion
  [PASS] Test 15: Target Leaving Frame
  [PASS] Test 16: Multiple Similar Objects
  [WARN] Test 17: Poor Lighting Conditions
  [PASS] Test 18: Fast-Moving Target
  [PASS] Test 19: System Recovery
  [PASS] Test 20: Boundary Conditions

========================================
Summary: 19/20 PASSED, 1/20 WARNING
Overall: PASS
========================================
```

---

## Continuous Integration

### GitHub Actions Workflow

Create `.github/workflows/test.yml`:

```yaml
name: ROS Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-20.04
    steps:
      - uses: actions/checkout@v2
      
      - name: Install ROS
        run: |
          sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
          sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
          sudo apt-get update
          sudo apt-get install -y ros-noetic-desktop-full
      
      - name: Build
        run: |
          source /opt/ros/noetic/setup.bash
          catkin_make
      
      - name: Run Tests
        run: |
          source devel/setup.bash
          ./scripts/run_tests.sh
```

---

## Test Data Collection

### Metrics to Track

1. **Detection Metrics:**
   - Precision, Recall, F1 Score
   - Detection latency
   - False positive rate

2. **Tracking Metrics:**
   - Track continuity
   - ID switches
   - MOTA (Multiple Object Tracking Accuracy)

3. **Control Metrics:**
   - Position error
   - Altitude hold accuracy
   - Centering error

4. **Mission Metrics:**
   - Success rate
   - Time to detection
   - Time to engagement
   - Total mission time

### Data Analysis

```python
# Example analysis script
import rosbag
import numpy as np

bag = rosbag.Bag('test_mission.bag')

# Extract detection times
detection_times = []
for topic, msg, t in bag.read_messages(topics=['/vision/detections']):
    if len(msg.detections) > 0:
        detection_times.append(t.to_sec())

# Calculate metrics
time_to_first_detection = detection_times[0] - start_time
detection_rate = len(detection_times) / total_time

print(f"Time to first detection: {time_to_first_detection:.2f}s")
print(f"Detection rate: {detection_rate:.2f} Hz")
```

---

## Reporting Issues

When reporting test failures, include:

1. **Test number and name**
2. **Expected vs. actual results**
3. **System configuration**
4. **Relevant logs**
5. **Steps to reproduce**

Example:
```
Test 3: Re-Identification - FAILED

Expected: Track ID maintained after occlusion
Actual: New track ID assigned (ID 1 → ID 2)

System: Ubuntu 20.04, ROS Noetic, YOLOv5s
Logs: Attached detector.log, tracker.log

Reproduction:
1. Launch full_mission.launch
2. Wait for detection
3. Move tank behind building for 5s
4. Observe new track ID assigned
```

---

## Conclusion

Regular testing ensures system reliability and performance. Run the full test suite before:
- Committing major changes
- Creating releases
- Deploying to real hardware

Maintain >90% test pass rate for production readiness.

