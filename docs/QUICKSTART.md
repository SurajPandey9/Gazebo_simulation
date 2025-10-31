# Quick Start Guide

Get the UAV simulation running in 5 minutes!

## Prerequisites Check

Before starting, ensure you have:
- Ubuntu 20.04 LTS
- At least 8GB RAM
- GPU recommended (for YOLOv5)
- Internet connection (for downloading models)

## Installation

### Option 1: Automated Setup (Recommended)

```bash
# Navigate to project directory
cd i2

# Make setup script executable
chmod +x scripts/setup.sh

# Run setup script
./scripts/setup.sh
```

The script will:
1. Install ROS Noetic (if needed)
2. Install required packages
3. Install Python dependencies
4. Build the workspace
5. Download YOLOv5 model

### Option 2: Manual Setup

```bash
# Install ROS Noetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install ros-noetic-desktop-full

# Install dependencies
sudo apt-get install -y \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-hector-quadrotor \
    ros-noetic-cv-bridge \
    python3-pip

# Install Python packages
pip3 install -r requirements.txt

# Build workspace
source /opt/ros/noetic/setup.bash
catkin_make

# Source workspace
source devel/setup.bash
```

## Running the Simulation

### Full Mission (All Components)

```bash
# Terminal 1: Launch everything
roslaunch uav_simulation full_mission.launch
```

This single command starts:
- Gazebo simulation with battlefield
- UAV and tank models
- Object detection node
- Object tracking node
- Flight controller
- RViz visualization (optional)

### Watch the Mission

The UAV will automatically:
1. **Takeoff** to 10m altitude (~8 seconds)
2. **Search** for the target in expanding spiral pattern (~15-30 seconds)
3. **Detect** the tank when it comes into view
4. **Track** the target, keeping it centered
5. **Approach** by descending toward the target
6. **Engage** when reaching 2m altitude

**Total mission time:** ~45-60 seconds

## Monitoring the Mission

### View Camera Feed

```bash
# In a new terminal
rqt_image_view
```

Select topics:
- `/uav/camera/image_raw` - Raw camera feed
- `/vision/debug_image` - Detections overlay
- `/vision/tracked_image` - Tracking visualization

### Monitor Mission Status

```bash
# UAV state
rostopic echo /uav/state

# Target state
rostopic echo /vision/target_state

# Detections
rostopic echo /vision/detections
```

### View in RViz

RViz should launch automatically. If not:

```bash
rviz -d src/uav_simulation/config/mission.rviz
```

## Understanding the Output

### Console Messages

**Takeoff Phase:**
```
[INFO] Flight Controller initialized
[INFO] Starting mission...
[INFO] Takeoff complete at 10.02m
```

**Search Phase:**
```
[INFO] Generated search pattern with 15 waypoints
[INFO] Reached waypoint 1/15
[INFO] Reached waypoint 2/15
...
```

**Detection Phase:**
```
[INFO] Processed 150 frames, 1 with detections
[INFO] Created new track 1
[INFO] Target detected! Switching to tracking mode
```

**Tracking Phase:**
```
[INFO] Target centered, starting approach
```

**Engagement Phase:**
```
[INFO] Reached engagement altitude
[INFO] ENGAGING TARGET!
[INFO] Target engaged successfully!
[INFO] MISSION COMPLETE
```

### Mission Statistics

At the end, you'll see:
```
==================================================
MISSION COMPLETE
==================================================
Total mission time: 52.34 seconds
Takeoff time: 8.12 seconds
Search time: 18.45 seconds
Tracking time: 11.23 seconds
==================================================
```

## Troubleshooting

### Gazebo doesn't start

**Problem:** Gazebo crashes or doesn't open

**Solutions:**
```bash
# Kill any existing Gazebo processes
killall gzserver gzclient

# Clear Gazebo cache
rm -rf ~/.gazebo/

# Try again
roslaunch uav_simulation full_mission.launch
```

### UAV doesn't take off

**Problem:** UAV stays on ground

**Solutions:**
```bash
# Check if controllers are loaded
rosservice list | grep controller

# Check UAV model spawned
rosservice call /gazebo/get_model_state '{model_name: quadrotor}'

# Restart simulation
```

### No detections

**Problem:** Vision system doesn't detect tank

**Solutions:**

1. **Check camera feed:**
```bash
rostopic hz /uav/camera/image_raw
# Should show ~30 Hz
```

2. **Verify tank is visible:**
```bash
# View camera in rqt_image_view
rqt_image_view
```

3. **Check detection node:**
```bash
# See if detector is running
rosnode list | grep detector

# Check for errors
rosnode info /object_detector
```

4. **Adjust confidence threshold:**
Edit `src/uav_simulation/launch/full_mission.launch`:
```xml
<param name="confidence_threshold" value="0.3"/>  <!-- Lower threshold -->
```

### YOLOv5 fails to load

**Problem:** "Failed to load YOLOv5 model"

**Solution:** System will automatically fall back to color-based detection

To fix YOLOv5:
```bash
# Download model manually
python3 -c "import torch; torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)"

# If GPU issues, use CPU
export CUDA_VISIBLE_DEVICES=""
```

### Simulation runs slowly

**Problem:** Low FPS, laggy simulation

**Solutions:**

1. **Reduce graphics quality:**
```bash
# Launch without GUI
roslaunch uav_simulation full_mission.launch gui:=false
```

2. **Use smaller YOLO model:**
Edit launch file:
```xml
<param name="model_size" value="yolov5n"/>  <!-- Nano model, faster -->
```

3. **Reduce camera resolution:**
Edit `src/uav_simulation/models/uav_with_camera/model.sdf`:
```xml
<width>320</width>
<height>240</height>
```

## Testing Different Scenarios

### Change Tank Position

Edit `src/uav_simulation/launch/full_mission.launch`:
```xml
<node name="spawn_tank" pkg="gazebo_ros" type="spawn_model" 
      args="-file $(find uav_simulation)/models/tank/model.sdf 
            -sdf 
            -model tank 
            -x 25    <!-- Change X position -->
            -y -15   <!-- Change Y position -->
            -z 0.3
            -Y 0.5"/>
```

### Change UAV Start Position

```xml
<include file="$(find hector_quadrotor_gazebo)/launch/spawn_quadrotor.launch">
  <arg name="x" value="5.0"/>   <!-- Change start X -->
  <arg name="y" value="5.0"/>   <!-- Change start Y -->
  <arg name="z" value="0.3"/>
</include>
```

### Adjust Flight Parameters

Edit `config/flight_params.yaml`:
```yaml
takeoff_altitude: 15.0    # Higher altitude
search_speed: 3.0         # Faster search
engagement_altitude: 1.0  # Lower engagement
```

Then restart the simulation.

## Recording a Demo

```bash
# Make script executable
chmod +x scripts/record_demo.sh

# Run recording script
./scripts/record_demo.sh
```

This will:
1. Record all relevant topics to a bag file
2. Optionally convert to video
3. Save in `recordings/` directory

## Next Steps

1. **Read the full documentation:**
   - `README.md` - Complete system overview
   - `docs/ALGORITHMS.md` - Algorithm details
   - `docs/REPORT_TEMPLATE.md` - Technical report template

2. **Experiment with parameters:**
   - Adjust detection thresholds
   - Modify flight speeds
   - Change search patterns

3. **Extend the system:**
   - Add obstacle avoidance
   - Implement multi-target tracking
   - Add more sophisticated re-ID

4. **Analyze performance:**
   - Record bag files
   - Extract metrics
   - Generate plots

## Common Commands Reference

```bash
# Launch full simulation
roslaunch uav_simulation full_mission.launch

# Launch only Gazebo
roslaunch uav_simulation battlefield.launch

# View camera
rqt_image_view

# Monitor topics
rostopic list
rostopic echo /uav/state
rostopic hz /uav/camera/image_raw

# Check nodes
rosnode list
rosnode info /flight_controller

# Kill all ROS nodes
rosnode kill -a

# Clean build
catkin_make clean
catkin_make

# Source workspace
source devel/setup.bash
```

## Getting Help

If you encounter issues:

1. **Check logs:**
```bash
# View node output
rosnode info /object_detector
rosnode info /flight_controller

# Check for errors
rqt_console
```

2. **Verify installation:**
```bash
# Check ROS
rosversion -d  # Should show "noetic"

# Check Python packages
pip3 list | grep torch
pip3 list | grep opencv
```

3. **Reset everything:**
```bash
# Kill all ROS processes
killall -9 roscore rosmaster gzserver gzclient

# Clean workspace
catkin_make clean
catkin_make

# Restart
roslaunch uav_simulation full_mission.launch
```

## Success Indicators

You know it's working when you see:

✅ Gazebo opens with battlefield environment  
✅ UAV and tank models visible  
✅ UAV takes off automatically  
✅ Console shows "Target detected!"  
✅ Camera feed shows bounding box around tank  
✅ UAV descends toward target  
✅ Console shows "MISSION COMPLETE"  

## Performance Expectations

On a typical system (i7 CPU, GTX 1060 GPU, 16GB RAM):
- Gazebo FPS: 30-60
- Detection rate: 30 Hz
- Mission time: 45-60 seconds
- CPU usage: 40-60%
- GPU usage: 30-50%

Enjoy your UAV simulation! 🚁

