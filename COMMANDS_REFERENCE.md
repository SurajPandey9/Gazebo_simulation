# Quick Commands Reference

Essential commands for working with the UAV simulation project.

## 🚀 Quick Start

```bash
# One-time setup
./scripts/setup.sh

# Build workspace
./scripts/build_workspace.sh

# Source workspace
source devel/setup.bash

# Run full mission
roslaunch uav_simulation full_mission.launch
```

## 🔧 Build Commands

```bash
# Clean build
./scripts/build_workspace.sh clean

# Manual build
source /opt/ros/noetic/setup.bash
catkin_make

# Build specific package
catkin_make --pkg uav_vision

# Clean and rebuild
rm -rf build/ devel/
catkin_make
```

## 🎮 Launch Commands

```bash
# Full mission (everything)
roslaunch uav_simulation full_mission.launch

# Gazebo only
roslaunch uav_simulation battlefield.launch

# Vision system only
roslaunch uav_vision detection_tracking.launch

# Flight controller only
roslaunch uav_control flight_controller.launch

# Engagement system only
roslaunch uav_engagement engagement.launch

# Visualization (RViz + viewers)
roslaunch uav_simulation visualization.launch
```

## 📊 Monitoring Commands

### View Topics
```bash
# List all topics
rostopic list

# Echo UAV state
rostopic echo /uav/state

# Echo target state
rostopic echo /vision/target_state

# Echo detections
rostopic echo /vision/detections

# Echo engagement status
rostopic echo /engagement/status

# Echo mission log
rostopic echo /mission/log

# Check topic frequency
rostopic hz /uav/camera/image_raw
rostopic hz /vision/detections
```

### View Nodes
```bash
# List all nodes
rosnode list

# Node information
rosnode info /flight_controller
rosnode info /object_detector

# Kill a node
rosnode kill /object_detector
```

### View Messages/Services
```bash
# List custom messages
rosmsg list | grep uav_msgs

# Show message definition
rosmsg show uav_msgs/Detection
rosmsg show uav_msgs/UAVState

# List services
rosservice list

# Call engagement service
rosservice call /engagement/engage_target "{}"
```

## 📹 Camera/Image Commands

```bash
# View raw camera feed
rqt_image_view /uav/camera/image_raw

# View tracked image
rqt_image_view /vision/tracked_image

# View debug image
rqt_image_view /vision/debug_image

# Save image
rosrun image_view image_saver image:=/uav/camera/image_raw
```

## 🎥 Recording Commands

```bash
# Record demo (automated)
./scripts/record_demo.sh

# Manual recording - all topics
rosbag record -a

# Record specific topics
rosbag record /uav/camera/image_raw /vision/tracked_image /uav/state

# Record with output file
rosbag record -O mission_test.bag /uav/state /vision/target_state

# Play back recording
rosbag play mission_test.bag

# Get bag info
rosbag info mission_test.bag
```

## 📈 Analysis Commands

```bash
# Analyze mission data
python3 scripts/analyze_mission.py recordings/mission.bag

# Analyze with plots
python3 scripts/analyze_mission.py recordings/mission.bag --plot

# Save plots to file
python3 scripts/analyze_mission.py recordings/mission.bag --plot --output results.png
```

## 🧪 Testing Commands

```bash
# Run all tests
./scripts/run_tests.sh

# Check dependencies
rosdep check --from-paths src --ignore-src

# Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# Verify messages
rosmsg list | grep uav_msgs

# Verify services
rossrv list | grep uav_msgs
```

## 🐛 Debugging Commands

```bash
# View console output
rqt_console

# View TF tree
rosrun rqt_tf_tree rqt_tf_tree

# View computation graph
rqt_graph

# Plot data in real-time
rqt_plot /uav/state/pose/position/z

# Monitor system resources
htop
```

## 🔍 Inspection Commands

```bash
# Check Gazebo models
rosrun gazebo_ros spawn_model -h

# Get model state
rosservice call /gazebo/get_model_state '{model_name: quadrotor}'
rosservice call /gazebo/get_model_state '{model_name: tank}'

# List Gazebo models
rosservice call /gazebo/get_world_properties

# Pause/unpause simulation
rosservice call /gazebo/pause_physics
rosservice call /gazebo/unpause_physics

# Reset simulation
rosservice call /gazebo/reset_simulation
```

## 🛑 Stop/Kill Commands

```bash
# Stop current launch (Ctrl+C in terminal)
# Or:

# Kill all ROS nodes
rosnode kill -a

# Kill specific processes
killall -9 gzserver
killall -9 gzclient
killall -9 roscore
killall -9 rosmaster

# Nuclear option - kill everything
killall -9 gzserver gzclient roscore rosmaster

# Clean restart
killall -9 gzserver gzclient roscore rosmaster
sleep 2
roslaunch uav_simulation full_mission.launch
```

## 🔧 Configuration Commands

```bash
# Edit flight parameters
nano config/flight_params.yaml

# Edit vision parameters
nano config/vision_params.yaml

# Reload parameters (after editing)
rosparam load config/flight_params.yaml
rosparam load config/vision_params.yaml

# Get current parameter
rosparam get /flight_controller/takeoff_altitude

# Set parameter
rosparam set /flight_controller/takeoff_altitude 15.0
```

## 📦 Package Management

```bash
# List ROS packages
rospack list | grep uav

# Find package path
rospack find uav_simulation

# List package dependencies
rospack depends uav_vision

# Check package
roscd uav_simulation
```

## 🌐 Network Commands (Multi-machine)

```bash
# Set ROS master (if using multiple machines)
export ROS_MASTER_URI=http://192.168.1.100:11311
export ROS_IP=192.168.1.101

# Check ROS network
rostopic list
rosnode list
```

## 📝 Log Commands

```bash
# View ROS logs
cd ~/.ros/log/latest/
ls -la

# Tail specific node log
tail -f ~/.ros/log/latest/flight_controller*.log

# Clear old logs
rosclean check
rosclean purge
```

## 🎯 Mission Control Commands

```bash
# Monitor mission status
rostopic echo /mission/status

# Monitor mission log
rostopic echo /mission/log

# Monitor vision status
rostopic echo /vision/status

# Monitor engagement status
rostopic echo /engagement/status

# Check if engagement ready
rostopic echo /engagement/ready
```

## 🔄 Restart Commands

```bash
# Restart specific node
rosnode kill /object_detector
rosrun uav_vision object_detector.py

# Restart with different parameters
rosnode kill /flight_controller
rosrun uav_control flight_controller.py _takeoff_altitude:=15.0

# Restart entire system
# Terminal 1: Kill all
killall -9 gzserver gzclient roscore rosmaster

# Terminal 1: Wait and relaunch
sleep 3
roslaunch uav_simulation full_mission.launch
```

## 📊 Performance Monitoring

```bash
# Monitor topic bandwidth
rostopic bw /uav/camera/image_raw

# Monitor topic frequency
rostopic hz /vision/detections

# Monitor node CPU/memory
top -p $(pgrep -f flight_controller)

# System-wide monitoring
htop

# GPU monitoring (if using YOLO)
nvidia-smi -l 1
```

## 🎨 Visualization Tools

```bash
# RViz
rviz -d src/uav_simulation/config/mission.rviz

# Image viewer
rqt_image_view

# Plot tool
rqt_plot

# Console
rqt_console

# Graph
rqt_graph

# TF tree
rqt_tf_tree

# All-in-one
rqt
```

## 🔐 Permission Commands

```bash
# Make scripts executable
chmod +x scripts/*.sh
chmod +x scripts/*.py

# Make all Python nodes executable
find src -name "*.py" -type f -exec chmod +x {} \;

# Check permissions
ls -la scripts/
ls -la src/*/nodes/
```

## 💾 Backup Commands

```bash
# Backup workspace
tar -czf uav_backup_$(date +%Y%m%d).tar.gz src/ config/ scripts/

# Backup recordings
tar -czf recordings_backup.tar.gz recordings/

# Backup specific mission
cp recordings/mission_20240115.bag backups/
```

## 🔍 Search Commands

```bash
# Find files
find . -name "*.launch"
find . -name "*.py"
find . -name "*.yaml"

# Search in files
grep -r "takeoff_altitude" src/
grep -r "YOLOv5" src/

# Search ROS packages
rospack find uav_vision
roscd uav_simulation
```

## 📚 Help Commands

```bash
# ROS help
ros -h

# Topic help
rostopic -h

# Node help
rosnode -h

# Launch help
roslaunch --help

# Package help
rospack -h

# Bag help
rosbag -h
```

## 🎓 Learning Commands

```bash
# Show message structure
rosmsg show uav_msgs/Detection

# Show service structure
rossrv show uav_msgs/EngageTarget

# Show node connections
rosnode info /flight_controller

# Show topic info
rostopic info /uav/state

# Show parameter tree
rosparam list
```

---

## 💡 Pro Tips

### Quick Mission Restart
```bash
alias restart_mission='killall -9 gzserver gzclient roscore rosmaster; sleep 3; roslaunch uav_simulation full_mission.launch'
```

### Monitor Everything
```bash
# Terminal 1: Launch
roslaunch uav_simulation full_mission.launch

# Terminal 2: Monitor UAV
rostopic echo /uav/state

# Terminal 3: Monitor vision
rostopic echo /vision/target_state

# Terminal 4: View camera
rqt_image_view /vision/tracked_image
```

### Quick Debug
```bash
# Check if everything is running
rosnode list && rostopic list && rosservice list
```

### Performance Check
```bash
# Check all frequencies
rostopic hz /uav/camera/image_raw &
rostopic hz /vision/detections &
rostopic hz /cmd_vel &
wait
```

---

**For more details, see:**
- `README.md` - Full documentation
- `QUICKSTART.md` - Getting started guide
- `VERIFICATION_CHECKLIST.md` - Complete verification

