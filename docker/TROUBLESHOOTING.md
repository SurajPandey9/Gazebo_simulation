# 🔧 Docker Troubleshooting Guide

Complete troubleshooting guide for running UAV simulation in Docker on Windows.

---

## 🚨 Common Issues and Solutions

### Issue 1: "Cannot open display: host.docker.internal:0"

**Symptoms:**
- GUI applications fail to start
- Error message about display
- xclock doesn't show window

**Diagnosis:**
```bash
# Check DISPLAY variable
echo $DISPLAY
# Should show: host.docker.internal:0
```

**Solutions:**

**Solution 1A: Check VcXsrv is running**
```batch
REM On Windows, check system tray for X icon
REM If not running:
docker\start_xserver.bat
```

**Solution 1B: Restart VcXsrv with correct settings**
1. Close VcXsrv (right-click system tray icon → Exit)
2. Run XLaunch
3. **IMPORTANT:** Check "Disable access control"
4. Start

**Solution 1C: Set DISPLAY manually**
```bash
# Inside container
export DISPLAY=host.docker.internal:0

# Test
xclock
```

**Solution 1D: Use Windows IP instead**
```bash
# Get Windows IP
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0

# Test
xclock
```

**Solution 1E: Check Windows Firewall**
1. Windows Security → Firewall & network protection
2. Allow an app through firewall
3. Find VcXsrv
4. Check both Private and Public
5. Restart VcXsrv

---

### Issue 2: Gazebo crashes immediately

**Symptoms:**
- Gazebo starts then crashes
- "Segmentation fault" error
- "symbol lookup error" messages

**Solutions:**

**Solution 2A: Use software rendering**
```bash
export LIBGL_ALWAYS_SOFTWARE=1
roslaunch uav_simulation full_mission.launch
```

**Solution 2B: Increase shared memory**
```bash
# Exit container and restart with more shared memory
docker run -it --rm \
  --shm-size=2gb \
  -e DISPLAY=host.docker.internal:0 \
  -e QT_X11_NO_MITSHM=1 \
  -v /e/i5_psWaste/i2:/workspace \
  osrf/ros:noetic-desktop-full bash
```

**Solution 2C: Run headless (no GUI)**
```bash
roslaunch uav_simulation full_mission.launch gui:=false
```

**Solution 2D: Update Docker resources**
1. Docker Desktop → Settings → Resources
2. Memory: 8GB minimum
3. CPUs: 4 minimum
4. Swap: 2GB
5. Apply & Restart

---

### Issue 3: "Permission denied" errors

**Symptoms:**
- Cannot execute scripts
- "./script.sh: Permission denied"

**Solutions:**

**Solution 3A: Fix permissions**
```bash
chmod +x scripts/*.sh scripts/*.py
find src -name "*.py" -exec chmod +x {} \;
```

**Solution 3B: Run with bash explicitly**
```bash
bash scripts/setup.sh
```

**Solution 3C: Check file ownership**
```bash
# Inside container
ls -la scripts/
# If owned by different user:
chown -R root:root /workspace
```

---

### Issue 4: Python module not found

**Symptoms:**
- "ModuleNotFoundError: No module named 'cv2'"
- "No module named 'torch'"
- Import errors

**Solutions:**

**Solution 4A: Install missing packages**
```bash
pip3 install opencv-python torch torchvision ultralytics filterpy scikit-learn
```

**Solution 4B: Install from requirements.txt**
```bash
cd /workspace
pip3 install -r requirements.txt
```

**Solution 4C: Use system OpenCV**
```bash
apt-get update
apt-get install -y python3-opencv
```

**Solution 4D: Check Python version**
```bash
python3 --version
# Should be Python 3.8.x

which python3
# Should be /usr/bin/python3
```

---

### Issue 5: catkin_make fails

**Symptoms:**
- Build errors
- "Could not find a package configuration file"
- CMake errors

**Solutions:**

**Solution 5A: Install missing ROS packages**
```bash
apt-get update
apt-get install -y \
  ros-noetic-hector-quadrotor \
  ros-noetic-hector-quadrotor-gazebo \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport
```

**Solution 5B: Source ROS environment**
```bash
source /opt/ros/noetic/setup.bash
catkin_make
```

**Solution 5C: Clean build**
```bash
cd /workspace
rm -rf build/ devel/
catkin_make
```

**Solution 5D: Check CMakeLists.txt**
```bash
# Verify all package.xml and CMakeLists.txt exist
find src -name "package.xml"
find src -name "CMakeLists.txt"
```

---

### Issue 6: Docker container won't start

**Symptoms:**
- "docker: Error response from daemon"
- Container exits immediately
- Cannot connect to Docker daemon

**Solutions:**

**Solution 6A: Check Docker is running**
```batch
REM Windows
docker info
```
If error, start Docker Desktop.

**Solution 6B: Check path mounting**
```bash
# Verify path format
# Windows: e:\i5_psWaste\i2
# Docker:  /e/i5_psWaste/i2

# Correct format:
docker run -v /e/i5_psWaste/i2:/workspace ...
```

**Solution 6C: Remove conflicting containers**
```bash
docker ps -a
docker rm uav_sim
```

**Solution 6D: Restart Docker**
1. Docker Desktop → Troubleshoot → Restart Docker
2. Wait for Docker to fully start
3. Try again

---

### Issue 7: Slow performance / Lag

**Symptoms:**
- Gazebo runs slowly
- Low FPS
- Delayed response

**Solutions:**

**Solution 7A: Allocate more resources**
1. Docker Desktop → Settings → Resources
2. CPUs: 6-8 cores
3. Memory: 12-16 GB
4. Apply & Restart

**Solution 7B: Use smaller YOLO model**
Edit `src/uav_simulation/launch/full_mission.launch`:
```xml
<param name="model_size" value="yolov5n"/>  <!-- Nano model -->
```

**Solution 7C: Reduce camera resolution**
Edit `src/uav_simulation/models/uav_with_camera/model.sdf`:
```xml
<width>320</width>
<height>240</height>
```

**Solution 7D: Disable debug visualization**
```bash
roslaunch uav_simulation full_mission.launch debug:=false
```

**Solution 7E: Use GPU acceleration (if available)**
```bash
# Requires NVIDIA GPU and nvidia-docker
docker run --gpus all ...
```

---

### Issue 8: "roslaunch: command not found"

**Symptoms:**
- ROS commands not recognized
- "roslaunch: command not found"
- "roscore: command not found"

**Solutions:**

**Solution 8A: Source ROS environment**
```bash
source /opt/ros/noetic/setup.bash
```

**Solution 8B: Add to bashrc**
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Solution 8C: Verify ROS installation**
```bash
ls /opt/ros/noetic/
# Should show ROS installation
```

---

### Issue 9: No camera image / Black screen

**Symptoms:**
- Camera topic exists but no image
- Black screen in rqt_image_view
- No detections

**Solutions:**

**Solution 9A: Check camera topic**
```bash
rostopic list | grep camera
rostopic hz /camera/rgb/image_raw
# Should show ~30 Hz
```

**Solution 9B: View raw camera**
```bash
rqt_image_view
# Select: /camera/rgb/image_raw
```

**Solution 9C: Check Gazebo camera plugin**
```bash
# Verify UAV spawned correctly
rosservice call /gazebo/get_model_state '{model_name: quadrotor}'
```

**Solution 9D: Restart simulation**
```bash
# Ctrl+C to stop
# Relaunch
roslaunch uav_simulation full_mission.launch
```

---

### Issue 10: UAV doesn't take off

**Symptoms:**
- UAV stays on ground
- No movement
- Flight controller not responding

**Solutions:**

**Solution 10A: Check flight controller node**
```bash
rosnode list | grep flight_controller
# Should show: /flight_controller

rosnode info /flight_controller
```

**Solution 10B: Check UAV state**
```bash
rostopic echo /uav/state
# Should show flight_mode changing
```

**Solution 10C: Manually send takeoff command**
```bash
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 1.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

**Solution 10D: Check for errors**
```bash
rqt_console
# Look for ERROR or WARN messages
```

---

## 🔍 Diagnostic Commands

### Check Docker Status
```bash
# Windows
docker info
docker ps
docker images
```

### Check X Server
```bash
# Inside container
echo $DISPLAY
xclock
glxinfo | grep "OpenGL"
```

### Check ROS
```bash
# Inside container
rosversion -d
# Should show: noetic

roscore &
# Should start without errors
```

### Check Workspace
```bash
# Inside container
cd /workspace
ls -la
source devel/setup.bash
rospack list | grep uav
```

### Check Topics and Nodes
```bash
# After launching simulation
rosnode list
rostopic list
rostopic hz /camera/rgb/image_raw
rostopic echo /uav/state
```

---

## 📋 Pre-Flight Checklist

Before launching simulation:

- [ ] Docker Desktop is running
- [ ] VcXsrv X Server is running (system tray icon)
- [ ] Container started successfully
- [ ] In /workspace directory
- [ ] ROS environment sourced
- [ ] Workspace built (catkin_make completed)
- [ ] Workspace sourced (devel/setup.bash)
- [ ] GUI test passed (xclock works)

---

## 🆘 Emergency Procedures

### Complete Reset

If nothing works, try complete reset:

```bash
# 1. Exit container
exit

# 2. Stop all containers
docker stop $(docker ps -aq)

# 3. Remove all containers
docker rm $(docker ps -aq)

# 4. Restart Docker Desktop
# Docker Desktop → Troubleshoot → Restart

# 5. Restart VcXsrv
# Close from system tray, run start_xserver.bat

# 6. Clean workspace (Windows)
cd e:\i5_psWaste\i2
rm -rf build/ devel/

# 7. Start fresh
docker\docker_run.bat
```

### Nuclear Option

Complete Docker reset:

```bash
# WARNING: This removes ALL Docker data!

# Stop Docker Desktop
# Open PowerShell as Administrator
docker system prune -a --volumes
# Type 'y' to confirm

# Restart Docker Desktop
# Pull image again
docker pull osrf/ros:noetic-desktop-full
```

---

## 📞 Getting Help

### Check Logs

```bash
# Container logs
docker logs uav_sim

# ROS logs
tail -f ~/.ros/log/latest/*.log

# Gazebo logs
tail -f ~/.gazebo/server.log
```

### Collect Debug Info

```bash
# System info
docker info
docker version

# Container info
docker inspect uav_sim

# ROS info
rosversion -d
rospack list

# Environment
env | grep ROS
env | grep DISPLAY
```

---

## ✅ Verification Tests

Run these to verify everything works:

```bash
# Test 1: Docker
docker run hello-world

# Test 2: X Server
docker run -it --rm -e DISPLAY=host.docker.internal:0 osrf/ros:noetic-desktop-full xclock

# Test 3: ROS
docker run -it --rm osrf/ros:noetic-desktop-full rosversion -d

# Test 4: Workspace mount
docker run -it --rm -v /e/i5_psWaste/i2:/workspace osrf/ros:noetic-desktop-full ls /workspace
```

All should complete successfully.

---

**Still having issues?** Check the full documentation:
- `DOCKER_SETUP.md` - Complete setup guide
- `DOCKER_QUICK_START.md` - Quick reference
- `README.md` - Project documentation

