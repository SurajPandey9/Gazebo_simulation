# 🐋 Docker Setup Guide for UAV Simulation on Windows

Complete guide to run the ROS + Gazebo UAV simulation using Docker on Windows.

---

## 📚 Documentation Overview

This is the **complete detailed guide**. For other resources:

- **Quick Start (1-page):** `docker/DOCKER_QUICK_START.md` - Start here!
- **Troubleshooting:** `docker/TROUBLESHOOTING.md` - Having issues?
- **Docker Directory:** `docker/README.md` - File overview
- **Batch Scripts:** `docker/*.bat` - Automated Windows scripts
- **Setup Script:** `docker/container_setup.sh` - Container automation

---

## 📋 Prerequisites

### Required Software

1. **Docker Desktop for Windows**
   - Download: https://www.docker.com/products/docker-desktop
   - Install and ensure it's running
   - Enable WSL2 backend (recommended)

2. **VcXsrv Windows X Server** (for GUI support)
   - Download: https://sourceforge.net/projects/vcxsrv/
   - Install with default settings

3. **Git Bash** (you already have this)

---

## 🎯 Quick Start (TL;DR)

```bash
# 1. Start X Server (run VcXsrv)
# 2. Run Docker container
docker run -it --rm \
  -e DISPLAY=host.docker.internal:0 \
  -v /e/i5_psWaste/i2:/workspace \
  --name uav_sim \
  osrf/ros:noetic-desktop-full \
  bash

# 3. Inside container
cd /workspace
./scripts/setup.sh
./scripts/build_workspace.sh
source devel/setup.bash
roslaunch uav_simulation full_mission.launch
```

---

## 📖 Detailed Step-by-Step Guide

### Step 1: Install and Configure VcXsrv (X Server)

#### 1.1 Install VcXsrv
- Download from: https://sourceforge.net/projects/vcxsrv/
- Run installer with default options
- Install to default location

#### 1.2 Configure and Start VcXsrv

**Option A: Using XLaunch (GUI)**

1. Launch **XLaunch** from Start Menu
2. **Display settings:**
   - Select: "Multiple windows"
   - Display number: 0
   - Click "Next"

3. **Client startup:**
   - Select: "Start no client"
   - Click "Next"

4. **Extra settings:**
   - ✅ Check: "Disable access control" (IMPORTANT!)
   - ✅ Check: "Clipboard"
   - ✅ Check: "Primary Selection"
   - Click "Next"

5. Click "Finish"

**Option B: Using Command Line (Recommended)**

Create a shortcut or batch file with:
```batch
"C:\Program Files\VcXsrv\vcxsrv.exe" :0 -ac -terminate -lesspointer -multiwindow -clipboard -wgl
```

Save as `start_xserver.bat` in your project directory.

#### 1.3 Verify X Server is Running

- Look for XLaunch icon in system tray
- Should show "0.0" or similar

---

### Step 2: Pull ROS Docker Image

Open **PowerShell** or **Git Bash** and run:

```bash
# Pull ROS Noetic with full desktop (includes Gazebo, RViz, etc.)
docker pull osrf/ros:noetic-desktop-full

# Verify image downloaded
docker images | grep ros
```

**Expected output:**
```
osrf/ros    noetic-desktop-full    <image-id>    <size>
```

---

### Step 3: Create Docker Container

#### 3.1 Basic Container (No GUI)

```bash
docker run -it --rm \
  -v /e/i5_psWaste/i2:/workspace \
  --name uav_sim \
  osrf/ros:noetic-desktop-full \
  bash
```

#### 3.2 Container with GUI Support (Recommended)

```bash
docker run -it --rm \
  -e DISPLAY=host.docker.internal:0 \
  -e QT_X11_NO_MITSHM=1 \
  -v /e/i5_psWaste/i2:/workspace \
  --name uav_sim \
  osrf/ros:noetic-desktop-full \
  bash
```

**Explanation of flags:**
- `-it` - Interactive terminal
- `--rm` - Remove container on exit
- `-e DISPLAY=host.docker.internal:0` - Connect to Windows X server
- `-e QT_X11_NO_MITSHM=1` - Fix Qt shared memory issues
- `-v /e/i5_psWaste/i2:/workspace` - Mount your project directory
- `--name uav_sim` - Container name
- `osrf/ros:noetic-desktop-full` - Image to use
- `bash` - Command to run

#### 3.3 Advanced Container with GPU Support (Optional)

If you have NVIDIA GPU and want hardware acceleration:

```bash
docker run -it --rm \
  --gpus all \
  -e DISPLAY=host.docker.internal:0 \
  -e QT_X11_NO_MITSHM=1 \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /e/i5_psWaste/i2:/workspace \
  --name uav_sim \
  osrf/ros:noetic-desktop-full \
  bash
```

---

### Step 4: Setup Inside Container

Once inside the container, you'll see a prompt like: `root@<container-id>:/#`

#### 4.1 Navigate to Workspace

```bash
cd /workspace
ls -la
```

You should see your project files.

#### 4.2 Install Additional Dependencies

```bash
# Update package lists
apt-get update

# Install required packages
apt-get install -y \
  python3-pip \
  python3-opencv \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  ros-noetic-vision-opencv \
  ros-noetic-hector-quadrotor \
  ros-noetic-hector-quadrotor-gazebo \
  ros-noetic-hector-quadrotor-description \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-gazebo-ros-control

# Install Python packages
pip3 install --upgrade pip
pip3 install -r requirements.txt
```

**Note:** This may take 5-10 minutes on first run.

#### 4.3 Fix File Permissions (Important!)

```bash
# Make scripts executable
chmod +x scripts/*.sh scripts/*.py
find src -name "*.py" -type f -exec chmod +x {} \;
```

---

### Step 5: Build the Workspace

#### 5.1 Source ROS Environment

```bash
source /opt/ros/noetic/setup.bash
```

#### 5.2 Build with catkin_make

```bash
# Clean build (recommended for first time)
rm -rf build/ devel/

# Build all packages
catkin_make

# Or use the build script
./scripts/build_workspace.sh
```

**Expected output:**
```
[100%] Built target uav_msgs_generate_messages
[100%] Built target uav_simulation
[100%] Built target uav_vision
[100%] Built target uav_control
[100%] Built target uav_engagement
```

#### 5.3 Source Workspace

```bash
source devel/setup.bash
```

#### 5.4 Verify Build

```bash
# Check messages
rosmsg list | grep uav_msgs

# Expected output:
# uav_msgs/Detection
# uav_msgs/DetectionArray
# uav_msgs/MissionStatus
# uav_msgs/TargetState
# uav_msgs/UAVState

# Check services
rossrv list | grep uav_msgs

# Expected output:
# uav_msgs/EngageTarget
```

---

### Step 6: Test GUI Connection

Before launching the full simulation, test if GUI works:

```bash
# Test 1: Simple GUI test
xclock
```

**Expected:** A clock window should appear on your Windows desktop.

If it doesn't appear:
- Check VcXsrv is running (system tray icon)
- Verify DISPLAY variable: `echo $DISPLAY`
- Should show: `host.docker.internal:0`

```bash
# Test 2: Test Gazebo
gazebo --verbose
```

**Expected:** Gazebo window opens (may take 30-60 seconds first time)

Press `Ctrl+C` to close Gazebo.

---

### Step 7: Launch Full Mission

#### 7.1 Launch the Simulation

```bash
# Make sure you're in the workspace
cd /workspace

# Source the workspace
source devel/setup.bash

# Launch full mission
roslaunch uav_simulation full_mission.launch
```

#### 7.2 What to Expect

**Console Output:**
```
... logging to /root/.ros/log/...
Checking log directory for disk usage...
Done checking log directory disk usage.

started roslaunch server http://...
SUMMARY
========

PARAMETERS
 * /rosdistro: noetic
 * /rosversion: 1.15.x
 ...

NODES
  /
    flight_controller (uav_control/flight_controller.py)
    gazebo (gazebo_ros/gzserver)
    gazebo_gui (gazebo_ros/gzclient)
    object_detector (uav_vision/object_detector.py)
    object_tracker (uav_vision/object_tracker.py)
    ...

process[gazebo-1]: started with pid [...]
process[gazebo_gui-2]: started with pid [...]
...
```

**Gazebo Window:**
- Should open showing battlefield environment
- UAV (quadrotor) visible
- Tank (ground vehicle) visible
- Desert terrain with buildings

**Mission Progress:**
1. **TAKEOFF** (0-10s): UAV ascends to 10m
2. **SEARCH** (10-30s): Expanding spiral pattern
3. **DETECTION** (~30s): Tank detected, bounding box appears
4. **TRACKING** (30-45s): UAV centers on target
5. **APPROACH** (45-55s): Descends toward target
6. **ENGAGE** (~55s): Target engagement
7. **COMPLETE** (60s): Mission success message

---

## 🔧 Troubleshooting

### Issue 1: "Cannot open display"

**Error:**
```
Error: cannot open display: host.docker.internal:0
```

**Solutions:**

1. **Check VcXsrv is running:**
   - Look for X icon in system tray
   - Restart VcXsrv if needed

2. **Verify DISPLAY variable:**
   ```bash
   echo $DISPLAY
   # Should show: host.docker.internal:0
   ```

3. **Set DISPLAY manually:**
   ```bash
   export DISPLAY=host.docker.internal:0
   ```

4. **Check Windows Firewall:**
   - Allow VcXsrv through firewall
   - Windows Security → Firewall → Allow an app

5. **Try alternative DISPLAY:**
   ```bash
   # Get your Windows IP
   export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
   ```

---

### Issue 2: Gazebo crashes or doesn't start

**Error:**
```
gzclient: symbol lookup error: /usr/lib/x86_64-linux-gnu/libgazebo_common.so...
```

**Solutions:**

1. **Use software rendering:**
   ```bash
   export LIBGL_ALWAYS_SOFTWARE=1
   roslaunch uav_simulation full_mission.launch
   ```

2. **Run without GUI:**
   ```bash
   roslaunch uav_simulation full_mission.launch gui:=false
   ```

3. **Increase shared memory:**
   ```bash
   docker run -it --rm \
     --shm-size=1gb \
     -e DISPLAY=host.docker.internal:0 \
     -v /e/i5_psWaste/i2:/workspace \
     osrf/ros:noetic-desktop-full bash
   ```

---

### Issue 3: Permission denied errors

**Error:**
```
bash: ./scripts/setup.sh: Permission denied
```

**Solution:**
```bash
chmod +x scripts/*.sh scripts/*.py
find src -name "*.py" -exec chmod +x {} \;
```

---

### Issue 4: Python packages not found

**Error:**
```
ModuleNotFoundError: No module named 'cv2'
```

**Solution:**
```bash
pip3 install opencv-python torch torchvision ultralytics filterpy scikit-learn
```

---

### Issue 5: Slow performance

**Solutions:**

1. **Allocate more resources to Docker:**
   - Docker Desktop → Settings → Resources
   - Increase CPU: 4+ cores
   - Increase Memory: 8+ GB

2. **Use smaller YOLO model:**
   Edit `src/uav_simulation/launch/full_mission.launch`:
   ```xml
   <param name="model_size" value="yolov5n"/>  <!-- Nano model -->
   ```

3. **Reduce camera resolution:**
   Edit `src/uav_simulation/models/uav_with_camera/model.sdf`:
   ```xml
   <width>320</width>
   <height>240</height>
   ```

---

## 📝 Useful Docker Commands

### Container Management

```bash
# List running containers
docker ps

# List all containers
docker ps -a

# Stop container
docker stop uav_sim

# Remove container
docker rm uav_sim

# Attach to running container
docker exec -it uav_sim bash
```

### Image Management

```bash
# List images
docker images

# Remove image
docker rmi osrf/ros:noetic-desktop-full

# Pull latest image
docker pull osrf/ros:noetic-desktop-full
```

### Cleanup

```bash
# Remove stopped containers
docker container prune

# Remove unused images
docker image prune

# Remove everything (careful!)
docker system prune -a
```

---

## 🎯 Complete Workflow Summary

### One-Time Setup

1. Install Docker Desktop
2. Install VcXsrv
3. Pull ROS image: `docker pull osrf/ros:noetic-desktop-full`

### Every Time You Want to Run

1. **Start VcXsrv** (run `start_xserver.bat` or XLaunch)

2. **Start Docker container:**
   ```bash
   docker run -it --rm \
     -e DISPLAY=host.docker.internal:0 \
     -e QT_X11_NO_MITSHM=1 \
     -v /e/i5_psWaste/i2:/workspace \
     --name uav_sim \
     osrf/ros:noetic-desktop-full bash
   ```

3. **Inside container:**
   ```bash
   cd /workspace
   source /opt/ros/noetic/setup.bash
   source devel/setup.bash  # If already built
   roslaunch uav_simulation full_mission.launch
   ```

---

## 🚀 Next Steps

After successful launch, you can:

1. **Monitor mission:**
   ```bash
   # In another terminal, attach to container
   docker exec -it uav_sim bash
   source /workspace/devel/setup.bash
   rostopic echo /uav/state
   ```

2. **Record demo:**
   ```bash
   ./scripts/record_demo.sh
   ```

3. **Analyze results:**
   ```bash
   python3 scripts/analyze_mission.py recordings/mission.bag --plot
   ```

---

**Ready to start!** Follow the steps above and you'll have the UAV simulation running in Docker on Windows! 🐋🚁

