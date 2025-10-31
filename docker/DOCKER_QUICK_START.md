# 🐋 Docker Quick Start - UAV Simulation

**One-page guide to run the UAV simulation in Docker on Windows**

---

## ⚡ Super Quick Start (3 Steps)

### Step 1: Start X Server
```batch
REM Double-click this file:
docker\start_xserver.bat
```

### Step 2: Run Docker Container
```batch
REM Double-click this file:
docker\docker_run.bat
```

### Step 3: Inside Container
```bash
# Run setup (first time only)
cd /workspace
./docker/container_setup.sh

# Launch simulation
source devel/setup.bash
roslaunch uav_simulation full_mission.launch
```

**Done!** Gazebo should open with the UAV simulation.

---

## 📋 Prerequisites

### One-Time Installation

1. **Docker Desktop**
   - Download: https://www.docker.com/products/docker-desktop
   - Install and start Docker Desktop
   - Ensure it's running (check system tray)

2. **VcXsrv (X Server)**
   - Download: https://sourceforge.net/projects/vcxsrv/
   - Install with default settings

3. **Pull ROS Image** (first time only)
   ```bash
   docker pull osrf/ros:noetic-desktop-full
   ```
   This downloads ~2GB, takes 5-10 minutes.

---

## 🎯 Detailed Workflow

### Every Time You Want to Run the Simulation

#### 1. Start X Server (Windows)

**Option A: Use batch file**
```batch
docker\start_xserver.bat
```

**Option B: Manual**
- Run XLaunch from Start Menu
- Select "Multiple windows", Display 0
- Select "Start no client"
- **CHECK "Disable access control"** ✅
- Finish

**Verify:** Look for X icon in system tray

---

#### 2. Start Docker Container (Windows)

**Option A: Use batch file**
```batch
docker\docker_run.bat
```

**Option B: Manual (Git Bash or PowerShell)**
```bash
docker run -it --rm \
  -e DISPLAY=host.docker.internal:0 \
  -e QT_X11_NO_MITSHM=1 \
  -v /e/i5_psWaste/i2:/workspace \
  --name uav_sim \
  osrf/ros:noetic-desktop-full \
  bash
```

**You're now inside the container!** Prompt shows: `root@<id>:/#`

---

#### 3. Setup Workspace (First Time Only)

```bash
# Navigate to workspace
cd /workspace

# Run automated setup
./docker/container_setup.sh
```

This will:
- Install dependencies (~5 minutes)
- Build ROS workspace (~3 minutes)
- Set permissions
- Verify build

**Skip this step on subsequent runs if already built.**

---

#### 4. Launch Simulation (Every Time)

```bash
# Source ROS and workspace
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# Launch full mission
roslaunch uav_simulation full_mission.launch
```

**Expected:**
- Gazebo window opens on Windows desktop
- UAV and tank visible in battlefield
- Console shows mission progress
- Mission completes in ~60 seconds

---

## 🧪 Testing GUI Connection

Before launching the full simulation, test if GUI works:

```bash
# Test 1: Simple GUI
xclock
```
**Expected:** Clock window appears on Windows desktop.

```bash
# Test 2: Gazebo
gazebo --verbose
```
**Expected:** Gazebo opens (may take 30-60 seconds first time).

Press `Ctrl+C` to close.

---

## 🔧 Common Issues & Fixes

### Issue: "Cannot open display"

**Fix 1: Check X Server**
- Ensure VcXsrv is running (system tray icon)
- Restart: `docker\start_xserver.bat`

**Fix 2: Set DISPLAY**
```bash
export DISPLAY=host.docker.internal:0
```

**Fix 3: Check firewall**
- Allow VcXsrv through Windows Firewall

---

### Issue: Gazebo crashes

**Fix 1: Use software rendering**
```bash
export LIBGL_ALWAYS_SOFTWARE=1
roslaunch uav_simulation full_mission.launch
```

**Fix 2: Run without GUI**
```bash
roslaunch uav_simulation full_mission.launch gui:=false
```

**Fix 3: Increase shared memory**
```bash
# Exit container and restart with:
docker run -it --rm \
  --shm-size=2gb \
  -e DISPLAY=host.docker.internal:0 \
  -v /e/i5_psWaste/i2:/workspace \
  osrf/ros:noetic-desktop-full bash
```

---

### Issue: Permission denied

**Fix:**
```bash
chmod +x scripts/*.sh scripts/*.py
find src -name "*.py" -exec chmod +x {} \;
```

---

### Issue: Module not found (Python)

**Fix:**
```bash
pip3 install opencv-python torch ultralytics filterpy scikit-learn
```

---

### Issue: Slow performance

**Fix 1: Allocate more resources**
- Docker Desktop → Settings → Resources
- CPU: 4+ cores
- Memory: 8+ GB

**Fix 2: Use smaller YOLO**
Edit `src/uav_simulation/launch/full_mission.launch`:
```xml
<param name="model_size" value="yolov5n"/>
```

---

## 📝 Useful Commands

### Inside Container

```bash
# Source environment (always needed)
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# Rebuild workspace
cd /workspace
catkin_make

# Check messages
rosmsg list | grep uav_msgs

# Check nodes
rosnode list

# Check topics
rostopic list

# Monitor UAV state
rostopic echo /uav/state

# View camera
rqt_image_view
```

### Outside Container (Windows)

```bash
# List running containers
docker ps

# Stop container
docker stop uav_sim

# Attach to running container (new terminal)
docker exec -it uav_sim bash

# View container logs
docker logs uav_sim

# Remove all stopped containers
docker container prune
```

---

## 🎬 Complete Example Session

**Windows Terminal:**
```batch
REM 1. Start X Server
docker\start_xserver.bat

REM 2. Start container
docker\docker_run.bat
```

**Inside Container (first time):**
```bash
# Setup
cd /workspace
./docker/container_setup.sh

# Launch
source devel/setup.bash
roslaunch uav_simulation full_mission.launch
```

**Inside Container (subsequent times):**
```bash
# Just launch
cd /workspace
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch uav_simulation full_mission.launch
```

---

## 🎯 Mission Phases to Watch

1. **TAKEOFF** (0-10s) - UAV ascends to 10m
2. **SEARCH** (10-30s) - Spiral search pattern
3. **DETECTION** (~30s) - Tank detected, bounding box
4. **TRACKING** (30-45s) - UAV centers on target
5. **APPROACH** (45-55s) - Descends to 2m
6. **ENGAGE** (~55s) - Target engagement
7. **COMPLETE** (60s) - Mission success! ✅

---

## 📊 Monitoring

### View Camera Feed
```bash
# In container
rqt_image_view
# Select: /vision/tracked_image
```

### Monitor Status
```bash
# UAV state
rostopic echo /uav/state

# Target tracking
rostopic echo /vision/target_state

# Mission status
rostopic echo /mission/status
```

### Multiple Terminals
```bash
# Windows: Open new terminal
docker exec -it uav_sim bash

# Inside new terminal
source /opt/ros/noetic/setup.bash
source /workspace/devel/setup.bash
rostopic echo /uav/state
```

---

## 🛑 Stopping

### Stop Simulation
- Press `Ctrl+C` in container terminal

### Exit Container
```bash
exit
```
Container auto-removes (--rm flag)

### Stop X Server
- Right-click X icon in system tray
- Exit

---

## 💾 Saving Work

**Your code changes are automatically saved!**

The workspace is mounted from Windows:
- Container: `/workspace`
- Windows: `e:\i5_psWaste\i2`

Any changes in either location are immediately reflected.

---

## 🚀 Next Steps

After successful launch:

1. **Record demo:**
   ```bash
   ./scripts/record_demo.sh
   ```

2. **Analyze mission:**
   ```bash
   python3 scripts/analyze_mission.py recordings/mission.bag --plot
   ```

3. **Modify parameters:**
   - Edit `config/flight_params.yaml`
   - Edit `config/vision_params.yaml`
   - Restart simulation

---

## 📚 More Help

- **Full Docker Guide:** `DOCKER_SETUP.md`
- **Project README:** `README.md`
- **Quick Start:** `QUICKSTART.md`
- **Commands:** `COMMANDS_REFERENCE.md`

---

## ✅ Checklist

Before running:
- [ ] Docker Desktop running
- [ ] VcXsrv X Server running
- [ ] ROS image pulled
- [ ] Workspace mounted correctly

First time:
- [ ] Run `container_setup.sh`
- [ ] Verify build successful
- [ ] Test GUI with `xclock`

Every time:
- [ ] Source ROS environment
- [ ] Source workspace
- [ ] Launch simulation

---

**Ready to fly!** 🚁🐋

