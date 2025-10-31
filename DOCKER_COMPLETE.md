# 🎉 Docker Setup Complete!

## UAV Simulation - Docker on Windows

**Status:** ✅ **COMPLETE AND READY TO USE**

---

## 📦 What's Been Created

### Complete Docker Setup Package

I've created a comprehensive Docker setup for running your UAV simulation on Windows without needing WSL2 Ubuntu or dual-boot Linux.

### 📁 New Files Created

#### Windows Batch Scripts (in `docker/` directory)

1. **`start_xserver.bat`** - Start VcXsrv X Server
   - Automatically finds VcXsrv installation
   - Checks if already running
   - Starts with optimal settings

2. **`docker_run.bat`** - Run Docker container
   - Checks Docker is running
   - Warns if X Server not running
   - Mounts workspace correctly
   - Sets up GUI support

3. **`docker_build_and_run.bat`** - Complete automated workflow
   - Pulls Docker image if needed
   - Starts X Server
   - Creates setup script
   - Runs container

4. **`build_custom_image.bat`** - Build custom Docker image (optional)
   - Creates image with all dependencies pre-installed
   - Faster subsequent runs
   - ~4GB image with everything ready

#### Linux Shell Scripts (in `docker/` directory)

5. **`container_setup.sh`** - Automated setup inside container
   - Installs all dependencies
   - Builds ROS workspace
   - Sets permissions
   - Verifies build

#### Docker Configuration

6. **`Dockerfile`** - Custom image definition
   - Based on ROS Noetic Desktop Full
   - All dependencies pre-installed
   - Python packages included
   - Ready to build workspace

#### Documentation (in `docker/` directory)

7. **`DOCKER_QUICK_START.md`** (300 lines)
   - One-page quick reference
   - Super quick start (3 steps)
   - Common issues & fixes
   - Complete workflow examples

8. **`TROUBLESHOOTING.md`** (300 lines)
   - 10 common issues with solutions
   - Diagnostic commands
   - Pre-flight checklist
   - Emergency procedures
   - Verification tests

9. **`README.md`** (300 lines)
   - Docker directory overview
   - File descriptions
   - Workflow diagrams
   - Tips & tricks
   - Resource usage info

#### Main Documentation

10. **`DOCKER_SETUP.md`** (Updated, 300 lines)
    - Complete detailed guide
    - Step-by-step instructions
    - GUI setup (VcXsrv)
    - Build and launch procedures
    - Comprehensive troubleshooting

---

## 🚀 How to Use

### Option 1: Automated (Recommended for First Time)

```batch
REM 1. Start X Server
docker\start_xserver.bat

REM 2. Run container
docker\docker_run.bat

REM 3. Inside container, run setup
cd /workspace
./docker/container_setup.sh

REM 4. Launch simulation
source devel/setup.bash
roslaunch uav_simulation full_mission.launch
```

### Option 2: Manual (Full Control)

**Step 1: Install Prerequisites (One-Time)**

1. Install Docker Desktop: https://www.docker.com/products/docker-desktop
2. Install VcXsrv: https://sourceforge.net/projects/vcxsrv/
3. Pull ROS image:
   ```bash
   docker pull osrf/ros:noetic-desktop-full
   ```

**Step 2: Start X Server (Every Time)**

```batch
docker\start_xserver.bat
```

Or manually:
- Run XLaunch
- Select "Multiple windows", Display 0
- Select "Start no client"
- **CHECK "Disable access control"** ✅
- Finish

**Step 3: Run Docker Container (Every Time)**

```batch
docker\docker_run.bat
```

Or manually:
```bash
docker run -it --rm \
  -e DISPLAY=host.docker.internal:0 \
  -e QT_X11_NO_MITSHM=1 \
  -v /e/i5_psWaste/i2:/workspace \
  --name uav_sim \
  osrf/ros:noetic-desktop-full \
  bash
```

**Step 4: Setup Inside Container (First Time Only)**

```bash
cd /workspace
./docker/container_setup.sh
```

This installs dependencies and builds workspace (~10 minutes first time).

**Step 5: Launch Simulation (Every Time)**

```bash
cd /workspace
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch uav_simulation full_mission.launch
```

### Option 3: Custom Image (Fastest After Initial Build)

**One-Time Build:**
```batch
docker\build_custom_image.bat
```

**Then Every Time:**
```batch
REM 1. Start X Server
docker\start_xserver.bat

REM 2. Run custom container
docker run -it --rm ^
  -e DISPLAY=host.docker.internal:0 ^
  -v /e/i5_psWaste/i2:/workspace ^
  uav_simulation:latest bash

REM 3. Inside container
cd /workspace
catkin_make  # Only if not built yet
source devel/setup.bash
roslaunch uav_simulation full_mission.launch
```

---

## 📖 Documentation Guide

### Start Here

1. **`docker/DOCKER_QUICK_START.md`** - Read this first!
   - One-page guide
   - Quick 3-step start
   - Common issues

2. **`DOCKER_SETUP.md`** - Complete detailed guide
   - Full step-by-step instructions
   - All options explained
   - Comprehensive troubleshooting

3. **`docker/TROUBLESHOOTING.md`** - When you have problems
   - 10 common issues
   - Detailed solutions
   - Diagnostic commands

### Reference

4. **`docker/README.md`** - Docker directory overview
5. **`COMMANDS_REFERENCE.md`** - ROS commands reference
6. **`README.md`** - Main project documentation

---

## 🎯 What to Expect

### When You Launch the Simulation

**Gazebo Window Opens:**
- Battlefield environment with desert terrain
- UAV (quadrotor) visible
- Tank (ground vehicle) visible
- Buildings and obstacles

**Mission Phases (60 seconds total):**

1. **TAKEOFF** (0-10s)
   - UAV ascends to 10 meters
   - Stabilizes in hover

2. **SEARCH** (10-30s)
   - Expanding spiral search pattern
   - Camera scanning for target

3. **DETECTION** (~30s)
   - Tank appears in camera view
   - Green bounding box appears
   - Track ID assigned

4. **TRACKING** (30-45s)
   - UAV centers on target
   - Maintains visual lock
   - Follows target movement

5. **APPROACH** (45-55s)
   - UAV descends toward target
   - Keeps target centered
   - Altitude decreases to 2m

6. **ENGAGE** (~55s)
   - Engagement conditions met
   - Weapon fire simulated
   - Target hit confirmed

7. **COMPLETE** (60s)
   - Mission summary displayed
   - Statistics printed
   - Success! ✅

**Console Output:**
```
[INFO] UAV State: TAKEOFF
[INFO] Altitude: 5.2m
[INFO] UAV State: SEARCH
[INFO] Executing search pattern...
[INFO] Target detected! ID: 1
[INFO] UAV State: TRACKING
[INFO] Target locked!
[INFO] UAV State: APPROACH
[INFO] Descending to engagement altitude...
[INFO] UAV State: ENGAGE
[INFO] Engagement conditions met!
[INFO] FIRE!
[INFO] Target hit!
[INFO] UAV State: MISSION_COMPLETE
[INFO] Mission completed successfully!
```

---

## 🔧 Troubleshooting Quick Reference

### Issue: GUI doesn't work

**Quick Fix:**
1. Check VcXsrv is running (system tray icon)
2. Restart: `docker\start_xserver.bat`
3. In container: `export DISPLAY=host.docker.internal:0`
4. Test: `xclock`

**Full Solution:** See `docker/TROUBLESHOOTING.md` → Issue 1

### Issue: Gazebo crashes

**Quick Fix:**
```bash
export LIBGL_ALWAYS_SOFTWARE=1
roslaunch uav_simulation full_mission.launch
```

**Full Solution:** See `docker/TROUBLESHOOTING.md` → Issue 2

### Issue: Slow performance

**Quick Fix:**
- Docker Desktop → Settings → Resources
- CPU: 6+ cores, Memory: 12+ GB

**Full Solution:** See `docker/TROUBLESHOOTING.md` → Issue 7

### Issue: Permission denied

**Quick Fix:**
```bash
chmod +x scripts/*.sh scripts/*.py
find src -name "*.py" -exec chmod +x {} \;
```

**Full Solution:** See `docker/TROUBLESHOOTING.md` → Issue 3

---

## ✅ Verification Checklist

Before launching simulation:

- [ ] Docker Desktop installed and running
- [ ] VcXsrv installed
- [ ] ROS image pulled (`docker images | grep ros`)
- [ ] X Server running (system tray icon)
- [ ] Container started successfully
- [ ] In `/workspace` directory
- [ ] Dependencies installed (first time)
- [ ] Workspace built (`catkin_make` completed)
- [ ] ROS environment sourced
- [ ] Workspace sourced
- [ ] GUI test passed (`xclock` works)

---

## 📊 System Requirements

### Minimum

- **OS:** Windows 10/11 with Docker Desktop
- **CPU:** 4 cores
- **RAM:** 8 GB
- **Disk:** 10 GB free
- **GPU:** Not required (software rendering available)

### Recommended

- **OS:** Windows 11 with Docker Desktop + WSL2
- **CPU:** 6-8 cores
- **RAM:** 16 GB
- **Disk:** 20 GB free
- **GPU:** NVIDIA GPU (optional, for hardware acceleration)

---

## 🎓 Learning Resources

### Docker + ROS

- **ROS Docker Tutorial:** https://wiki.ros.org/docker/Tutorials/Docker
- **ROS Docker Images:** https://hub.docker.com/_/ros
- **Docker Desktop Docs:** https://docs.docker.com/desktop/

### X Server on Windows

- **VcXsrv:** https://sourceforge.net/projects/vcxsrv/
- **X11 Forwarding:** https://wiki.ros.org/docker/Tutorials/GUI

---

## 💡 Pro Tips

### Tip 1: Keep Container Running

Instead of `--rm`, use persistent container:
```bash
docker run -it \
  --name uav_sim_persistent \
  -e DISPLAY=host.docker.internal:0 \
  -v /e/i5_psWaste/i2:/workspace \
  osrf/ros:noetic-desktop-full bash

# Later, restart:
docker start -ai uav_sim_persistent
```

### Tip 2: Multiple Terminals

```bash
# Terminal 1: Main
docker exec -it uav_sim bash

# Terminal 2: Monitoring
docker exec -it uav_sim bash
source /workspace/devel/setup.bash
rostopic echo /uav/state
```

### Tip 3: Use Custom Image

Build once, run fast:
```batch
docker\build_custom_image.bat
```

Then use `uav_simulation:latest` instead of `osrf/ros:noetic-desktop-full`.

---

## 🎬 Complete Example Session

**Windows Command Prompt:**

```batch
REM Step 1: Start X Server
C:\> cd e:\i5_psWaste\i2
C:\> docker\start_xserver.bat
[VcXsrv started successfully!]

REM Step 2: Run container
C:\> docker\docker_run.bat
[Docker container starting...]
```

**Inside Docker Container (First Time):**

```bash
root@abc123:/# cd /workspace
root@abc123:/workspace# ./docker/container_setup.sh
[Installing dependencies...]
[Building workspace...]
[Setup Complete!]

root@abc123:/workspace# source devel/setup.bash
root@abc123:/workspace# roslaunch uav_simulation full_mission.launch
[Gazebo opens, mission starts...]
[Mission completes in 60 seconds]
```

**Inside Docker Container (Subsequent Times):**

```bash
root@abc123:/# cd /workspace
root@abc123:/workspace# source /opt/ros/noetic/setup.bash
root@abc123:/workspace# source devel/setup.bash
root@abc123:/workspace# roslaunch uav_simulation full_mission.launch
[Simulation runs immediately]
```

---

## 🎉 Success!

You now have a complete Docker setup for running the UAV simulation on Windows!

### What You Can Do Now

1. ✅ Run full UAV simulation in Docker
2. ✅ See Gazebo GUI on Windows desktop
3. ✅ Monitor mission progress
4. ✅ Record demonstrations
5. ✅ Analyze mission data
6. ✅ Modify and test changes

### Next Steps

1. **Run the simulation** - Follow `docker/DOCKER_QUICK_START.md`
2. **Customize parameters** - Edit `config/*.yaml` files
3. **Record demo** - Use `scripts/record_demo.sh`
4. **Analyze results** - Use `scripts/analyze_mission.py`
5. **Write report** - Use `docs/REPORT_TEMPLATE.md`

---

## 📞 Support

**Quick Help:**
- `docker/DOCKER_QUICK_START.md` - Quick reference
- `docker/TROUBLESHOOTING.md` - Problem solving

**Detailed Help:**
- `DOCKER_SETUP.md` - Complete guide
- `README.md` - Project documentation
- `COMMANDS_REFERENCE.md` - Command reference

---

**Ready to launch!** 🚀🐋

Start with: `docker/DOCKER_QUICK_START.md`

