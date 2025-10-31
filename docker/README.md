# 🐋 Docker Files for UAV Simulation

This directory contains all Docker-related files for running the UAV simulation on Windows.

---

## 📁 Files Overview

### Windows Batch Scripts (.bat)

| File | Purpose | When to Use |
|------|---------|-------------|
| `start_xserver.bat` | Start VcXsrv X Server | Before running container |
| `docker_run.bat` | Run Docker container | Every time you want to run simulation |
| `docker_build_and_run.bat` | Complete automated workflow | First time setup |
| `build_custom_image.bat` | Build custom Docker image | Optional, for faster startup |

### Linux Shell Scripts (.sh)

| File | Purpose | When to Use |
|------|---------|-------------|
| `container_setup.sh` | Setup inside container | First time in container |

### Documentation (.md)

| File | Purpose | Read When |
|------|---------|-----------|
| `DOCKER_QUICK_START.md` | One-page quick reference | Getting started |
| `DOCKER_SETUP.md` | Complete detailed guide | Full setup instructions |
| `TROUBLESHOOTING.md` | Problem solving | Having issues |
| `README.md` | This file | Overview |

### Docker Files

| File | Purpose | When to Use |
|------|---------|-------------|
| `Dockerfile` | Custom image definition | Building custom image |

---

## 🚀 Quick Start

### First Time Setup

1. **Install Prerequisites:**
   - Docker Desktop: https://www.docker.com/products/docker-desktop
   - VcXsrv: https://sourceforge.net/projects/vcxsrv/

2. **Pull ROS Image:**
   ```bash
   docker pull osrf/ros:noetic-desktop-full
   ```

3. **Start X Server:**
   ```batch
   docker\start_xserver.bat
   ```

4. **Run Container:**
   ```batch
   docker\docker_run.bat
   ```

5. **Inside Container:**
   ```bash
   cd /workspace
   ./docker/container_setup.sh
   source devel/setup.bash
   roslaunch uav_simulation full_mission.launch
   ```

### Subsequent Runs

1. **Start X Server:**
   ```batch
   docker\start_xserver.bat
   ```

2. **Run Container:**
   ```batch
   docker\docker_run.bat
   ```

3. **Inside Container:**
   ```bash
   cd /workspace
   source /opt/ros/noetic/setup.bash
   source devel/setup.bash
   roslaunch uav_simulation full_mission.launch
   ```

---

## 📖 Documentation Guide

**Start here:**
1. Read `DOCKER_QUICK_START.md` for immediate usage
2. Read `DOCKER_SETUP.md` for detailed explanations
3. Refer to `TROUBLESHOOTING.md` if you encounter issues

---

## 🎯 Workflow Diagrams

### Standard Workflow

```
Windows
  ├─ Start VcXsrv (start_xserver.bat)
  └─ Start Docker (docker_run.bat)
       │
       └─ Inside Container
            ├─ First Time: ./docker/container_setup.sh
            ├─ Source: source devel/setup.bash
            └─ Launch: roslaunch uav_simulation full_mission.launch
```

### Custom Image Workflow (Optional)

```
Windows
  ├─ Build Custom Image (build_custom_image.bat)
  │    └─ Creates: uav_simulation:latest
  │
  ├─ Start VcXsrv (start_xserver.bat)
  └─ Run Custom Container
       └─ Inside Container
            ├─ Build: catkin_make (dependencies already installed)
            └─ Launch: roslaunch uav_simulation full_mission.launch
```

---

## 🔧 Customization

### Using Custom Image

After building custom image with `build_custom_image.bat`:

Edit `docker_run.bat`, change:
```batch
REM FROM:
osrf/ros:noetic-desktop-full

REM TO:
uav_simulation:latest
```

### Changing Resources

Edit Docker Desktop settings:
- Settings → Resources
- Adjust CPU, Memory, Swap
- Apply & Restart

### Changing Display

If `host.docker.internal:0` doesn't work, edit `docker_run.bat`:

```batch
REM Get your Windows IP
ipconfig

REM Use that IP instead
-e DISPLAY=192.168.1.100:0
```

---

## 🐛 Common Issues

### Issue: X Server not working
**Solution:** Read `TROUBLESHOOTING.md` → Issue 1

### Issue: Gazebo crashes
**Solution:** Read `TROUBLESHOOTING.md` → Issue 2

### Issue: Permission denied
**Solution:** Read `TROUBLESHOOTING.md` → Issue 3

### Issue: Slow performance
**Solution:** Read `TROUBLESHOOTING.md` → Issue 7

**For all issues:** See `TROUBLESHOOTING.md`

---

## 📋 File Permissions

All scripts should be executable:

```bash
# Inside container
chmod +x docker/*.sh
chmod +x scripts/*.sh scripts/*.py
find src -name "*.py" -exec chmod +x {} \;
```

---

## 🎓 Learning Resources

### Docker Basics
- Docker Desktop Documentation: https://docs.docker.com/desktop/
- Docker CLI Reference: https://docs.docker.com/engine/reference/commandline/cli/

### X Server on Windows
- VcXsrv Documentation: https://sourceforge.net/projects/vcxsrv/
- X11 Forwarding Guide: https://wiki.ros.org/docker/Tutorials/GUI

### ROS in Docker
- ROS Docker Images: https://hub.docker.com/_/ros
- ROS Docker Tutorial: https://wiki.ros.org/docker/Tutorials/Docker

---

## 💡 Tips & Tricks

### Tip 1: Keep Container Running

Instead of `--rm`, use named container:
```bash
docker run -it \
  -e DISPLAY=host.docker.internal:0 \
  -v /e/i5_psWaste/i2:/workspace \
  --name uav_sim_persistent \
  osrf/ros:noetic-desktop-full bash

# Later, restart it:
docker start -ai uav_sim_persistent
```

### Tip 2: Multiple Terminals

```bash
# Terminal 1: Main simulation
docker exec -it uav_sim bash

# Terminal 2: Monitoring
docker exec -it uav_sim bash
source /workspace/devel/setup.bash
rostopic echo /uav/state

# Terminal 3: Camera view
docker exec -it uav_sim bash
source /workspace/devel/setup.bash
rqt_image_view
```

### Tip 3: Save Custom Image

After setting up container:
```bash
# In another terminal
docker commit uav_sim my_uav_sim:v1

# Use it later
docker run -it my_uav_sim:v1 bash
```

### Tip 4: Faster Builds

Use custom image (see `build_custom_image.bat`):
- Dependencies pre-installed
- Faster startup
- Consistent environment

---

## 🔄 Update Workflow

### Update ROS Image

```bash
docker pull osrf/ros:noetic-desktop-full
```

### Update Custom Image

```bash
docker\build_custom_image.bat
```

### Update Workspace

Changes in `e:\i5_psWaste\i2` are immediately reflected in container (mounted volume).

---

## 🧹 Cleanup

### Remove Stopped Containers
```bash
docker container prune
```

### Remove Unused Images
```bash
docker image prune
```

### Complete Cleanup
```bash
docker system prune -a
```

**Warning:** This removes all Docker data!

---

## 📊 Resource Usage

### Typical Usage

- **Disk Space:** 
  - Base image: ~2 GB
  - Custom image: ~4 GB
  - Container: ~500 MB

- **Memory:**
  - Minimum: 4 GB
  - Recommended: 8 GB
  - Optimal: 12+ GB

- **CPU:**
  - Minimum: 2 cores
  - Recommended: 4 cores
  - Optimal: 6+ cores

---

## ✅ Verification

### Test Docker Setup

```bash
# Test 1: Docker works
docker run hello-world

# Test 2: ROS image works
docker run --rm osrf/ros:noetic-desktop-full rosversion -d

# Test 3: Volume mount works
docker run --rm -v /e/i5_psWaste/i2:/workspace osrf/ros:noetic-desktop-full ls /workspace

# Test 4: X Server works
docker run --rm -e DISPLAY=host.docker.internal:0 osrf/ros:noetic-desktop-full xclock
```

All should succeed.

---

## 🎯 Next Steps

After successful Docker setup:

1. **Run simulation** - Follow `DOCKER_QUICK_START.md`
2. **Customize parameters** - Edit config files
3. **Record demo** - Use `record_demo.sh`
4. **Analyze results** - Use `analyze_mission.py`

---

## 📞 Support

**Documentation:**
- Quick Start: `DOCKER_QUICK_START.md`
- Full Guide: `DOCKER_SETUP.md`
- Troubleshooting: `TROUBLESHOOTING.md`

**Project Documentation:**
- Main README: `../README.md`
- Quick Start: `../docs/QUICKSTART.md`
- Commands: `../COMMANDS_REFERENCE.md`

---

**Ready to run!** Start with `DOCKER_QUICK_START.md` 🚀

