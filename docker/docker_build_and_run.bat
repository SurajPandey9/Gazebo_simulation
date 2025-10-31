@echo off
REM Complete Docker workflow: Pull image, start X server, run container, build workspace, launch simulation

echo ========================================
echo UAV Simulation - Complete Docker Setup
echo ========================================
echo.

REM Check Docker
docker info >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: Docker is not running!
    echo Please start Docker Desktop and try again.
    pause
    exit /b 1
)

echo [1/5] Checking Docker image...
docker images osrf/ros:noetic-desktop-full | find "noetic-desktop-full" >nul
if %ERRORLEVEL% NEQ 0 (
    echo Docker image not found. Pulling...
    echo This may take 10-20 minutes on first run...
    docker pull osrf/ros:noetic-desktop-full
) else (
    echo Docker image already exists.
)
echo.

echo [2/5] Checking VcXsrv X Server...
tasklist /FI "IMAGENAME eq vcxsrv.exe" 2>NUL | find /I /N "vcxsrv.exe">NUL
if "%ERRORLEVEL%"=="1" (
    echo Starting VcXsrv...
    call docker\start_xserver.bat
    timeout /t 3 >nul
) else (
    echo VcXsrv is already running.
)
echo.

echo [3/5] Creating setup script for container...
echo #!/bin/bash > docker\container_setup.sh
echo set -e >> docker\container_setup.sh
echo echo "========================================" >> docker\container_setup.sh
echo echo "Setting up UAV Simulation Environment" >> docker\container_setup.sh
echo echo "========================================" >> docker\container_setup.sh
echo cd /workspace >> docker\container_setup.sh
echo echo "" >> docker\container_setup.sh
echo echo "[1/4] Installing dependencies..." >> docker\container_setup.sh
echo apt-get update ^>^/dev/null 2^>^&1 >> docker\container_setup.sh
echo apt-get install -y python3-pip python3-opencv ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-vision-opencv ros-noetic-hector-quadrotor ros-noetic-hector-quadrotor-gazebo ros-noetic-gazebo-ros-pkgs ^>^/dev/null 2^>^&1 >> docker\container_setup.sh
echo pip3 install -q -r requirements.txt >> docker\container_setup.sh
echo echo "" >> docker\container_setup.sh
echo echo "[2/4] Setting permissions..." >> docker\container_setup.sh
echo chmod +x scripts/*.sh scripts/*.py >> docker\container_setup.sh
echo find src -name "*.py" -type f -exec chmod +x {} \; >> docker\container_setup.sh
echo echo "" >> docker\container_setup.sh
echo echo "[3/4] Building workspace..." >> docker\container_setup.sh
echo source /opt/ros/noetic/setup.bash >> docker\container_setup.sh
echo catkin_make >> docker\container_setup.sh
echo echo "" >> docker\container_setup.sh
echo echo "[4/4] Sourcing workspace..." >> docker\container_setup.sh
echo source devel/setup.bash >> docker\container_setup.sh
echo echo "" >> docker\container_setup.sh
echo echo "========================================" >> docker\container_setup.sh
echo echo "Setup Complete!" >> docker\container_setup.sh
echo echo "========================================" >> docker\container_setup.sh
echo echo "" >> docker\container_setup.sh
echo echo "To launch the simulation:" >> docker\container_setup.sh
echo echo "  roslaunch uav_simulation full_mission.launch" >> docker\container_setup.sh
echo echo "" >> docker\container_setup.sh
echo /bin/bash >> docker\container_setup.sh

echo.
echo [4/5] Starting Docker container...
echo.

REM Get current directory in Unix format
set CURRENT_DIR=%CD%
set UNIX_PATH=%CURRENT_DIR:\=/%
set UNIX_PATH=%UNIX_PATH:C:=/c%
set UNIX_PATH=%UNIX_PATH:D:=/d%
set UNIX_PATH=%UNIX_PATH:E:=/e%
set UNIX_PATH=%UNIX_PATH:F:=/f%

echo Container will start with interactive shell.
echo.
echo Inside the container, run:
echo   source /opt/ros/noetic/setup.bash
echo   cd /workspace
echo   ./docker/container_setup.sh
echo.
echo Or manually:
echo   catkin_make
echo   source devel/setup.bash
echo   roslaunch uav_simulation full_mission.launch
echo.
pause

docker run -it --rm ^
  -e DISPLAY=host.docker.internal:0 ^
  -e QT_X11_NO_MITSHM=1 ^
  -v "%UNIX_PATH%:/workspace" ^
  --name uav_sim ^
  osrf/ros:noetic-desktop-full ^
  bash

echo.
echo Container exited.
pause

