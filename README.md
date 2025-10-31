# UAV Target Detection and Engagement Simulation

A ROS + Gazebo simulation project demonstrating autonomous UAV (drone) capabilities for detecting, tracking, and engaging ground targets (tanks) in a simulated battlefield environment.

## Project Overview

This project implements a complete autonomous system featuring:
- **Autonomous Flight Control**: Takeoff, search patterns, and target approach
- **Computer Vision Pipeline**: Object detection, tracking, and re-identification
- **Target Engagement**: Autonomous approach and engagement capabilities
- **Simulated Environment**: Realistic battlefield scenario in Gazebo

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Gazebo Simulation                         │
│  ┌──────────────┐              ┌──────────────┐            │
│  │  UAV Model   │              │  Tank Model  │            │
│  │  + Camera    │              │  (Target)    │            │
│  └──────────────┘              └──────────────┘            │
└─────────────────────────────────────────────────────────────┘
           │                              │
           ▼                              ▼
┌─────────────────────────────────────────────────────────────┐
│                      ROS Middleware                          │
│  /camera/image_raw    /uav/pose    /tank/pose              │
└─────────────────────────────────────────────────────────────┘
           │                              │
           ▼                              ▼
┌──────────────────────┐      ┌─────────────────────────────┐
│  Vision Pipeline     │      │  Flight Controller          │
│  - Detection         │◄────►│  - Takeoff                  │
│  - Tracking          │      │  - Search Pattern           │
│  - Re-ID             │      │  - Target Approach          │
└──────────────────────┘      └─────────────────────────────┘
           │                              │
           └──────────────┬───────────────┘
                          ▼
                ┌──────────────────┐
                │ Engagement System │
                └──────────────────┘
```

## Package Structure

- **uav_simulation**: Main simulation package with Gazebo worlds and models
- **uav_control**: Flight controller and autonomous navigation
- **uav_vision**: Computer vision pipeline (detection, tracking, re-ID)
- **uav_engagement**: Target engagement logic
- **uav_msgs**: Custom ROS message definitions

## Prerequisites

### Required Software
- Ubuntu 20.04 LTS (recommended)
- ROS Noetic
- Gazebo 11
- Python 3.8+

### Required ROS Packages
```bash
sudo apt-get update
sudo apt-get install -y \
    ros-noetic-desktop-full \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-hector-quadrotor \
    ros-noetic-hector-quadrotor-gazebo \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-vision-opencv
```

### Python Dependencies
```bash
pip3 install -r requirements.txt
```

## Installation

1. **Clone the repository**:
```bash
git clone <repository-url>
cd i2
```

2. **Build the workspace**:
```bash
catkin_make
# or if using catkin tools:
# catkin build
```

3. **Source the workspace**:
```bash
source devel/setup.bash
```

## Usage

### Quick Start - Full Simulation

Launch the complete simulation with all components:
```bash
roslaunch uav_simulation full_mission.launch
```

This will:
1. Start Gazebo with the battlefield environment
2. Spawn the UAV and tank models
3. Launch the flight controller
4. Start the computer vision pipeline
5. Initialize the engagement system

### Step-by-Step Launch

For debugging or testing individual components:

1. **Launch Gazebo environment only**:
```bash
roslaunch uav_simulation battlefield.launch
```

2. **Launch UAV control**:
```bash
roslaunch uav_control flight_controller.launch
```

3. **Launch vision pipeline**:
```bash
roslaunch uav_vision detection_tracking.launch
```

4. **Launch engagement system**:
```bash
roslaunch uav_engagement target_engagement.launch
```

## Mission Workflow

1. **Takeoff Phase**: UAV autonomously takes off to 10m altitude
2. **Search Phase**: UAV executes search pattern to locate target
3. **Detection Phase**: Computer vision detects tank in camera feed
4. **Tracking Phase**: System locks onto and tracks the target
5. **Approach Phase**: UAV descends and approaches the target
6. **Engagement Phase**: UAV executes target engagement
7. **Mission Complete**: System reports success

## Computer Vision Pipeline

### Object Detection
- **Algorithm**: YOLOv5 (optimized for real-time detection)
- **Input**: Camera feed from UAV (640x480 @ 30fps)
- **Output**: Bounding boxes with confidence scores
- **Detection Range**: Effective from 10m altitude

### Object Tracking
- **Algorithm**: DeepSORT (Deep Learning + SORT)
- **Features**: 
  - Kalman filtering for motion prediction
  - Deep appearance features for re-identification
  - Handles occlusions and temporary losses

### Re-Identification
- **Method**: Deep feature extraction + cosine similarity
- **Capabilities**:
  - Maintains target identity across frames
  - Recovers correct target after temporary loss
  - Distinguishes between similar objects

## Flight Control

### Search Pattern
- **Type**: Expanding square spiral
- **Altitude**: 10m (constant during search)
- **Speed**: 2 m/s
- **Coverage**: 100m x 100m area

### Approach Trajectory
- **Strategy**: Proportional descent with visual servoing
- **Descent Rate**: Proportional to distance from target
- **Lateral Control**: Keeps target centered in frame
- **Safety**: Minimum altitude 1m, collision avoidance enabled

## Configuration

Key parameters can be adjusted in configuration files:

- `config/flight_params.yaml`: Flight control parameters
- `config/vision_params.yaml`: Computer vision settings
- `config/mission_params.yaml`: Mission-specific settings

## Monitoring and Visualization

### RViz Visualization
```bash
roslaunch uav_simulation visualization.launch
```

### View Camera Feed
```bash
rosrun rqt_image_view rqt_image_view
```

### Monitor Topics
```bash
# UAV state
rostopic echo /uav/state

# Detection results
rostopic echo /vision/detections

# Mission status
rostopic echo /mission/status
```

## Troubleshooting

### Gazebo crashes on startup
- Ensure you have sufficient GPU resources
- Try reducing camera resolution in config

### UAV doesn't take off
- Check that controllers are loaded: `rosservice list | grep controller`
- Verify UAV model spawned: `rosservice call /gazebo/get_model_state '{model_name: uav}'`

### No detections
- Verify camera feed: `rostopic hz /camera/image_raw`
- Check if tank is in view: Use RViz to visualize
- Adjust detection confidence threshold in config

## Project Structure

```
i2/
├── src/
│   ├── uav_simulation/      # Gazebo worlds, models, launch files
│   ├── uav_control/         # Flight controller nodes
│   ├── uav_vision/          # Computer vision nodes
│   ├── uav_engagement/      # Engagement logic
│   └── uav_msgs/            # Custom message definitions
├── config/                  # Configuration files
├── docs/                    # Documentation and report
├── scripts/                 # Utility scripts
├── requirements.txt         # Python dependencies
└── README.md               # This file
```

## Development

### Adding New Features
1. Create feature branch: `git checkout -b feature/your-feature`
2. Implement changes
3. Test in simulation
4. Submit pull request

### Running Tests
```bash
catkin_make run_tests
```

## Contributing

Contributions are welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License

This project is licensed under the MIT License - see LICENSE file for details.

## Authors

- Your Name - Initial work

## Acknowledgments

- ROS community for excellent documentation
- Gazebo team for simulation tools
- YOLOv5 and DeepSORT authors for computer vision algorithms

## References

1. Bewley, A., et al. "Simple online and realtime tracking." ICIP 2016.
2. Wojke, N., et al. "Simple online and realtime tracking with a deep association metric." ICIP 2017.
3. Redmon, J., et al. "You only look once: Unified, real-time object detection." CVPR 2016.

## Contact

For questions or issues, please open an issue on GitHub or contact [your-email].

