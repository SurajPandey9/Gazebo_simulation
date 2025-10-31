# Dockerfile for UAV Simulation with all dependencies pre-installed
# This creates a custom image with everything ready to go

FROM osrf/ros:noetic-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

# Update and install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-vision-opencv \
    ros-noetic-hector-quadrotor \
    ros-noetic-hector-quadrotor-gazebo \
    ros-noetic-hector-quadrotor-description \
    ros-noetic-hector-quadrotor-teleop \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-rqt \
    ros-noetic-rqt-common-plugins \
    ros-noetic-rqt-image-view \
    x11-apps \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip
RUN pip3 install --upgrade pip

# Install Python dependencies
RUN pip3 install \
    opencv-python==4.8.0.74 \
    numpy==1.24.3 \
    scipy==1.10.1 \
    torch==2.0.1 \
    torchvision==0.15.2 \
    ultralytics==8.0.134 \
    filterpy==1.4.5 \
    scikit-learn==1.3.0 \
    matplotlib==3.7.2 \
    pyyaml==6.0

# Set up ROS environment
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Create workspace directory
RUN mkdir -p /workspace

# Set working directory
WORKDIR /workspace

# Default command
CMD ["/bin/bash"]

