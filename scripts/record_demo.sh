#!/bin/bash
# Script to record demonstration video of the UAV mission

echo "========================================="
echo "UAV Mission Demo Recording Script"
echo "========================================="

# Check if required tools are installed
if ! command -v rosbag &> /dev/null; then
    echo "Error: rosbag not found. Please install ROS."
    exit 1
fi

if ! command -v ffmpeg &> /dev/null; then
    echo "Warning: ffmpeg not found. Installing..."
    sudo apt-get install -y ffmpeg
fi

# Create output directory
OUTPUT_DIR="$(dirname "$0")/../recordings"
mkdir -p "$OUTPUT_DIR"

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BAG_FILE="$OUTPUT_DIR/mission_$TIMESTAMP.bag"
VIDEO_FILE="$OUTPUT_DIR/mission_$TIMESTAMP.mp4"

echo ""
echo "Recording will be saved to:"
echo "  Bag file: $BAG_FILE"
echo "  Video: $VIDEO_FILE"
echo ""

# Topics to record
TOPICS="/uav/camera/image_raw \
        /vision/debug_image \
        /vision/tracked_image \
        /vision/detections \
        /vision/target_state \
        /uav/state \
        /uav/ground_truth/pose \
        /tank/ground_truth/pose \
        /cmd_vel"

echo "Starting rosbag recording..."
echo "Topics: $TOPICS"
echo ""
echo "Press Ctrl+C to stop recording"
echo ""

# Start recording
rosbag record -O "$BAG_FILE" $TOPICS &
ROSBAG_PID=$!

# Wait for user to stop
trap "kill $ROSBAG_PID 2>/dev/null" EXIT

wait $ROSBAG_PID

echo ""
echo "Recording stopped."
echo "Bag file saved to: $BAG_FILE"
echo ""

# Ask if user wants to convert to video
read -p "Convert camera feed to video? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Converting to video..."
    echo "This may take a few minutes..."
    
    # Play bag and convert to video
    rosrun image_view video_recorder image:=/vision/tracked_image _filename:="$VIDEO_FILE" _fps:=30 &
    VIDEO_PID=$!
    
    sleep 2
    rosbag play "$BAG_FILE"
    
    sleep 2
    kill $VIDEO_PID 2>/dev/null
    
    echo "Video saved to: $VIDEO_FILE"
fi

echo ""
echo "========================================="
echo "Recording Complete!"
echo "========================================="
echo ""
echo "Files saved in: $OUTPUT_DIR"
echo ""

