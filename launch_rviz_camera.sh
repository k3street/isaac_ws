#!/bin/bash
"""
Launch RViz2 to visualize the rotating camera feed
"""

echo "Starting RViz2 to visualize Isaac Sim camera feed..."
echo "Camera topics available:"
echo "  - /camera/rgb (Image)"
echo "  - /camera/depth (Image)" 
echo "  - /camera/camera_info (CameraInfo)"
echo ""

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

# Launch RViz2 with camera visualization
rviz2 &

echo "RViz2 launched. To view camera feed:"
echo "1. Click 'Add' button"
echo "2. Select 'Image' display type"
echo "3. Set Image Topic to '/camera/rgb' or '/camera/depth'"
echo "4. You should see the rotating camera view!"
echo ""
echo "Press Ctrl+C to stop"

wait
