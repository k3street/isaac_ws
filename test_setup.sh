#!/bin/bash
# Test script to verify ROS2 camera control setup

echo "=== Testing ROS2 Camera Control Setup ==="

# Build the ROS2 package
echo "1. Building ROS2 package..."
cd /home/kimate/isaac_ws
colcon build --packages-select isaac_test

if [ $? -eq 0 ]; then
    echo "✓ Package built successfully"
else
    echo "✗ Package build failed"
    exit 1
fi

# Source the workspace
echo "2. Sourcing workspace..."
source install/setup.bash

echo "3. Available ROS2 nodes:"
echo "   - camera_subscriber: listens to camera data"
echo "   - camera_controller: standalone ROS2 control node"
echo "   - isaac_camera_controlled: integrated Isaac Sim + ROS2 control"

echo ""
echo "4. To test camera control:"
echo "   Terminal 1: ros2 run isaac_test isaac_camera_controlled"
echo "   Terminal 2: ros2 topic pub /camera/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}'"
echo ""
echo "5. Available control topics:"
echo "   - /camera/cmd_vel (geometry_msgs/Twist): velocity control"
echo "   - /camera/set_pose (geometry_msgs/PoseStamped): position control"
echo "   - /camera/set_fov (std_msgs/Float32): field of view"
echo "   - /camera/enable (std_msgs/Bool): enable/disable camera"

echo ""
echo "✓ Test setup complete! Ready to test camera control."
