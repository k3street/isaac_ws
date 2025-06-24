#!/bin/bash
# Diagnostic script to test ROS2 camera control

echo "=== ROS2 Camera Control Diagnostics ==="

# Source the workspace
source /home/kimate/isaac_ws/install/setup.bash

echo "1. Checking available ROS2 nodes..."
echo "Available executables:"
ros2 pkg executables isaac_test

echo ""
echo "2. Testing ROS2 topic publishing..."

# Test if we can publish to control topics
echo "Testing /camera/cmd_vel topic..."
timeout 2s ros2 topic pub /camera/cmd_vel geometry_msgs/msg/Twist '{}' --once
if [ $? -eq 0 ]; then
    echo "✓ Can publish to /camera/cmd_vel"
else
    echo "✗ Failed to publish to /camera/cmd_vel"
fi

echo "Testing /camera/enable topic..."
timeout 2s ros2 topic pub /camera/enable std_msgs/msg/Bool '{"data": true}' --once
if [ $? -eq 0 ]; then
    echo "✓ Can publish to /camera/enable"
else
    echo "✗ Failed to publish to /camera/enable"
fi

echo ""
echo "3. Checking for running camera nodes..."
ros2 node list | grep -i camera
if [ $? -eq 0 ]; then
    echo "✓ Camera nodes are running"
else
    echo "ℹ No camera nodes currently running"
fi

echo ""
echo "4. Available ROS2 topics related to camera:"
ros2 topic list | grep -i camera

echo ""
echo "=== Diagnostic Complete ==="
echo "To start camera control: ros2 run isaac_test isaac_camera_controlled"
