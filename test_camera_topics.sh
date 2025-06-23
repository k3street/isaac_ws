#!/bin/bash
"""
Test Isaac Sim Camera Topics
Checks if camera topics are available and working
"""

echo "🔍 Testing Isaac Sim Camera Topics..."
echo ""

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

echo "📋 Available ROS2 topics:"
ros2 topic list | grep -E "(camera|isaac)" || echo "❌ No camera topics found"
echo ""

echo "🏃 Topic rates (if available):"
timeout 5s ros2 topic hz /camera/rgb 2>/dev/null || echo "❌ /camera/rgb not publishing"
timeout 5s ros2 topic hz /camera/depth 2>/dev/null || echo "❌ /camera/depth not publishing" 
timeout 5s ros2 topic hz /camera/camera_info 2>/dev/null || echo "❌ /camera/camera_info not publishing"
echo ""

echo "📄 Camera info sample (if available):"
timeout 3s ros2 topic echo /camera/camera_info --once 2>/dev/null || echo "❌ No camera info available"
echo ""

echo "🎯 To run the full pipeline:"
echo "  Terminal 1: \$ISAAC_SIM_PATH/python.sh isaac_camera_node_final.py"
echo "  Terminal 2: ./run_camera_subscriber.sh"
