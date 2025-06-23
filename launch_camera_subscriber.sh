#!/bin/bash

# Isaac Camera Subscriber Launch Script
# This script builds the ROS2 package and launches the camera subscriber node

echo "🚀 Starting Isaac Camera Subscriber Node"
echo "================================================"

# Navigate to workspace
cd /home/kimate/isaac_ws

# Source ROS2 environment
echo "📦 Sourcing ROS2 environment..."
source /opt/ros/jazzy/setup.bash

# Build the package
echo "🔨 Building isaac_test package..."
colcon build --packages-select isaac_test

# Check if build was successful
if [ $? -eq 0 ]; then
    echo "✅ Build successful!"
    
    # Source the workspace
    echo "📦 Sourcing workspace..."
    source install/setup.bash
    
    # Launch the camera subscriber node
    echo "🚀 Launching camera subscriber node..."
    echo "Press Ctrl+C to stop the node"
    echo ""
    
    ros2 run isaac_test camera_subscriber
    
else
    echo "❌ Build failed! Please check the error messages above."
    exit 1
fi
