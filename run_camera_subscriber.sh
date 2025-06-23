#!/bin/bash
"""
Launch Isaac Sim Camera Subscriber Node
Subscribes to camera data from Isaac Sim and displays live feed
"""

echo "🚀 Starting Isaac Sim Camera Subscriber..."
echo ""
echo "This node will:"
echo "  📹 Subscribe to Isaac Sim camera topics (/camera/rgb, /camera/depth, /camera/camera_info)"
echo "  🖼️  Display live RGB and depth camera feeds"
echo "  📊 Publish analysis and status information"
echo "  🔄 Show the rotating camera view from Isaac Sim"
echo ""
echo "Make sure Isaac Sim camera node is running first:"
echo "  \$ISAAC_SIM_PATH/python.sh isaac_camera_node_final.py"
echo ""

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

# Source the local workspace if built
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "✅ Using built workspace"
else
    echo "⚠️  Workspace not built yet. Run: colcon build --packages-select isaac_test"
fi

echo "🎯 Starting camera subscriber node..."
echo "Press Ctrl+C to stop"
echo ""

# Run the subscriber node
ros2 run isaac_test camera_subscriber
