#!/bin/bash

# Isaac Controllable Camera Launch Script
# This script launches the controllable camera node with proper environment

echo "🎮 Starting Isaac Sim Controllable Camera Node"
echo "================================================"

# Check if Isaac Sim path is set
if [ -z "$ISAAC_SIM_PATH" ]; then
    echo "❌ ISAAC_SIM_PATH environment variable not set"
    echo "Please set it to your Isaac Sim installation path:"
    echo "export ISAAC_SIM_PATH=\"/path/to/isaac-sim\""
    exit 1
fi

# Check if Isaac Sim exists
if [ ! -f "$ISAAC_SIM_PATH/python.sh" ]; then
    echo "❌ Isaac Sim python.sh not found at: $ISAAC_SIM_PATH/python.sh"
    echo "Please check your ISAAC_SIM_PATH"
    exit 1
fi

# Navigate to workspace
cd /home/kimate/isaac_ws

# Source ROS2 environment first
echo "📦 Sourcing ROS2 environment..."
source /opt/ros/jazzy/setup.bash

# Export ROS2 settings for Isaac Sim
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "🚀 Launching controllable camera node..."
echo ""
echo "🎮 Camera Control Topics:"
echo "  /camera/cmd_vel       - Velocity control (geometry_msgs/Twist)"
echo "  /camera/set_pose      - Absolute positioning (geometry_msgs/PoseStamped)"  
echo "  /camera/set_fov       - Field of view control (std_msgs/Float32)"
echo "  /camera/enable        - Enable/disable camera (std_msgs/Bool)"
echo ""
echo "📊 Camera Output Topics:"
echo "  /camera/rgb           - RGB image stream"
echo "  /camera/camera_info   - Camera calibration info"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Launch with Isaac Sim python
$ISAAC_SIM_PATH/python.sh isaac_camera_controllable.py
