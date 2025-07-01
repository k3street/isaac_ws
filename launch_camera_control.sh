#!/bin/bash

# Isaac Sim Camera Control System Launcher
echo "🎮 Starting Isaac Sim Camera Control System"
echo "============================================="

# Check Isaac Sim
if [ -z "$ISAAC_SIM_PATH" ]; then
    echo "❌ ISAAC_SIM_PATH not set"
    exit 1
fi

# Navigate to workspace
cd /home/kimate/isaac_ws

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Export necessary environment variables for ROS2 bridge
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Launch Camera Control Node
echo "🎥 Starting camera control node with movement capabilities..."
echo "Expected topics: /camera/rgb, /camera/depth, /camera/camera_info"
echo "Movement control: JSON commands via /tmp/isaac_camera_commands.json"
echo ""
echo "🎮 Control commands:"
echo "  - python3 camera_control_sender.py --move-forward"
echo "  - python3 camera_control_sender.py --move-back"
echo "  - echo '{\"type\":\"position\",\"data\":{\"x\":1.0,\"y\":2.0,\"z\":3.0}}' > /tmp/isaac_camera_commands.json"
echo ""

$ISAAC_SIM_PATH/python.sh camera_control_node.py
