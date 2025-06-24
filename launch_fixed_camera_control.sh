#!/bin/bash

# Fixed Isaac Sim + ROS2 Camera Control System Launcher
# Camera will NOT auto-rotate - controlled by ROS2 commands only

echo "🚀 LAUNCHING FIXED ISAAC SIM CAMERA CONTROL SYSTEM"
echo "============================================================"
echo "🔧 FIXES APPLIED:"
echo "  ✅ Camera automatic rotation DISABLED"
echo "  ✅ Camera responds to ROS2 control commands"
echo "  ✅ Camera stays stationary until ROS2 commands received"
echo ""
echo "This will start:"
echo "  1. Isaac Sim with stationary camera (NO auto-rotation)"
echo "  2. ROS2 camera controller for real-time control"
echo "  3. Camera data subscriber and processor"
echo ""

# Check dependencies
echo "📋 Checking dependencies..."

# Check Isaac Sim
if [ -z "$ISAAC_SIM_PATH" ]; then
    echo "❌ ISAAC_SIM_PATH not set. Please export ISAAC_SIM_PATH=\"/path/to/isaac-sim\""
    exit 1
fi

if [ ! -f "$ISAAC_SIM_PATH/python.sh" ]; then
    echo "❌ Isaac Sim not found at: $ISAAC_SIM_PATH"
    exit 1
fi

# Check ROS2
if ! command -v ros2 &> /dev/null; then
    echo "❌ ROS2 not found. Please install ROS2 Jazzy"
    exit 1
fi

echo "✅ All dependencies found"
echo "📍 Isaac Sim Path: $ISAAC_SIM_PATH"

# Navigate to workspace
cd /home/kimate/isaac_ws

# Source ROS2 environment
echo "📦 Sourcing ROS2 environment..."
source /opt/ros/jazzy/setup.bash

# Build the ROS2 package
echo "🔨 Building isaac_test package..."
colcon build --packages-select isaac_test

if [ $? -ne 0 ]; then
    echo "❌ Build failed!"
    exit 1
fi

# Source the workspace
source install/setup.bash

echo "✅ Build successful"

# Set ROS2 environment for Isaac Sim
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Function to start Isaac Sim with controlled camera
start_isaac_sim() {
    echo "🎮 Starting Isaac Sim with controlled camera (NO auto-rotation)..."
    cd /home/kimate/isaac_ws
    $ISAAC_SIM_PATH/python.sh isaac_camera_node_controlled.py &
    ISAAC_PID=$!
    echo "Isaac Sim PID: $ISAAC_PID"
}

# Function to start camera subscriber
start_camera_subscriber() {
    echo "📸 Starting camera subscriber..."
    ros2 run isaac_test camera_subscriber &
    SUBSCRIBER_PID=$!
    echo "Subscriber PID: $SUBSCRIBER_PID"
}

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "🧹 Cleaning up processes..."
    kill $ISAAC_PID 2>/dev/null
    kill $SUBSCRIBER_PID 2>/dev/null
    echo "✅ Cleanup complete"
    exit 0
}

# Set trap for cleanup
trap cleanup SIGINT SIGTERM

echo ""
echo "🚀 LAUNCHING SYSTEM COMPONENTS..."
echo ""

# Start Isaac Sim first
start_isaac_sim
echo "⏳ Waiting for Isaac Sim to initialize (30 seconds)..."
sleep 30

# Check if Isaac Sim topics are available
echo "🔍 Checking for Isaac Sim camera topics..."
if ros2 topic list | grep -q "/camera/rgb"; then
    echo "✅ Isaac Sim camera topics detected"
    ISAAC_WORKING=true
else
    echo "⚠️  Isaac Sim camera topics not detected - check console for errors"
    ISAAC_WORKING=false
fi

# Start ROS2 components
start_camera_subscriber
sleep 3

echo ""
echo "🎉 SYSTEM LAUNCH COMPLETE!"
echo "============================================================"
echo ""
echo "📊 System Status:"
if [ "$ISAAC_WORKING" = true ]; then
    echo "  ✅ Isaac Sim Camera: Running and publishing (NO auto-rotation)"
else
    echo "  ⚠️  Isaac Sim Camera: Check console for errors"
fi
echo "  ✅ Camera Subscriber: Running"
echo ""
echo "🎮 Camera Control Commands (camera will respond immediately):"
echo ""
echo "  # Move camera forward"
echo "  ros2 topic pub --once /camera/cmd_vel geometry_msgs/msg/Twist '{\"linear\": {\"x\": 1.0}}'"
echo ""
echo "  # Move camera backward"
echo "  ros2 topic pub --once /camera/cmd_vel geometry_msgs/msg/Twist '{\"linear\": {\"x\": -1.0}}'"
echo ""
echo "  # Rotate camera (yaw)"
echo "  ros2 topic pub --once /camera/cmd_vel geometry_msgs/msg/Twist '{\"angular\": {\"z\": 0.5}}'"
echo ""
echo "  # Stop camera movement"
echo "  ros2 topic pub --once /camera/cmd_vel geometry_msgs/msg/Twist '{}'"
echo ""
echo "  # Set absolute position"
echo "  ros2 topic pub --once /camera/set_pose geometry_msgs/msg/PoseStamped '{\"pose\": {\"position\": {\"x\": 5.0, \"y\": 0.0, \"z\": 3.0}}}'"
echo ""
echo "  # Disable camera movement"
echo "  ros2 topic pub --once /camera/enable std_msgs/msg/Bool '{\"data\": false}'"
echo ""
echo "  # Enable camera movement"
echo "  ros2 topic pub --once /camera/enable std_msgs/msg/Bool '{\"data\": true}'"
echo ""
echo "📊 Monitoring Commands:"
echo ""
echo "  # Watch camera state"
echo "  ros2 topic echo /camera/state"
echo ""
echo "  # Watch camera position"
echo "  ros2 topic echo /camera/current_pose"
echo ""
echo "  # View available topics"
echo "  ros2 topic list | grep camera"
echo ""
echo "  # View camera RGB feed info"
echo "  ros2 topic info /camera/rgb"
echo ""
echo "🎯 Key Features:"
echo "  ✅ Camera starts STATIONARY (fixed position)"
echo "  ✅ NO automatic rotation"
echo "  ✅ Responds immediately to ROS2 commands"
echo "  ✅ Real-time position/orientation control"
echo "  ✅ Enable/disable functionality"
echo ""
echo "Press Ctrl+C to stop all components"
echo ""

# Keep script running and show live status
while true; do
    sleep 5
    echo "$(date '+%H:%M:%S') - System running... (Ctrl+C to stop)"
    
    # Show topic count
    TOPIC_COUNT=$(ros2 topic list | grep camera | wc -l)
    echo "  Camera topics active: $TOPIC_COUNT"
    
    # Show camera state if available
    if ros2 topic list | grep -q "/camera/state"; then
        STATE=$(timeout 1s ros2 topic echo /camera/state --once 2>/dev/null | grep "data:" | cut -d'"' -f2)
        if [ ! -z "$STATE" ]; then
            echo "  Current state: $STATE"
        fi
    fi
    echo ""
done
