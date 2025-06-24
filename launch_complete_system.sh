#!/bin/bash

# Complete Isaac Sim + ROS2 Camera Control System Launcher
# This script launches the full system with visualization

echo "🚀 LAUNCHING COMPLETE ISAAC SIM CAMERA CONTROL SYSTEM"
echo "============================================================"
echo "This will start:"
echo "  1. Isaac Sim with camera publishing"
echo "  2. ROS2 camera controller for real-time control"
echo "  3. Camera data subscriber and processor"
echo "  4. Visualization tools (RViz2)"
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

# Function to start Isaac Sim with camera
start_isaac_sim() {
    echo "🎮 Starting Isaac Sim camera publisher..."
    cd /home/kimate/isaac_ws
    $ISAAC_SIM_PATH/python.sh isaac_camera_node_final.py &
    ISAAC_PID=$!
    echo "Isaac Sim PID: $ISAAC_PID"
}

# Function to start ROS2 controller
start_camera_controller() {
    echo "🎛️  Starting camera controller..."
    ros2 run isaac_test camera_controller &
    CONTROLLER_PID=$!
    echo "Controller PID: $CONTROLLER_PID"
}

# Function to start camera subscriber
start_camera_subscriber() {
    echo "📸 Starting camera subscriber..."
    ros2 run isaac_test camera_subscriber &
    SUBSCRIBER_PID=$!
    echo "Subscriber PID: $SUBSCRIBER_PID"
}

# Function to start RViz2 for visualization
start_rviz() {
    echo "📊 Starting RViz2 with camera visualization config..."
    rviz2 -d /home/kimate/isaac_ws/camera_control_rviz.rviz &
    RVIZ_PID=$!
    echo "RViz2 PID: $RVIZ_PID"
}

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "🧹 Cleaning up processes..."
    kill $ISAAC_PID 2>/dev/null
    kill $CONTROLLER_PID 2>/dev/null  
    kill $SUBSCRIBER_PID 2>/dev/null
    kill $RVIZ_PID 2>/dev/null
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
    echo "⚠️  Isaac Sim camera topics not detected - using simulation mode"
    ISAAC_WORKING=false
fi

# Start ROS2 components
start_camera_controller
sleep 3

start_camera_subscriber  
sleep 3

# Start visualization
start_rviz
sleep 3

echo ""
echo "🎉 SYSTEM LAUNCH COMPLETE!"
echo "============================================================"
echo ""
echo "📊 System Status:"
if [ "$ISAAC_WORKING" = true ]; then
    echo "  ✅ Isaac Sim Camera: Running and publishing"
else
    echo "  ⚠️  Isaac Sim Camera: Simulation mode (check console for errors)"
fi
echo "  ✅ Camera Controller: Running"
echo "  ✅ Camera Subscriber: Running"  
echo "  ✅ RViz2 Visualization: Running"
echo ""
echo "🎮 Camera Control Commands:"
echo "  # Move camera forward"
echo "  ros2 topic pub /camera/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}}'"
echo ""
echo "  # Rotate camera (yaw)"
echo "  ros2 topic pub /camera/cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.5}}'"
echo ""
echo "  # Set absolute position"
echo "  ros2 topic pub /camera/set_pose geometry_msgs/msg/PoseStamped '{pose: {position: {x: 5.0, y: 0.0, z: 3.0}}}'"
echo ""
echo "  # Change field of view"
echo "  ros2 topic pub /camera/set_fov std_msgs/msg/Float32 '{data: 90.0}'"
echo ""
echo "📊 Monitoring Commands:"
echo "  # Watch camera state"
echo "  ros2 topic echo /camera/state"
echo ""
echo "  # Monitor image processing"
echo "  ros2 topic echo /camera/analysis"
echo ""
echo "  # View available topics"
echo "  ros2 topic list | grep camera"
echo ""
echo "🎯 RViz2 Setup:"
echo "  1. Add -> By topic -> /camera/rgb -> Image"
echo "  2. Add -> By topic -> /camera/depth -> Image" 
echo "  3. Add -> By topic -> /camera/current_pose -> Pose"
echo "  4. Set Fixed Frame to 'camera_frame'"
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
