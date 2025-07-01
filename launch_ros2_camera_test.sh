#!/bin/bash
# Launch script for testing ROS2-based camera control
# This script starts the camera control node and waits for user input to run tests

echo "=== Isaac Sim ROS2 Camera Control Test ==="
echo "This script will:"
echo "1. Launch the camera control node with ROS2 integration"
echo "2. Wait for the system to initialize"
echo "3. Optionally run automated movement tests"
echo ""

# Check if Isaac Sim is available
if ! command -v isaac-sim.headless.native.sh &> /dev/null; then
    echo "WARNING: Isaac Sim not found in PATH"
    echo "Make sure Isaac Sim is properly installed and sourced."
    echo ""
fi

# Source ROS2 environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "📦 Sourcing ROS2 Humble..."
    source /opt/ros/humble/setup.bash
else
    echo "❌ ROS2 Humble not found. Please install ROS2 Humble."
    exit 1
fi

# Build the workspace if needed
if [ -d "build" ]; then
    echo "🔧 Sourcing local workspace..."
    source install/setup.bash
fi

echo ""
echo "🚀 Starting camera control node with ROS2 integration..."
echo "Press Ctrl+C to stop"
echo ""

# Launch the camera control node in the background
python3 camera_control_node.py &
CAMERA_PID=$!

# Wait for initialization
echo "⏳ Waiting for camera node to initialize (10 seconds)..."
sleep 10

# Check if camera node is still running
if ! kill -0 $CAMERA_PID 2>/dev/null; then
    echo "❌ Camera node failed to start properly"
    exit 1
fi

echo ""
echo "✅ Camera node is running (PID: $CAMERA_PID)"
echo ""
echo "🎮 Available control methods:"
echo "1. File-based: python3 camera_control_sender.py"
echo "2. ROS2 topics:"
echo "   - ros2 topic pub --once /camera/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}}'"
echo "   - ros2 topic pub --once /camera/cmd_pose geometry_msgs/msg/PoseStamped '{pose: {position: {x: 3.0, y: 3.0, z: 3.0}}}'"
echo ""
echo "📊 Monitor topics:"
echo "   - ros2 topic list"
echo "   - ros2 topic echo /camera/rgb"
echo "   - ros2 topic echo /camera/depth"
echo ""

# Ask user if they want to run automated tests
read -p "🤖 Run automated ROS2 movement tests? [y/N]: " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo ""
    echo "🧪 Starting automated tests..."
    python3 test_ros2_camera_control.py
else
    echo ""
    echo "💡 Manual testing mode. Use the commands above to test camera movement."
    echo "Press Ctrl+C to stop the camera node when done."
    
    # Wait for user to stop
    wait $CAMERA_PID
fi

# Clean up
echo ""
echo "🧹 Cleaning up..."
if kill -0 $CAMERA_PID 2>/dev/null; then
    kill $CAMERA_PID
    echo "✅ Camera node stopped"
fi

echo "Done!"
