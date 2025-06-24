#!/bin/bash

# Camera Controller Test Script
# This script demonstrates various camera control commands

echo "🎮 Isaac Camera Controller Test Script"
echo "================================================"

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash
cd /home/kimate/isaac_ws
source install/setup.bash

echo ""
echo "1️⃣ Testing Camera Controller Availability..."
if ros2 pkg executables isaac_test | grep -q "camera_controller"; then
    echo "✅ camera_controller executable found"
else
    echo "❌ camera_controller executable not found"
    echo "Build the package first: colcon build --packages-select isaac_test"
    exit 1
fi

echo ""
echo "2️⃣ Starting Camera Controller (background)..."
ros2 run isaac_test camera_controller &
CONTROLLER_PID=$!
sleep 2

echo ""
echo "3️⃣ Testing Basic Camera Commands..."

echo "  📍 Setting camera position..."
ros2 topic pub --once /camera/set_pose geometry_msgs/msg/PoseStamped "
header:
  frame_id: 'world'
pose:
  position:
    x: 1.0
    y: 2.0 
    z: 3.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"

sleep 1

echo "  🔍 Setting camera FOV to 45 degrees..."
ros2 topic pub --once /camera/set_fov std_msgs/msg/Float32 "data: 45.0"

sleep 1

echo "  📷 Setting focus distance to 10 meters..."
ros2 topic pub --once /camera/set_focus std_msgs/msg/Float32 "data: 10.0"

sleep 1

echo "  🎥 Testing velocity control (move forward)..."
ros2 topic pub --once /camera/cmd_vel geometry_msgs/msg/Twist "
linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.1"

sleep 2

echo ""
echo "4️⃣ Checking Camera State..."
echo "Current camera state:"
timeout 3s ros2 topic echo /camera/state --once

echo ""
echo "Current camera pose:"
timeout 3s ros2 topic echo /camera/current_pose --once

echo ""
echo "5️⃣ Testing Camera Disable/Enable..."
echo "  ⏸️  Disabling camera..."
ros2 topic pub --once /camera/enable std_msgs/msg/Bool "data: false"

sleep 1

echo "  ▶️  Enabling camera..."
ros2 topic pub --once /camera/enable std_msgs/msg/Bool "data: true"

echo ""
echo "6️⃣ Available Topics:"
echo "Control topics:"
ros2 topic list | grep "/camera" | grep -E "(cmd_vel|set_|enable)"

echo ""
echo "Status topics:"
ros2 topic list | grep "/camera" | grep -E "(state|current_pose)"

echo ""
echo "🔚 Test completed!"

# Clean up
echo "Stopping camera controller..."
kill $CONTROLLER_PID 2>/dev/null

echo ""
echo "📋 Usage Examples:"
echo ""
echo "# Move camera forward at 0.5 m/s"
echo "ros2 topic pub /camera/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}'"
echo ""
echo "# Rotate camera (yaw) at 0.2 rad/s"  
echo "ros2 topic pub /camera/cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.2}}'"
echo ""
echo "# Set absolute position"
echo "ros2 topic pub /camera/set_pose geometry_msgs/msg/PoseStamped '{pose: {position: {x: 5.0, y: 0.0, z: 2.0}}}'"
echo ""
echo "# Change field of view to 90 degrees"
echo "ros2 topic pub /camera/set_fov std_msgs/msg/Float32 '{data: 90.0}'"
echo ""
echo "# Monitor camera state continuously"
echo "ros2 topic echo /camera/state"
