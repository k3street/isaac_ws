#!/bin/bash

# Camera Control Demo Script
# Run this after the complete system is launched

echo "🎮 CAMERA CONTROL DEMONSTRATION"
echo "=================================="

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash
cd /home/kimate/isaac_ws
source install/setup.bash

echo ""
echo "📊 Current System Status:"
echo "Active camera topics: $(ros2 topic list | grep camera | wc -l)"
echo ""

# Wait for user input
read -p "Press Enter to start camera control demonstration..."

echo ""
echo "🎯 Demo 1: Setting camera to overhead view..."
ros2 topic pub --once /camera/set_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.0, y: 0.0, z: 10.0}, orientation: {x: 0.7071, y: 0.0, z: 0.0, w: 0.7071}}}"

sleep 2
echo "Current camera state:"
timeout 2s ros2 topic echo /camera/state --once

read -p "Press Enter for next demo..."

echo ""
echo "🎯 Demo 2: Moving camera in a circle..."
for i in {0..8}; do
    angle=$(echo "scale=3; $i * 3.14159 / 4" | bc -l)
    x=$(echo "scale=3; 5 * c($angle)" | bc -l)
    y=$(echo "scale=3; 5 * s($angle)" | bc -l)
    
    echo "Moving to position ($x, $y, 3.0)..."
    ros2 topic pub --once /camera/set_pose geometry_msgs/msg/PoseStamped \
      "{pose: {position: {x: $x, y: $y, z: 3.0}}}"
    
    sleep 1
done

read -p "Press Enter for next demo..."

echo ""
echo "🎯 Demo 3: Velocity control - smooth movement..."
echo "Moving forward for 3 seconds..."
ros2 topic pub --times 30 --rate 10 /camera/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0}}" &

sleep 3
pkill -f "ros2 topic pub.*cmd_vel"

echo "Rotating for 2 seconds..."
ros2 topic pub --times 20 --rate 10 /camera/cmd_vel geometry_msgs/msg/Twist \
  "{angular: {z: 0.5}}" &

sleep 2
pkill -f "ros2 topic pub.*cmd_vel"

read -p "Press Enter for next demo..."

echo ""
echo "🎯 Demo 4: Camera parameter control..."
echo "Changing field of view to wide angle (120°)..."
ros2 topic pub --once /camera/set_fov std_msgs/msg/Float32 "{data: 120.0}"

sleep 1

echo "Changing focus distance to 50m..."
ros2 topic pub --once /camera/set_focus std_msgs/msg/Float32 "{data: 50.0}"

sleep 1

echo "Final camera state:"
timeout 2s ros2 topic echo /camera/state --once

echo ""
echo "🎯 Demo 5: Camera disable/enable..."
echo "Disabling camera..."
ros2 topic pub --once /camera/enable std_msgs/msg/Bool "{data: false}"

sleep 1
timeout 2s ros2 topic echo /camera/state --once

echo "Re-enabling camera..."
ros2 topic pub --once /camera/enable std_msgs/msg/Bool "{data: true}"

sleep 1
timeout 2s ros2 topic echo /camera/state --once

echo ""
echo "🎉 DEMONSTRATION COMPLETE!"
echo ""
echo "📋 Available Commands for Manual Control:"
echo ""
echo "# Basic movement:"
echo "ros2 topic pub /camera/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}}'"
echo "ros2 topic pub /camera/cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.5}}'"
echo ""
echo "# Absolute positioning:"
echo "ros2 topic pub /camera/set_pose geometry_msgs/msg/PoseStamped '{pose: {position: {x: 5.0, y: 0.0, z: 3.0}}}'"
echo ""
echo "# Camera settings:"
echo "ros2 topic pub /camera/set_fov std_msgs/msg/Float32 '{data: 90.0}'"
echo "ros2 topic pub /camera/set_focus std_msgs/msg/Float32 '{data: 10.0}'"
echo ""
echo "# Monitoring:"
echo "ros2 topic echo /camera/state"
echo "ros2 topic echo /camera/current_pose"
