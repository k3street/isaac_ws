#!/bin/bash

# Camera Topic Test Script
# This script tests the availability of Isaac Sim camera topics and the subscriber node

echo "🔍 Isaac Sim Camera Topic Test"
echo "================================================"

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash
cd /home/kimate/isaac_ws
source install/setup.bash

echo ""
echo "1️⃣ Checking available ROS2 topics..."
echo "Looking for Isaac Sim camera topics:"
ros2 topic list | grep -E "(camera|isaac)" || echo "❌ No camera topics found"

echo ""
echo "2️⃣ Checking /camera/rgb topic specifically..."
if ros2 topic list | grep -q "/camera/rgb"; then
    echo "✅ /camera/rgb topic is available"
    
    echo ""
    echo "3️⃣ Getting topic info for /camera/rgb..."
    ros2 topic info /camera/rgb
    
    echo ""
    echo "4️⃣ Checking message rate (5 second sample)..."
    timeout 5s ros2 topic hz /camera/rgb || echo "⚠️  No messages received in 5 seconds"
    
    echo ""
    echo "5️⃣ Getting a sample message..."
    timeout 3s ros2 topic echo /camera/rgb --once || echo "⚠️  Could not get sample message"
    
else
    echo "❌ /camera/rgb topic not found"
    echo "Make sure Isaac Sim is running with the camera node"
fi

echo ""
echo "6️⃣ Checking /camera/depth topic..."
if ros2 topic list | grep -q "/camera/depth"; then
    echo "✅ /camera/depth topic is available"
    ros2 topic info /camera/depth
else
    echo "❌ /camera/depth topic not found"
fi

echo ""
echo "7️⃣ Checking /camera/camera_info topic..."
if ros2 topic list | grep -q "/camera/camera_info"; then
    echo "✅ /camera/camera_info topic is available"
    ros2 topic info /camera/camera_info
else
    echo "❌ /camera/camera_info topic not found"
fi

echo ""
echo "8️⃣ Checking isaac_test package..."
if ros2 pkg list | grep -q "isaac_test"; then
    echo "✅ isaac_test package is available"
    
    echo ""
    echo "9️⃣ Checking camera_subscriber executable..."
    if ros2 pkg executables isaac_test | grep -q "camera_subscriber"; then
        echo "✅ camera_subscriber executable is available"
    else
        echo "❌ camera_subscriber executable not found"
        echo "Try rebuilding the package with: colcon build --packages-select isaac_test"
    fi
else
    echo "❌ isaac_test package not found"
    echo "Build the package first with: colcon build --packages-select isaac_test"
fi

echo ""
echo "🔚 Test completed!"
echo ""
echo "📋 Next Steps:"
echo "   - If camera topics are available, run: ./launch_camera_subscriber.sh"
echo "   - If topics are missing, start Isaac Sim with camera node first"
echo "   - Monitor the subscriber with: ros2 topic echo /isaac_test/status"
