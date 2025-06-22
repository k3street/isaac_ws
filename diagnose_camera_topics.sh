#!/bin/bash

echo "========================================"
echo "Camera Topic Diagnostic Script"
echo "========================================"
echo ""

echo "1. Current ROS2 Environment:"
echo "   ROS_DISTRO: $ROS_DISTRO"
echo "   RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo ""

echo "2. All ROS2 Topics:"
ros2 topic list | head -20
echo ""

echo "3. Looking for camera topics:"
ros2 topic list | grep -i camera || echo "   No camera topics found"
echo ""

echo "4. Topic types:"
echo "   /clock topic:"
ros2 topic info /clock 2>/dev/null || echo "   /clock topic not found"
echo ""

echo "5. ROS2 Node List:"
ros2 node list | head -10
echo ""

echo "6. Checking for Isaac Sim ROS2 nodes:"
ros2 node list | grep -i isaac || echo "   No Isaac Sim nodes found"
echo ""

echo "========================================"
echo "Diagnostic complete"
echo "========================================"
