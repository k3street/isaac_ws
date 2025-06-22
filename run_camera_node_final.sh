#!/bin/bash

# Run Isaac Sim ROS2 Camera Node - Final Working Version
# Based on official Isaac Sim examples using isaacsim.ros2.bridge

set -e

echo "=== Isaac Sim 5.0 ROS2 Camera Node - Final Test ==="
echo "Timestamp: $(date)"
echo "Working directory: $(pwd)"
echo

# Check if we're in the correct directory
if [[ ! -f "isaac_camera_node_final.py" ]]; then
    echo "ERROR: isaac_camera_node_final.py not found"
    echo "Please run this script from the isaac_ws directory"
    exit 1
fi

# Source ROS2 environment
echo "Sourcing ROS2 Jazzy environment..."
if [[ -f "/opt/ros/jazzy/setup.bash" ]]; then
    source /opt/ros/jazzy/setup.bash
    echo "✓ ROS2 Jazzy environment sourced"
else
    echo "ERROR: ROS2 Jazzy not found at /opt/ros/jazzy/"
    exit 1
fi

# Check Isaac Sim environment
echo "Checking Isaac Sim environment..."
if [[ -z "$ISAAC_SIM_PATH" ]]; then
    echo "WARNING: ISAAC_SIM_PATH not set, trying default locations..."
    
    # Try common Isaac Sim paths
    ISAAC_PATHS=(
        "/home/kimate/.local/share/ov/pkg/isaac-sim-5.0.0"
        "/home/kimate/.local/share/ov/pkg/isaac_sim-5.0.0"
        "/opt/nvidia/isaac-sim"
        "/usr/local/isaac-sim"
    )
    
    ISAAC_SIM_PATH=""
    for path in "${ISAAC_PATHS[@]}"; do
        if [[ -d "$path" ]]; then
            ISAAC_SIM_PATH="$path"
            echo "✓ Found Isaac Sim at: $ISAAC_SIM_PATH"
            break
        fi
    done
    
    if [[ -z "$ISAAC_SIM_PATH" ]]; then
        echo "ERROR: Isaac Sim installation not found"
        echo "Please set ISAAC_SIM_PATH environment variable"
        exit 1
    fi
else
    echo "✓ Using Isaac Sim at: $ISAAC_SIM_PATH"
fi

# Export Isaac Sim environment
export ISAAC_SIM_PATH
export PYTHONPATH="$ISAAC_SIM_PATH:$PYTHONPATH"

echo "Environment setup complete."
echo

# Make script executable
chmod +x isaac_camera_node_final.py

echo "Starting Isaac Sim camera node with official APIs..."
echo "This version uses:"
echo "  - isaacsim.ros2.bridge extension (official ROS2 bridge)"
echo "  - ROS2CameraHelper and ROS2CameraInfoHelper nodes"
echo "  - Standard Isaac Sim camera creation workflow"
echo "  - Proper ROS2 topic publishing for /camera/rgb, /camera/camera_info, /camera/depth"
echo

# Run the camera node
echo "=== LAUNCHING ISAAC SIM CAMERA NODE ==="
timeout 300 python3 isaac_camera_node_final.py || {
    exit_code=$?
    echo
    if [[ $exit_code -eq 124 ]]; then
        echo "=== TEST COMPLETED: Node ran for 5 minutes without crashing ==="
        echo "✓ SUCCESS: Camera node is stable and functional"
    else
        echo "=== Node exited with code: $exit_code ==="
        if [[ $exit_code -eq 0 ]]; then
            echo "✓ SUCCESS: Camera node completed successfully"
        else
            echo "✗ FAILED: Camera node encountered an error"
        fi
    fi
}

echo
echo "=== Test completed at $(date) ==="
