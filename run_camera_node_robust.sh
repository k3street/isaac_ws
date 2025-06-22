#!/bin/bash

# Robust Camera Node Launcher
# Runs the crash-safe camera node that avoids the SdCreateRenderProduct issue

echo "=== Isaac Sim Robust Camera Node Launcher ==="
echo "Starting robust camera node (avoiding known crashes)..."

# Set Isaac Sim environment (from proven working setup)
export CARB_APP_PATH="/home/kimate/isaacsim/_build/linux-x86_64/release/kit"
export EXP_PATH="/home/kimate/isaacsim/_build/linux-x86_64/release/apps"
export ISAAC_PATH="/home/kimate/isaacsim"

# Run the robust camera node using Isaac Sim's Python
echo "Launching robust camera node..."
/home/kimate/isaacsim/_build/linux-x86_64/release/python.sh /home/kimate/isaac_ws/isaac_camera_node_robust.py

echo "Robust camera node finished."
