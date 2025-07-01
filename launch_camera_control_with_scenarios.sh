#!/bin/bash
# Launch Isaac Sim Camera Control with Scenario Support
# Enhanced version of the original launch script

echo "🚀 Isaac Sim Camera Control with Scenario Support"
echo "=================================================="

# Check if Isaac Sim is installed
if [ -z "$ISAAC_SIM_PATH" ]; then
    echo "❌ ISAAC_SIM_PATH not set. Please set it to your Isaac Sim installation directory."
    echo "   Example: export ISAAC_SIM_PATH=/home/user/.local/share/ov/pkg/isaac_sim-5.0.0"
    exit 1
fi

if [ ! -f "$ISAAC_SIM_PATH/python.sh" ]; then
    echo "❌ Isaac Sim not found at $ISAAC_SIM_PATH"
    echo "   Please check your ISAAC_SIM_PATH setting."
    exit 1
fi

# Check ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2 environment not sourced. Please source your ROS2 setup."
    echo "   Example: source /opt/ros/jazzy/setup.bash"
    exit 1
fi

echo "✅ Isaac Sim path: $ISAAC_SIM_PATH"
echo "✅ ROS2 distro: $ROS_DISTRO"

# Make scripts executable
chmod +x scenario_cli.py
chmod +x scenario_service_node.py

echo ""
echo "🎯 Scenario Management Features:"
echo "   - Natural language scenario requests"
echo "   - LLM-driven scene setup"
echo "   - Multi-robot support"
echo "   - Dynamic camera positioning"
echo ""

# Check if scenario request is provided as argument
if [ ! -z "$1" ]; then
    echo "📝 Scenario request: '$1'"
    echo "💾 Saving scenario request..."
    
    # Create scenario request using Python
    python3 -c "
from scenario_manager import ScenarioManager
import sys
manager = ScenarioManager()
config = manager.generate_scenario_config('$1')
manager.save_scenario_config(config)
print('✅ Scenario configured')
"
fi

echo ""
echo "🎬 Starting Isaac Sim Camera Control Node..."
echo "   (This will take 30-60 seconds to initialize)"
echo ""
echo "💡 While Isaac Sim loads, you can:"
echo "   - Send scenario requests in another terminal:"
echo "     python3 scenario_cli.py request -s \"warehouse with carter robot\""
echo "   - View available examples:"
echo "     python3 scenario_cli.py examples"
echo "   - List available assets:"
echo "     python3 scenario_cli.py --list-scenes"
echo ""
echo "Press Ctrl+C to stop all processes"
echo ""

# Start the enhanced camera control node with scenario support
exec $ISAAC_SIM_PATH/python.sh camera_control_node.py
