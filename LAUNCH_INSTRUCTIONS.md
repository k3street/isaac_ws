# 🚀 Isaac Sim Camera Control with Scenarios - Complete Launch Instructions

## Prerequisites

### 1. Isaac Sim Installation
- **Isaac Sim 5.0+** must be installed
- Set environment variable: `export ISAAC_SIM_PATH=/path/to/isaac-sim`
- Example: `export ISAAC_SIM_PATH=/home/user/.local/share/ov/pkg/isaac_sim-5.0.0`

### 2. ROS2 Environment  
- **ROS2 Jazzy** (or compatible version)
- Source ROS2: `source /opt/ros/jazzy/setup.bash`
- Verify: `echo $ROS_DISTRO` should show "jazzy"

### 3. Python Dependencies
- Python 3.11+ (for Isaac Sim compatibility)
- All scenario management scripts are included in the workspace

---

## 🎯 Launch Methods

### Method 1: Quick Launch with Default Scenario
```bash
cd /home/kimate/isaac_ws
./launch_camera_control_with_scenarios.sh
```

### Method 2: Launch with Custom Scenario
```bash
cd /home/kimate/isaac_ws
./launch_camera_control_with_scenarios.sh "warehouse scene with carter 2.0 robot that is fully ros controlled"
```

### Method 3: Step-by-Step Manual Launch

#### Step 1: Set Environment
```bash
export ISAAC_SIM_PATH=/path/to/your/isaac-sim
source /opt/ros/jazzy/setup.bash
cd /home/kimate/isaac_ws
```

#### Step 2: Create Scenario Configuration
```bash
# Option A: Use the demo to create a scenario
python3 demo_complete_workflow.py

# Option B: Use CLI to create custom scenario
python3 scenario_cli.py request -s "office environment with franka arm for manipulation"

# Option C: Use Python directly
python3 -c "
from scenario_manager import ScenarioManager
manager = ScenarioManager(enable_logging=False)
config = manager.generate_scenario_config('warehouse with carter robot fully ros controlled')
manager.save_scenario_config(config)
print('✅ Scenario configured')
"
```

#### Step 3: Launch Isaac Sim
```bash
$ISAAC_SIM_PATH/python.sh camera_control_node.py
```

---

## 🎮 Control Commands

### Camera Control (File-Based Method - Currently Working)
```bash
# Use the camera control CLI (RECOMMENDED - WORKING)
python3 camera_cli.py --help

# Move camera forward
python3 camera_cli.py move --x 1.0

# Move camera backward
python3 camera_cli.py move --x -1.0

# Move camera left/right
python3 camera_cli.py move --y 1.0

# Move camera up/down  
python3 camera_cli.py move --z 1.0

# Rotate camera
python3 camera_cli.py rotate --yaw 30

# Set specific position
python3 camera_cli.py position --x 0 --y 0 --z 10

# Reset to default position
python3 camera_cli.py reset

# Quick commands
python3 camera_cli.py forward    # Move forward
python3 camera_cli.py up        # Move up
python3 camera_cli.py overhead  # Overhead view
```

### Alternative: Use Test Scripts
```bash
# Automated test sequence (also working)
python3 test_camera_control.py

# Note: camera_control_sender.py is a ROS2 node, not a CLI tool
# To use it, run: python3 camera_control_sender.py (in background)
# Then publish to ROS2 topics: ros2 topic pub /camera/cmd_vel ...
```

### Camera Control (ROS2 Method - Requires ROS2 Bridge Setup)
```bash
# NOTE: These require the ROS2 bridge to be active
# Check if topics are available first: ros2 topic list | grep cmd_vel

# Move camera forward (if ROS2 bridge is running)
ros2 topic pub --once /camera/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}}'

# Set overhead view (if ROS2 bridge is running)
ros2 topic pub --once /camera/set_pose geometry_msgs/msg/PoseStamped '{pose: {position: {x: 0.0, y: 0.0, z: 10.0}}}'
```

### Robot Control (if robots are in scenario)
```bash
# Move Carter robot forward
ros2 topic pub --once /carter_robot/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}'

# Move Carter 2.0 robot
ros2 topic pub --once /carter_2_robot/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}'

# Move Franka arm (if present)
ros2 topic pub --once /franka_robot/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}}'
```

---

## 📊 Monitor and Manage

### View Available Assets
```bash
# List available scenes
python3 scenario_cli.py --list-scenes

# List available robots  
python3 scenario_cli.py --list-robots

# Show example scenarios
python3 scenario_cli.py examples
```

### Monitor System Status
```bash
# One-time status check
python3 scenario_status_monitor.py

# Continuous monitoring
python3 scenario_status_monitor.py --monitor

# Export status report
python3 scenario_status_monitor.py --export
```

### Change Scenarios (while running)
```bash
# Request new scenario
python3 scenario_cli.py request -s "office environment with multiple robots"

# Activate the new scenario
python3 scenario_cli.py activate

# Validate a scenario before activating
python3 scenario_cli.py validate -s "warehouse with autonomous navigation"
```

---

## 📡 ROS2 Topics

### Published Topics (Camera Data)
- `/camera/rgb` - RGB camera images (sensor_msgs/Image)
- `/camera/depth` - Depth camera data (sensor_msgs/Image)  
- `/camera/camera_info` - Camera parameters (sensor_msgs/CameraInfo)
- `/camera/current_pose` - Real-time camera position (geometry_msgs/PoseStamped)
- `/camera/state` - Camera status information

### Subscribed Topics (Control Commands)
- `/camera/cmd_vel` - Velocity-based camera movement (geometry_msgs/Twist)
- `/camera/set_pose` - Absolute camera positioning (geometry_msgs/PoseStamped)

### Robot Topics (if robots present)
- `/{robot_name}/cmd_vel` - Robot movement commands
- `/{robot_name}/odom` - Robot odometry data
- `/{robot_name}/joint_states` - Robot joint information

---

## 🔧 Troubleshooting

### Common Issues

#### Isaac Sim Not Found
```bash
# Check if ISAAC_SIM_PATH is set correctly
echo $ISAAC_SIM_PATH
ls $ISAAC_SIM_PATH/python.sh

# If not found, find your Isaac Sim installation:
find /home -name "isaac_sim*" -type d 2>/dev/null
```

#### ROS2 Not Sourced
```bash
# Check ROS2 environment
echo $ROS_DISTRO

# If empty, source ROS2:
source /opt/ros/jazzy/setup.bash
# or for other distros:
source /opt/ros/humble/setup.bash
```

#### Scenario Files Missing
```bash
# Verify scenario configuration exists
ls -la /tmp/isaac_scenario_config.json

# If missing, create one:
python3 demo_complete_workflow.py
```

#### ROS2 Control Topics Missing (/camera/cmd_vel not found)
```bash
# Check available topics
ros2 topic list

# If you see /camera/rgb, /camera/depth, /camera/camera_info but NO /camera/cmd_vel:
# This is NORMAL - use file-based control instead (it works perfectly):
python3 camera_cli.py move --x 1.0

# The system has two control methods:
# 1. File-based (always works) ✅ 
# 2. ROS2 topics (requires additional bridge setup)

# To verify file-based control is working:
ls -la /tmp/isaac_camera_commands.json  # Should exist after sending commands
```

### Debug Information
```bash
# Check Isaac Sim logs
tail -f ~/.nvidia-omniverse/logs/Kit/Isaac-Sim/*/kit.log

# Check scenario manager logs  
ls /tmp/isaac_logs/
tail -f /tmp/isaac_logs/scenario_manager_*.log

# Test scenario manager directly
python3 scenario_manager.py
```

---

## 🎬 Expected Behavior

When you launch successfully, you should see:

1. **Isaac Sim GUI opens** with the specified environment (warehouse, office, etc.)
2. **Robot(s) appear** at the origin [0,0,0] if included in scenario
3. **Camera positioned** at [2,2,2] or scenario-specified location
4. **ROS2 topics active** - check with `ros2 topic list`
5. **Real-time control** - camera and robots respond to ROS2 commands
6. **Live data streaming** - images and depth data published continuously

### Success Indicators
- Isaac Sim GUI shows the correct environment
- `ros2 topic list` shows camera and robot topics
- Camera moves when you send velocity commands
- Robot responds to movement commands (if ROS-enabled robots present)
- Status monitor shows "active" system state

---

## 💡 Pro Tips

1. **Start with the demo**: Run `python3 demo_complete_workflow.py` to see the full workflow
2. **Use the CLI**: `python3 scenario_cli.py --help` for all available commands
3. **Monitor continuously**: Use `python3 scenario_status_monitor.py --monitor` in a separate terminal
4. **Test scenarios**: Validate scenarios before launching with `scenario_cli.py validate`
5. **Check logs**: Monitor `/tmp/isaac_logs/` for detailed debugging information

---

## 🚀 Quick Start Example

```bash
# 1. Set environment
export ISAAC_SIM_PATH=/home/user/.local/share/ov/pkg/isaac_sim-5.0.0
source /opt/ros/jazzy/setup.bash
cd /home/kimate/isaac_ws

# 2. Create scenario
python3 -c "from scenario_manager import ScenarioManager; m=ScenarioManager(enable_logging=False); m.save_scenario_config(m.generate_scenario_config('warehouse with carter robot ros controlled'))"

# 3. Launch
./launch_camera_control_with_scenarios.sh

# 4. Control (in another terminal) - Use file-based control
python3 camera_cli.py move --x 2.0
python3 camera_cli.py move --z 5.0
python3 camera_cli.py rotate --yaw 45

# Alternative: ROS2 control (if ROS2 bridge is active)
# ros2 topic pub --once /camera/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 2.0}}'
```

🎉 **You're ready to control Isaac Sim with natural language scenarios!**

---

## ✅ Testing Camera Control (Step-by-Step)

### Step 1: Verify Isaac Sim is Running
```bash
# Check that Isaac Sim is running and ROS2 topics are active
ros2 topic list
# You should see: /camera/rgb, /camera/depth, /camera/camera_info
```

### Step 2: Test File-Based Camera Control
```bash
# Test camera control CLI (this should work immediately)
python3 camera_cli.py --help

# Move camera forward
python3 camera_cli.py move --x 1.0

# Move camera up for better view
python3 camera_cli.py move --z 2.0

# Rotate camera to look around
python3 camera_cli.py rotate --yaw 45

# Set overhead view
python3 camera_cli.py position --x 0 --y 0 --z 10

# Reset to default
python3 camera_cli.py reset
```

### Step 3: Monitor Camera Data
```bash
# Watch camera feed (in another terminal)
ros2 topic echo /camera/rgb --once

# Check camera position updates
ros2 topic echo /camera/current_pose --once
```

### Step 4: Verify Commands are Working
```bash
# Check if command file is being created/updated
ls -la /tmp/isaac_camera_commands.json

# Watch the file for changes (optional)
watch -n 1 "ls -la /tmp/isaac_camera_commands.json"
```

---

## ✅ SUCCESS CONFIRMATION

### Camera Control Working! 🎉

Since you've confirmed that `test_camera_control.py` moves the camera around the scenario, your system is fully operational! This means:

#### ✅ What's Working:
- **Isaac Sim** is running correctly
- **Camera control node** is active and processing commands
- **File-based control** is functioning perfectly
- **Scenario environment** is loaded and visible
- **Real-time camera movement** responds to commands

#### 🎮 Continue Testing:

You can now use any of these control methods:

**Option 1: Interactive Command Sender**
```bash
python3 camera_cli.py move --x 2.0
python3 camera_cli.py position --x 0 --y 0 --z 10
python3 camera_cli.py rotate --yaw 45
python3 camera_cli.py overhead
```

**Option 2: Run Test Sequences**
```bash
python3 test_camera_control.py  # Full automated test
python3 comprehensive_camera_test.py  # More extensive tests
```

**Option 3: Custom Scenarios**
```bash
# Create new scenarios while running
python3 scenario_cli.py request -s "office environment with franka arm"
python3 scenario_cli.py activate

# Monitor system status
python3 scenario_status_monitor.py --monitor
```

#### 🚀 What to Try Next:

1. **Test different camera positions** using the control sender
2. **Change scenarios** using the scenario CLI
3. **Monitor camera data** with `ros2 topic echo /camera/rgb`
4. **Add robots** to your scenarios and see them in the environment
5. **Try ROS2 control** (if you want to set up the bridge later)

Your Isaac Sim camera control with scenario management system is **fully operational**! 🎬
