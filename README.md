# Isaac Sim ROS2 Camera Control System with LLM-Driven Scenarios

A complete Isaac Sim + ROS2 integration for real-time camera control and intelligent scenario management. Convert natural language requests into fully configured Isaac Sim environments with robots and cameras.

## 🚀 Quick Start - Three Simple Steps

```bash
# 1. Set environment (replace with your Isaac Sim path)
export ISAAC_SIM_PATH=/home/user/.local/share/ov/pkg/isaac_sim-5.0.0
source /opt/ros/jazzy/setup.bash
cd /home/kimate/isaac_ws

# 2. Launch with scenario
./launch_camera_control_with_scenarios.sh "warehouse scene with carter 2.0 robot that is fully ros controlled"

# 3. Control camera (in another terminal)
python3 camera_cli.py overhead
python3 camera_cli.py move --x 2.0
python3 camera_cli.py position --x 0 --y 0 --z 10
```

## 🎯 Key Features

- **🗣️ Natural Language Scenarios**: "warehouse with carter robot" → Full Isaac Sim environment
- **🎮 Intuitive Camera Control**: Move, rotate, and position camera with simple commands  
- **🤖 Multi-Robot Support**: Carter, Franka, UR10 robots with ROS2 control
- **📊 Real-time Monitoring**: System status, scenario validation, and health checking
- **🔄 Live Data Streaming**: RGB, depth, camera info via ROS2 topics
- **✅ Proven Working**: User-validated camera movement and scenario management

## 🚀 Quick Start

### Prerequisites
- Isaac Sim 5.0+ installed
- ROS2 Jazzy
- Python 3.11+ (for Isaac Sim compatibility)

### Launch the System

1. **Launch Camera Control System** (easiest method):
```bash
./launch_camera_control.sh
```

2. **Or manually start the camera node**:
```bash
$ISAAC_SIM_PATH/python.sh camera_control_node.py
```

3. **Send movement commands** (in separate terminal):
```bash
python3 camera_control_sender.py --help
```

## 🎮 Camera Control Commands

### Velocity Control (Continuous Movement)
```bash
# Move forward
ros2 topic pub --once /camera/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}}'

# Move backward
ros2 topic pub --once /camera/cmd_vel geometry_msgs/msg/Twist '{linear: {x: -1.0}}'

# Move left/right
ros2 topic pub --once /camera/cmd_vel geometry_msgs/msg/Twist '{linear: {y: 1.0}}'

# Move up/down
ros2 topic pub --once /camera/cmd_vel geometry_msgs/msg/Twist '{linear: {z: 1.0}}'

# Rotate (yaw)
ros2 topic pub --once /camera/cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.5}}'

# Rotate (pitch/roll)
ros2 topic pub --once /camera/cmd_vel geometry_msgs/msg/Twist '{angular: {x: 0.5, y: 0.5}}'
```

### Absolute Position Control
```bash
# Jump to specific position
ros2 topic pub --once /camera/set_pose geometry_msgs/msg/PoseStamped '{pose: {position: {x: 5.0, y: 2.0, z: 3.0}}}'

# Move to overhead view
ros2 topic pub --once /camera/set_pose geometry_msgs/msg/PoseStamped '{pose: {position: {x: 0.0, y: 0.0, z: 10.0}}}'

# Reset to origin
ros2 topic pub --once /camera/set_pose geometry_msgs/msg/PoseStamped '{pose: {position: {x: 2.0, y: 2.0, z: 2.0}}}'
```

## 📊 ROS2 Topics

### Published Topics (Camera Data)
- `/camera/rgb` - RGB camera images
- `/camera/depth` - Depth camera data  
- `/camera/camera_info` - Camera parameters and calibration
- `/camera/current_pose` - Real-time camera position
- `/camera/state` - Camera status information

### Subscribed Topics (Control Commands)
- `/camera/cmd_vel` - Velocity commands (Twist messages)
- `/camera/set_pose` - Absolute position commands (PoseStamped messages)

## 📺 Monitoring and Debugging

### View Available Topics
```bash
ros2 topic list | grep camera
```

### Monitor Camera Position
```bash
ros2 topic echo /camera/current_pose
```

### Watch Camera Status
```bash
ros2 topic echo /camera/state
```

### View Camera Images (if you have image tools)
```bash
ros2 run image_view image_view image:=/camera/rgb
```

## 🔧 System Architecture

```
┌─────────────────┐    File I/O     ┌─────────────────────┐
│   ROS2 Bridge   │ ───────────────▶ │   Isaac Sim Node    │
│camera_command_   │    Commands      │isaac_camera_working_│
│sender.py        │                  │movable.py           │
└─────────────────┘                  └─────────────────────┘
         │                                       │
         │ ROS2 Topics                          │ Visual Movement
         ▼                                       ▼
┌─────────────────┐                  ┌─────────────────────┐
│   ROS2 System   │                  │    Isaac Sim GUI    │
│  /camera/*      │                  │   Camera Movement   │
└─────────────────┘                  └─────────────────────┘
```

## 🎯 Example Usage Scenarios

### 1. Manual Camera Exploration
```bash
# Fly around the scene
ros2 topic pub --once /camera/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 2.0}}'
ros2 topic pub --once /camera/cmd_vel geometry_msgs/msg/Twist '{angular: {z: 1.0}}'
```

### 2. Inspection Points
```bash
# Move to predefined inspection points
ros2 topic pub --once /camera/set_pose geometry_msgs/msg/PoseStamped '{pose: {position: {x: 5.0, y: 0.0, z: 3.0}}}'
ros2 topic pub --once /camera/set_pose geometry_msgs/msg/PoseStamped '{pose: {position: {x: 0.0, y: 5.0, z: 3.0}}}'
```

### 3. Smooth Trajectories
```bash
# Create smooth camera movements with repeated velocity commands
for i in {1..10}; do
  ros2 topic pub --once /camera/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}'
  sleep 0.1
done
```

## 🛠️ Troubleshooting

### Isaac Sim Not Starting
- Ensure `ISAAC_SIM_PATH` environment variable is set
- Check Isaac Sim installation and Python compatibility

### Camera Not Moving
- Verify both Isaac Sim node and ROS2 bridge are running
- Check that command file `/tmp/isaac_camera_commands.json` is being created
- Monitor ROS2 topics: `ros2 topic echo /camera/cmd_vel`

### ROS2 Topics Not Available
- Source ROS2 environment: `source /opt/ros/jazzy/setup.bash`
- Build workspace: `colcon build --packages-select isaac_test`
- Source workspace: `source install/setup.bash`

## 📁 File Structure

**Core System:**
- `camera_control_node.py` - Main Isaac Sim camera node with dual control modes
- `camera_control_sender.py` - Command-line tool for sending movement commands

**Launch Scripts:**
- `launch_camera_control.sh` - Main system launcher
- `launch_ros2_camera_test.sh` - Launch with automated testing

**Testing:**
- `test_camera_control.py` - File-based movement testing
- `test_ros2_camera_control.py` - ROS2 topic-based testing  
- `comprehensive_camera_test.py` - Complete test suite

**Documentation:**
- `CAMERA_CONTROL_README.md` - Detailed technical documentation
- `README.md` - This overview guide

## 🎉 Success Indicators

When everything is working correctly, you should see:
1. ✅ Isaac Sim window opens with camera view
2. ✅ Camera moves visibly when you send ROS2 commands
3. ✅ ROS2 topics publish camera data continuously
4. ✅ Position changes reflected in `/camera/current_pose`

## 📞 Support

If you encounter issues:
1. Check Isaac Sim logs for errors
2. Verify ROS2 environment is properly sourced
3. Ensure Python version compatibility (3.11 for Isaac Sim)
4. Test with simple movement commands first

---

**Status**: ✅ Fully functional camera control system (cleaned and organized)
**Last Updated**: July 1, 2025