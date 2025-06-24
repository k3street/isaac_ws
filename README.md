# Isaac Sim ROS2 Camera Control System

A complete Isaac Sim + ROS2 integration for real-time camera control and data streaming.

## 🎯 Features

- **Real-time Camera Control**: Move and rotate camera in Isaac Sim via ROS2 commands
- **Live Data Streaming**: RGB images, depth data, and camera info published to ROS2
- **Multiple Control Modes**: Velocity-based movement and absolute positioning
- **Visual Feedback**: See camera movement in real-time within Isaac Sim interface

## 🚀 Quick Start

### Prerequisites
- Isaac Sim 5.0+ installed
- ROS2 Jazzy
- Python 3.11+ (for Isaac Sim compatibility)

### Launch the System

1. **Start Isaac Sim Camera Node** (with movement capability):
```bash
$ISAAC_SIM_PATH/python.sh isaac_camera_working_movable.py
```

2. **Start ROS2 Command Bridge** (in separate terminal):
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 camera_command_sender.py
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

- `isaac_camera_working_movable.py` - Main Isaac Sim camera node with movement
- `camera_command_sender.py` - ROS2 to Isaac Sim command bridge
- `src/isaac_test/` - ROS2 package for additional controllers
- `launch_complete_system.sh` - Complete system launcher script

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

**Status**: ✅ Fully functional ROS2 camera control system
**Last Updated**: June 24, 2025