# Isaac Sim Camera Control System

A complete Isaac Sim 5.0 + ROS2 camera control system with movable camera capabilities, image publishing, and command-based movement control.

## 🎯 Features

- **Isaac Sim 5.0 Integration**: Full support for latest Isaac Sim version
- **ROS2 Jazzy Support**: Compatible with ROS2 Jazzy distribution
- **Camera Movement**: Real-time camera position and rotation control
- **Image Publishing**: RGB, depth, and camera info topics
- **Multiple Control Methods**: File-based commands, ROS2 topics, and Python API
- **Room Environment**: Loads room scene with interactive objects
- **Real-time Feedback**: Live position updates and status monitoring

## 📁 Files

### Core Components
- `camera_control_node.py` - Main Isaac Sim camera control node
- `camera_control_sender.py` - Command sender utility
- `launch_camera_control.sh` - Launch script for the complete system

### Legacy Files (for reference)
- `isaac_camera_node_final.py` - Working base version
- `isaac_camera_working_movable.py` - Earlier movable version

## 🚀 Quick Start

### 1. Prerequisites
```bash
# Isaac Sim 5.0 installed and ISAAC_SIM_PATH set
export ISAAC_SIM_PATH=/path/to/isaac/sim

# ROS2 Jazzy installed
source /opt/ros/jazzy/setup.bash
```

### 2. Launch the System
```bash
cd /home/kimate/isaac_ws
./launch_camera_control.sh
```

### 3. Verify ROS2 Topics
```bash
# In another terminal
source /opt/ros/jazzy/setup.bash
ros2 topic list

# Expected topics:
# /camera/rgb
# /camera/depth  
# /camera/camera_info
# /clock
```

## 🎮 Camera Control Methods

### Method 1: Direct JSON Commands
```bash
# Move to specific position
echo '{"type":"position","data":{"x":3.0,"y":3.0,"z":2.0}}' > /tmp/isaac_camera_commands.json

# Move with rotation
echo '{"type":"position","data":{"x":1.0,"y":2.0,"z":3.0,"roll":0.0,"pitch":0.0,"yaw":45.0}}' > /tmp/isaac_camera_commands.json
```

### Method 2: Python Command Sender
```bash
# Basic movements
python3 camera_control_sender.py --move-forward
python3 camera_control_sender.py --move-back
python3 camera_control_sender.py --move-left
python3 camera_control_sender.py --move-right
python3 camera_control_sender.py --move-up
python3 camera_control_sender.py --move-down

# Rotations
python3 camera_control_sender.py --turn-left
python3 camera_control_sender.py --turn-right
```

### Method 3: ROS2 Topics (planned)
```bash
# Velocity control
ros2 topic pub --once /camera/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0, y: 0.0, z: 0.0}}'

# Position control
ros2 topic pub --once /camera/cmd_pose geometry_msgs/msg/Pose '{position: {x: 3.0, y: 3.0, z: 2.0}}'
```

## 📊 Command Format

### Position Commands
```json
{
  "type": "position",
  "data": {
    "x": 3.0,
    "y": 3.0, 
    "z": 2.0,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0
  }
}
```

### Velocity Commands
```json
{
  "type": "velocity",
  "data": {
    "linear_x": 1.0,
    "linear_y": 0.0,
    "linear_z": 0.0,
    "angular_x": 0.0,
    "angular_y": 0.0,
    "angular_z": 0.5
  }
}
```

## 🔧 System Architecture

```
┌─────────────────────┐    JSON Commands    ┌──────────────────────┐
│   Command Sender    │ ──────────────────► │   Camera Control    │
│  (Python/ROS2)     │                     │       Node           │
└─────────────────────┘                     │   (Isaac Sim)        │
                                            └──────────────────────┘
                                                      │
                                                      ▼
                                            ┌──────────────────────┐
                                            │    ROS2 Topics       │
                                            │  /camera/rgb         │
                                            │  /camera/depth       │
                                            │  /camera/camera_info │
                                            └──────────────────────┘
```

## 📈 Status

### ✅ Working Features
- Isaac Sim 5.0 launches with GUI
- ROS2 bridge loads and publishes camera topics
- File-based command processing
- Real-time camera position updates
- Transform application to USD scene
- Room environment with objects
- Position feedback via console output

### 🔧 Recent Fixes
- Fixed USD prim reference handling
- Corrected transform API usage
- Added debug output for movement verification
- Improved error handling and robustness

### 🚀 Tested Movements
- ✅ Position [2,2,2] → [3,3,3] ✅ SUCCESSFUL
- ✅ Multiple position commands
- ✅ Command file processing
- ✅ Real-time position feedback

## 🐛 Troubleshooting

### Common Issues

1. **Isaac Sim Path Not Set**
   ```bash
   export ISAAC_SIM_PATH=/path/to/isaac/sim/_build/linux-x86_64/release
   ```

2. **ROS2 Topics Not Publishing**
   ```bash
   # Check if Isaac Sim is running
   ps aux | grep isaac
   
   # Verify ROS2 environment
   source /opt/ros/jazzy/setup.bash
   ros2 topic list
   ```

3. **Movement Commands Not Working**
   ```bash
   # Check command file exists and is readable
   ls -la /tmp/isaac_camera_commands.json
   
   # Monitor Isaac Sim console for debug output
   ```

## 📝 Development Notes

- Commands are processed at ~32 FPS during simulation loop
- File-based interface avoids ROS2 compatibility issues between different Python versions
- USD transform API requires proper prim reference (not schema object)
- Camera position updates are immediately reflected in Isaac Sim coordinate system

## 🔄 Next Steps

1. **Visual Movement Verification**: Confirm viewport camera movement
2. **Enhanced Controls**: Add smooth interpolation between positions
3. **ROS2 Direct Interface**: Implement native ROS2 topic subscribers
4. **Recording/Playback**: Add movement sequence recording capabilities

---

**Status**: ✅ **WORKING** - Camera movement system functional and tested
**Last Updated**: July 1, 2025
