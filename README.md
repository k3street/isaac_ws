# Isaac Sim 5.0 + ROS2 Jazzy Camera Node - ✅ COMPLETED

## 🎥 Demo Video
[![Isaac Sim Camera Node Demo](https://img.youtube.com/vi/bT7uk9ZliIE/0.jpg)](https://youtu.be/bT7uk9ZliIE)

*Click the image above to watch the complete Isaac Sim → ROS2 camera pipeline in action*

This project implements a **fully functional** camera node for Isaac Sim 5.0 with ROS2 Jazzy integration.

## ✅ SOLUTION COMPLETE

**SUCCESS**: The final implementation (`isaac_camera_node_final.py`) successfully publishes all required ROS2 topics without segmentation faults or crashes.

### ✅ Verified Working Topics
```bash
ros2 topic list
/camera/camera_info    # ✅ Camera calibration parameters
/camera/depth          # ✅ Depth image data
/camera/rgb            # ✅ RGB image data  
/parameter_events      # ✅ ROS2 system topic
/rosout               # ✅ ROS2 logging topic
```

## Final Implementation

### Working Solution
- **`isaac_camera_node_final.py`** - ✅ **WORKING** - Complete camera node using official Isaac Sim APIs
- **`run_camera_node_final.sh`** - ✅ **WORKING** - Launcher script with proper environment setup

### Key Technical Solutions Applied
1. **Correct Extension**: Uses `isaacsim.ros2.bridge` (not deprecated `omni.isaac.ros2_bridge`)
2. **Official APIs**: Uses `ROS2CameraHelper` and `ROS2CameraInfoHelper` nodes
3. **Proper Import**: Uses `isaacsim.simulation_app.SimulationApp` (not deprecated `omni.isaac.kit`)
4. **Standard Workflow**: Follows official Isaac Sim camera publishing examples
5. **Robust Error Handling**: Comprehensive error handling and graceful shutdown

### Architecture
```
Camera Creation → ROS2 Graph → Topic Publishing
    ↓               ↓              ↓
UsdGeom.Camera → ROS2CameraHelper → /camera/rgb
                → ROS2CameraInfoHelper → /camera/camera_info  
                → ROS2CameraHelper(depth) → /camera/depth
```

## Usage

### Quick Start
```bash
# Run the working camera node
$ISAAC_SIM_PATH/python.sh isaac_camera_node_final.py

# Verify topics in another terminal
ros2 topic list
ros2 topic echo /camera/camera_info
```

### Camera Controller Demo
The project includes both a **controllable Isaac Sim camera** and a **ROS2 camera controller node**:

#### 1. Controllable Isaac Sim Camera (`isaac_camera_controllable.py`)
Real-time camera control directly in Isaac Sim via ROS2 topics:

```bash
# Launch the controllable camera in Isaac Sim
./launch_camera_controller.sh

# Control camera movement (in another terminal)
# Move forward at 0.5 m/s
ros2 topic pub /camera/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}'

# Rotate camera (yaw) at 0.2 rad/s
ros2 topic pub /camera/cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.2}}'

# Set absolute position
ros2 topic pub /camera/set_pose geometry_msgs/msg/PoseStamped '{pose: {position: {x: 5.0, y: 0.0, z: 2.0}}}'

# Change field of view to 90 degrees
ros2 topic pub /camera/set_fov std_msgs/msg/Float32 '{data: 90.0}'

# Disable/enable camera
ros2 topic pub /camera/enable std_msgs/msg/Bool '{data: false}'
```

#### 2. ROS2 Camera Controller Node (`camera_controller.py`)
Standalone ROS2 node for camera state management and simulation:

```bash
# Build the ROS2 package
colcon build --packages-select isaac_test

# Source the workspace
source install/setup.bash

# Run the standalone controller
ros2 run isaac_test camera_controller

# Test the controller
./test_camera_controller.sh
```

### Camera Subscriber Demo
The project also includes a complete ROS2 subscriber node that processes camera data from Isaac Sim:

```bash
# Build the ROS2 package
colcon build --packages-select isaac_test

# Source the workspace
source install/setup.bash

# Run the camera subscriber (after starting Isaac Sim camera node)
ros2 run isaac_test camera_subscriber

# Or use the convenience script
./launch_camera_subscriber.sh
```

The subscriber node:
- ✅ **Subscribes** to `/camera/rgb`, `/camera/depth`, and `/camera/camera_info` topics
- ✅ **Processes** images with OpenCV for analysis
- ✅ **Publishes** processed information and statistics
- ✅ **Monitors** frame rates and image quality metrics
- ✅ **Handles** errors gracefully with comprehensive logging

### Environment Setup
```bash
# Set Isaac Sim path
export ISAAC_SIM_PATH="/path/to/isaac-sim"

# Source ROS2 environment  
source /opt/ros/jazzy/setup.bash

# Run with launcher script
./run_camera_node_final.sh
```

## Project Evolution

### ❌ Issues Resolved
- **Segmentation Faults**: Eliminated by avoiding deprecated APIs
- **CsRawData Errors**: Fixed by using proper extension (`isaacsim.ros2.bridge`)
- **Import Errors**: Resolved with correct module paths
- **Extension Loading**: Fixed with proper extension enablement

### 🔧 Technical Fixes Applied
1. Replaced `omni.isaac.sensor` → `isaacsim.ros2.bridge`
2. Replaced `omni.isaac.kit.SimulationApp` → `isaacsim.simulation_app.SimulationApp`
3. Used official `ROS2CameraHelper` nodes instead of custom implementations
4. Followed Isaac Sim 5.0 official examples and documentation

## Files Structure

```
isaac_ws/
├── isaac_camera_node_final.py     # ✅ WORKING - Original camera publisher
├── isaac_camera_controllable.py   # ✅ NEW - ROS2-controllable camera in Isaac Sim
├── run_camera_node_final.sh       # ✅ WORKING - Original camera launcher
├── launch_camera_controller.sh    # ✅ NEW - Controllable camera launcher
├── launch_camera_subscriber.sh    # ✅ WORKING - Subscriber launcher
├── test_camera_subscriber.sh      # ✅ WORKING - Subscriber test script
├── test_camera_controller.sh      # ✅ NEW - Controller test script
├── README.md                      # 📖 This documentation
├── STATUS.md                      # 📊 Project status log
└── src/isaac_test/               # 📦 ROS2 package structure
    ├── package.xml               # ✅ Package configuration
    ├── setup.py                  # ✅ Python package setup
    └── isaac_test/
        ├── __init__.py
        ├── camera_subscriber.py   # ✅ WORKING - Camera data subscriber
        └── camera_controller.py   # ✅ NEW - Standalone camera controller node
```

## Success Metrics ✅

- [x] **No Segmentation Faults**: Node runs indefinitely without crashes
- [x] **ROS2 Topic Publishing**: All camera topics publish successfully
- [x] **ROS2 Subscriber Node**: Complete subscriber implementation with image processing
- [x] **ROS2 Camera Control**: Real-time camera control via ROS2 topics
- [x] **Controllable Isaac Sim Camera**: Direct camera control in Isaac Sim simulation
- [x] **Stable Operation**: Runs for extended periods without issues  
- [x] **Official APIs**: Uses only supported, non-deprecated Isaac Sim APIs
- [x] **Error Handling**: Graceful shutdown and error recovery
- [x] **Complete Pipeline**: Full Isaac Sim → ROS2 → Processing → Control pipeline
- [x] **Documentation**: Complete usage instructions and troubleshooting

## Testing the Complete Pipeline

1. **Start Isaac Sim Camera Node**:
   ```bash
   ./run_camera_node_final.sh
   ```

2. **Verify Topics** (in new terminal):
   ```bash
   ./test_camera_subscriber.sh
   ```

3. **Run Subscriber Node** (in new terminal):
   ```bash
   ./launch_camera_subscriber.sh
   ```

4. **Monitor Processing** (in new terminal):
   ```bash
   ros2 topic echo /camera/analysis
   ```

## Final Status: ✅ **PROJECT COMPLETED SUCCESSFULLY**

The Isaac Sim 5.0 + ROS2 Jazzy camera node is now **fully functional** with a complete subscriber implementation, demonstrating the full pipeline from Isaac Sim camera data publishing to ROS2 processing and analysis.

### Complete Implementation Includes:
- ✅ **Camera Publisher**: Isaac Sim camera node publishing RGB, depth, and camera info
- ✅ **Camera Subscriber**: ROS2 node processing and analyzing camera data
- ✅ **Launch Scripts**: Automated startup and testing utilities
- ✅ **Error Handling**: Robust error handling and logging throughout
- ✅ **Documentation**: Complete setup and usage instructions

## Legacy Files (Kept for Reference)

The following files represent the iterative development process:
- `isaac_camera_node_robust.py` - Previous working version using Replicator API
- Various test logs and diagnostic scripts

These demonstrate the debugging process and can be removed if desired.
