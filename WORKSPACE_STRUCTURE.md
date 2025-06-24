# Isaac Sim + ROS2 Camera Control - Clean Workspace

## Essential Files

### Core Components
- **`isaac_camera_node_final.py`** - Original working camera node with image publishing
- **`isaac_camera_working_movable.py`** - Enhanced camera node with movement control + image publishing
- **`camera_command_sender.py`** - ROS2 command bridge for camera movement control
- **`launch_complete_system.sh`** - Main launcher script for the complete system

### Documentation
- **`README.md`** - Complete documentation of the ROS2 camera control workflow
- **`WORKSPACE_STRUCTURE.md`** - This file explaining the clean workspace structure

### ROS2 Package Structure
- **`src/isaac_test/`** - Minimal ROS2 package structure
  - `package.xml` - Package metadata
  - `setup.py` - Package setup configuration
  - `setup.cfg` - Setup configuration
  - `isaac_test/__init__.py` - Package initialization
  - `resource/isaac_test` - Package marker
  - `test/` - Standard ROS2 package tests

### Version Control
- **`.git/`** - Git repository (preserves development history)
- **`.gitignore`** - Git ignore rules

## Usage

1. **Setup ROS2 Jazzy environment:**
   ```bash
   source /opt/ros/jazzy/setup.bash
   ```

2. **Launch the complete system:**
   ```bash
   ./launch_complete_system.sh
   ```

3. **Control camera movement:**
   ```bash
   # Velocity control
   ros2 topic pub --once /camera/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}}'
   
   # Position control
   ros2 topic pub --once /camera/set_pose geometry_msgs/msg/PoseStamped '{pose: {position: {x: 5.0, z: 3.0}}}'
   ```

4. **Monitor camera topics:**
   ```bash
   ros2 topic list | grep camera
   ros2 topic echo /camera/current_pose
   ros2 topic echo /camera/rgb
   ```

## Removed Files

All temporary, duplicate, and outdated files have been removed:
- Build artifacts (`build/`, `install/`, `log/`, `__pycache__/`)
- Duplicate camera nodes and bridges
- Test scripts and simulators
- Old launch files and documentation
- Deprecated shell scripts

The workspace is now clean and contains only the essential working components.
