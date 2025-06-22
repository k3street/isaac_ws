# Contributing to Isaac Sim ROS2 Camera Node

This repository contains the working solution for Isaac Sim 5.0 + ROS2 Jazzy integration.

## Quick Start for Contributors

1. **Prerequisites**
   - Isaac Sim 5.0 installed
   - ROS2 Jazzy environment
   - Set `ISAAC_SIM_PATH` environment variable

2. **Clone and Run**
   ```bash
   git clone <your-repo-url>
   cd isaac_ws
   ./run_camera_node_final.sh
   ```

3. **Verify Topics**
   ```bash
   ros2 topic list
   # Should show /camera/camera_info, /camera/rgb, /camera/depth
   ```

## Repository Structure

- `isaac_camera_node_final.py` - **Main working implementation**
- `run_camera_node_final.sh` - Launcher script
- `diagnose_camera_topics.sh` - ROS2 diagnostics
- `src/isaac_test/` - ROS2 package structure

## Technical Notes

This solution uses:
- `isaacsim.ros2.bridge` (official extension)
- `ROS2CameraHelper` and `ROS2CameraInfoHelper` nodes
- Standard Isaac Sim camera creation workflow

## License

[Add your license here]
