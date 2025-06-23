# Camera Movement Visualization Guide

This guide shows different ways to visualize the rotating camera in Isaac Sim.

## Method 1: Isaac Sim GUI Mode (Recommended)

The camera node has been updated to run with GUI enabled. Simply run:

```bash
# Run with GUI to see camera movement directly
$ISAAC_SIM_PATH/python.sh isaac_camera_node_final.py
```

In the Isaac Sim GUI window:
1. **Viewport Control**: Use mouse to navigate the 3D scene
2. **Camera View**: You'll see the camera moving in a circle around the scene
3. **Live Feed**: The camera captures what it sees as it rotates

## Method 2: ROS2 Camera Feed Viewer

View the actual camera output using our custom viewer:

```bash
# Terminal 1: Start Isaac Sim camera node
$ISAAC_SIM_PATH/python.sh isaac_camera_node_final.py

# Terminal 2: View camera feed (requires cv_bridge and opencv)
pip3 install opencv-python
sudo apt install ros-jazzy-cv-bridge
python3 view_camera_feed.py
```

This shows:
- **RGB feed**: What the camera sees as it rotates
- **Depth feed**: Depth information with color mapping
- **Real-time**: Updates as camera moves every second

## Method 3: RViz2 Visualization

Use RViz2 to view camera topics:

```bash
# Terminal 1: Start camera node
$ISAAC_SIM_PATH/python.sh isaac_camera_node_final.py

# Terminal 2: Launch RViz2
./launch_rviz_camera.sh
```

In RViz2:
1. Click **Add** → **Image**
2. Set **Image Topic** to `/camera/rgb`
3. See the rotating camera view update in real-time!

## Method 4: Command Line Monitoring

Monitor camera topics from command line:

```bash
# List active topics
ros2 topic list

# See camera info
ros2 topic echo /camera/camera_info --once

# Monitor image publishing rate
ros2 topic hz /camera/rgb

# View raw image data
ros2 topic echo /camera/rgb --once
```

## Method 5: Console Logging

The camera node prints detailed movement information:

```
🔄 Camera rotated: Yaw=45.3° | Position=(2.00, 2.00, 2.00) | Looking at origin
🔄 Camera rotated: Yaw=178.9° | Position=(-2.82, 2.00, 0.14) | Looking at origin
🔄 Camera rotated: Yaw=267.1° | Position=(0.35, 2.00, -2.81) | Looking at origin
```

Track the camera's:
- **Yaw angle**: 0-360° rotation
- **Position**: X, Y, Z coordinates in 3D space  
- **Direction**: Always looking at origin (0, 0, 0)

## Expected Behavior

The camera should:
- ✅ Rotate randomly 360° every second
- ✅ Stay at constant distance (radius ≈ 2.83m) from origin
- ✅ Maintain height at Y = 2.0m
- ✅ Always point toward center of scene
- ✅ Publish live RGB and depth images to ROS2

## Troubleshooting

**No GUI showing?**
- Check that `headless=False` in isaac_camera_node_final.py
- Ensure X11 forwarding if using SSH: `ssh -X`

**ROS2 topics not visible?**
- Verify ROS2 environment: `source /opt/ros/jazzy/setup.bash`
- Check Isaac Sim ROS2 bridge is working: `ros2 topic list`

**Camera viewer not working?**
- Install dependencies: `pip3 install opencv-python`
- Install ROS2 CV bridge: `sudo apt install ros-jazzy-cv-bridge`

## Files for Visualization

- `isaac_camera_node_final.py` - Main camera node (now with GUI)
- `view_camera_feed.py` - ROS2 camera feed viewer
- `launch_rviz_camera.sh` - RViz2 launcher
- `camera_rotation_demo.py` - Standalone rotation test (no ROS2)

Choose the visualization method that works best for your setup!
