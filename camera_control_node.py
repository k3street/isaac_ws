#!/usr/bin/env python3
"""
Isaac Sim 5.0 ROS2 Camera Control Node
Movable camera with ROS2 integration and command-based control
Based on official Isaac Sim examples and documentation
"""
import sys
import signal
import time
import traceback
import math
import random
import json
from pathlib import Path

from isaacsim.simulation_app import SimulationApp
CONFIG = {"renderer": "RaytracedLighting", "headless": False}  # Enable GUI for visualization
simulation_app = SimulationApp(CONFIG)

# Standard imports after simulation app
import carb
import omni
import omni.graph.core as og
import usdrt.Sdf
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils import extensions, stage
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, UsdGeom, UsdLux

class CameraControlNode:
    def __init__(self):
        self.simulation_context = None
        self.camera_prim = None
        self.ros_camera_graph = None
        self.ros_control_graph = None  # New: For ROS2 control subscribers
        self.shutdown_requested = False
        self.last_rotation_time = 0
        self.current_yaw = 0.0  # Current camera yaw rotation
        
        # Movement control attributes
        self.command_file = Path("/tmp/isaac_camera_commands.json")
        self.camera_position = [2.0, 2.0, 2.0]  # x, y, z
        self.camera_rotation = [0.0, 0.0, 0.0]  # roll, pitch, yaw in degrees
        
        # ROS2 movement state
        self.cmd_vel_data = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 
                            'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}
        self.position_cmd_data = {'x': None, 'y': None, 'z': None}
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
    def _signal_handler(self, signum, frame):
        print(f"Received signal {signum}, initiating shutdown...")
        self.shutdown_requested = True
        
    def initialize_isaac_sim(self):
        """Initialize Isaac Sim following official examples"""
        try:
            print("Initializing Isaac Sim simulation context...")
            
            # Enable ROS2 bridge extension (correct extension name)
            print("Enabling omni.isaac.ros2_bridge extension...")
            extensions.enable_extension("omni.isaac.ros2_bridge")
            
            # Update to load extensions
            simulation_app.update()
            
            # Create simulation context
            self.simulation_context = SimulationContext(stage_units_in_meters=1.0)
            
            # Get assets path
            assets_root_path = get_assets_root_path()
            if assets_root_path is None:
                raise RuntimeError("Could not find Isaac Sim assets folder")
                
            print(f"Using assets root path: {assets_root_path}")
            
            # Load simple room environment
            environment_path = assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd"
            stage.add_reference_to_stage(usd_path=environment_path, prim_path="/World/simple_room")
            
            # Add some interesting objects to view
            # Add a cube at origin
            from omni.isaac.core.utils.stage import add_reference_to_stage
            try:
                cube_path = assets_root_path + "/Isaac/Props/Blocks/basic_block.usd"
                add_reference_to_stage(usd_path=cube_path, prim_path="/World/Cube1")
                
                # Position cubes around the scene
                cube_prim = omni.usd.get_context().get_stage().GetPrimAtPath("/World/Cube1")
                if cube_prim:
                    xform_api = UsdGeom.XformCommonAPI(cube_prim)
                    xform_api.SetTranslate(Gf.Vec3d(0, 0, 0.5))
                    xform_api.SetScale(Gf.Vec3f(0.5, 0.5, 0.5))
                    
                # Add another cube
                add_reference_to_stage(usd_path=cube_path, prim_path="/World/Cube2")
                cube2_prim = omni.usd.get_context().get_stage().GetPrimAtPath("/World/Cube2")
                if cube2_prim:
                    xform_api = UsdGeom.XformCommonAPI(cube2_prim)
                    xform_api.SetTranslate(Gf.Vec3d(1.5, 0, 0.3))
                    xform_api.SetScale(Gf.Vec3f(0.3, 0.3, 0.6))
                    
            except Exception as e:
                print(f"Note: Could not add props (continuing anyway): {e}")
            
            # Add lighting
            sphereLight = UsdLux.SphereLight.Define(omni.usd.get_context().get_stage(), "/World/SphereLight")
            sphereLight.CreateRadiusAttr(150)
            sphereLight.CreateIntensityAttr(30000)
            sphereLight.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
            
            simulation_app.update()
            print("✓ Isaac Sim initialized successfully")
            
            return True
            
        except Exception as e:
            print(f"ERROR: Failed to initialize Isaac Sim: {e}")
            print(traceback.format_exc())
            return False
            
    def create_camera_and_ros_graph(self):
        """Create camera and ROS2 publishing graph following official examples"""
        try:
            print("Creating camera and ROS2 publishing graph...")
            
            # Camera parameters
            CAMERA_STAGE_PATH = "/World/Camera"
            ROS_CAMERA_GRAPH_PATH = "/ROS_Camera"
            
            # Create camera prim (following camera_periodic.py example)
            stage = omni.usd.get_context().get_stage()
            camera_prim_usd = stage.DefinePrim(CAMERA_STAGE_PATH, "Camera")
            camera_prim = UsdGeom.Camera(camera_prim_usd)
            xform_api = UsdGeom.XformCommonAPI(camera_prim_usd)
            xform_api.SetTranslate(Gf.Vec3d(2, 2, 2))
            xform_api.SetRotate((0, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
            camera_prim.GetHorizontalApertureAttr().Set(21)
            camera_prim.GetVerticalApertureAttr().Set(16)
            camera_prim.GetProjectionAttr().Set("perspective")
            camera_prim.GetFocalLengthAttr().Set(24)
            camera_prim.GetFocusDistanceAttr().Set(400)
            
            # Store the USD prim for transform operations
            self.camera_prim = camera_prim_usd
            
            simulation_app.update()
            
            # Create ROS camera graph (following camera_periodic.py example)
            keys = og.Controller.Keys
            (self.ros_camera_graph, _, _, _) = og.Controller.edit(
                {
                    "graph_path": ROS_CAMERA_GRAPH_PATH,
                    "evaluator_name": "push",
                    "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
                },
                {
                    keys.CREATE_NODES: [
                        ("OnTick", "omni.graph.action.OnTick"),
                        ("createViewport", "isaacsim.core.nodes.IsaacCreateViewport"),
                        ("getRenderProduct", "isaacsim.core.nodes.IsaacGetViewportRenderProduct"),
                        ("setCamera", "isaacsim.core.nodes.IsaacSetCameraOnRenderProduct"),
                        ("cameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                        ("cameraHelperInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                        ("cameraHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ],
                    keys.CONNECT: [
                        ("OnTick.outputs:tick", "createViewport.inputs:execIn"),
                        ("createViewport.outputs:execOut", "getRenderProduct.inputs:execIn"),
                        ("createViewport.outputs:viewport", "getRenderProduct.inputs:viewport"),
                        ("getRenderProduct.outputs:execOut", "setCamera.inputs:execIn"),
                        ("getRenderProduct.outputs:renderProductPath", "setCamera.inputs:renderProductPath"),
                        ("setCamera.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
                        ("setCamera.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
                        ("setCamera.outputs:execOut", "cameraHelperDepth.inputs:execIn"),
                        ("getRenderProduct.outputs:renderProductPath", "cameraHelperRgb.inputs:renderProductPath"),
                        ("getRenderProduct.outputs:renderProductPath", "cameraHelperInfo.inputs:renderProductPath"),
                        ("getRenderProduct.outputs:renderProductPath", "cameraHelperDepth.inputs:renderProductPath"),
                    ],
                    keys.SET_VALUES: [
                        ("createViewport.inputs:viewportId", 0),
                        ("cameraHelperRgb.inputs:frameId", "camera_link"),
                        ("cameraHelperRgb.inputs:topicName", "camera/rgb"),
                        ("cameraHelperRgb.inputs:type", "rgb"),
                        ("cameraHelperInfo.inputs:frameId", "camera_link"),
                        ("cameraHelperInfo.inputs:topicName", "camera/camera_info"),
                        ("cameraHelperDepth.inputs:frameId", "camera_link"),
                        ("cameraHelperDepth.inputs:topicName", "camera/depth"),
                        ("cameraHelperDepth.inputs:type", "depth"),
                        ("setCamera.inputs:cameraPrim", [usdrt.Sdf.Path(CAMERA_STAGE_PATH)]),
                    ],
                },
            )
            
            print("✓ Camera and ROS2 graph created successfully")
            
            # Run the ROS Camera graph once to generate ROS image publishers
            og.Controller.evaluate_sync(self.ros_camera_graph)
            simulation_app.update()
            
            print("✓ ROS2 publishers initialized")
            return True
            
        except Exception as e:
            print(f"ERROR: Failed to create camera and ROS graph: {e}")
            print(traceback.format_exc())
            return False
    
    def create_ros_control_graph(self):
        """Create ROS2 control subscribers for camera movement"""
        try:
            print("Creating ROS2 control subscribers...")
            
            # Ensure ROS2 bridge extension is loaded
            extensions.enable_extension("omni.isaac.ros2_bridge")
            
            # Create ROS2 control graph for movement commands
            keys = og.Controller.Keys
            ros_graph_path = "/ROS_Control"
            
            (self.ros_control_graph, _, _, _) = og.Controller.edit(
                {
                    "graph_path": ros_graph_path,
                    "evaluator_name": "push",
                    "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
                },
                {
                    keys.CREATE_NODES: [
                        ("OnTick", "omni.graph.action.OnTick"),
                        ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
                        ("cmd_vel_subscriber", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
                        ("pose_subscriber", "omni.isaac.ros2_bridge.ROS2SubscribePoseStamped"),
                    ],
                    keys.SET_VALUES: [
                        ("ros2_context.inputs:domain_id", 0),
                        ("cmd_vel_subscriber.inputs:topicName", "/camera/cmd_vel"),
                        ("pose_subscriber.inputs:topicName", "/camera/cmd_pose"),
                    ],
                },
            )
            
            print("✓ ROS2 control subscribers created successfully")
            return True
            
        except Exception as e:
            print(f"WARNING: Failed to create ROS2 control graph: {e}")
            print("Continuing with file-based control only...")
            self.ros_control_graph = None
            return False
            
    def run_simulation(self):
        """Run the main simulation loop"""
        try:
            print("Starting simulation...")
            
            if not self.simulation_context:
                print("ERROR: Simulation context not initialized")
                return False
                
            # Start simulation
            self.simulation_context.play()
            print("✓ Simulation started successfully")
            
            step_count = 0
            start_time = time.time()
            
            print("\n=== Starting main simulation loop ===")
            print("📸 Publishing ROS2 topics:")
            print("  - /camera/rgb (sensor_msgs/Image)")
            print("  - /camera/camera_info (sensor_msgs/CameraInfo)")
            print("  - /camera/depth (sensor_msgs/Image)")
            print("  - /clock (rosgraph_msgs/Clock)")
            print("")
            print("🎮 Movement control available via:")
            print("  - File-based: python3 camera_control_sender.py")
            print(f"  - Command file: {self.command_file}")
            if self.ros_control_graph:
                print("  - ROS2 topics: /camera/cmd_vel, /camera/cmd_pose")
            print("")
            print("💡 Test movement with ROS2:")
            print("  # Move forward:")
            print("  ros2 topic pub --once /camera/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}}'")
            print("  # Move to position:")
            print("  ros2 topic pub --once /camera/cmd_pose geometry_msgs/msg/PoseStamped '{pose: {position: {x: 3.0, y: 3.0, z: 3.0}}}'")
            print("Press Ctrl+C to stop\n")
            
            while not self.shutdown_requested:
                try:
                    # Process movement commands from file
                    self.process_movement_commands()
                    
                    # Process ROS2 movement commands
                    self.process_ros_movement_commands()
                    
                    # Step simulation with rendering
                    self.simulation_context.step(render=True)
                    simulation_app.update()
                    
                    step_count += 1
                    
                    # Print status every 100 steps
                    if step_count % 100 == 0:
                        elapsed = time.time() - start_time
                        fps = step_count / elapsed if elapsed > 0 else 0
                        print(f"Step {step_count:5d} | Elapsed: {elapsed:6.1f}s | FPS: {fps:5.1f} | Pos: {self.camera_position}")
                        
                    # Limit FPS 
                    time.sleep(0.02)  # ~50 FPS
                    
                    # Safety check for very long runs
                    if step_count > 200000:  # Stop after 200k steps
                        print(f"Reached maximum step count ({step_count}), stopping...")
                        break
                        
                except KeyboardInterrupt:
                    print("\nKeyboard interrupt received, stopping...")
                    break
                except Exception as e:
                    print(f"ERROR during simulation step {step_count}: {e}")
                    # Log but continue unless critical
                    if "Segmentation fault" in str(e) or "core dumped" in str(e):
                        print("Critical error detected, stopping simulation")
                        break
                        
            print(f"\nSimulation completed successfully. Total steps: {step_count}")
            return True
            
        except Exception as e:
            print(f"ERROR: Simulation failed: {e}")
            print(traceback.format_exc())
            return False
        finally:
            self.cleanup()
            
    def cleanup(self):
        """Clean up resources"""
        try:
            print("\nCleaning up...")
            
            if self.simulation_context:
                self.simulation_context.stop()
                print("✓ Simulation stopped")
                
            simulation_app.update()
            
        except Exception as e:
            print(f"Warning: Error during cleanup: {e}")
            
    def rotate_camera_randomly(self):
        """DISABLED: Camera rotation controlled by ROS2 commands only"""
        # Automatic rotation disabled - camera controlled by ROS2 topics
        pass
    
    def update_camera_transform(self):
        """Update camera position and rotation in Isaac Sim"""
        try:
            if self.camera_prim:
                print(f"🎥 Updating camera to position: {self.camera_position}, rotation: {self.camera_rotation}")
                
                # Apply transform to camera using UsdGeom.XformCommonAPI
                xform_api = UsdGeom.XformCommonAPI(self.camera_prim)
                
                # Set position
                xform_api.SetTranslate(Gf.Vec3d(self.camera_position[0], self.camera_position[1], self.camera_position[2]))
                
                # Set rotation (use the same format as camera creation)
                rotation_tuple = (
                    self.camera_rotation[0],  # roll
                    self.camera_rotation[1],  # pitch  
                    self.camera_rotation[2]   # yaw
                )
                xform_api.SetRotate(rotation_tuple, UsdGeom.XformCommonAPI.RotationOrderXYZ)
                
                print(f"✅ Camera transform applied successfully")
            else:
                print(f"❌ Camera prim not found!")
                
        except Exception as e:
            print(f"❌ Error updating camera transform: {e}")
    
    def process_movement_commands(self):
        """Process movement commands from file with robust error handling"""
        try:
            if not self.command_file.exists():
                return
                
            # Read and immediately delete to prevent corruption
            try:
                with open(self.command_file, 'r') as f:
                    content = f.read().strip()
                
                # Delete the file immediately after reading
                self.command_file.unlink()
                
                if not content:
                    return
                    
                # Parse single JSON command
                try:
                    command = json.loads(content)
                    self.process_single_command(command)
                except json.JSONDecodeError as je:
                    # If JSON parsing fails, skip silently to avoid spam
                    pass
                        
            except FileNotFoundError:
                # File was already processed by another thread
                pass
            except Exception as e:
                # Try to clean up corrupted file
                if self.command_file.exists():
                    try:
                        self.command_file.unlink()
                    except:
                        pass
        except Exception as e:
            # Silently handle overall file processing errors
            pass
    
    def process_ros_movement_commands(self):
        """Process ROS2 movement commands if available"""
        try:
            if not self.ros_control_graph:
                return  # ROS2 control not available
                
            # Evaluate the ROS2 control graph to get latest messages
            og.Controller.evaluate_sync(self.ros_control_graph)
            
            # Process cmd_vel (continuous velocity commands)
            try:
                # Get velocity data from ROS2 subscriber
                linear_x_attr = og.Controller.attribute("cmd_vel_subscriber.outputs:linearVelocity")
                angular_z_attr = og.Controller.attribute("cmd_vel_subscriber.outputs:angularVelocity")
                
                if linear_x_attr.is_valid() and angular_z_attr.is_valid():
                    linear_vel = og.Controller.get(linear_x_attr)
                    angular_vel = og.Controller.get(angular_z_attr)
                    
                    # Apply velocity commands (integrate over time)
                    dt = 0.02  # 50Hz update rate
                    if linear_vel and len(linear_vel) >= 3:
                        self.camera_position[0] += linear_vel[0] * dt
                        self.camera_position[1] += linear_vel[1] * dt
                        self.camera_position[2] += linear_vel[2] * dt
                        
                    if angular_vel and len(angular_vel) >= 3:
                        self.camera_rotation[2] += angular_vel[2] * dt * 57.2958  # rad/s to deg/s
                        
                        # Apply transform
                        self.update_camera_transform()
                        
            except Exception as e:
                # Silently handle ROS2 command errors to avoid spam
                pass
                
            # Process pose commands (absolute positioning)
            try:
                # Get position data from ROS2 subscriber
                position_attr = og.Controller.attribute("pose_subscriber.outputs:position")
                orientation_attr = og.Controller.attribute("pose_subscriber.outputs:orientation")
                
                if position_attr.is_valid():
                    position = og.Controller.get(position_attr)
                    
                    # Apply absolute position if valid
                    if position and len(position) >= 3:
                        self.camera_position[0] = position[0]
                        self.camera_position[1] = position[1]
                        self.camera_position[2] = position[2]
                        
                        # Apply transform
                        self.update_camera_transform()
                        
            except Exception as e:
                # Silently handle ROS2 command errors to avoid spam
                pass
                
        except Exception as e:
            # Silently handle overall ROS2 processing errors
            pass
    
    def process_single_command(self, command):
        """Process a single movement command"""
        if not isinstance(command, dict):
            return
            
        if command.get('type') == 'velocity':
            # Handle velocity-based movement
            vel = command.get('data', {})
            dt = 0.02  # 50Hz update rate
            
            # Update position (m/s to m)
            self.camera_position[0] += vel.get('linear_x', 0) * dt
            self.camera_position[1] += vel.get('linear_y', 0) * dt
            self.camera_position[2] += vel.get('linear_z', 0) * dt
            
            # Update rotation (rad/s to degrees)
            self.camera_rotation[0] += vel.get('angular_x', 0) * dt * 57.2958
            self.camera_rotation[1] += vel.get('angular_y', 0) * dt * 57.2958
            self.camera_rotation[2] += vel.get('angular_z', 0) * dt * 57.2958
            
            # Apply the transform to Isaac Sim
            self.update_camera_transform()
            
        elif command.get('type') == 'position':
            # Set absolute position
            pos = command.get('data', {})
            self.camera_position = [
                pos.get('x', self.camera_position[0]),
                pos.get('y', self.camera_position[1]),
                pos.get('z', self.camera_position[2])
            ]
            
            # Apply the transform to Isaac Sim
            self.update_camera_transform()

    def run(self):
        """Main execution method"""
        print("=== Isaac Sim 5.0 ROS2 Camera Control Node ===")
        print("Movable camera with command-based control")
        print("Using official APIs: isaacsim.ros2.bridge, ROS2CameraHelper\n")
        
        try:
            # Step 1: Initialize Isaac Sim
            if not self.initialize_isaac_sim():
                print("FAILED: Could not initialize Isaac Sim")
                return False
                
            # Step 2: Create camera and ROS2 publishing graph
            if not self.create_camera_and_ros_graph():
                print("FAILED: Could not create camera and ROS2 graph")
                return False
                
            # Step 3: Create ROS2 control graph for movement commands
            self.create_ros_control_graph()  # Continue even if this fails
                
            # Step 4: Run simulation
            return self.run_simulation()
            
        except Exception as e:
            print(f"FATAL ERROR: {e}")
            print(traceback.format_exc())
            return False

def main():
    """Main entry point"""
    node = CameraControlNode()
    success = node.run()
    
    print(f"\n=== Final Status: {'SUCCESS' if success else 'FAILED'} ===")
    print("Camera node execution completed.")
    
    # Close simulation app
    try:
        simulation_app.close()
    except:
        pass
        
    return 0 if success else 1

if __name__ == "__main__":
    sys.exit(main())
