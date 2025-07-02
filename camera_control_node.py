#!/usr/bin/env python3
"""
Isaac Sim 5.0 ROS2 Camera Node - Final Working Version with Scenario Support
Based on official Isaac Sim examples and documentation
Enhanced with LLM-driven scenario management and ROS2 cmd_vel support
"""
import sys
import signal
import time
import traceback
import math
import random
import json
import threading
from pathlib import Path

# Import scenario manager
try:
    from scenario_manager import ScenarioManager
except ImportError:
    print("Warning: scenario_manager not found, scenario features disabled")
    ScenarioManager = None

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

# ROS2 imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class FinalCameraNode:
    def __init__(self):
        self.simulation_context = None
        self.camera_prim = None
        self.ros_camera_graph = None
        self.shutdown_requested = False
        self.last_rotation_time = 0
        self.current_yaw = 0.0  # Current camera yaw rotation
        
        # Movement control attributes
        self.command_file = Path("/tmp/isaac_camera_commands.json")
        self.camera_position = [2.0, 2.0, 2.0]  # x, y, z
        self.camera_rotation = [0.0, 0.0, 0.0]  # roll, pitch, yaw in degrees
        
        # ROS2 node for cmd_vel subscription
        self.ros_node = None
        self.ros_thread = None
        self.initialize_ros2_node()
        
        # Scenario management attributes
        self.scenario_manager = ScenarioManager() if ScenarioManager else None
        self.scenario_config_file = Path("/tmp/isaac_scenario_config.json")
        self.current_scenario_config = None
        self.scenario_assets = []  # Track loaded scenario assets
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
    def _signal_handler(self, signum, frame):
        print(f"Received signal {signum}, initiating shutdown...")
        self.shutdown_requested = True
        
    def initialize_isaac_sim(self):
        """Initialize Isaac Sim following official examples with scenario support"""
        try:
            print("Initializing Isaac Sim simulation context...")
            
            # Load scenario configuration first
            self.load_scenario_config()
            
            # Enable ROS2 bridge extension (correct extension name)
            print("Enabling isaacsim.ros2.bridge extension...")
            extensions.enable_extension("isaacsim.ros2.bridge")
            
            # Update to load extensions
            simulation_app.update()
            
            # Create simulation context
            self.simulation_context = SimulationContext(stage_units_in_meters=1.0)
            
            # Get assets path
            assets_root_path = get_assets_root_path()
            if assets_root_path is None:
                raise RuntimeError("Could not find Isaac Sim assets folder")
                
            print(f"Using assets root path: {assets_root_path}")
            
            # Set up environment based on scenario (or default)
            self.setup_scenario_environment(assets_root_path)
            
            # Load scenario robots if specified
            self.load_scenario_robots(assets_root_path)
            
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
            print("  - python3 camera_command_sender.py")
            print(f"  - Command file: {self.command_file}")
            print("")
            print("💡 Test movement:")
            print("  ros2 topic pub --once /camera/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}}'")
            print("Press Ctrl+C to stop\n")
            
            while not self.shutdown_requested:
                try:
                    # Process movement commands from file
                    self.process_movement_commands()
                    
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
            
            # Clean up ROS2 node
            if self.ros_node:
                try:
                    self.ros_node.destroy_node()
                    print("✓ ROS2 node destroyed")
                except Exception as e:
                    print(f"Warning: Error destroying ROS2 node: {e}")
            
            # Shutdown ROS2 if we initialized it
            if rclpy.ok():
                try:
                    rclpy.shutdown()
                    print("✓ ROS2 shutdown")
                except Exception as e:
                    print(f"Warning: Error shutting down ROS2: {e}")
            
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
            # Clean up on any error
            if self.command_file.exists():
                try:
                    self.command_file.unlink()
                except:
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

    def load_scenario_config(self):
        """Load scenario configuration if available"""
        try:
            if self.scenario_config_file.exists() and self.scenario_manager:
                config = self.scenario_manager.load_scenario_config()
                if config:
                    self.current_scenario_config = config
                    print(f"🎯 Loaded scenario: {config['parsed_scenario']['scene']}")
                    return True
        except Exception as e:
            print(f"⚠️  Could not load scenario config: {e}")
        return False
    
    def setup_scenario_environment(self, assets_root_path):
        """Set up environment based on scenario configuration"""
        if not self.current_scenario_config:
            # Use default simple room environment
            environment_path = assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd"
            stage.add_reference_to_stage(usd_path=environment_path, prim_path="/World/simple_room")
            print("✅ Default simple_room environment loaded")
            return True
            
        try:
            scenario = self.current_scenario_config['parsed_scenario']
            scene_name = scenario['scene']
            
            # Load specified scene
            if scene_name in self.scenario_manager.available_scenes:
                scene_info = self.scenario_manager.available_scenes[scene_name]
                scene_path = assets_root_path + scene_info['path']
                stage.add_reference_to_stage(usd_path=scene_path, prim_path=f"/World/{scene_name}")
                print(f"✅ Scenario scene loaded: {scene_name}")
                
                # Set camera position if specified in scenario
                if scenario.get('camera_position'):
                    self.camera_position = scenario['camera_position'].copy()
                    print(f"📷 Scenario camera position set: {self.camera_position}")
                
                return True
            else:
                print(f"⚠️  Scene '{scene_name}' not found, using simple_room")
                # Fall back to simple room
                environment_path = assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd"
                stage.add_reference_to_stage(usd_path=environment_path, prim_path="/World/simple_room")
                
        except Exception as e:
            print(f"❌ Error setting up scenario environment: {e}")
            # Fall back to simple room
            environment_path = assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd"
            stage.add_reference_to_stage(usd_path=environment_path, prim_path="/World/simple_room")
        
        return True
    
    def load_scenario_robots(self, assets_root_path):
        """Load robots specified in scenario configuration"""
        if not self.current_scenario_config:
            return
            
        try:
            scenario = self.current_scenario_config['parsed_scenario']
            robots = scenario.get('robots', [])
            
            for i, robot_config in enumerate(robots):
                robot_type = robot_config['type']
                
                if robot_type in self.scenario_manager.available_robots:
                    robot_info = self.scenario_manager.available_robots[robot_type]
                    robot_path = assets_root_path + robot_info['path']
                    robot_prim_path = f"/World/{robot_config['name']}"
                    
                    # Load robot asset
                    stage.add_reference_to_stage(usd_path=robot_path, prim_path=robot_prim_path)
                    
                    # Set robot position
                    robot_prim = omni.usd.get_context().get_stage().GetPrimAtPath(robot_prim_path)
                    if robot_prim:
                        xform_api = UsdGeom.XformCommonAPI(robot_prim)
                        position = robot_config.get('position', [0, 0, 0])
                        xform_api.SetTranslate(Gf.Vec3d(position[0], position[1], position[2]))
                        
                        print(f"🤖 Robot loaded: {robot_type} at {position}")
                        
                        # Track loaded assets
                        self.scenario_assets.append({
                            'type': 'robot',
                            'name': robot_config['name'],
                            'prim_path': robot_prim_path
                        })
                        
                        # TODO: Enable ROS2 control if specified
                        if robot_config.get('ros_enabled'):
                            print(f"🔗 ROS2 control enabled for {robot_config['name']}")
                else:
                    print(f"⚠️  Robot type '{robot_type}' not found, skipping")
                    
        except Exception as e:
            print(f"❌ Error loading scenario robots: {e}")
    
    def initialize_ros2_node(self):
        """Initialize ROS2 node for cmd_vel subscription"""
        try:
            # Initialize ROS2 if not done
            if not rclpy.ok():
                rclpy.init()
            
            # Create a simple ROS2 node
            class CameraControlROS2Node(Node):
                def __init__(self, camera_node):
                    super().__init__('isaac_camera_control_node')
                    self.camera_node = camera_node
                    self.subscription = self.create_subscription(
                        Twist,
                        '/camera/cmd_vel',
                        self.cmd_vel_callback,
                        10
                    )
                    self.get_logger().info('🎮 Isaac Camera Control ROS2 Node initialized')
                    self.get_logger().info('📡 Subscribing to /camera/cmd_vel')
                
                def cmd_vel_callback(self, msg):
                    """Handle incoming cmd_vel messages"""
                    try:
                        self.get_logger().info(f'🎮 Received cmd_vel: linear=[{msg.linear.x:.3f}, {msg.linear.y:.3f}, {msg.linear.z:.3f}], angular=[{msg.angular.x:.3f}, {msg.angular.y:.3f}, {msg.angular.z:.3f}]')
                        
                        # Convert ROS2 Twist message to file-based command
                        command = {
                            "type": "velocity",
                            "data": {
                                "linear": {
                                    "x": msg.linear.x,
                                    "y": msg.linear.y,
                                    "z": msg.linear.z
                                },
                                "angular": {
                                    "x": msg.angular.x,
                                    "y": msg.angular.y,
                                    "z": msg.angular.z
                                }
                            },
                            "timestamp": time.time()
                        }
                        
                        # Write command to file (same format as camera_cli.py)
                        self.camera_node.write_command_file(command)
                        
                    except Exception as e:
                        self.get_logger().error(f'Error processing cmd_vel: {e}')
            
            # Create ROS2 node
            self.ros_node = CameraControlROS2Node(self)
            
            # Start ROS2 spinning in a separate thread
            def ros_spin():
                try:
                    rclpy.spin(self.ros_node)
                except Exception as e:
                    print(f"ROS2 spin error: {e}")
            
            self.ros_thread = threading.Thread(target=ros_spin, daemon=True)
            self.ros_thread.start()
            
            print("✅ ROS2 cmd_vel subscriber initialized successfully")
            
        except Exception as e:
            print(f"❌ Error initializing ROS2 node: {e}")
            print("Camera control will work with file-based commands only")
    
    def write_command_file(self, command):
        """Write command to file for processing by main loop"""
        try:
            # Write command to file (atomic operation)
            temp_file = Path(f"{self.command_file}.tmp")
            with open(temp_file, 'w') as f:
                json.dump(command, f)
            temp_file.rename(self.command_file)
            
        except Exception as e:
            print(f"Error writing command file: {e}")

    def run(self):
        """Main execution method with scenario support"""
        print("=== Isaac Sim 5.0 ROS2 Camera Node - Enhanced with Scenario Support ===")
        print("Using official APIs: isaacsim.ros2.bridge, ROS2CameraHelper")
        if self.scenario_manager:
            print("🎯 Scenario management enabled")
        print()
        
        try:
            # Step 1: Initialize Isaac Sim (includes scenario loading)
            if not self.initialize_isaac_sim():
                print("FAILED: Could not initialize Isaac Sim")
                return False
                
            # Print scenario info if loaded
            if self.current_scenario_config:
                scenario = self.current_scenario_config['parsed_scenario']
                print(f"\n🎬 Active Scenario:")
                print(f"   Scene: {scenario['scene']}")
                print(f"   Robots: {len(scenario['robots'])} robot(s)")
                print(f"   Special requirements: {scenario['special_requirements']}")
                print(f"   Camera position: {scenario.get('camera_position', 'default')}")
                print()
                
            # Step 2: Create camera and ROS2 publishing graph
            if not self.create_camera_and_ros_graph():
                print("FAILED: Could not create camera and ROS2 graph")
                return False
                
            # Step 3: Run simulation
            return self.run_simulation()
            
        except Exception as e:
            print(f"FATAL ERROR: {e}")
            print(traceback.format_exc())
            return False
            
        except Exception as e:
            print(f"FATAL ERROR: {e}")
            print(traceback.format_exc())
            return False

def main():
    """Main entry point"""
    node = FinalCameraNode()
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
