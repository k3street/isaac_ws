#!/usr/bin/env python3
"""
Isaac Sim Camera Node - Isaac Sim Only Version (No ROS2)
This version runs inside Isaac Sim's Python 3.11 environment
Loads scenarios and publishes camera data via Isaac Sim's built-in ROS2 bridge
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
    sys.path.append('/home/kimate/isaac_ws')
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

class IsaacSimCameraNode:
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
            
            # Enable required extensions
            print("Enabling isaacsim.ros2.bridge extension...")
            extensions.enable_extension("isaacsim.ros2.bridge")
            
            print("Enabling omni.isaac.core_nodes extension...")
            extensions.enable_extension("omni.isaac.core_nodes")
            
            print("Enabling omni.isaac.core extension...")
            extensions.enable_extension("omni.isaac.core")
            
            # Update to load extensions
            simulation_app.update()
            time.sleep(1)  # Give extensions time to load
            
            # Get assets root path
            assets_root_path = get_assets_root_path()
            print(f"Assets root path: {assets_root_path}")
            
            # Create simulation context
            self.simulation_context = SimulationContext(stage_units_in_meters=1.0)
            
            # Set up environment based on scenario (or default)
            self.setup_scenario_environment(assets_root_path)
            
            # Load scenario robots if specified
            self.load_scenario_robots(assets_root_path)
            
            # Add some interesting objects to view
            # Add a cube at origin
            try:
                from isaacsim.core.utils.stage import add_reference_to_stage
            except ImportError:
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
            print("Creating camera and ROS2 graph...")
            
            # Create camera prim
            try:
                from isaacsim.core.utils.prims import create_prim
            except ImportError:
                from omni.isaac.core.utils.prims import create_prim
            
            # Set initial camera position based on scenario or default
            cam_pos = self.camera_position.copy()
            if self.current_scenario_config:
                scenario = self.current_scenario_config['parsed_scenario']
                if scenario.get('camera_position'):
                    cam_pos = scenario['camera_position']
                    print(f"📷 Using scenario camera position: {cam_pos}")
            
            self.camera_prim = create_prim(
                prim_path="/World/Camera",
                prim_type="Camera",
                position=cam_pos,
                orientation=[0.0, 0.0, 0.0, 1.0]  # No initial rotation (identity quaternion)
            )
            
            # Set camera properties
            camera_prim = omni.usd.get_context().get_stage().GetPrimAtPath("/World/Camera")
            if camera_prim:
                from pxr import UsdGeom
                camera = UsdGeom.Camera(camera_prim)
                camera.CreateFocalLengthAttr(24.0)  # Wide angle for better coverage
                camera.CreateFocusDistanceAttr(400.0)  # Focus distance
                
            simulation_app.update()
            
            # Create ROS2 camera helper graph following IsaacSim examples
            print("Creating ROS2 camera publisher graph...")
            
            try:
                # Clear any existing action graphs to avoid conflicts
                keys = og.Controller.Keys
                graph_path = "/World/ActionGraph"
                
                # Try to remove existing graph if it exists
                try:
                    existing_graph = og.Controller.graph(graph_path)
                    if existing_graph.is_valid():
                        print("Removing existing ActionGraph...")
                        og.Controller.edit({"graph_path": graph_path}, {keys.DELETE_GRAPH: True})
                        simulation_app.update()
                except:
                    pass  # Graph doesn't exist, that's fine
                
                # Try with omni.isaac.core_nodes first
                try:
                    (ros_camera_graph, nodes, _, _) = og.Controller.edit(
                        {
                            "graph_path": graph_path,
                            "evaluator_name": "execution",
                            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
                        },
                        {
                            keys.CREATE_NODES: [
                                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                                ("CreateRenderProduct", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                                ("ROS2CameraHelper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                            ],
                            keys.CONNECT: [
                                ("OnPlaybackTick.outputs:tick", "ROS2CameraHelper.inputs:execIn"),
                                ("CreateRenderProduct.outputs:renderProductPath", "ROS2CameraHelper.inputs:renderProductPath"),
                            ],
                            keys.SET_VALUES: [
                                # Camera setup
                                ("CreateRenderProduct.inputs:cameraPrim", "/World/Camera"),
                                ("CreateRenderProduct.inputs:resolution", [640, 480]),
                                
                                # ROS2 topics
                                ("ROS2CameraHelper.inputs:topicName", "camera"),
                                ("ROS2CameraHelper.inputs:frameId", "camera_link"),
                                ("ROS2CameraHelper.inputs:nodeNamespace", ""),
                                ("ROS2CameraHelper.inputs:queueSize", 10),
                                ("ROS2CameraHelper.inputs:publishRGB", True),
                                ("ROS2CameraHelper.inputs:publishDepth", True),
                                ("ROS2CameraHelper.inputs:publishInfo", True),
                                ("ROS2CameraHelper.inputs:publishDepthPCL", False),
                            ],
                        },
                    )
                    print("✓ ROS2 camera publisher created successfully")
                    
                except Exception as core_nodes_error:
                    print(f"Warning: omni.isaac.core_nodes not available: {core_nodes_error}")
                    print("Trying simplified ROS2 camera setup...")
                    
                    # Alternative: Try using just the ROS2 bridge without render product
                    try:
                        # Create a simplified graph with just the camera helper
                        (ros_camera_graph, nodes, _, _) = og.Controller.edit(
                            {
                                "graph_path": graph_path + "_Simple",  # Use different path to avoid conflict
                                "evaluator_name": "execution",
                                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
                            },
                            {
                                keys.CREATE_NODES: [
                                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                                    ("ROS2CameraHelper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                                ],
                                keys.CONNECT: [
                                    ("OnPlaybackTick.outputs:tick", "ROS2CameraHelper.inputs:execIn"),
                                ],
                                keys.SET_VALUES: [
                                    # ROS2 topics
                                    ("ROS2CameraHelper.inputs:topicName", "camera"),
                                    ("ROS2CameraHelper.inputs:frameId", "camera_link"),
                                    ("ROS2CameraHelper.inputs:nodeNamespace", ""),
                                    ("ROS2CameraHelper.inputs:queueSize", 10),
                                    ("ROS2CameraHelper.inputs:publishRGB", True),
                                    ("ROS2CameraHelper.inputs:publishDepth", True),
                                    ("ROS2CameraHelper.inputs:publishInfo", True),
                                    ("ROS2CameraHelper.inputs:publishDepthPCL", False),
                                ],
                            },
                        )
                        print("✓ Simplified ROS2 camera setup successful")
                        
                    except Exception as alt_error:
                        print(f"Warning: Simplified ROS2 setup also failed: {alt_error}")
                        print("Camera will work in Isaac Sim, but no ROS2 publishing")
                        
                self.ros_camera_graph = ros_camera_graph if 'ros_camera_graph' in locals() else None
                
            except Exception as e:
                print(f"Warning: Could not create ROS2 camera graph: {e}")
                print("Camera will still work, but no ROS2 publishing")
                
            simulation_app.update()
            return True
            
        except Exception as e:
            print(f"ERROR: Failed to create camera: {e}")
            print(traceback.format_exc())
            return False

    def load_scenario_config(self):
        """Load scenario configuration if available"""
        try:
            if self.scenario_config_file.exists() and self.scenario_manager:
                config = self.scenario_manager.load_scenario_config()
                if config:
                    self.current_scenario_config = config
                    print(f"🎯 Loaded scenario: {config['parsed_scenario']['scene']}")
                    print(f"   Robots: {len(config['parsed_scenario'].get('robots', []))}")
                    return True
        except Exception as e:
            print(f"⚠️  Could not load scenario config: {e}")
        return False
    
    def setup_scenario_environment(self, assets_root_path):
        """Set up environment based on scenario configuration"""
        if not self.current_scenario_config:
            # Use default simple room environment
            environment_path = assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd"
            print(f"🏠 Loading default environment from: {environment_path}")
            stage.add_reference_to_stage(usd_path=environment_path, prim_path="/World/simple_room")
            print("✅ Default simple_room environment loaded")
            return True
            
        try:
            scenario = self.current_scenario_config['parsed_scenario']
            scene_name = scenario['scene']
            
            # Handle special case for no environment
            if scene_name == 'none':
                print("🏠 No environment loading requested - using empty scene")
                return True
            
            # Load specified scene
            if scene_name in self.scenario_manager.available_scenes:
                scene_info = self.scenario_manager.available_scenes[scene_name]
                scene_path = assets_root_path + scene_info['path']
                print(f"🏠 Loading scenario environment: {scene_name}")
                print(f"   Path: {scene_path}")
                
                # Check if file exists (only for local paths)
                import os
                is_remote = scene_path.startswith(('http://', 'https://'))
                if not is_remote and not os.path.exists(scene_path):
                    print(f"❌ Local scene file not found: {scene_path}")
                    # Fall back to simple room
                    environment_path = assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd"
                    print(f"🏠 Falling back to: {environment_path}")
                    stage.add_reference_to_stage(usd_path=environment_path, prim_path="/World/simple_room")
                    return True
                elif is_remote:
                    print(f"📡 Using remote asset: {scene_path}")
                
                stage.add_reference_to_stage(usd_path=scene_path, prim_path=f"/World/{scene_name}")
                print(f"✅ Scenario scene loaded: {scene_name} at /World/{scene_name}")
                
                # Set camera position if specified in scenario
                if scenario.get('camera_position'):
                    self.camera_position = scenario['camera_position'].copy()
                    print(f"📷 Scenario camera position set: {self.camera_position}")
                
                return True
            else:
                print(f"⚠️  Scene '{scene_name}' not found in available scenes: {list(self.scenario_manager.available_scenes.keys())}")
                # Fall back to simple room
                environment_path = assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd"
                print(f"🏠 Falling back to: {environment_path}")
                stage.add_reference_to_stage(usd_path=environment_path, prim_path="/World/simple_room")
                
        except Exception as e:
            print(f"❌ Error setting up scenario environment: {e}")
            import traceback
            print(traceback.format_exc())
            # Fall back to simple room
            environment_path = assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd"
            print(f"🏠 Falling back to: {environment_path}")
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
                else:
                    print(f"⚠️  Robot type '{robot_type}' not found, skipping")
                    
        except Exception as e:
            print(f"❌ Error loading scenario robots: {e}")

    def start_simulation_loop(self):
        """Start the main simulation loop"""
        try:
            print("Starting simulation...")
            
            # Reset simulation to start
            self.simulation_context.reset()
            self.simulation_context.play()
            
            print("🚀 Isaac Sim simulation running with scenario support")
            print("📷 Camera publishing on ROS2 topics: /camera/rgb, /camera/depth, /camera/camera_info")
            
            if self.current_scenario_config:
                scenario = self.current_scenario_config['parsed_scenario']
                print(f"🎯 Active scenario: {scenario['scene']} with {len(scenario.get('robots', []))} robots")
            
            print("Press Ctrl+C to exit")
            
            # Main simulation loop
            while not self.shutdown_requested:
                # Keep simulation running
                self.simulation_context.step(render=True)
                
                # Small delay to prevent excessive CPU usage
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("Received keyboard interrupt, shutting down...")
        except Exception as e:
            print(f"ERROR in simulation loop: {e}")
            print(traceback.format_exc())
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        try:
            print("Cleaning up...")
            if self.simulation_context:
                self.simulation_context.stop()
            simulation_app.close()
        except Exception as e:
            print(f"Error during cleanup: {e}")

def main():
    print("🎬 Isaac Sim Camera Node (Scenario-Enabled)")
    print("=" * 50)
    
    camera_node = IsaacSimCameraNode()
    
    # Initialize Isaac Sim
    if not camera_node.initialize_isaac_sim():
        print("Failed to initialize Isaac Sim")
        return 1
    
    # Create camera and ROS graph
    if not camera_node.create_camera_and_ros_graph():
        print("Failed to create camera")
        return 1
    
    # Start simulation
    camera_node.start_simulation_loop()
    
    return 0

if __name__ == "__main__":
    exit(main())
