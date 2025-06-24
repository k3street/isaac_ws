#!/usr/bin/env python3
"""
Isaac Sim Camera with Movement Control - Fixed Version
Based on the working isaac_camera_node_final.py but adds camera movement capability
"""
import sys
import signal
import time
import traceback
import math
import json
from pathlib import Path

from isaacsim.simulation_app import SimulationApp
CONFIG = {"renderer": "RaytracedLighting", "headless": False}
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


class MovableCameraNode:
    def __init__(self):
        self.simulation_context = None
        self.camera_prim = None
        self.ros_camera_graph = None
        self.shutdown_requested = False
        
        # Camera movement state
        self.camera_position = [2.0, 2.0, 2.0]  # x, y, z
        self.camera_rotation = [0.0, 0.0, 0.0]  # euler angles in degrees
        
        # Command file for movement
        self.command_file = Path("/tmp/isaac_camera_commands.json")
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
    def _signal_handler(self, signum, frame):
        print(f"Received signal {signum}, initiating shutdown...")
        self.shutdown_requested = True
        
    def initialize_isaac_sim(self):
        """Initialize Isaac Sim following the working example"""
        try:
            print("Initializing Isaac Sim simulation context...")
            
            # Enable ROS2 bridge extension
            print("Enabling isaacsim.ros2.bridge extension...")
            extensions.enable_extension("isaacsim.ros2.bridge")
            
            # Create simulation context
            self.simulation_context = SimulationContext(
                stage_units_in_meters=1.0,
                physics_dt=1.0/60.0,
                rendering_dt=1.0/60.0,
                backend="torch"
            )
            
            # Add environment objects for reference
            stage_context = omni.usd.get_context()
            stage = stage_context.get_stage()
            
            # Create ground plane
            plane_prim = UsdGeom.Plane.Define(stage, "/World/GroundPlane")
            plane_geom = UsdGeom.Plane(plane_prim)
            plane_geom.CreateAxisAttr().Set("Y")
            
            # Set ground size and position
            xform_api = UsdGeom.XformCommonAPI(plane_prim)
            xform_api.SetTranslate(Gf.Vec3d(0, 0, 0))
            xform_api.SetScale(Gf.Vec3f(10.0, 1.0, 10.0))
            
            # Create some reference cubes
            for i in range(3):
                cube_prim = UsdGeom.Cube.Define(stage, f"/World/Cube{i+1}")
                xform_api = UsdGeom.XformCommonAPI(cube_prim)
                xform_api.SetTranslate(Gf.Vec3d(i*2, 0, 0.5))
                
            # Add lighting
            light_prim = UsdLux.DistantLight.Define(stage, "/World/DistantLight")
            light = UsdLux.DistantLight(light_prim)
            light.CreateIntensityAttr(3000)
            
            xform_api = UsdGeom.XformCommonAPI(light_prim)
            xform_api.SetTranslate(Gf.Vec3d(0, 0, 0.5))
            xform_api.SetRotate(Gf.Vec3f(1.5, 0, 0))
            
            simulation_app.update()
            print("✓ Isaac Sim initialized successfully")
            
            return True
            
        except Exception as e:
            print(f"❌ Failed to initialize Isaac Sim: {e}")
            traceback.print_exc()
            return False
            
    def create_camera_and_ros_graph(self):
        """Create camera and ROS2 publishing graph using the working method"""
        try:
            print("Creating camera and ROS2 publishing graph...")
            
            # Camera parameters (same as working version)
            CAMERA_STAGE_PATH = "/World/Camera"
            ROS_CAMERA_GRAPH_PATH = "/ROS_Camera"
            
            # Create camera prim (following working example)
            camera_prim = UsdGeom.Camera(omni.usd.get_context().get_stage().DefinePrim(CAMERA_STAGE_PATH, "Camera"))
            xform_api = UsdGeom.XformCommonAPI(camera_prim)
            xform_api.SetTranslate(Gf.Vec3d(self.camera_position[0], self.camera_position[1], self.camera_position[2]))
            xform_api.SetRotate((self.camera_rotation[0], self.camera_rotation[1], self.camera_rotation[2]), UsdGeom.XformCommonAPI.RotationOrderXYZ)
            camera_prim.GetHorizontalApertureAttr().Set(21)
            camera_prim.GetVerticalApertureAttr().Set(16)
            camera_prim.GetProjectionAttr().Set("perspective")
            camera_prim.GetFocalLengthAttr().Set(24)
            camera_prim.GetFocusDistanceAttr().Set(400)
            
            self.camera_prim = camera_prim
            
            simulation_app.update()
            
            # Create ROS camera graph (exact copy from working version)
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
            print(f"❌ Failed to create camera and ROS graph: {e}")
            traceback.print_exc()
            return False
            
    def update_camera_transform(self):
        """Update camera position in Isaac Sim - THIS IS THE KEY FUNCTION"""
        if self.camera_prim:
            try:
                # Get the camera prim
                camera_prim_path = self.camera_prim.GetPrim()
                
                # Update transform using XformCommonAPI
                xform_api = UsdGeom.XformCommonAPI(camera_prim_path)
                
                # Set new translation
                new_translation = Gf.Vec3d(
                    self.camera_position[0], 
                    self.camera_position[1], 
                    self.camera_position[2]
                )
                xform_api.SetTranslate(new_translation)
                
                # Set new rotation
                new_rotation = (
                    self.camera_rotation[0], 
                    self.camera_rotation[1], 
                    self.camera_rotation[2]
                )
                xform_api.SetRotate(new_rotation, UsdGeom.XformCommonAPI.RotationOrderXYZ)
                
                print(f"📷 Camera moved to: position={self.camera_position}, rotation={self.camera_rotation}")
                
            except Exception as e:
                print(f"❌ Failed to update camera position: {e}")
                
    def process_movement_commands(self):
        """Process movement commands from file"""
        if self.command_file.exists():
            try:
                with open(self.command_file, 'r') as f:
                    commands = json.load(f)
                
                for command in commands:
                    if command.get('type') == 'velocity':
                        # Apply velocity for one timestep
                        vel = command.get('data', {})
                        dt = 0.1  # timestep
                        
                        # Update position
                        self.camera_position[0] += vel.get('linear_x', 0) * dt
                        self.camera_position[1] += vel.get('linear_y', 0) * dt
                        self.camera_position[2] += vel.get('linear_z', 0) * dt
                        
                        # Update rotation (convert rad/s to degrees)
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
                
                # Clear processed commands
                self.command_file.unlink()
                
            except Exception as e:
                print(f"❌ Error processing commands: {e}")
                
    def run_simulation(self):
        """Run the main simulation loop with movement processing"""
        try:
            print("🚀 Starting simulation with camera movement control...")
            print(f"📂 Listening for commands at: {self.command_file}")
            print("📊 Publishing ROS2 topics:")
            print("  /camera/rgb - Camera image")
            print("  /camera/depth - Depth image") 
            print("  /camera/camera_info - Camera parameters")
            print("")
            print("💡 To move camera:")
            print("  ros2 topic pub --once /camera/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}}'")
            print("  ros2 topic pub --once /camera/set_pose geometry_msgs/msg/PoseStamped '{pose: {position: {x: 5.0, z: 3.0}}}'")
            
            frame_count = 0
            
            while simulation_app.is_running() and not self.shutdown_requested:
                # Process any movement commands
                self.process_movement_commands()
                
                # Step the simulation context
                if self.simulation_context:
                    self.simulation_context.step(render=True)
                
                # Update ROS2 publishers periodically
                if frame_count % 10 == 0:  # Every 10 frames
                    if self.ros_camera_graph:
                        og.Controller.evaluate_sync(self.ros_camera_graph)
                
                simulation_app.update()
                frame_count += 1
                
                # Small delay
                time.sleep(0.02)  # ~50 FPS
                
        except KeyboardInterrupt:
            print("🛑 Received interrupt signal")
        except Exception as e:
            print(f"❌ Simulation error: {e}")
            traceback.print_exc()
        finally:
            print("🧹 Cleaning up...")
            if self.command_file.exists():
                self.command_file.unlink()


def main():
    print("=== Isaac Sim Movable Camera - Fixed Version ===")
    print("Camera will actually move in Isaac Sim based on ROS2 commands\n")
    
    camera_node = MovableCameraNode()
    
    try:
        # Step 1: Initialize Isaac Sim
        if not camera_node.initialize_isaac_sim():
            return False
            
        # Step 2: Create camera and ROS2 publishing
        if not camera_node.create_camera_and_ros_graph():
            return False
        
        # Step 3: Run simulation loop with movement
        camera_node.run_simulation()
        
    except Exception as e:
        print(f"❌ Fatal error: {e}")
        traceback.print_exc()
        return False
    finally:
        simulation_app.close()
        
    return True


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
