#!/usr/bin/env python3
"""
Isaac Sim Camera with ROS2 Control - Working Version
This version actually moves the camera in Isaac Sim based on ROS2 commands
"""

import sys
import signal
import time
import traceback
import math
import threading
import asyncio
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


class MovableIsaacCamera:
    def __init__(self):
        self.simulation_context = None
        self.camera_prim = None
        self.ros_camera_graph = None
        self.shutdown_requested = False
        
        # Camera position and control
        self.camera_position = [2.0, 2.0, 2.0]  # x, y, z
        self.camera_rotation = [0.0, 0.0, 0.0]  # euler angles in degrees
        
        # Command file for ROS2 communication
        self.command_file = Path("/tmp/isaac_camera_commands.json")
        
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
            
            print("✅ Isaac Sim initialization complete")
            return True
            
        except Exception as e:
            print(f"❌ Isaac Sim initialization failed: {e}")
            traceback.print_exc()
            return False
            
    def setup_scene(self):
        """Setup the scene with objects and camera"""
        try:
            print("Setting up Isaac Sim scene...")
            
            # Get stage
            stage = omni.usd.get_context().get_stage()
            
            # Create ground plane
            UsdGeom.Mesh.Define(stage, "/World/Ground")
            ground_prim = stage.GetPrimAtPath("/World/Ground")
            ground_geom = UsdGeom.Mesh(ground_prim)
            
            # Create some cubes for reference
            for i in range(3):
                cube_path = f"/World/Cube_{i}"
                UsdGeom.Cube.Define(stage, cube_path)
                cube_prim = stage.GetPrimAtPath(cube_path)
                
                # Set position
                xform_api = UsdGeom.XformCommonAPI(cube_prim)
                xform_api.SetTranslate(Gf.Vec3d(i*2, 0, 0.5))
                xform_api.SetScale(Gf.Vec3f(1.0, 1.0, 1.0))
            
            # Add lighting
            UsdLux.DistantLight.Define(stage, "/World/DistantLight")
            light_prim = stage.GetPrimAtPath("/World/DistantLight")
            light = UsdLux.DistantLight(light_prim)
            light.CreateIntensityAttr(3000)
            
            # Set light direction
            xform_api = UsdGeom.XformCommonAPI(light_prim)
            xform_api.SetRotate(Gf.Vec3f(-45, 0, 0))
            
            print("✅ Scene setup complete")
            return True
            
        except Exception as e:
            print(f"❌ Scene setup failed: {e}")
            traceback.print_exc()
            return False
            
    def create_camera(self):
        """Create camera and ROS2 publishing graph"""
        try:
            print("Creating camera and ROS2 publishing graph...")
            
            # Get stage
            stage = omni.usd.get_context().get_stage()
            
            # Create camera prim
            camera_path = "/World/Camera"
            self.camera_prim = UsdGeom.Camera.Define(stage, camera_path)
            
            # Set camera properties
            self.camera_prim.GetFocalLengthAttr().Set(24.0)
            self.camera_prim.GetClippingRangeAttr().Set(Gf.Vec2f(0.1, 1000.0))
            
            # Set initial camera transform
            self.update_camera_position()
            
            # Create ROS2 camera graph
            keys = og.Controller.Keys
            (ros_camera_graph, nodes, _, _) = og.Controller.edit(
                {
                    "graph_path": "/World/CameraGraph",
                    "evaluator_name": "execution"
                },
                {
                    keys.CREATE_NODES: [
                        ("cameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                        ("cameraHelperInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                        ("cameraHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ],
                    keys.SET_VALUES: [
                        # RGB camera settings
                        ("cameraHelperRgb.inputs:cameraPrim", camera_path),
                        ("cameraHelperRgb.inputs:topicName", "/camera/rgb"),
                        ("cameraHelperRgb.inputs:frameId", "camera_frame"),
                        ("cameraHelperRgb.inputs:type", "rgb"),
                        ("cameraHelperRgb.inputs:queueSize", 10),
                        
                        # Camera info settings  
                        ("cameraHelperInfo.inputs:cameraPrim", camera_path),
                        ("cameraHelperInfo.inputs:topicName", "/camera/camera_info"),
                        ("cameraHelperInfo.inputs:frameId", "camera_frame"),
                        ("cameraHelperInfo.inputs:queueSize", 10),
                        
                        # Depth camera settings
                        ("cameraHelperDepth.inputs:cameraPrim", camera_path),
                        ("cameraHelperDepth.inputs:topicName", "/camera/depth"),
                        ("cameraHelperDepth.inputs:frameId", "camera_frame"),
                        ("cameraHelperDepth.inputs:type", "depth"),
                        ("cameraHelperDepth.inputs:queueSize", 10),
                    ]
                }
            )
            
            self.ros_camera_graph = ros_camera_graph
            
            print("✅ Camera and ROS2 graph created successfully")
            
            # Initialize ROS2 publishers
            og.Controller.set(og.Controller.attribute("outputs:execOut", nodes[0]), None)
            print("✅ ROS2 publishers initialized")
            
            return True
            
        except Exception as e:
            print(f"❌ Camera creation failed: {e}")
            traceback.print_exc()
            return False
            
    def update_camera_position(self):
        """Update camera position in Isaac Sim"""
        if self.camera_prim:
            try:
                # Get the camera prim
                camera_prim = self.camera_prim.GetPrim()
                
                # Create transform
                xform_api = UsdGeom.XformCommonAPI(camera_prim)
                
                # Set translation
                translation = Gf.Vec3d(self.camera_position[0], self.camera_position[1], self.camera_position[2])
                xform_api.SetTranslate(translation)
                
                # Set rotation (convert degrees to radians)
                rotation = Gf.Vec3f(
                    math.radians(self.camera_rotation[0]),  # roll
                    math.radians(self.camera_rotation[1]),  # pitch  
                    math.radians(self.camera_rotation[2])   # yaw
                )
                xform_api.SetRotate(rotation, UsdGeom.XformCommonAPI.RotationOrderXYZ)
                
                print(f"📷 Camera moved to: position={self.camera_position}, rotation={self.camera_rotation}")
                
            except Exception as e:
                print(f"❌ Failed to update camera position: {e}")
                
    def process_commands(self):
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
                        self.camera_rotation[0] += vel.get('angular_x', 0) * dt * 57.2958  # rad to deg
                        self.camera_rotation[1] += vel.get('angular_y', 0) * dt * 57.2958
                        self.camera_rotation[2] += vel.get('angular_z', 0) * dt * 57.2958
                        
                        self.update_camera_position()
                        
                    elif command.get('type') == 'position':
                        # Set absolute position
                        pos = command.get('data', {})
                        self.camera_position = [
                            pos.get('x', self.camera_position[0]),
                            pos.get('y', self.camera_position[1]),
                            pos.get('z', self.camera_position[2])
                        ]
                        self.update_camera_position()
                
                # Clear processed commands
                self.command_file.unlink()
                
            except Exception as e:
                print(f"❌ Error processing commands: {e}")
                
    def run_simulation(self):
        """Main simulation loop"""
        print("🚀 Starting camera simulation with movement control...")
        print(f"📂 Listening for commands at: {self.command_file}")
        print("📊 Publishing ROS2 topics:")
        print("  /camera/rgb - Camera image")
        print("  /camera/depth - Depth image") 
        print("  /camera/camera_info - Camera parameters")
        
        try:
            while simulation_app.is_running() and not self.shutdown_requested:
                # Process any movement commands
                self.process_commands()
                
                # Step simulation
                self.simulation_context.step(render=True)
                
                # Small delay
                time.sleep(0.05)
                
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
    print("=== Isaac Sim Camera with Movement Control ===")
    print("This version actually moves the camera based on commands\n")
    
    camera_node = MovableIsaacCamera()
    
    try:
        # Step 1: Initialize Isaac Sim
        if not camera_node.initialize_isaac_sim():
            return False
            
        # Step 2: Setup scene
        if not camera_node.setup_scene():
            return False
            
        # Step 3: Create camera and ROS2 publishing
        if not camera_node.create_camera():
            return False
        
        # Step 4: Run simulation loop
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
