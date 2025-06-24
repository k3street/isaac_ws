#!/usr/bin/env python3
"""
Isaac Sim Camera Bridge
Simple bridge to control Isaac Sim camera from ROS2 commands
"""

import asyncio
import json
import time
from pathlib import Path

# Isaac Sim imports
import omni
from omni.isaac.kit import SimulationApp

# Configuration
CONFIG = {
    "width": 1280,
    "height": 720,
    "headless": False,
    "renderer": "RayTracedLighting"
}

class IsaacCameraBridge:
    def __init__(self):
        self.simulation_app = SimulationApp(CONFIG)
        
        # Import after SimulationApp
        import omni.usd
        from omni.isaac.core import World
        from omni.isaac.core.objects import DynamicCuboid
        from pxr import Gf, UsdGeom
        
        # Store references
        self.omni_usd = omni.usd
        self.World = World
        self.DynamicCuboid = DynamicCuboid
        self.Gf = Gf
        self.UsdGeom = UsdGeom
        
        # Initialize world and camera
        self.world = None
        self.camera_prim = None
        self.camera_position = [0, 0, 3]  # Default position
        self.camera_rotation = [0, 0, 0]  # Default rotation
        
        # Control state file for communication with ROS2
        self.control_file = Path("/tmp/isaac_camera_control.json")
        self.status_file = Path("/tmp/isaac_camera_status.json")
        
        self.setup_scene()
        
    def setup_scene(self):
        """Setup the Isaac Sim scene with camera"""
        print("🎮 Setting up Isaac Sim scene...")
        
        # Create world
        self.world = self.World(stage_units_in_meters=1.0)
        
        # Add some objects to the scene
        cube1 = self.DynamicCuboid(
            prim_path="/World/Cube1",
            name="cube1",
            position=[2, 2, 0.5],
            size=[1.0, 1.0, 1.0],
            color=[1.0, 0.0, 0.0]  # Red
        )
        
        cube2 = self.DynamicCuboid(
            prim_path="/World/Cube2", 
            name="cube2",
            position=[-2, -2, 0.5],
            size=[1.0, 1.0, 1.0],
            color=[0.0, 1.0, 0.0]  # Green
        )
        
        # Setup camera
        stage = self.omni_usd.get_context().get_stage()
        
        # Create camera prim
        camera_path = "/World/Camera"
        self.camera_prim = self.UsdGeom.Camera.Define(stage, camera_path)
        
        # Set camera properties
        self.camera_prim.GetFocalLengthAttr().Set(24.0)
        self.camera_prim.GetClippingRangeAttr().Set(self.Gf.Vec2f(0.1, 1000.0))
        
        # Set initial position
        self.update_camera_transform()
        
        # Reset world
        self.world.reset()
        
        print("✅ Scene setup complete")
        
    def update_camera_transform(self):
        """Update camera position and rotation in Isaac Sim"""
        if self.camera_prim:
            # Create transform matrix
            translate = self.Gf.Vec3d(*self.camera_position)
            rotation = self.Gf.Rotation(self.Gf.Vec3d(1,0,0), self.camera_rotation[0]) * \
                      self.Gf.Rotation(self.Gf.Vec3d(0,1,0), self.camera_rotation[1]) * \
                      self.Gf.Rotation(self.Gf.Vec3d(0,0,1), self.camera_rotation[2])
            
            transform = self.Gf.Matrix4d()
            transform.SetTranslateOnly(translate)
            transform = transform * self.Gf.Matrix4d(rotation, self.Gf.Vec3d(0,0,0))
            
            # Apply transform
            xform = self.UsdGeom.Xformable(self.camera_prim)
            xform_op = xform.AddTransformOp()
            xform_op.Set(transform)
            
    def process_commands(self):
        """Process ROS2 commands from file"""
        if self.control_file.exists():
            try:
                with open(self.control_file, 'r') as f:
                    command = json.load(f)
                
                if command.get('type') == 'velocity':
                    # Apply velocity command
                    vel = command.get('velocity', {})
                    dt = 0.1  # Time step
                    
                    # Update position
                    self.camera_position[0] += vel.get('linear_x', 0) * dt
                    self.camera_position[1] += vel.get('linear_y', 0) * dt  
                    self.camera_position[2] += vel.get('linear_z', 0) * dt
                    
                    # Update rotation (in degrees)
                    self.camera_rotation[0] += vel.get('angular_x', 0) * dt * 57.3  # rad to deg
                    self.camera_rotation[1] += vel.get('angular_y', 0) * dt * 57.3
                    self.camera_rotation[2] += vel.get('angular_z', 0) * dt * 57.3
                    
                    self.update_camera_transform()
                    print(f"📷 Camera moved to: {self.camera_position}")
                    
                elif command.get('type') == 'position':
                    # Set absolute position
                    pos = command.get('position', {})
                    self.camera_position = [
                        pos.get('x', self.camera_position[0]),
                        pos.get('y', self.camera_position[1]), 
                        pos.get('z', self.camera_position[2])
                    ]
                    self.update_camera_transform()
                    print(f"📷 Camera set to: {self.camera_position}")
                
                # Remove processed command
                self.control_file.unlink()
                
            except Exception as e:
                print(f"❌ Error processing command: {e}")
                
    def update_status(self):
        """Update status file for ROS2"""
        status = {
            'position': self.camera_position,
            'rotation': self.camera_rotation,
            'timestamp': time.time()
        }
        
        try:
            with open(self.status_file, 'w') as f:
                json.dump(status, f)
        except Exception as e:
            print(f"❌ Error updating status: {e}")
            
    def run(self):
        """Main execution loop"""
        print("🚀 Isaac Sim Camera Bridge running...")
        print("📂 Control file:", self.control_file)
        print("📊 Status file:", self.status_file)
        
        try:
            while self.simulation_app.is_running():
                # Process any pending commands
                self.process_commands()
                
                # Update status
                self.update_status()
                
                # Step simulation
                self.world.step(render=True)
                
                # Small delay
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("🛑 Stopping Isaac Sim...")
        
        finally:
            # Cleanup
            if self.control_file.exists():
                self.control_file.unlink()
            if self.status_file.exists():
                self.status_file.unlink()
            self.simulation_app.close()

if __name__ == "__main__":
    bridge = IsaacCameraBridge()
    bridge.run()
