#!/usr/bin/env python3
"""
Isaac Sim 5.0 Camera Node with Random Rotation - Test Version
Simple camera rotation demo without ROS2 dependencies
"""
import sys
import signal
import time
import traceback
import math
import random

from isaacsim.simulation_app import SimulationApp
CONFIG = {"renderer": "RaytracedLighting", "headless": True}
simulation_app = SimulationApp(CONFIG)

# Standard imports after simulation app
import carb
import omni
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils import extensions, stage
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, UsdGeom, UsdLux

class CameraRotationDemo:
    def __init__(self):
        self.simulation_context = None
        self.camera_prim = None
        self.shutdown_requested = False
        self.last_rotation_time = 0
        self.current_yaw = 0.0
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
    def _signal_handler(self, signum, frame):
        print(f"Received signal {signum}, initiating shutdown...")
        self.shutdown_requested = True
        
    def rotate_camera_randomly(self):
        """Rotate camera randomly around Y-axis (yaw) every second"""
        current_time = time.time()
        
        # Rotate every 1 second
        if current_time - self.last_rotation_time >= 1.0:
            self.last_rotation_time = current_time
            
            # Generate random yaw rotation (0-360 degrees)
            self.current_yaw = random.uniform(0, 360)
            
            if self.camera_prim:
                try:
                    # Get the camera's transform API
                    xform_api = UsdGeom.XformCommonAPI(self.camera_prim)
                    
                    # Keep camera at same distance but rotate around Y-axis
                    radius = 3.0   # Distance from origin
                    height = 2.0   # Keep same height
                    
                    # Calculate new position based on yaw rotation
                    yaw_rad = math.radians(self.current_yaw)
                    new_x = radius * math.cos(yaw_rad)
                    new_z = radius * math.sin(yaw_rad)
                    
                    # Set new position
                    xform_api.SetTranslate(Gf.Vec3d(new_x, height, new_z))
                    
                    # Calculate rotation to look at origin
                    look_yaw = self.current_yaw + 180  # Face inward toward origin
                    xform_api.SetRotate((0, look_yaw, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
                    
                    print(f"🔄 Camera rotated: Yaw={self.current_yaw:.1f}° | Position=({new_x:.2f}, {height:.2f}, {new_z:.2f})")
                    
                except Exception as e:
                    print(f"Warning: Failed to rotate camera: {e}")
                    
    def initialize_isaac_sim(self):
        """Initialize Isaac Sim"""
        try:
            print("Initializing Isaac Sim simulation context...")
            
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
            
            # Add some basic shapes to make rotation visible
            # Create a simple cube at origin
            stage_ref = omni.usd.get_context().get_stage()
            cube_prim = UsdGeom.Cube.Define(stage_ref, "/World/Cube")
            cube_prim.GetSizeAttr().Set(1.0)
            xform_api = UsdGeom.XformCommonAPI(cube_prim)
            xform_api.SetTranslate(Gf.Vec3d(0, 0, 0.5))
            
            # Add a second cube 
            cube2_prim = UsdGeom.Cube.Define(stage_ref, "/World/Cube2") 
            cube2_prim.GetSizeAttr().Set(0.5)
            xform_api2 = UsdGeom.XformCommonAPI(cube2_prim)
            xform_api2.SetTranslate(Gf.Vec3d(2, 0, 0.25))
            
            # Add lighting
            sphereLight = UsdLux.SphereLight.Define(stage_ref, "/World/SphereLight")
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
            
    def create_camera(self):
        """Create camera for rotation demo"""
        try:
            print("Creating camera...")
            
            CAMERA_STAGE_PATH = "/World/Camera"
            
            # Create camera prim
            camera_prim = UsdGeom.Camera(omni.usd.get_context().get_stage().DefinePrim(CAMERA_STAGE_PATH, "Camera"))
            xform_api = UsdGeom.XformCommonAPI(camera_prim)
            xform_api.SetTranslate(Gf.Vec3d(3, 2, 0))  # Start position
            xform_api.SetRotate((0, 90, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
            camera_prim.GetHorizontalApertureAttr().Set(21)
            camera_prim.GetVerticalApertureAttr().Set(16)
            camera_prim.GetProjectionAttr().Set("perspective")
            camera_prim.GetFocalLengthAttr().Set(24)
            camera_prim.GetFocusDistanceAttr().Set(400)
            
            self.camera_prim = camera_prim
            
            simulation_app.update()
            print("✓ Camera created successfully")
            
            return True
            
        except Exception as e:
            print(f"ERROR: Failed to create camera: {e}")
            print(traceback.format_exc())
            return False
            
    def run_simulation(self):
        """Run the main simulation loop with camera rotation"""
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
            
            print("\n=== Starting camera rotation demo ===")
            print("🔄 Camera will rotate randomly 360° every second")
            print("📹 Watch the camera pan around the scene")
            print("Press Ctrl+C to stop\n")
            
            while not self.shutdown_requested:
                try:
                    # Rotate camera randomly every second
                    self.rotate_camera_randomly()
                    
                    # Step simulation with rendering
                    self.simulation_context.step(render=True)
                    simulation_app.update()
                    
                    step_count += 1
                    
                    # Print status every 100 steps
                    if step_count % 100 == 0:
                        elapsed = time.time() - start_time
                        fps = step_count / elapsed if elapsed > 0 else 0
                        print(f"Step {step_count:5d} | Elapsed: {elapsed:6.1f}s | FPS: {fps:5.1f} | Yaw: {self.current_yaw:6.1f}°")
                        
                    # Limit FPS 
                    time.sleep(0.016)  # ~60 FPS
                    
                    # Safety check for demo
                    if step_count > 5000:  # Stop after reasonable demo time
                        print(f"Demo completed ({step_count} steps), stopping...")
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
                        
            print(f"\nCamera rotation demo completed successfully. Total steps: {step_count}")
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
            
    def run(self):
        """Main execution method"""
        print("=== Isaac Sim 5.0 Camera Rotation Demo ===")
        print("Testing random 360° camera rotation every second\n")
        
        try:
            # Step 1: Initialize Isaac Sim
            if not self.initialize_isaac_sim():
                print("FAILED: Could not initialize Isaac Sim")
                return False
                
            # Step 2: Create camera
            if not self.create_camera():
                print("FAILED: Could not create camera")
                return False
                
            # Step 3: Run simulation with rotation
            return self.run_simulation()
            
        except Exception as e:
            print(f"FATAL ERROR: {e}")
            print(traceback.format_exc())
            return False

def main():
    """Main entry point"""
    node = CameraRotationDemo()
    success = node.run()
    
    print(f"\n=== Demo Status: {'SUCCESS' if success else 'FAILED'} ===")
    print("Camera rotation demo completed.")
    
    # Close simulation app
    try:
        simulation_app.close()
    except:
        pass
        
    return 0 if success else 1

if __name__ == "__main__":
    sys.exit(main())
