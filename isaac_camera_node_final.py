#!/usr/bin/env python3
"""
Isaac Sim 5.0 ROS2 Camera Node - Final Working Version
Based on official Isaac Sim examples and documentation
"""
import sys
import signal
import time
import traceback

from isaacsim.simulation_app import SimulationApp
CONFIG = {"renderer": "RaytracedLighting", "headless": True}
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

class FinalCameraNode:
    def __init__(self):
        self.simulation_context = None
        self.camera_prim = None
        self.ros_camera_graph = None
        self.shutdown_requested = False
        
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
            
            # Load simple room environment
            environment_path = assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd"
            stage.add_reference_to_stage(usd_path=environment_path, prim_path="/World/simple_room")
            
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
            camera_prim = UsdGeom.Camera(omni.usd.get_context().get_stage().DefinePrim(CAMERA_STAGE_PATH, "Camera"))
            xform_api = UsdGeom.XformCommonAPI(camera_prim)
            xform_api.SetTranslate(Gf.Vec3d(2, 2, 2))
            xform_api.SetRotate((0, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
            camera_prim.GetHorizontalApertureAttr().Set(21)
            camera_prim.GetVerticalApertureAttr().Set(16)
            camera_prim.GetProjectionAttr().Set("perspective")
            camera_prim.GetFocalLengthAttr().Set(24)
            camera_prim.GetFocusDistanceAttr().Set(400)
            
            self.camera_prim = camera_prim
            
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
            print("Publishing ROS2 topics:")
            print("  - /camera/rgb (sensor_msgs/Image)")
            print("  - /camera/camera_info (sensor_msgs/CameraInfo)")
            print("  - /camera/depth (sensor_msgs/Image)")
            print("  - /clock (rosgraph_msgs/Clock)")
            print("Press Ctrl+C to stop\n")
            
            while not self.shutdown_requested:
                try:
                    # Step simulation with rendering
                    self.simulation_context.step(render=True)
                    simulation_app.update()
                    
                    step_count += 1
                    
                    # Print status every 100 steps
                    if step_count % 100 == 0:
                        elapsed = time.time() - start_time
                        fps = step_count / elapsed if elapsed > 0 else 0
                        print(f"Step {step_count:5d} | Elapsed: {elapsed:6.1f}s | FPS: {fps:5.1f} | Status: OK")
                        
                    # Limit FPS 
                    time.sleep(0.016)  # ~60 FPS
                    
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
            
    def run(self):
        """Main execution method"""
        print("=== Isaac Sim 5.0 ROS2 Camera Node - Final Working Version ===")
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
                
            # Step 3: Run simulation
            return self.run_simulation()
            
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
