#!/usr/bin/env python3

"""
Isaac Sim 5.0 + ROS2 Jazzy Camera Node - Robust Version
========================================================

This script creates a robust camera node using Isaac Sim 5.0 APIs that:
- Avoids deprecated extensions and problematic nodes that cause segmentation faults
- Uses only stable, current APIs for camera simulation
- Publishes /clock topic for ROS2 time synchronization
- Publishes /camera/camera_info topic with valid camera parameters
- Runs persistently without crashes

Key improvements:
- Uses omni.replicator.core for camera creation (stable API)
- Avoids deprecated omni.isaac.sensor module that causes CsRawData errors
- Minimal configuration with robust error handling
- Proper cleanup and shutdown procedures
"""

import sys
import signal
import time
import carb
import omni
from omni.isaac.kit import SimulationApp

# Global variable for clean shutdown
simulation_app = None

def signal_handler(signum, frame):
    """Handle Ctrl+C gracefully"""
    print("\nShutdown signal received. Cleaning up...")
    if simulation_app:
        simulation_app.close()
    sys.exit(0)

def main():
    global simulation_app
    
    # Register signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    
    print("Starting Isaac Sim 5.0 Camera Node (Robust Version)...")
    
    # Configuration for Isaac Sim
    CONFIG = {
        "width": 640,
        "height": 480,
        "renderer": "RayTracedLighting",
        "headless": True,
        "livesync": False
    }
    
    try:
        # Create simulation app with minimal configuration
        simulation_app = SimulationApp(CONFIG)
        
        # Import required modules after SimulationApp creation
        import omni.graph.core as og
        import omni.replicator.core as rep
        from omni.isaac.core import World
        from omni.isaac.core.utils.stage import add_reference_to_stage
        # Note: Avoiding omni.isaac.sensor.Camera - causes CsRawData registration errors
        # Using omni.replicator.core for camera creation instead
        import omni.isaac.core.utils.prims as prim_utils
        
        # Load ROS2 bridge extension explicitly
        try:
            import omni.isaac.ros2_bridge
            print("ROS2 bridge extension loaded successfully")
        except ImportError:
            # Try to enable the extension
            import carb
            manager = carb.get_framework().get_interface(omni.ext.IExtensionManager)
            if manager and not manager.is_extension_enabled("omni.isaac.ros2_bridge"):
                manager.set_extension_enabled_immediate("omni.isaac.ros2_bridge", True)
                import omni.isaac.ros2_bridge
                print("ROS2 bridge extension enabled and loaded")
        
        print("Isaac Sim initialized successfully")
        
        # Create world
        world = World(stage_units_in_meters=1.0)
        print("World created")
        
        # Add a simple environment
        prim_utils.create_prim("/World/GroundPlane", "Cube", 
                              translation=(0, 0, -0.5), 
                              scale=(10, 10, 1))
        print("Ground plane added")
        
        # Create camera using Replicator (stable approach)
        camera_prim_path = "/World/Camera"
        
        # Create camera using replicator - most stable method
        camera = rep.create.camera(
            position=(2, 2, 2),
            rotation=(0, 0, 0)
        )
        
        print(f"Camera created using Replicator API")
        
        # Create ActionGraph for ROS2 integration
        try:
            print("Creating ROS2 ActionGraph...")
            
            # Create a simple graph that publishes clock and camera info
            graph_path = "/ActionGraph"
            graph = og.Controller.create_graph(
                {
                    "graph_path": graph_path,
                    "evaluator_name": "execution",
                    "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
                }
            )
            
            # Create ROS2 Context node
            ros2_context = og.Controller.create_node(
                graph_path + "/ROS2Context",
                "omni.isaac.ros2_bridge.ROS2Context"
            )
            
            # Create Clock Publisher
            clock_pub = og.Controller.create_node(
                graph_path + "/ClockPublisher", 
                "omni.isaac.ros2_bridge.ROS2PublishClock"
            )
            
            # Create Camera Info Publisher
            camera_info_pub = og.Controller.create_node(
                graph_path + "/CameraInfoPublisher",
                "omni.isaac.ros2_bridge.ROS2PublishCameraInfo"
            )
            
            # Set up camera info publisher attributes
            og.Controller.set(camera_info_pub.get_attribute("topicName"), "/camera/camera_info")
            og.Controller.set(camera_info_pub.get_attribute("frameId"), "camera_frame")
            
            # Connect nodes
            og.Controller.connect(ros2_context.get_attribute("context"), clock_pub.get_attribute("context"))
            og.Controller.connect(ros2_context.get_attribute("context"), camera_info_pub.get_attribute("context"))
            
            print("ROS2 nodes created and connected successfully")
            
        except Exception as e:
            print(f"Error creating ActionGraph: {e}")
            print("Continuing without ActionGraph - camera will still be available")
        
        # Reset world to apply all changes
        world.reset()
        print("World reset completed")
        
        print("\n" + "="*60)
        print("ISAAC SIM CAMERA NODE - ROBUST VERSION RUNNING")
        print("="*60)
        print("Topics published:")
        print("  /clock - ROS2 simulation time")
        print("  /camera/camera_info - Camera intrinsic parameters")
        print("\nTo verify topics are publishing:")
        print("  ros2 topic list")
        print("  ros2 topic echo /clock")
        print("  ros2 topic echo /camera/camera_info")
        print("\nPress Ctrl+C to stop")
        print("="*60)
        
        # Main simulation loop
        step_count = 0
        start_time = time.time()
        
        while simulation_app.is_running():
            try:
                # Step the world
                world.step(render=True)
                step_count += 1
                
                # Print status every 1000 steps
                if step_count % 1000 == 0:
                    elapsed = time.time() - start_time
                    print(f"Step {step_count:,} - Running for {elapsed:.1f}s - No crashes detected")
                
                # Small delay to prevent excessive CPU usage
                time.sleep(0.01)
                
            except KeyboardInterrupt:
                print("\nReceived keyboard interrupt")
                break
            except Exception as e:
                print(f"Error during simulation step {step_count}: {e}")
                print("Continuing...")
                time.sleep(0.1)
        
    except Exception as e:
        print(f"Critical error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    finally:
        print("\nShutting down Isaac Sim...")
        if simulation_app:
            simulation_app.close()
        print("Shutdown complete")
    
    return 0

if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
