#!/usr/bin/env python3
"""
Isaac Sim Camera Node with ROS2 Control Integration
- Publishes camera data (RGB, Depth, CameraInfo) 
- Subscribes to ROS2 control commands
- Controls camera position/orientation in Isaac Sim
- NO automatic rotation - waits for ROS2 commands
"""
import sys
import signal
import time
import traceback
import math
import threading

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

# ROS2 imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Image, CameraInfo

class IsaacCameraControlNode(Node):
    def __init__(self):
        super().__init__('isaac_camera_control_node')
        
        # Isaac Sim components
        self.simulation_context = None
        self.camera_prim = None
        self.ros_camera_graph = None
        self.shutdown_requested = False
        
        # Camera state
        self.camera_position = [2.0, 2.0, 2.0]  # Initial position
        self.camera_orientation = [0.0, 0.0, 0.0]  # roll, pitch, yaw in radians
        self.camera_enabled = True
        
        # Movement parameters
        self.max_velocity = 2.0  # m/s
        self.max_angular_velocity = 1.0  # rad/s
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        # Create ROS2 subscribers for camera control
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/camera/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/camera/set_pose',
            self.pose_callback,
            10)
        
        self.fov_sub = self.create_subscription(
            Float32,
            '/camera/set_fov',
            self.fov_callback,
            10)
        
        self.enable_sub = self.create_subscription(
            Bool,
            '/camera/enable',
            self.enable_callback,
            10)
        
        # Timer for applying camera updates
        self.update_timer = self.create_timer(0.1, self.update_camera_in_sim)
        
        self.get_logger().info('Isaac Camera Control Node initialized')
        self.get_logger().info('Waiting for ROS2 control commands...')
        
    def _signal_handler(self, signum, frame):
        self.get_logger().info(f"Received signal {signum}, shutting down...")
        self.shutdown_requested = True
        
    def cmd_vel_callback(self, msg):
        """Handle velocity commands"""
        if not self.camera_enabled:
            return
            
        # Clamp velocities
        linear_x = max(-self.max_velocity, min(self.max_velocity, msg.linear.x))
        linear_y = max(-self.max_velocity, min(self.max_velocity, msg.linear.y))
        linear_z = max(-self.max_velocity, min(self.max_velocity, msg.linear.z))
        
        angular_x = max(-self.max_angular_velocity, min(self.max_angular_velocity, msg.angular.x))
        angular_y = max(-self.max_angular_velocity, min(self.max_angular_velocity, msg.angular.y))
        angular_z = max(-self.max_angular_velocity, min(self.max_angular_velocity, msg.angular.z))
        
        # Update camera state (integrate velocity)
        dt = 0.1  # Update rate
        self.camera_position[0] += linear_x * dt
        self.camera_position[1] += linear_y * dt
        self.camera_position[2] += linear_z * dt
        
        self.camera_orientation[0] += angular_x * dt  # roll
        self.camera_orientation[1] += angular_y * dt  # pitch
        self.camera_orientation[2] += angular_z * dt  # yaw
        
        # Normalize angles
        for i in range(3):
            while self.camera_orientation[i] > math.pi:
                self.camera_orientation[i] -= 2 * math.pi
            while self.camera_orientation[i] < -math.pi:
                self.camera_orientation[i] += 2 * math.pi
        
        self.get_logger().info(f'Camera moving: pos=({self.camera_position[0]:.2f}, {self.camera_position[1]:.2f}, {self.camera_position[2]:.2f})')
        
    def pose_callback(self, msg):
        """Handle absolute pose commands"""
        if not self.camera_enabled:
            return
            
        # Extract position
        self.camera_position[0] = msg.pose.position.x
        self.camera_position[1] = msg.pose.position.y
        self.camera_position[2] = msg.pose.position.z
        
        # Convert quaternion to Euler angles
        qw = msg.pose.orientation.w
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        
        # Convert to Euler angles (roll, pitch, yaw)
        self.camera_orientation[0] = math.atan2(2*(qw*qx + qy*qz), 1-2*(qx*qx + qy*qy))  # roll
        self.camera_orientation[1] = math.asin(2*(qw*qy - qz*qx))  # pitch
        self.camera_orientation[2] = math.atan2(2*(qw*qz + qx*qy), 1-2*(qy*qy + qz*qz))  # yaw
        
        self.get_logger().info(f'Camera pose set to: pos=({self.camera_position[0]:.2f}, {self.camera_position[1]:.2f}, {self.camera_position[2]:.2f})')
        
    def fov_callback(self, msg):
        """Handle FOV changes"""
        if not self.camera_enabled:
            return
        # TODO: Implement FOV control in Isaac Sim
        self.get_logger().info(f'FOV change requested: {msg.data:.1f}°')
        
    def enable_callback(self, msg):
        """Handle camera enable/disable"""
        self.camera_enabled = msg.data
        status = "enabled" if self.camera_enabled else "disabled"
        self.get_logger().info(f'Camera {status}')
        
    def update_camera_in_sim(self):
        """Apply camera position/orientation to Isaac Sim camera"""
        if not self.camera_prim or not self.camera_enabled:
            return
            
        try:
            # Get the camera's transform API
            xform_api = UsdGeom.XformCommonAPI(self.camera_prim)
            
            # Set position
            position = Gf.Vec3d(self.camera_position[0], self.camera_position[1], self.camera_position[2])
            xform_api.SetTranslate(position)
            
            # Set rotation (convert radians to degrees for USD)
            roll_deg = math.degrees(self.camera_orientation[0])
            pitch_deg = math.degrees(self.camera_orientation[1])
            yaw_deg = math.degrees(self.camera_orientation[2])
            
            xform_api.SetRotate((roll_deg, pitch_deg, yaw_deg), UsdGeom.XformCommonAPI.RotationOrderXYZ)
            
        except Exception as e:
            self.get_logger().warn(f"Failed to update camera in sim: {e}")

    def initialize_isaac_sim(self):
        """Initialize Isaac Sim simulation"""
        try:
            print("Initializing Isaac Sim simulation context...")
            
            # Enable ROS2 bridge extension
            print("Enabling isaacsim.ros2.bridge extension...")
            extensions.enable_extension("isaacsim.ros2.bridge")
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
            
            # Add some objects to view
            try:
                from omni.isaac.core.utils.stage import add_reference_to_stage
                cube_path = assets_root_path + "/Isaac/Props/Blocks/basic_block.usd"
                add_reference_to_stage(usd_path=cube_path, prim_path="/World/Cube1")
                
                cube_prim = omni.usd.get_context().get_stage().GetPrimAtPath("/World/Cube1")
                if cube_prim:
                    xform_api = UsdGeom.XformCommonAPI(cube_prim)
                    xform_api.SetTranslate(Gf.Vec3d(0, 0, 0.5))
                    xform_api.SetScale(Gf.Vec3f(0.5, 0.5, 0.5))
                    
            except Exception as e:
                print(f"Note: Could not add props: {e}")
            
            # Add lighting
            sphereLight = UsdLux.SphereLight.Define(omni.usd.get_context().get_stage(), "/World/SphereLight")
            sphereLight.CreateRadiusAttr(150)
            sphereLight.CreateIntensityAttr(30000)
            sphereLight.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
            
            print("✓ Isaac Sim simulation initialized")
            return True
            
        except Exception as e:
            print(f"FAILED to initialize Isaac Sim: {e}")
            traceback.print_exc()
            return False

    def create_camera_and_ros_graph(self):
        """Create camera and ROS2 publishing graph"""
        try:
            print("Creating camera and ROS2 publishing graph...")
            
            # Create camera prim - STATIONARY by default
            camera_prim_path = "/World/Camera"
            camera_prim = UsdGeom.Camera.Define(omni.usd.get_context().get_stage(), camera_prim_path)
            self.camera_prim = camera_prim.GetPrim()
            
            # Set initial camera position - FIXED until ROS2 commands
            xform_api = UsdGeom.XformCommonAPI(self.camera_prim)
            xform_api.SetTranslate(Gf.Vec3d(self.camera_position[0], self.camera_position[1], self.camera_position[2]))
            xform_api.SetRotate((0, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
            
            # Set camera properties
            camera_prim.GetAttribute('focalLength').Set(24.0)
            camera_prim.GetAttribute('focusDistance').Set(400.0)
            camera_prim.GetAttribute('horizontalAperture').Set(20.955)
            camera_prim.GetAttribute('verticalAperture').Set(15.2908)
            camera_prim.GetAttribute('clippingRange').Set(Gf.Vec2f(0.1, 1000000.0))
            
            # Create render product for the camera
            render_product_path = omni.syntheticdata.SyntheticData.convert_sensor_type_to_renderproduct(camera_prim_path)
            
            # Create ROS2 camera graph
            try:
                ros_camera_graph_path = "/ActionGraph"
                
                (self.ros_camera_graph, _, _, _) = og.Controller.edit(
                    {"graph_path": ros_camera_graph_path, "evaluator_name": "execution"},
                    {
                        og.Controller.Keys.CREATE_NODES: [
                            ("onPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                            ("cameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                            ("cameraHelperInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                            ("cameraHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                        ],
                        og.Controller.Keys.SET_VALUES: [
                            ("cameraHelperRgb.inputs:type", "rgb"),
                            ("cameraHelperRgb.inputs:topicName", "/camera/rgb"),
                            ("cameraHelperRgb.inputs:frameId", "camera_frame"),
                            ("cameraHelperRgb.inputs:renderProductPath", render_product_path),
                            
                            ("cameraHelperInfo.inputs:topicName", "/camera/camera_info"),
                            ("cameraHelperInfo.inputs:frameId", "camera_frame"),  
                            ("cameraHelperInfo.inputs:renderProductPath", render_product_path),
                            
                            ("cameraHelperDepth.inputs:type", "depth"),
                            ("cameraHelperDepth.inputs:topicName", "/camera/depth"),
                            ("cameraHelperDepth.inputs:frameId", "camera_frame"),
                            ("cameraHelperDepth.inputs:renderProductPath", render_product_path),
                        ],
                        og.Controller.Keys.CONNECT: [
                            ("onPlaybackTick.outputs:tick", "cameraHelperRgb.inputs:execIn"),
                            ("onPlaybackTick.outputs:tick", "cameraHelperInfo.inputs:execIn"),
                            ("onPlaybackTick.outputs:tick", "cameraHelperDepth.inputs:execIn"),
                        ],
                    },
                )
                
                print("✓ Camera and ROS2 graph created successfully")
                return True
                
            except Exception as e:
                print(f"FAILED to create ROS2 camera graph: {e}")
                traceback.print_exc()
                return False
                
        except Exception as e:
            print(f"FAILED to create camera: {e}")
            traceback.print_exc()
            return False

    def start_simulation(self):
        """Start the simulation"""
        try:
            print("Starting simulation...")
            self.simulation_context.play()
            
            # Wait for simulation to be ready
            simulation_app.update()
            time.sleep(1.0)
            
            print("✓ Simulation started successfully")
            print("📷 Camera is STATIONARY - waiting for ROS2 control commands")
            print("\nPublishing ROS2 topics:")
            print("  /camera/rgb - RGB images")
            print("  /camera/depth - Depth images") 
            print("  /camera/camera_info - Camera information")
            print("\nListening for ROS2 control topics:")
            print("  /camera/cmd_vel - Velocity commands")
            print("  /camera/set_pose - Position commands")
            print("  /camera/set_fov - Field of view")
            print("  /camera/enable - Enable/disable camera")
            
            return True
            
        except Exception as e:
            print(f"FAILED to start simulation: {e}")
            traceback.print_exc()
            return False

    def run(self):
        """Main execution method"""
        print("=== Isaac Sim Camera with ROS2 Control ===")
        print("Camera will NOT auto-rotate - waiting for ROS2 commands\n")
        
        try:
            # Step 1: Initialize Isaac Sim
            if not self.initialize_isaac_sim():
                print("FAILED: Could not initialize Isaac Sim")
                return False
                
            # Step 2: Create camera and ROS2 publishing graph
            if not self.create_camera_and_ros_graph():
                print("FAILED: Could not create camera and ROS2 graph")
                return False
                
            # Step 3: Start simulation
            if not self.start_simulation():
                print("FAILED: Could not start simulation")
                return False
            
            print("\n🎯 SUCCESS: Isaac Sim camera node ready for ROS2 control!")
            print("💡 Test with: ros2 topic pub /camera/cmd_vel geometry_msgs/msg/Twist ...")
            
            # Main loop - keep simulation running and handle ROS2
            while not self.shutdown_requested:
                simulation_app.update()
                rclpy.spin_once(self, timeout_sec=0.01)
                time.sleep(0.01)
                
            return True
            
        except Exception as e:
            print(f"FAILED: Unexpected error: {e}")
            traceback.print_exc()
            return False
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
            print(f"Warning during cleanup: {e}")


def main():
    """Main function"""
    # Initialize ROS2
    rclpy.init()
    
    # Create and run the camera node
    camera_node = IsaacCameraControlNode()
    
    # Run Isaac Sim in separate thread
    def run_isaac_sim():
        success = camera_node.run()
        if not success:
            print("Isaac Sim node failed to start properly")
    
    isaac_thread = threading.Thread(target=run_isaac_sim, daemon=True)
    isaac_thread.start()
    
    try:
        # Keep ROS2 node alive
        time.sleep(2)  # Wait for Isaac Sim to initialize
        print("ROS2 node spinning...")
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        print("Keyboard interrupt received")
    finally:
        camera_node.shutdown_requested = True
        camera_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
