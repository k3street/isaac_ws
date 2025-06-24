#!/usr/bin/env python3
"""
Isaac Sim Controllable Camera Node
Combines camera publishing with ROS2 control interface
"""
import sys
import signal
import time
import traceback
import math

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
from pxr import Gf, UsdGeom, UsdLux

# ROS2 imports (after Isaac Sim initialization)
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32, Bool

class ControllableCameraNode:
    def __init__(self):
        self.simulation_context = None
        self.camera_prim = None
        self.ros_camera_graph = None
        self.ros_node = None
        self.shutdown_requested = False
        
        # Camera control state
        self.camera_position = [0.0, 0.0, 2.0]  # x, y, z
        self.camera_orientation = [0.0, 0.0, 0.0]  # roll, pitch, yaw
        self.camera_enabled = True
        self.last_update_time = time.time()
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
    def _signal_handler(self, signum, frame):
        print(f"Received signal {signum}, initiating shutdown...")
        self.shutdown_requested = True
        
    def initialize_isaac_sim(self):
        """Initialize Isaac Sim with controllable camera"""
        try:
            print("Initializing Isaac Sim simulation context...")
            
            # Enable ROS2 bridge extension
            print("Enabling isaacsim.ros2.bridge extension...")
            extensions.enable_extension("isaacsim.ros2.bridge")
            
            # Create simulation context
            self.simulation_context = SimulationContext(stage_units_in_meters=1.0)
            
            # Get the current stage
            stage_handle = omni.usd.get_context().get_stage()
            
            # Add lighting
            print("Setting up scene lighting...")
            UsdLux.DistantLight.Define(stage_handle, "/World/defaultLight")
            
            # Create controllable camera
            self.create_controllable_camera(stage_handle)
            
            # Initialize ROS2 node
            self.initialize_ros2()
            
            # Create ROS2 camera graph
            self.create_ros_camera_graph()
            
            print("✅ Isaac Sim controllable camera node initialized successfully!")
            return True
            
        except Exception as e:
            print(f"❌ Failed to initialize Isaac Sim: {str(e)}")
            traceback.print_exc()
            return False
    
    def create_controllable_camera(self, stage):
        """Create a camera that can be controlled via ROS2"""
        try:
            print("Creating controllable camera...")
            
            # Create camera prim
            camera_path = "/World/ControllableCamera"
            self.camera_prim = UsdGeom.Camera.Define(stage, camera_path)
            
            # Set initial camera properties
            self.camera_prim.CreateFocalLengthAttr(24.0)
            self.camera_prim.CreateFocusDistanceAttr(400.0)
            self.camera_prim.CreateFStopAttr(0.0)
            self.camera_prim.CreateHorizontalApertureAttr(20.955)
            self.camera_prim.CreateVerticalApertureAttr(15.2908)
            self.camera_prim.CreateClippingRangeAttr(Gf.Vec2f(0.1, 1000000.0))
            
            # Set initial position and orientation
            self.update_camera_transform()
            
            print(f"✅ Controllable camera created at {camera_path}")
            return True
            
        except Exception as e:
            print(f"❌ Failed to create camera: {str(e)}")
            traceback.print_exc()
            return False
    
    def initialize_ros2(self):
        """Initialize ROS2 node for camera control"""
        try:
            print("Initializing ROS2 node...")
            
            if not rclpy.ok():
                rclpy.init()
            
            self.ros_node = CameraControlNode(self)
            print("✅ ROS2 control node initialized")
            return True
            
        except Exception as e:
            print(f"❌ Failed to initialize ROS2: {str(e)}")
            traceback.print_exc()
            return False
    
    def create_ros_camera_graph(self):
        """Create ROS2 camera publishing graph"""
        try:
            print("Creating ROS2 camera graph...")
            
            # Create the graph
            graph_path = "/World/CameraGraph"
            (self.ros_camera_graph, nodes, _, _) = og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnTick", "omni.graph.action.OnTick"),
                        ("CameraHelper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                        ("CameraInfoHelper", "omni.isaac.ros2_bridge.ROS2CameraInfoHelper"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnTick.outputs:tick", "CameraHelper.inputs:execIn"),
                        ("OnTick.outputs:tick", "CameraInfoHelper.inputs:execIn"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("CameraHelper.inputs:cameraPath", "/World/ControllableCamera"),
                        ("CameraHelper.inputs:topicName", "/camera/rgb"),
                        ("CameraHelper.inputs:frameId", "camera_frame"),
                        ("CameraInfoHelper.inputs:cameraPath", "/World/ControllableCamera"),
                        ("CameraInfoHelper.inputs:topicName", "/camera/camera_info"),
                        ("CameraInfoHelper.inputs:frameId", "camera_frame"),
                    ],
                },
            )
            
            print("✅ ROS2 camera graph created successfully!")
            return True
            
        except Exception as e:
            print(f"❌ Failed to create ROS2 camera graph: {str(e)}")
            traceback.print_exc()
            return False
    
    def update_camera_transform(self):
        """Update camera position and orientation based on control state"""
        if self.camera_prim and self.camera_enabled:
            # Create transform matrix
            position = Gf.Vec3d(self.camera_position[0], self.camera_position[1], self.camera_position[2])
            
            # Convert Euler angles to rotation matrix
            roll, pitch, yaw = self.camera_orientation
            rotation = Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(yaw)) * \
                      Gf.Rotation(Gf.Vec3d(1, 0, 0), math.degrees(pitch)) * \
                      Gf.Rotation(Gf.Vec3d(0, 1, 0), math.degrees(roll))
            
            # Apply transform
            transform = Gf.Transform()
            transform.SetTranslation(position)
            transform.SetRotation(rotation)
            
            # Set the transform
            xform_prim = self.camera_prim.GetPrim()
            UsdGeom.XformCommonAPI(xform_prim).SetTransform(transform.GetMatrix())
    
    def run_simulation(self):
        """Main simulation loop with ROS2 integration"""
        print("🚀 Starting controllable camera simulation...")
        print("Use ROS2 topics to control the camera:")
        print("  /camera/cmd_vel - Velocity control")
        print("  /camera/set_pose - Position control")
        print("  /camera/set_fov - Field of view")
        print("  /camera/enable - Enable/disable")
        
        try:
            # Start simulation
            self.simulation_context.play()
            
            # Main loop
            while simulation_app.is_running() and not self.shutdown_requested:
                # Step simulation
                self.simulation_context.step(render=True)
                
                # Process ROS2 callbacks
                if self.ros_node:
                    rclpy.spin_once(self.ros_node, timeout_sec=0.001)
                
                # Update camera transform
                self.update_camera_transform()
                
                # Small delay to prevent excessive CPU usage
                time.sleep(0.01)
                
        except Exception as e:
            print(f"❌ Error during simulation: {str(e)}")
            traceback.print_exc()
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Cleanup resources"""
        print("🧹 Cleaning up controllable camera node...")
        
        if self.simulation_context:
            self.simulation_context.stop()
        
        if self.ros_node:
            self.ros_node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()
        
        if simulation_app:
            simulation_app.close()
        
        print("✅ Cleanup completed")


class CameraControlNode(Node):
    """ROS2 node for camera control"""
    
    def __init__(self, camera_node):
        super().__init__('isaac_camera_control')
        self.camera_node = camera_node
        
        # Create subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/camera/cmd_vel', self.cmd_vel_callback, 10)
        
        self.pose_sub = self.create_subscription(
            PoseStamped, '/camera/set_pose', self.pose_callback, 10)
        
        self.fov_sub = self.create_subscription(
            Float32, '/camera/set_fov', self.fov_callback, 10)
        
        self.enable_sub = self.create_subscription(
            Bool, '/camera/enable', self.enable_callback, 10)
        
        self.get_logger().info('Camera control node initialized')
    
    def cmd_vel_callback(self, msg):
        """Handle velocity commands"""
        if not self.camera_node.camera_enabled:
            return
        
        dt = 0.1  # Update rate
        
        # Update position
        self.camera_node.camera_position[0] += msg.linear.x * dt
        self.camera_node.camera_position[1] += msg.linear.y * dt
        self.camera_node.camera_position[2] += msg.linear.z * dt
        
        # Update orientation
        self.camera_node.camera_orientation[0] += msg.angular.x * dt
        self.camera_node.camera_orientation[1] += msg.angular.y * dt
        self.camera_node.camera_orientation[2] += msg.angular.z * dt
    
    def pose_callback(self, msg):
        """Handle pose commands"""
        if not self.camera_node.camera_enabled:
            return
        
        # Update position
        self.camera_node.camera_position[0] = msg.pose.position.x
        self.camera_node.camera_position[1] = msg.pose.position.y
        self.camera_node.camera_position[2] = msg.pose.position.z
        
        # Convert quaternion to Euler (simplified)
        # In a full implementation, use proper quaternion math
        qw = msg.pose.orientation.w
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        
        self.camera_node.camera_orientation[0] = math.atan2(2*(qw*qx + qy*qz), 1-2*(qx*qx + qy*qy))
        self.camera_node.camera_orientation[1] = math.asin(2*(qw*qy - qz*qx))
        self.camera_node.camera_orientation[2] = math.atan2(2*(qw*qz + qx*qy), 1-2*(qy*qy + qz*qz))
    
    def fov_callback(self, msg):
        """Handle FOV commands"""
        if self.camera_node.camera_prim:
            # Convert FOV to focal length (simplified)
            horizontal_aperture = 20.955  # mm
            focal_length = horizontal_aperture / (2 * math.tan(math.radians(msg.data / 2)))
            self.camera_node.camera_prim.CreateFocalLengthAttr(focal_length)
    
    def enable_callback(self, msg):
        """Handle enable/disable commands"""
        self.camera_node.camera_enabled = msg.data
        status = "enabled" if msg.data else "disabled"
        self.get_logger().info(f'Camera {status}')


def main():
    """Main function"""
    print("🎮 Starting Isaac Sim Controllable Camera Node")
    print("=" * 50)
    
    camera_node = ControllableCameraNode()
    
    try:
        if camera_node.initialize_isaac_sim():
            camera_node.run_simulation()
        else:
            print("❌ Failed to initialize Isaac Sim")
            return 1
            
    except KeyboardInterrupt:
        print("\n⏹️ Keyboard interrupt received")
    except Exception as e:
        print(f"❌ Unexpected error: {str(e)}")
        traceback.print_exc()
        return 1
    finally:
        camera_node.cleanup()
    
    print("👋 Controllable camera node finished")
    return 0


if __name__ == "__main__":
    sys.exit(main())
