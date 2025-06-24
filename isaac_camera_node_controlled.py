#!/usr/bin/env python3
"""
Isaac Sim Camera Control Node - Fixed Version
- NO automatic rotation
- ROS2 control integration
- Camera stays stationary until ROS2 commands received
"""
import sys
import signal
import time
import traceback
import math
import threading

# Isaac Sim imports
from isaacsim.simulation_app import SimulationApp
CONFIG = {"renderer": "RaytracedLighting", "headless": False}
simulation_app = SimulationApp(CONFIG)

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
from std_msgs.msg import Float32, Bool, String


class IsaacCameraControlNode(Node):
    def __init__(self):
        super().__init__('isaac_camera_control')
        
        # Isaac Sim components
        self.simulation_context = None
        self.camera_prim = None
        self.ros_camera_graph = None
        self.shutdown_requested = False
        
        # Camera state - FIXED position initially
        self.camera_position = [2.0, 2.0, 2.0]  # x, y, z
        self.camera_orientation = [0.0, 0.0, 0.0]  # roll, pitch, yaw (radians)
        self.camera_enabled = True
        
        # Movement parameters
        self.max_velocity = 2.0  # m/s
        self.max_angular_velocity = 1.0  # rad/s
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        # Create ROS2 subscribers for camera control
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/camera/cmd_vel', self.cmd_vel_callback, 10)
        
        self.pose_sub = self.create_subscription(
            PoseStamped, '/camera/set_pose', self.pose_callback, 10)
        
        self.fov_sub = self.create_subscription(
            Float32, '/camera/set_fov', self.fov_callback, 10)
        
        self.enable_sub = self.create_subscription(
            Bool, '/camera/enable', self.enable_callback, 10)
        
        # Create publishers for feedback
        self.state_pub = self.create_publisher(String, '/camera/state', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/camera/current_pose', 10)
        
        # Timer for updating camera and publishing state
        self.update_timer = self.create_timer(0.1, self.update_camera)
        self.state_timer = self.create_timer(1.0, self.publish_state)
        
        self.get_logger().info('🎥 Isaac Sim Camera Control Node Started')
        self.get_logger().info('📷 Camera is STATIONARY - waiting for ROS2 commands')
        
    def _signal_handler(self, signum, frame):
        self.get_logger().info(f"Received signal {signum}, shutting down...")
        self.shutdown_requested = True
        
    def cmd_vel_callback(self, msg):
        """Handle velocity commands"""
        if not self.camera_enabled:
            self.get_logger().warn('Camera disabled - ignoring velocity command')
            return
            
        # Clamp velocities
        linear_x = max(-self.max_velocity, min(self.max_velocity, msg.linear.x))
        linear_y = max(-self.max_velocity, min(self.max_velocity, msg.linear.y))
        linear_z = max(-self.max_velocity, min(self.max_velocity, msg.linear.z))
        
        angular_x = max(-self.max_angular_velocity, min(self.max_angular_velocity, msg.angular.x))
        angular_y = max(-self.max_angular_velocity, min(self.max_angular_velocity, msg.angular.y))
        angular_z = max(-self.max_angular_velocity, min(self.max_angular_velocity, msg.angular.z))
        
        # Update camera state (integrate velocity)
        dt = 0.1
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
        
        self.get_logger().info(f'📹 Camera moving: pos=({self.camera_position[0]:.2f}, {self.camera_position[1]:.2f}, {self.camera_position[2]:.2f})')
        
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
        
        self.camera_orientation[0] = math.atan2(2*(qw*qx + qy*qz), 1-2*(qx*qx + qy*qy))
        self.camera_orientation[1] = math.asin(2*(qw*qy - qz*qx))
        self.camera_orientation[2] = math.atan2(2*(qw*qz + qx*qy), 1-2*(qy*qy + qz*qz))
        
        self.get_logger().info(f'🎯 Camera pose set: pos=({self.camera_position[0]:.2f}, {self.camera_position[1]:.2f}, {self.camera_position[2]:.2f})')
        
    def fov_callback(self, msg):
        """Handle FOV changes"""
        if not self.camera_enabled:
            return
        self.get_logger().info(f'🔍 FOV change requested: {msg.data:.1f}°')
        
    def enable_callback(self, msg):
        """Handle camera enable/disable"""
        self.camera_enabled = msg.data
        status = "enabled" if self.camera_enabled else "disabled"
        self.get_logger().info(f'Camera {status}')
        
    def update_camera(self):
        """Apply camera position/orientation to Isaac Sim"""
        if not self.camera_prim or not self.camera_enabled:
            return
            
        try:
            xform_api = UsdGeom.XformCommonAPI(self.camera_prim)
            
            # Set position
            position = Gf.Vec3d(self.camera_position[0], self.camera_position[1], self.camera_position[2])
            xform_api.SetTranslate(position)
            
            # Set rotation (convert radians to degrees)
            roll_deg = math.degrees(self.camera_orientation[0])
            pitch_deg = math.degrees(self.camera_orientation[1])
            yaw_deg = math.degrees(self.camera_orientation[2])
            
            xform_api.SetRotate((roll_deg, pitch_deg, yaw_deg), UsdGeom.XformCommonAPI.RotationOrderXYZ)
            
        except Exception as e:
            self.get_logger().warn(f"Failed to update camera: {e}")
            
    def publish_state(self):
        """Publish current camera state"""
        state_msg = String()
        state_msg.data = (f"Camera State - "
                         f"Pos: [{self.camera_position[0]:.2f}, {self.camera_position[1]:.2f}, {self.camera_position[2]:.2f}], "
                         f"Rot: [{math.degrees(self.camera_orientation[0]):.1f}°, {math.degrees(self.camera_orientation[1]):.1f}°, {math.degrees(self.camera_orientation[2]):.1f}°], "
                         f"Enabled: {self.camera_enabled}")
        
        self.state_pub.publish(state_msg)
        
        # Publish current pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "camera_frame"
        
        pose_msg.pose.position.x = self.camera_position[0]
        pose_msg.pose.position.y = self.camera_position[1]
        pose_msg.pose.position.z = self.camera_position[2]
        
        # Convert Euler to quaternion
        roll, pitch, yaw = self.camera_orientation
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        pose_msg.pose.orientation.w = cy * cp * cr + sy * sp * sr
        pose_msg.pose.orientation.x = cy * cp * sr - sy * sp * cr
        pose_msg.pose.orientation.y = sy * cp * sr + cy * sp * cr
        pose_msg.pose.orientation.z = sy * cp * cr - cy * sp * sr
        
        self.pose_pub.publish(pose_msg)


class IsaacSimCameraPublisher:
    def __init__(self, ros_node):
        self.ros_node = ros_node
        self.simulation_context = None
        self.camera_prim = None
        self.ros_camera_graph = None
        
    def initialize_isaac_sim(self):
        """Initialize Isaac Sim simulation"""
        try:
            print("🔧 Initializing Isaac Sim...")
            
            # Enable ROS2 bridge extension
            extensions.enable_extension("isaacsim.ros2.bridge")
            simulation_app.update()
            
            # Create simulation context
            self.simulation_context = SimulationContext(stage_units_in_meters=1.0)
            
            # Get assets path
            assets_root_path = get_assets_root_path()
            if assets_root_path is None:
                raise RuntimeError("Could not find Isaac Sim assets folder")
            
            print(f"Using assets root path: {assets_root_path}")
            
            # Load environment
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
            
            print("✅ Isaac Sim simulation initialized")
            return True
            
        except Exception as e:
            print(f"❌ Failed to initialize Isaac Sim: {e}")
            traceback.print_exc()
            return False

    def create_camera_and_ros_graph(self):
        """Create camera and ROS2 publishing graph"""
        try:
            print("📷 Creating camera and ROS2 graph...")
            
            # Create camera prim - STATIONARY initially
            camera_prim_path = "/World/Camera"
            camera_prim = UsdGeom.Camera.Define(omni.usd.get_context().get_stage(), camera_prim_path)
            self.camera_prim = camera_prim.GetPrim()
            
            # Set camera in ROS node so it can be controlled
            self.ros_node.camera_prim = self.camera_prim
            
            # Set initial camera position - FIXED until ROS2 commands
            xform_api = UsdGeom.XformCommonAPI(self.camera_prim)
            xform_api.SetTranslate(Gf.Vec3d(2.0, 2.0, 2.0))
            xform_api.SetRotate((0, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
            
            # Set camera properties
            camera_prim.GetAttribute('focalLength').Set(24.0)
            camera_prim.GetAttribute('focusDistance').Set(400.0)
            camera_prim.GetAttribute('horizontalAperture').Set(20.955)
            camera_prim.GetAttribute('verticalAperture').Set(15.2908)
            camera_prim.GetAttribute('clippingRange').Set(Gf.Vec2f(0.1, 1000000.0))
            
            # Create render product
            render_product_path = omni.syntheticdata.SyntheticData.convert_sensor_type_to_renderproduct(camera_prim_path)
            
            # Create ROS2 camera graph
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
            
            print("✅ Camera and ROS2 graph created")
            return True
            
        except Exception as e:
            print(f"❌ Failed to create camera/ROS2 graph: {e}")
            traceback.print_exc()
            return False

    def start_simulation(self):
        """Start the simulation"""
        try:
            print("▶️  Starting simulation...")
            self.simulation_context.play()
            
            simulation_app.update()
            time.sleep(1.0)
            
            print("✅ Simulation started")
            print("📷 Camera is STATIONARY - waiting for ROS2 control commands")
            print("\n🎮 Published topics:")
            print("  /camera/rgb - RGB images")
            print("  /camera/depth - Depth images")
            print("  /camera/camera_info - Camera info")
            print("\n🕹️  Control topics:")
            print("  /camera/cmd_vel - Velocity commands")
            print("  /camera/set_pose - Position commands")
            print("  /camera/enable - Enable/disable")
            
            return True
            
        except Exception as e:
            print(f"❌ Failed to start simulation: {e}")
            traceback.print_exc()
            return False

    def cleanup(self):
        """Clean up resources"""
        try:
            if self.simulation_context:
                self.simulation_context.stop()
            simulation_app.close()
        except Exception as e:
            print(f"Warning during cleanup: {e}")


def main():
    """Main function"""
    print("=" * 60)
    print("🎥 ISAAC SIM CAMERA WITH ROS2 CONTROL")
    print("=" * 60)
    print("Camera will NOT auto-rotate - controlled by ROS2 only")
    print("")
    
    # Initialize ROS2
    rclpy.init()
    
    # Create ROS2 node
    ros_node = IsaacCameraControlNode()
    
    # Create Isaac Sim publisher
    isaac_publisher = IsaacSimCameraPublisher(ros_node)
    
    try:
        # Initialize Isaac Sim
        if not isaac_publisher.initialize_isaac_sim():
            print("❌ Failed to initialize Isaac Sim")
            return False
            
        # Create camera and ROS2 graph
        if not isaac_publisher.create_camera_and_ros_graph():
            print("❌ Failed to create camera/ROS2 graph")
            return False
            
        # Start simulation
        if not isaac_publisher.start_simulation():
            print("❌ Failed to start simulation")
            return False
        
        print("\n🎉 SUCCESS: Isaac Sim ready for ROS2 control!")
        print("💡 Test with: ros2 topic pub /camera/cmd_vel geometry_msgs/msg/Twist ...")
        
        # Main loop
        while not ros_node.shutdown_requested:
            simulation_app.update()
            rclpy.spin_once(ros_node, timeout_sec=0.01)
            time.sleep(0.01)
            
        return True
        
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        traceback.print_exc()
        return False
    finally:
        isaac_publisher.cleanup()
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
