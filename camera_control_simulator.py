#!/usr/bin/env python3
"""
Standalone Camera Control Simulator
Simulates camera movement without Isaac Sim for testing ROS2 control interface
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32, Bool, String
import math
import time
import threading
import numpy as np

class CameraControlSimulator(Node):
    """
    Simulates camera control for testing ROS2 interface without Isaac Sim
    """
    
    def __init__(self):
        super().__init__('camera_control_simulator')
        
        # Camera state
        self.camera_position = [2.0, 2.0, 2.0]  # x, y, z
        self.camera_orientation = [0.0, 0.0, 0.0]  # roll, pitch, yaw (radians)
        self.camera_fov = 60.0  # degrees
        self.camera_enabled = True
        
        # Movement parameters
        self.max_velocity = 2.0  # m/s
        self.max_angular_velocity = 1.0  # rad/s
        
        # Create subscribers for camera control
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
        
        # Create publishers for camera data simulation
        self.rgb_pub = self.create_publisher(Image, '/camera/rgb', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.state_pub = self.create_publisher(String, '/camera/state', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/camera/current_pose', 10)
        
        # Timers
        self.update_timer = self.create_timer(0.1, self.update_camera)  # 10 Hz
        self.state_timer = self.create_timer(1.0, self.publish_state)   # 1 Hz
        self.image_timer = self.create_timer(0.033, self.publish_simulated_images)  # 30 fps
        
        self.get_logger().info('🎥 Camera Control Simulator Started')
        self.get_logger().info('📷 Camera is STATIONARY - waiting for ROS2 commands')
        self.get_logger().info('')
        self.get_logger().info('Available control topics:')
        self.get_logger().info('  /camera/cmd_vel - Velocity commands')
        self.get_logger().info('  /camera/set_pose - Position commands')
        self.get_logger().info('  /camera/set_fov - Field of view')
        self.get_logger().info('  /camera/enable - Enable/disable camera')
        self.get_logger().info('')
        self.get_logger().info('Published data topics:')
        self.get_logger().info('  /camera/rgb - Simulated RGB images')
        self.get_logger().info('  /camera/depth - Simulated depth images')
        self.get_logger().info('  /camera/camera_info - Camera parameters')
        self.get_logger().info('  /camera/state - Camera status')
        
    def cmd_vel_callback(self, msg):
        """Handle velocity commands"""
        if not self.camera_enabled:
            self.get_logger().warn('Camera is disabled - ignoring velocity command')
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
        
        self.get_logger().info(f'📹 Camera moving: pos=({self.camera_position[0]:.2f}, {self.camera_position[1]:.2f}, {self.camera_position[2]:.2f}) | '
                              f'rot=({math.degrees(self.camera_orientation[0]):.1f}°, {math.degrees(self.camera_orientation[1]):.1f}°, {math.degrees(self.camera_orientation[2]):.1f}°)')
        
    def pose_callback(self, msg):
        """Handle absolute pose commands"""
        if not self.camera_enabled:
            self.get_logger().warn('Camera is disabled - ignoring pose command')
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
        
        self.get_logger().info(f'🎯 Camera pose set: pos=({self.camera_position[0]:.2f}, {self.camera_position[1]:.2f}, {self.camera_position[2]:.2f}) | '
                              f'rot=({math.degrees(self.camera_orientation[0]):.1f}°, {math.degrees(self.camera_orientation[1]):.1f}°, {math.degrees(self.camera_orientation[2]):.1f}°)')
        
    def fov_callback(self, msg):
        """Handle FOV changes"""
        if not self.camera_enabled:
            return
            
        old_fov = self.camera_fov
        self.camera_fov = max(10.0, min(120.0, msg.data))
        self.get_logger().info(f'🔍 Camera FOV changed: {old_fov:.1f}° → {self.camera_fov:.1f}°')
        
    def enable_callback(self, msg):
        """Handle camera enable/disable"""
        self.camera_enabled = msg.data
        status = "🟢 enabled" if self.camera_enabled else "🔴 disabled"
        self.get_logger().info(f'Camera {status}')
        
    def update_camera(self):
        """Update camera - this would control Isaac Sim in real implementation"""
        # In a real implementation, this would send commands to Isaac Sim
        pass
        
    def publish_state(self):
        """Publish current camera state"""
        # Create state message
        state_msg = String()
        state_msg.data = (f"Camera State - "
                         f"Pos: [{self.camera_position[0]:.2f}, {self.camera_position[1]:.2f}, {self.camera_position[2]:.2f}], "
                         f"Rot: [{math.degrees(self.camera_orientation[0]):.1f}°, {math.degrees(self.camera_orientation[1]):.1f}°, {math.degrees(self.camera_orientation[2]):.1f}°], "
                         f"FOV: {self.camera_fov:.1f}°, "
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
        
    def publish_simulated_images(self):
        """Publish simulated camera images"""
        if not self.camera_enabled:
            return
            
        # Create simulated RGB image (colorful gradient based on camera position)
        rgb_msg = Image()
        rgb_msg.header.stamp = self.get_clock().now().to_msg()
        rgb_msg.header.frame_id = "camera_frame"
        rgb_msg.height = 480
        rgb_msg.width = 640
        rgb_msg.encoding = "rgb8"
        rgb_msg.step = rgb_msg.width * 3
        
        # Create gradient based on camera position
        rgb_data = np.zeros((rgb_msg.height, rgb_msg.width, 3), dtype=np.uint8)
        pos_factor = (self.camera_position[0] + 10) / 20.0  # Normalize position
        rgb_data[:, :, 0] = int(255 * min(1.0, max(0.0, pos_factor)))  # Red
        rgb_data[:, :, 1] = int(255 * min(1.0, max(0.0, (self.camera_position[1] + 10) / 20.0)))  # Green  
        rgb_data[:, :, 2] = int(255 * min(1.0, max(0.0, (self.camera_position[2] + 10) / 20.0)))  # Blue
        
        rgb_msg.data = rgb_data.flatten().tolist()
        self.rgb_pub.publish(rgb_msg)
        
        # Create simulated depth image
        depth_msg = Image()
        depth_msg.header.stamp = rgb_msg.header.stamp
        depth_msg.header.frame_id = "camera_frame"
        depth_msg.height = 480
        depth_msg.width = 640
        depth_msg.encoding = "32FC1"
        depth_msg.step = depth_msg.width * 4
        
        # Simple depth pattern
        depth_data = np.ones((depth_msg.height, depth_msg.width), dtype=np.float32) * 5.0
        depth_msg.data = depth_data.flatten().tobytes()
        self.depth_pub.publish(depth_msg)
        
        # Publish camera info
        info_msg = CameraInfo()
        info_msg.header.stamp = rgb_msg.header.stamp
        info_msg.header.frame_id = "camera_frame"
        info_msg.height = 480
        info_msg.width = 640
        info_msg.distortion_model = "plumb_bob"
        focal_length = 640.0 / (2.0 * math.tan(math.radians(self.camera_fov / 2.0)))
        info_msg.k = [focal_length, 0.0, 320.0, 0.0, focal_length, 240.0, 0.0, 0.0, 1.0]
        info_msg.p = [focal_length, 0.0, 320.0, 0.0, 0.0, focal_length, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        self.camera_info_pub.publish(info_msg)


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    print("=" * 60)
    print("🎥 CAMERA CONTROL SIMULATOR")
    print("=" * 60)
    print("This simulates camera control without Isaac Sim")
    print("Camera starts STATIONARY - use ROS2 commands to control")
    print("")
    
    camera_sim = CameraControlSimulator()
    
    try:
        print("🚀 Simulator ready! Try these commands in another terminal:")
        print("")
        print("# Move camera forward:")
        print("ros2 topic pub --once /camera/cmd_vel geometry_msgs/msg/Twist '{\"linear\": {\"x\": 0.5}}'")
        print("")
        print("# Rotate camera:")
        print("ros2 topic pub --once /camera/cmd_vel geometry_msgs/msg/Twist '{\"angular\": {\"z\": 0.5}}'")
        print("")
        print("# Set specific position:")
        print("ros2 topic pub --once /camera/set_pose geometry_msgs/msg/PoseStamped '{\"pose\": {\"position\": {\"x\": 3.0, \"y\": 3.0, \"z\": 3.0}}}'")
        print("")
        print("# Stop camera:")
        print("ros2 topic pub --once /camera/cmd_vel geometry_msgs/msg/Twist '{}'")
        print("")
        print("# Monitor camera state:")
        print("ros2 topic echo /camera/state")
        print("")
        print("=" * 60)
        
        rclpy.spin(camera_sim)
    except KeyboardInterrupt:
        camera_sim.get_logger().info('🛑 Shutting down camera control simulator')
    finally:
        camera_sim.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
