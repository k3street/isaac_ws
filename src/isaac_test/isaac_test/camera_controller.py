#!/usr/bin/env python3
"""
Isaac Sim Camera Controller Node

This node provides ROS2 control interface for Isaac Sim camera:
- Position control (x, y, z)
- Orientation control (roll, pitch, yaw)
- Camera parameter control (FOV, focus distance)
- Real-time command response
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32, String, Bool
import math


class CameraController(Node):
    """
    ROS2 node that provides control interface for Isaac Sim camera.
    """

    def __init__(self):
        super().__init__('isaac_camera_controller')
        
        # Camera state variables
        self.camera_position = [0.0, 0.0, 2.0]  # x, y, z
        self.camera_orientation = [0.0, 0.0, 0.0]  # roll, pitch, yaw (radians)
        self.camera_fov = 60.0  # Field of view in degrees
        self.camera_focus_distance = 5.0  # Focus distance in meters
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
        
        self.focus_sub = self.create_subscription(
            Float32,
            '/camera/set_focus',
            self.focus_callback,
            10)
        
        self.enable_sub = self.create_subscription(
            Bool,
            '/camera/enable',
            self.enable_callback,
            10)
        
        # Create publishers for camera feedback
        self.state_pub = self.create_publisher(
            String,  # Will use custom message later
            '/camera/state',
            10)
        
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/camera/current_pose',
            10)
        
        # Create timer for regular updates
        self.update_timer = self.create_timer(0.1, self.update_camera)  # 10 Hz
        self.state_timer = self.create_timer(1.0, self.publish_state)   # 1 Hz
        
        self.get_logger().info('Isaac Camera Controller Node started')
        self.get_logger().info('Subscribed to:')
        self.get_logger().info('  /camera/cmd_vel - Velocity commands')
        self.get_logger().info('  /camera/set_pose - Position commands')
        self.get_logger().info('  /camera/set_fov - Field of view')
        self.get_logger().info('  /camera/set_focus - Focus distance')
        self.get_logger().info('  /camera/enable - Enable/disable camera')

    def cmd_vel_callback(self, msg):
        """
        Handle velocity commands for camera movement.
        
        Args:
            msg (geometry_msgs.msg.Twist): Velocity command
        """
        if not self.camera_enabled:
            return
            
        # Linear velocities (m/s)
        linear_x = max(-self.max_velocity, min(self.max_velocity, msg.linear.x))
        linear_y = max(-self.max_velocity, min(self.max_velocity, msg.linear.y))
        linear_z = max(-self.max_velocity, min(self.max_velocity, msg.linear.z))
        
        # Angular velocities (rad/s)
        angular_x = max(-self.max_angular_velocity, min(self.max_angular_velocity, msg.angular.x))
        angular_y = max(-self.max_angular_velocity, min(self.max_angular_velocity, msg.angular.y))
        angular_z = max(-self.max_angular_velocity, min(self.max_angular_velocity, msg.angular.z))
        
        # Update camera position (integrate velocity)
        dt = 0.1  # Update rate
        self.camera_position[0] += linear_x * dt
        self.camera_position[1] += linear_y * dt
        self.camera_position[2] += linear_z * dt
        
        # Update camera orientation (integrate angular velocity)
        self.camera_orientation[0] += angular_x * dt  # roll
        self.camera_orientation[1] += angular_y * dt  # pitch
        self.camera_orientation[2] += angular_z * dt  # yaw
        
        # Normalize angles to [-pi, pi]
        for i in range(3):
            while self.camera_orientation[i] > math.pi:
                self.camera_orientation[i] -= 2 * math.pi
            while self.camera_orientation[i] < -math.pi:
                self.camera_orientation[i] += 2 * math.pi
        
        self.get_logger().debug(f'Camera velocity: linear=({linear_x:.2f}, {linear_y:.2f}, {linear_z:.2f}), '
                               f'angular=({angular_x:.2f}, {angular_y:.2f}, {angular_z:.2f})')

    def pose_callback(self, msg):
        """
        Handle absolute pose commands for camera positioning.
        
        Args:
            msg (geometry_msgs.msg.PoseStamped): Target pose
        """
        if not self.camera_enabled:
            return
            
        # Extract position
        self.camera_position[0] = msg.pose.position.x
        self.camera_position[1] = msg.pose.position.y
        self.camera_position[2] = msg.pose.position.z
        
        # Convert quaternion to Euler angles
        # Simplified conversion (for full implementation, use proper quaternion math)
        qw = msg.pose.orientation.w
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        
        # Convert to Euler angles (roll, pitch, yaw)
        self.camera_orientation[0] = math.atan2(2*(qw*qx + qy*qz), 1-2*(qx*qx + qy*qy))  # roll
        self.camera_orientation[1] = math.asin(2*(qw*qy - qz*qx))  # pitch
        self.camera_orientation[2] = math.atan2(2*(qw*qz + qx*qy), 1-2*(qy*qy + qz*qz))  # yaw
        
        self.get_logger().info(f'Camera pose set to: pos=({self.camera_position[0]:.2f}, '
                              f'{self.camera_position[1]:.2f}, {self.camera_position[2]:.2f}), '
                              f'rot=({math.degrees(self.camera_orientation[0]):.1f}°, '
                              f'{math.degrees(self.camera_orientation[1]):.1f}°, '
                              f'{math.degrees(self.camera_orientation[2]):.1f}°)')

    def fov_callback(self, msg):
        """
        Handle field of view adjustment commands.
        
        Args:
            msg (std_msgs.msg.Float32): FOV in degrees
        """
        if not self.camera_enabled:
            return
            
        # Clamp FOV to reasonable range
        self.camera_fov = max(10.0, min(120.0, msg.data))
        self.get_logger().info(f'Camera FOV set to: {self.camera_fov:.1f}°')

    def focus_callback(self, msg):
        """
        Handle focus distance adjustment commands.
        
        Args:
            msg (std_msgs.msg.Float32): Focus distance in meters
        """
        if not self.camera_enabled:
            return
            
        # Clamp focus distance to reasonable range
        self.camera_focus_distance = max(0.1, min(100.0, msg.data))
        self.get_logger().info(f'Camera focus distance set to: {self.camera_focus_distance:.2f}m')

    def enable_callback(self, msg):
        """
        Handle camera enable/disable commands.
        
        Args:
            msg (std_msgs.msg.Bool): Enable state
        """
        self.camera_enabled = msg.data
        status = "enabled" if self.camera_enabled else "disabled"
        self.get_logger().info(f'Camera {status}')

    def update_camera(self):
        """
        Update camera parameters - this would interface with Isaac Sim.
        In a full implementation, this would call Isaac Sim APIs to update the camera.
        """
        # This is where you would interface with Isaac Sim to actually move the camera
        # For now, we just maintain state and publish feedback
        pass

    def publish_state(self):
        """
        Publish current camera state for monitoring.
        """
        # Create state message
        state_msg = String()
        state_msg.data = (f"Camera State - "
                         f"Pos: [{self.camera_position[0]:.2f}, {self.camera_position[1]:.2f}, {self.camera_position[2]:.2f}], "
                         f"Rot: [{math.degrees(self.camera_orientation[0]):.1f}°, {math.degrees(self.camera_orientation[1]):.1f}°, {math.degrees(self.camera_orientation[2]):.1f}°], "
                         f"FOV: {self.camera_fov:.1f}°, "
                         f"Focus: {self.camera_focus_distance:.2f}m, "
                         f"Enabled: {self.camera_enabled}")
        
        self.state_pub.publish(state_msg)
        
        # Publish current pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "camera_frame"
        
        pose_msg.pose.position.x = self.camera_position[0]
        pose_msg.pose.position.y = self.camera_position[1]
        pose_msg.pose.position.z = self.camera_position[2]
        
        # Convert Euler to quaternion (simplified)
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

    def destroy_node(self):
        """Clean up when the node is destroyed."""
        self.get_logger().info('Shutting down Isaac Camera Controller Node')
        super().destroy_node()


def main(args=None):
    """
    Main function to initialize and run the camera controller node.
    
    Args:
        args: Command line arguments (optional)
    """
    rclpy.init(args=args)
    
    camera_controller = CameraController()
    
    try:
        rclpy.spin(camera_controller)
    except KeyboardInterrupt:
        camera_controller.get_logger().info('Keyboard interrupt received')
    finally:
        camera_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
