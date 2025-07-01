#!/usr/bin/env python3
"""
Camera Control Command Sender
Sends movement commands to Isaac Sim Camera Control Node via file-based interface
"""

import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32, String


class CameraCommandSender(Node):
    def __init__(self):
        super().__init__('camera_command_sender')
        
        # Command file for Isaac Sim communication
        self.command_file = Path("/tmp/isaac_camera_commands.json")
        
        # ROS2 subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/camera/cmd_vel', self.cmd_vel_callback, 10)
        
        self.set_pose_sub = self.create_subscription(
            PoseStamped, '/camera/set_pose', self.set_pose_callback, 10)
            
        # ROS2 publishers for status
        self.status_pub = self.create_publisher(String, '/camera/state', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/camera/current_pose', 10)
        
        # Keep track of camera state
        self.camera_position = [2.0, 2.0, 2.0]
        self.camera_rotation = [0.0, 0.0, 0.0]
        
        # Timer for status updates
        self.status_timer = self.create_timer(0.2, self.publish_status)
        
        self.get_logger().info("🚀 Camera Command Sender started")
        self.get_logger().info(f"📂 Sending commands to: {self.command_file}")
        
    def cmd_vel_callback(self, msg):
        """Handle velocity commands and send to Isaac Sim"""
        command = {
            'type': 'velocity',
            'data': {
                'linear_x': msg.linear.x,
                'linear_y': msg.linear.y, 
                'linear_z': msg.linear.z,
                'angular_x': msg.angular.x,
                'angular_y': msg.angular.y,
                'angular_z': msg.angular.z
            },
            'timestamp': time.time()
        }
        
        self.send_command(command)
        
        # Update estimated position (for status)
        dt = 0.1
        self.camera_position[0] += msg.linear.x * dt
        self.camera_position[1] += msg.linear.y * dt
        self.camera_position[2] += msg.linear.z * dt
        
        self.camera_rotation[0] += msg.angular.x * dt * 57.2958
        self.camera_rotation[1] += msg.angular.y * dt * 57.2958
        self.camera_rotation[2] += msg.angular.z * dt * 57.2958
        
        self.get_logger().info(f"📤 Velocity: linear=({msg.linear.x:.2f}, {msg.linear.y:.2f}, {msg.linear.z:.2f}), angular=({msg.angular.x:.2f}, {msg.angular.y:.2f}, {msg.angular.z:.2f})")
        
    def set_pose_callback(self, msg):
        """Handle absolute position commands and send to Isaac Sim"""
        command = {
            'type': 'position',
            'data': {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z
            },
            'timestamp': time.time()
        }
        
        self.send_command(command)
        
        # Update tracked position
        self.camera_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        
        self.get_logger().info(f"📤 Position: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})")
        
    def send_command(self, command):
        """Send single command to Isaac Sim via file with improved error handling"""
        try:
            # Rate limiting - don't send commands too frequently
            current_time = time.time()
            if hasattr(self, '_last_command_time'):
                if current_time - self._last_command_time < 0.05:  # 50ms minimum interval
                    return
            self._last_command_time = current_time
            
            # Use simple direct write since Isaac Sim processes files quickly
            with open(self.command_file, 'w') as f:
                json.dump(command, f)
                f.flush()  # Ensure data is written immediately
                
        except Exception as e:
            self.get_logger().error(f"❌ Failed to send command: {e}")
            
    def publish_status(self):
        """Publish camera status to ROS2"""
        # Publish status string
        status_msg = String()
        status_msg.data = f"Position: [{self.camera_position[0]:.2f}, {self.camera_position[1]:.2f}, {self.camera_position[2]:.2f}], Rotation: [{self.camera_rotation[0]:.1f}°, {self.camera_rotation[1]:.1f}°, {self.camera_rotation[2]:.1f}°]"
        self.status_pub.publish(status_msg)
        
        # Publish current pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "camera_frame"
        
        pose_msg.pose.position.x = float(self.camera_position[0])
        pose_msg.pose.position.y = float(self.camera_position[1])
        pose_msg.pose.position.z = float(self.camera_position[2])
        
        # Simple quaternion (no rotation for now)
        pose_msg.pose.orientation.w = 1.0
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        
        self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        sender = CameraCommandSender()
        rclpy.spin(sender)
    except KeyboardInterrupt:
        pass
    finally:
        sender.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
