#!/usr/bin/env python3
"""
ROS2 to Isaac Sim Bridge
Receives ROS2 commands and sends them to Isaac Sim via file communication
"""

import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32, String


class ROS2IsaacBridge(Node):
    def __init__(self):
        super().__init__('ros2_isaac_bridge')
        
        # File communication paths
        self.control_file = Path("/tmp/isaac_camera_control.json")
        self.status_file = Path("/tmp/isaac_camera_status.json")
        
        # ROS2 subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/camera/cmd_vel', self.cmd_vel_callback, 10)
        
        self.set_pose_sub = self.create_subscription(
            PoseStamped, '/camera/set_pose', self.set_pose_callback, 10)
            
        self.set_fov_sub = self.create_subscription(
            Float32, '/camera/set_fov', self.set_fov_callback, 10)
        
        # ROS2 publishers
        self.status_pub = self.create_publisher(String, '/camera/state', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/camera/current_pose', 10)
        
        # Timer for status updates
        self.status_timer = self.create_timer(0.1, self.publish_status)
        
        self.get_logger().info("🌉 ROS2-Isaac Bridge started")
        self.get_logger().info(f"📂 Control file: {self.control_file}")
        self.get_logger().info(f"📊 Status file: {self.status_file}")
        
    def cmd_vel_callback(self, msg):
        """Handle velocity commands"""
        command = {
            'type': 'velocity',
            'velocity': {
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
        self.get_logger().info(f"📤 Sent velocity command: linear=({msg.linear.x:.2f}, {msg.linear.y:.2f}, {msg.linear.z:.2f})")
        
    def set_pose_callback(self, msg):
        """Handle absolute position commands"""
        command = {
            'type': 'position',
            'position': {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z
            },
            'orientation': {
                'x': msg.pose.orientation.x,
                'y': msg.pose.orientation.y,
                'z': msg.pose.orientation.z,
                'w': msg.pose.orientation.w
            },
            'timestamp': time.time()
        }
        
        self.send_command(command)
        self.get_logger().info(f"📤 Sent position command: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})")
        
    def set_fov_callback(self, msg):
        """Handle field of view commands"""
        command = {
            'type': 'fov',
            'fov': msg.data,
            'timestamp': time.time()
        }
        
        self.send_command(command)
        self.get_logger().info(f"📤 Sent FOV command: {msg.data}°")
        
    def send_command(self, command):
        """Send command to Isaac Sim via file"""
        try:
            with open(self.control_file, 'w') as f:
                json.dump(command, f)
        except Exception as e:
            self.get_logger().error(f"❌ Failed to send command: {e}")
            
    def publish_status(self):
        """Read status from Isaac Sim and publish to ROS2"""
        if self.status_file.exists():
            try:
                with open(self.status_file, 'r') as f:
                    status = json.load(f)
                
                # Publish status string
                status_msg = String()
                status_msg.data = f"Position: {status['position']}, Rotation: {status['rotation']}"
                self.status_pub.publish(status_msg)
                
                # Publish current pose
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "camera_frame"
                
                pose_msg.pose.position.x = float(status['position'][0])
                pose_msg.pose.position.y = float(status['position'][1])
                pose_msg.pose.position.z = float(status['position'][2])
                
                # Convert rotation to quaternion (simplified)
                pose_msg.pose.orientation.w = 1.0
                pose_msg.pose.orientation.x = 0.0
                pose_msg.pose.orientation.y = 0.0
                pose_msg.pose.orientation.z = 0.0
                
                self.pose_pub.publish(pose_msg)
                
            except Exception as e:
                self.get_logger().warn(f"⚠️ Failed to read status: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        bridge = ROS2IsaacBridge()
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
