#!/usr/bin/env python3
"""
ROS2 Camera Control Node
Subscribes to /camera/cmd_vel topic and sends commands to Isaac Sim camera
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3Stamped
from sensor_msgs.msg import Image
import json
from pathlib import Path
import time

class CameraROS2Control(Node):
    def __init__(self):
        super().__init__('camera_ros2_control')
        
        # Command file for communicating with Isaac Sim
        self.command_file = Path("/tmp/isaac_camera_commands.json")
        
        # Create subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/camera/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.position_sub = self.create_subscription(
            Vector3Stamped,
            '/camera/position',
            self.position_callback,
            10
        )
        
        # Create image subscriber to check if Isaac Sim is running
        self.image_sub = self.create_subscription(
            Image,
            '/camera/rgb',
            self.image_callback,
            10
        )
        
        self.isaac_sim_connected = False
        self.last_image_time = time.time()
        
        # Create timer to check connection status
        self.timer = self.create_timer(1.0, self.status_timer_callback)
        
        self.get_logger().info('🎮 Camera ROS2 Control Node started')
        self.get_logger().info('📺 Listening on:')
        self.get_logger().info('   /camera/cmd_vel (geometry_msgs/Twist) - for velocity control')
        self.get_logger().info('   /camera/position (geometry_msgs/Vector3Stamped) - for absolute positioning')
        self.get_logger().info('💡 Example commands:')
        self.get_logger().info('   ros2 topic pub --once /camera/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"')
        self.get_logger().info('   ros2 topic pub --once /camera/position geometry_msgs/msg/Vector3Stamped "{vector: {x: 5.0, y: 0.0, z: 5.0}}"')
        
    def cmd_vel_callback(self, msg):
        """Handle velocity commands"""
        try:
            command = {
                "type": "velocity",
                "data": {
                    "linear_x": msg.linear.x,
                    "linear_y": msg.linear.y,
                    "linear_z": msg.linear.z,
                    "angular_x": msg.angular.x,
                    "angular_y": msg.angular.y,
                    "angular_z": msg.angular.z
                }
            }
            
            self.send_command(command)
            self.get_logger().info(f'🎮 Velocity command: linear=[{msg.linear.x:.2f}, {msg.linear.y:.2f}, {msg.linear.z:.2f}], angular=[{msg.angular.x:.2f}, {msg.angular.y:.2f}, {msg.angular.z:.2f}]')
            
        except Exception as e:
            self.get_logger().error(f'❌ Error processing velocity command: {e}')
    
    def position_callback(self, msg):
        """Handle absolute position commands"""
        try:
            command = {
                "type": "position",
                "data": {
                    "x": msg.vector.x,
                    "y": msg.vector.y,
                    "z": msg.vector.z
                }
            }
            
            self.send_command(command)
            self.get_logger().info(f'📍 Position command: [{msg.vector.x:.2f}, {msg.vector.y:.2f}, {msg.vector.z:.2f}]')
            
        except Exception as e:
            self.get_logger().error(f'❌ Error processing position command: {e}')
    
    def image_callback(self, msg):
        """Track Isaac Sim connection status"""
        self.isaac_sim_connected = True
        self.last_image_time = time.time()
    
    def status_timer_callback(self):
        """Check connection status"""
        current_time = time.time()
        if current_time - self.last_image_time > 3.0:  # No images for 3 seconds
            if self.isaac_sim_connected:
                self.get_logger().warn('⚠️  Lost connection to Isaac Sim camera')
                self.isaac_sim_connected = False
        elif not self.isaac_sim_connected:
            self.get_logger().info('✅ Connected to Isaac Sim camera')
            self.isaac_sim_connected = True
    
    def send_command(self, command):
        """Send command to Isaac Sim via file"""
        try:
            # Write command to file atomically
            temp_file = self.command_file.with_suffix('.tmp')
            with open(temp_file, 'w') as f:
                json.dump(command, f, indent=2)
            
            # Atomic move
            temp_file.rename(self.command_file)
            
        except Exception as e:
            self.get_logger().error(f'❌ Error sending command: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CameraROS2Control()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
