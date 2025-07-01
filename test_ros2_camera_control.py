#!/usr/bin/env python3
"""
Test script for ROS2-based camera control
Sends movement commands via ROS2 topics to test the integrated camera control system
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import time
import math

class CameraControlTester(Node):
    def __init__(self):
        super().__init__('camera_control_tester')
        
        # Create publishers for camera control
        self.cmd_vel_pub = self.create_publisher(Twist, '/camera/cmd_vel', 10)
        self.cmd_pose_pub = self.create_publisher(PoseStamped, '/camera/cmd_pose', 10)
        
        self.get_logger().info('Camera Control Tester initialized')
        
    def send_velocity_command(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_z=0.0):
        """Send a velocity command to move the camera"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.linear.z = linear_z
        msg.angular.z = angular_z
        
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info(f'Sent velocity command: linear=({linear_x:.2f}, {linear_y:.2f}, {linear_z:.2f}), angular_z={angular_z:.2f}')
        
    def send_position_command(self, x, y, z):
        """Send a position command to move the camera to specific coordinates"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        
        # Default orientation (no rotation)
        msg.pose.orientation.w = 1.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        
        self.cmd_pose_pub.publish(msg)
        self.get_logger().info(f'Sent position command: ({x:.2f}, {y:.2f}, {z:.2f})')
        
    def run_test_sequence(self):
        """Run a test sequence of camera movements"""
        self.get_logger().info('Starting camera control test sequence...')
        
        # Wait for publishers to be established
        time.sleep(2)
        
        # Test 1: Move forward for 2 seconds
        self.get_logger().info('Test 1: Moving forward...')
        for i in range(20):  # 2 seconds at 10Hz
            self.send_velocity_command(linear_x=1.0)
            time.sleep(0.1)
        
        # Stop movement
        self.send_velocity_command()
        time.sleep(1)
        
        # Test 2: Move sideways
        self.get_logger().info('Test 2: Moving sideways...')
        for i in range(20):  # 2 seconds at 10Hz
            self.send_velocity_command(linear_y=0.5)
            time.sleep(0.1)
        
        # Stop movement
        self.send_velocity_command()
        time.sleep(1)
        
        # Test 3: Move up
        self.get_logger().info('Test 3: Moving up...')
        for i in range(20):  # 2 seconds at 10Hz
            self.send_velocity_command(linear_z=0.5)
            time.sleep(0.1)
        
        # Stop movement
        self.send_velocity_command()
        time.sleep(1)
        
        # Test 4: Rotate
        self.get_logger().info('Test 4: Rotating...')
        for i in range(20):  # 2 seconds at 10Hz
            self.send_velocity_command(angular_z=0.5)
            time.sleep(0.1)
        
        # Stop movement
        self.send_velocity_command()
        time.sleep(2)
        
        # Test 5: Absolute positioning
        positions = [
            (3.0, 3.0, 3.0),
            (1.0, 1.0, 4.0),
            (0.0, 2.0, 2.0),
            (2.0, 0.0, 3.5),
            (2.0, 2.0, 2.0),  # Return to original
        ]
        
        for i, (x, y, z) in enumerate(positions):
            self.get_logger().info(f'Test 5.{i+1}: Moving to position ({x}, {y}, {z})')
            self.send_position_command(x, y, z)
            time.sleep(3)  # Wait longer for position changes
        
        self.get_logger().info('Camera control test sequence completed!')

def main():
    rclpy.init()
    
    tester = CameraControlTester()
    
    try:
        # Run the test sequence
        tester.run_test_sequence()
        
        # Keep the node alive for a bit
        time.sleep(5)
        
    except KeyboardInterrupt:
        tester.get_logger().info('Test interrupted by user')
    except Exception as e:
        tester.get_logger().error(f'Test failed: {e}')
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
