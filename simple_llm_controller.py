#!/usr/bin/env python3
"""
Simple LLM Camera Controller - Working Version
Analyzes camera feed and sends intelligent movement commands
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import cv2
import numpy as np
from cv_bridge import CvBridge
import json
import time

class SimpleLLMController(Node):
    def __init__(self):
        super().__init__('simple_llm_controller')
        
        self.bridge = CvBridge()
        self.current_image = None
        self.last_command_time = 0
        self.command_interval = 3.0  # Send command every 3 seconds
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/rgb', self.image_callback, 10
        )
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/camera/cmd_vel', 10)
        self.analysis_pub = self.create_publisher(String, '/llm_analysis', 10)
        
        # Timer for periodic analysis
        self.timer = self.create_timer(2.0, self.analyze_and_move)
        
        self.get_logger().info('🤖 Simple LLM Controller started')
        self.get_logger().info('📸 Watching /camera/rgb')
        self.get_logger().info('🎮 Publishing to /camera/cmd_vel')
        
    def image_callback(self, msg):
        """Store current image"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')
    
    def analyze_and_move(self):
        """Analyze current image and generate movement"""
        if self.current_image is None:
            return
            
        current_time = time.time()
        if current_time - self.last_command_time < self.command_interval:
            return
            
        try:
            # Simple analysis
            analysis = self.simple_analysis(self.current_image)
            
            # Publish analysis
            analysis_msg = String()
            analysis_msg.data = json.dumps(analysis, indent=2)
            self.analysis_pub.publish(analysis_msg)
            
            # Generate movement command
            twist = self.create_twist_command(analysis['action'])
            if twist:
                self.cmd_pub.publish(twist)
                self.get_logger().info(f'🎮 Action: {analysis["action"]} - {analysis["reasoning"]}')
                self.last_command_time = current_time
                
        except Exception as e:
            self.get_logger().error(f'Analysis error: {e}')
    
    def simple_analysis(self, image):
        """Simple computer vision analysis"""
        height, width = image.shape[:2]
        
        # Convert to grayscale and find edges
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        
        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        analysis = {
            "timestamp": time.time(),
            "action": "hold_position",
            "reasoning": "No clear action needed",
            "num_objects": len(contours)
        }
        
        if contours:
            # Find largest object
            largest = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest)
            
            obj_center_x = x + w // 2
            img_center_x = width // 2
            x_offset = obj_center_x - img_center_x
            
            # Simple decision logic
            if abs(x_offset) > width * 0.3:
                if x_offset > 0:
                    analysis["action"] = "rotate_left"
                    analysis["reasoning"] = "Object too far right, rotating left"
                else:
                    analysis["action"] = "rotate_right" 
                    analysis["reasoning"] = "Object too far left, rotating right"
            elif w * h < (width * height) * 0.05:
                analysis["action"] = "move_closer"
                analysis["reasoning"] = "Object too small, moving closer"
            elif w * h > (width * height) * 0.7:
                analysis["action"] = "move_back"
                analysis["reasoning"] = "Object too large, moving back"
            else:
                # Randomly explore different viewpoints
                import random
                actions = ["move_up", "move_down", "tilt_up", "tilt_down"]
                analysis["action"] = random.choice(actions)
                analysis["reasoning"] = "Exploring different viewpoints"
        
        return analysis
    
    def create_twist_command(self, action):
        """Create Twist command for given action"""
        twist = Twist()
        
        # Ensure all values are proper floats
        move_speed = 0.5
        rotate_speed = 0.3
        
        if action == "move_closer":
            twist.linear.x = float(move_speed)
        elif action == "move_back":
            twist.linear.x = float(-move_speed)
        elif action == "move_left":
            twist.linear.y = float(move_speed)
        elif action == "move_right":
            twist.linear.y = float(-move_speed)
        elif action == "move_up":
            twist.linear.z = float(move_speed)
        elif action == "move_down":
            twist.linear.z = float(-move_speed)
        elif action == "rotate_left":
            twist.angular.y = float(rotate_speed)
        elif action == "rotate_right":
            twist.angular.y = float(-rotate_speed)
        elif action == "tilt_up":
            twist.angular.x = float(-rotate_speed)
        elif action == "tilt_down":
            twist.angular.x = float(rotate_speed)
        else:
            return None
            
        return twist

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SimpleLLMController()
        
        print("\n" + "="*60)
        print("🤖 SIMPLE LLM CAMERA CONTROLLER")
        print("="*60)
        print("📸 Analyzing camera feed every 2 seconds")
        print("🎮 Sending intelligent camera commands every 3 seconds")
        print("\n📊 Monitor:")
        print("   ros2 topic echo /llm_analysis")
        print("   ros2 topic echo /camera/cmd_vel")
        print("\n🚀 Intelligent camera control active!")
        print("="*60)
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n🛑 LLM Controller stopped")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
