#!/usr/bin/env python3
"""
LLM-Powered Camera Control Node for Isaac Sim
Subscribes to RGB camera feed, analyzes images with LLM, and publishes camera movement commands
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import cv2
import numpy as np
from cv_bridge import CvBridge
import base64
import json
import time
import threading
from pathlib import Path

class LLMCameraController(Node):
    def __init__(self):
        super().__init__('llm_camera_controller')
        
        # Initialize CV bridge for image conversion
        self.bridge = CvBridge()
        
        # Control parameters
        self.analysis_interval = 2.0  # Analyze image every 2 seconds
        self.last_analysis_time = 0
        self.current_image = None
        self.analysis_active = True
        
        # Camera control parameters
        self.movement_scale = 0.5  # Scale factor for movements
        self.rotation_scale = 0.3  # Scale factor for rotations
        
        # Create subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/rgb',
            self.image_callback,
            10
        )
        
        # Create publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/camera/cmd_vel',
            10
        )
        
        self.analysis_pub = self.create_publisher(
            String,
            '/camera/llm_analysis',
            10
        )
        
        # Create timer for periodic analysis
        self.analysis_timer = self.create_timer(self.analysis_interval, self.analyze_current_image)
        
        # LLM analysis thread
        self.analysis_queue = []
        self.analysis_thread = threading.Thread(target=self.llm_analysis_worker, daemon=True)
        self.analysis_thread.start()
        
        self.get_logger().info('🤖 LLM Camera Controller Node started')
        self.get_logger().info('📸 Subscribing to /camera/rgb')
        self.get_logger().info('🎮 Publishing to /camera/cmd_vel')
        self.get_logger().info('📝 Publishing analysis to /camera/llm_analysis')
        self.get_logger().info(f'🔄 Analysis interval: {self.analysis_interval}s')
    
    def image_callback(self, msg):
        """Store latest image for analysis"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_image = cv_image
        except Exception as e:
            self.get_logger().error(f'❌ Error converting image: {e}')
    
    def analyze_current_image(self):
        """Trigger analysis of current image"""
        if self.current_image is not None and self.analysis_active:
            current_time = time.time()
            if current_time - self.last_analysis_time >= self.analysis_interval:
                # Add to analysis queue
                self.analysis_queue.append(self.current_image.copy())
                self.last_analysis_time = current_time
                self.get_logger().info('📸 Image queued for LLM analysis')
    
    def llm_analysis_worker(self):
        """Worker thread for LLM analysis"""
        while True:
            if self.analysis_queue:
                image = self.analysis_queue.pop(0)
                try:
                    self.analyze_image_with_llm(image)
                except Exception as e:
                    self.get_logger().error(f'❌ LLM analysis error: {e}')
            time.sleep(0.1)
    
    def analyze_image_with_llm(self, image):
        """Analyze image with LLM and generate camera movement commands"""
        try:
            # Resize image for faster processing
            height, width = image.shape[:2]
            if width > 640:
                scale = 640 / width
                new_width = int(width * scale)
                new_height = int(height * scale)
                image = cv2.resize(image, (new_width, new_height))
            
            # Convert to base64 for LLM analysis
            _, buffer = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            
            # Create analysis prompt
            analysis_prompt = self.create_analysis_prompt()
            
            # For now, simulate LLM analysis with computer vision heuristics
            # In production, this would call an actual LLM API
            analysis_result = self.simulate_llm_analysis(image)
            
            # Generate camera movement commands based on analysis
            camera_command = self.generate_camera_command(analysis_result)
            
            # Publish analysis result
            analysis_msg = String()
            analysis_msg.data = json.dumps(analysis_result, indent=2)
            self.analysis_pub.publish(analysis_msg)
            
            # Publish camera movement command
            if camera_command:
                self.cmd_vel_pub.publish(camera_command)
                self.get_logger().info(f'🎮 Camera command: {analysis_result["action"]}')
            
        except Exception as e:
            self.get_logger().error(f'❌ Error in LLM analysis: {e}')
    
    def create_analysis_prompt(self):
        """Create prompt for LLM analysis"""
        return """
        Analyze this camera image from Isaac Sim and provide camera movement recommendations.
        
        Consider:
        1. Scene composition - are important objects centered or visible?
        2. Depth and perspective - would a different angle provide better view?
        3. Object interactions - are robots/objects clearly visible?
        4. Lighting and clarity - can the scene be seen clearly?
        
        Respond with one of these actions:
        - "move_closer": Move camera closer to objects
        - "move_back": Move camera further from objects  
        - "move_left": Pan camera left
        - "move_right": Pan camera right
        - "move_up": Move camera higher
        - "move_down": Move camera lower
        - "rotate_left": Rotate camera left (yaw)
        - "rotate_right": Rotate camera right (yaw)
        - "tilt_up": Tilt camera up (pitch)
        - "tilt_down": Tilt camera down (pitch)
        - "hold_position": Current view is optimal
        
        Also provide reasoning for the decision.
        """
    
    def simulate_llm_analysis(self, image):
        """Simulate LLM analysis using computer vision heuristics"""
        try:
            height, width = image.shape[:2]
            
            # Convert to grayscale for analysis
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # Detect edges to find objects
            edges = cv2.Canny(gray, 50, 150)
            
            # Find contours (objects)
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Analyze scene composition
            analysis = {
                "timestamp": time.time(),
                "scene_analysis": {},
                "action": "hold_position",
                "reasoning": "Scene analysis in progress",
                "confidence": 0.5
            }
            
            if len(contours) > 0:
                # Find largest contour (main object)
                largest_contour = max(contours, key=cv2.contourArea)
                
                # Get bounding box of main object
                x, y, w, h = cv2.boundingRect(largest_contour)
                
                # Calculate object center
                obj_center_x = x + w // 2
                obj_center_y = y + h // 2
                
                # Calculate image center
                img_center_x = width // 2
                img_center_y = height // 2
                
                # Analyze object positioning
                x_offset = obj_center_x - img_center_x
                y_offset = obj_center_y - img_center_y
                
                # Determine action based on object position
                if abs(x_offset) > width * 0.2:  # Object significantly off-center horizontally
                    if x_offset > 0:
                        analysis["action"] = "rotate_left"
                        analysis["reasoning"] = "Main object is too far right, rotating camera left to center it"
                    else:
                        analysis["action"] = "rotate_right"
                        analysis["reasoning"] = "Main object is too far left, rotating camera right to center it"
                elif abs(y_offset) > height * 0.2:  # Object significantly off-center vertically
                    if y_offset > 0:
                        analysis["action"] = "tilt_up"
                        analysis["reasoning"] = "Main object is too low in frame, tilting camera up"
                    else:
                        analysis["action"] = "tilt_down"
                        analysis["reasoning"] = "Main object is too high in frame, tilting camera down"
                elif w * h < (width * height) * 0.1:  # Object too small
                    analysis["action"] = "move_closer"
                    analysis["reasoning"] = "Main object appears small, moving closer for better view"
                elif w * h > (width * height) * 0.6:  # Object too large
                    analysis["action"] = "move_back"
                    analysis["reasoning"] = "Main object fills most of frame, moving back for better perspective"
                else:
                    analysis["action"] = "hold_position"
                    analysis["reasoning"] = "Scene composition looks good, maintaining current position"
                
                analysis["scene_analysis"] = {
                    "main_object_center": [int(obj_center_x), int(obj_center_y)],
                    "image_center": [int(img_center_x), int(img_center_y)],
                    "offset": [int(x_offset), int(y_offset)],
                    "object_size_ratio": round((w * h) / (width * height), 3),
                    "num_objects": len(contours)
                }
                analysis["confidence"] = 0.8
            else:
                analysis["action"] = "move_back"
                analysis["reasoning"] = "No clear objects detected, moving back to get wider view"
                analysis["confidence"] = 0.6
            
            return analysis
            
        except Exception as e:
            self.get_logger().error(f'❌ Error in simulated LLM analysis: {e}')
            return {
                "timestamp": time.time(),
                "action": "hold_position",
                "reasoning": f"Analysis error: {e}",
                "confidence": 0.0
            }
    
    def generate_camera_command(self, analysis):
        """Generate Twist message based on analysis result"""
        try:
            action = analysis.get("action", "hold_position")
            confidence = analysis.get("confidence", 0.0)
            
            # Only act if confidence is above threshold
            if confidence < 0.5:
                return None
            
            twist = Twist()
            
            # Map actions to movement commands
            if action == "move_closer":
                twist.linear.x = self.movement_scale
            elif action == "move_back":
                twist.linear.x = -self.movement_scale
            elif action == "move_left":
                twist.linear.y = self.movement_scale
            elif action == "move_right":
                twist.linear.y = -self.movement_scale
            elif action == "move_up":
                twist.linear.z = self.movement_scale
            elif action == "move_down":
                twist.linear.z = -self.movement_scale
            elif action == "rotate_left":
                twist.angular.y = self.rotation_scale
            elif action == "rotate_right":
                twist.angular.y = -self.rotation_scale
            elif action == "tilt_up":
                twist.angular.x = -self.rotation_scale
            elif action == "tilt_down":
                twist.angular.x = self.rotation_scale
            elif action == "hold_position":
                # No movement
                pass
            
            return twist
            
        except Exception as e:
            self.get_logger().error(f'❌ Error generating camera command: {e}')
            return None
    
    def set_analysis_active(self, active):
        """Enable/disable automatic analysis"""
        self.analysis_active = active
        status = "enabled" if active else "disabled"
        self.get_logger().info(f'🔄 Automatic analysis {status}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LLMCameraController()
        
        print("\n" + "="*60)
        print("🤖 LLM CAMERA CONTROLLER ACTIVE")
        print("="*60)
        print("📸 Subscribing to /camera/rgb")
        print("🎮 Publishing to /camera/cmd_vel")
        print("📝 Publishing analysis to /camera/llm_analysis")
        print("\n💡 Monitor commands:")
        print("   ros2 topic echo /camera/llm_analysis")
        print("   ros2 topic echo /camera/cmd_vel")
        print("\n🔄 The system will automatically:")
        print("   1. Analyze camera images every 2 seconds")
        print("   2. Detect objects and composition")
        print("   3. Generate intelligent camera movements")
        print("   4. Publish movement commands to Isaac Sim")
        print("\nPress Ctrl+C to stop")
        print("="*60)
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n🛑 LLM Camera Controller stopped")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
