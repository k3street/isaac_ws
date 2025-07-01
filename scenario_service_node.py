#!/usr/bin/env python3
"""
ROS2 Scenario Service Node
Accepts scenario requests via ROS2 service and integrates with camera control
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist, PoseStamped
import json
import time
import threading
from pathlib import Path
from scenario_manager import ScenarioManager

class ScenarioServiceNode(Node):
    """ROS2 service node for handling scenario requests"""
    
    def __init__(self):
        super().__init__('scenario_service_node')
        
        # Initialize scenario manager
        self.scenario_manager = ScenarioManager()
        
        # ROS2 Services
        self.scenario_service = self.create_service(
            SetBool, 
            '/isaac/setup_scenario', 
            self.setup_scenario_callback
        )
        
        # ROS2 Subscribers for scenario requests
        self.scenario_request_sub = self.create_subscription(
            String,
            '/isaac/scenario_request',
            self.scenario_request_callback,
            10
        )
        
        # ROS2 Publishers
        self.scenario_status_pub = self.create_publisher(String, '/isaac/scenario_status', 10)
        self.camera_cmd_vel_pub = self.create_publisher(Twist, '/camera/cmd_vel', 10)
        self.camera_pose_pub = self.create_publisher(PoseStamped, '/camera/set_pose', 10)
        
        # State management
        self.current_scenario = None
        self.scenario_active = False
        self.scenario_config_file = Path("/tmp/isaac_scenario_config.json")
        self.camera_command_file = Path("/tmp/isaac_camera_commands.json")
        
        self.get_logger().info("🎯 Scenario Service Node initialized")
        self.get_logger().info("   Service: /isaac/setup_scenario")
        self.get_logger().info("   Topic: /isaac/scenario_request")
        self.get_logger().info("   Status: /isaac/scenario_status")
        
        # Publish initial status
        self.publish_status("Scenario Service Ready - Send requests to /isaac/scenario_request")
    
    def scenario_request_callback(self, msg):
        """Handle scenario requests from ROS2 topic"""
        self.get_logger().info(f"📥 Received scenario request: '{msg.data}'")
        
        try:
            # Parse and generate scenario configuration
            config = self.scenario_manager.generate_scenario_config(msg.data)
            
            # Save configuration for the camera node to use
            self.scenario_manager.save_scenario_config(config)
            
            # Store current scenario
            self.current_scenario = config
            
            # Set up camera positioning if specified
            if config['parsed_scenario'].get('camera_position'):
                self.setup_camera_position(config['parsed_scenario']['camera_position'])
            
            # Publish status update
            status_msg = f"Scenario configured: {config['parsed_scenario']['scene']} with {len(config['parsed_scenario']['robots'])} robot(s)"
            self.publish_status(status_msg)
            
            self.get_logger().info(f"✅ Scenario configured successfully")
            
        except Exception as e:
            error_msg = f"Failed to configure scenario: {str(e)}"
            self.get_logger().error(f"❌ {error_msg}")
            self.publish_status(error_msg)
    
    def setup_scenario_callback(self, request, response):
        """Handle scenario setup service calls"""
        self.get_logger().info(f"🔧 Service call received: setup_scenario = {request.data}")
        
        try:
            if request.data:  # True = setup scenario
                if self.current_scenario:
                    # Scenario already configured, just activate it
                    self.scenario_active = True
                    response.success = True
                    response.message = f"Scenario activated: {self.current_scenario['parsed_scenario']['scene']}"
                else:
                    response.success = False
                    response.message = "No scenario configured. Send a scenario request first."
            else:  # False = reset/clear scenario
                self.current_scenario = None
                self.scenario_active = False
                # Clear configuration file
                if self.scenario_config_file.exists():
                    self.scenario_config_file.unlink()
                response.success = True
                response.message = "Scenario cleared"
                
            self.publish_status(response.message)
            
        except Exception as e:
            response.success = False
            response.message = f"Service error: {str(e)}"
            self.get_logger().error(f"❌ Service error: {str(e)}")
            
        return response
    
    def setup_camera_position(self, position):
        """Set up camera position based on scenario requirements"""
        try:
            # Create camera position command
            camera_command = {
                "type": "position",
                "data": {
                    "x": position[0],
                    "y": position[1], 
                    "z": position[2]
                },
                "timestamp": time.time()
            }
            
            # Write to camera command file
            with open(self.camera_command_file, 'w') as f:
                json.dump(camera_command, f)
            
            # Also send via ROS2 topic
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "world"
            pose_msg.pose.position.x = float(position[0])
            pose_msg.pose.position.y = float(position[1])
            pose_msg.pose.position.z = float(position[2])
            
            self.camera_pose_pub.publish(pose_msg)
            
            self.get_logger().info(f"📷 Camera position set to: {position}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to set camera position: {str(e)}")
    
    def publish_status(self, message):
        """Publish status message to ROS2 topic"""
        status_msg = String()
        status_msg.data = f"[{time.strftime('%H:%M:%S')}] {message}"
        self.scenario_status_pub.publish(status_msg)
    
    def get_current_scenario_info(self):
        """Get information about current scenario"""
        if self.current_scenario:
            return {
                "active": self.scenario_active,
                "scene": self.current_scenario['parsed_scenario']['scene'],
                "robots": len(self.current_scenario['parsed_scenario']['robots']),
                "requirements": self.current_scenario['parsed_scenario']['special_requirements']
            }
        return {"active": False, "scene": None}

def main(args=None):
    """Main entry point"""
    print("🚀 Starting Isaac Sim Scenario Service Node...")
    
    rclpy.init(args=args)
    
    try:
        node = ScenarioServiceNode()
        
        # Use multi-threaded executor for better performance
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        print("✅ Scenario Service Node running!")
        print("\n📝 Usage:")
        print("  # Send scenario request:")
        print("  ros2 topic pub --once /isaac/scenario_request std_msgs/msg/String \"data: 'warehouse with carter 2.0 robot'\"")
        print("\n  # Activate scenario:")
        print("  ros2 service call /isaac/setup_scenario std_srvs/srv/SetBool \"data: true\"")
        print("\n  # Monitor status:")
        print("  ros2 topic echo /isaac/scenario_status")
        print("\nPress Ctrl+C to stop...")
        
        executor.spin()
        
    except KeyboardInterrupt:
        print("\n🛑 Shutting down scenario service...")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        print("✅ Scenario Service Node stopped")

if __name__ == '__main__':
    main()
