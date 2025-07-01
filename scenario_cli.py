#!/usr/bin/env python3
"""
Isaac Sim Scenario Command Line Interface
Easy tool for testing and sending scenario requests
"""

import argparse
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
from scenario_manager import ScenarioManager

class ScenarioCLI(Node):
    """Command line interface for scenario management"""
    
    def __init__(self):
        super().__init__('scenario_cli')
        
        # ROS2 Publishers and Clients
        self.scenario_request_pub = self.create_publisher(String, '/isaac/scenario_request', 10)
        self.setup_service_client = self.create_client(SetBool, '/isaac/setup_scenario')
        
        # Scenario manager for validation
        self.scenario_manager = ScenarioManager()
    
    def send_scenario_request(self, request_text):
        """Send scenario request via ROS2"""
        try:
            # Validate the request first
            config = self.scenario_manager.generate_scenario_config(request_text)
            
            if config['is_valid']:
                print(f"✅ Scenario request validated")
            else:
                print(f"⚠️  Scenario has warnings:")
                for warning in config['warnings']:
                    print(f"   - {warning}")
            
            # Send the request
            msg = String()
            msg.data = request_text
            self.scenario_request_pub.publish(msg)
            
            print(f"📤 Scenario request sent: '{request_text}'")
            return True
            
        except Exception as e:
            print(f"❌ Error sending scenario request: {e}")
            return False
    
    def activate_scenario(self, activate=True):
        """Activate or deactivate scenario via service"""
        try:
            if not self.setup_service_client.wait_for_service(timeout_sec=5.0):
                print("❌ Scenario service not available")
                return False
            
            request = SetBool.Request()
            request.data = activate
            
            future = self.setup_service_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    action = "activated" if activate else "deactivated"
                    print(f"✅ Scenario {action}: {response.message}")
                    return True
                else:
                    print(f"❌ Service call failed: {response.message}")
            else:
                print("❌ Service call timed out")
                
        except Exception as e:
            print(f"❌ Error calling scenario service: {e}")
            
        return False

def main():
    """Main CLI function"""
    parser = argparse.ArgumentParser(description='Isaac Sim Scenario Management CLI')
    parser.add_argument('command', nargs='?', choices=['request', 'activate', 'deactivate', 'validate', 'examples'], 
                       help='Command to execute')
    parser.add_argument('--scenario', '-s', type=str, 
                       help='Scenario description (for request/validate commands)')
    parser.add_argument('--list-scenes', action='store_true',
                       help='List available scenes')
    parser.add_argument('--list-robots', action='store_true', 
                       help='List available robots')
    
    args = parser.parse_args()
    
    # Initialize scenario manager for validation/info commands
    scenario_manager = ScenarioManager()
    
    # Handle list commands first (don't require command)
    if args.list_scenes:
        print("🏗️  Available scenes:")
        for name, info in scenario_manager.available_scenes.items():
            print(f"  - {name}: {info['description']}")
        return
        
    if args.list_robots:
        print("🤖 Available robots:")
        for name, info in scenario_manager.available_robots.items():
            ros_status = "✅ ROS2" if info['ros_enabled'] else "❌ No ROS2"
            print(f"  - {name}: {info['description']} ({ros_status})")
        return
    
    # Check if command is provided for other operations
    if not args.command:
        print("❌ Please provide a command. Use --help for usage information.")
        parser.print_help()
        return
    
    if args.command == 'examples':
        print("📝 Example scenario requests:")
        examples = [
            "warehouse scene with carter 2.0 robot that is fully ros controlled",
            "office environment with franka arm for manipulation",
            "simple room with multiple carter robots",
            "warehouse with overhead camera view",
            "grid room with ur10 arm and autonomous navigation"
        ]
        for i, example in enumerate(examples, 1):
            print(f"  {i}. \"{example}\"")
        return
        if not args.scenario:
            print("❌ Please provide a scenario description with --scenario")
            return
            
        print(f"🔍 Validating scenario: '{args.scenario}'")
        config = scenario_manager.generate_scenario_config(args.scenario)
        
        print(f"\n📋 Parsed scenario:")
        print(f"   Scene: {config['parsed_scenario']['scene']}")
        print(f"   Robots: {len(config['parsed_scenario']['robots'])} robot(s)")
        print(f"   Requirements: {config['parsed_scenario']['special_requirements']}")
        print(f"   Camera position: {config['parsed_scenario'].get('camera_position', 'default')}")
        
        if config['warnings']:
            print(f"\n⚠️  Warnings:")
            for warning in config['warnings']:
                print(f"   - {warning}")
        else:
            print(f"\n✅ Scenario is valid")
        return
    
    # Commands that require ROS2
    if args.command in ['request', 'activate', 'deactivate']:
        try:
            rclpy.init()
            cli = ScenarioCLI()
            
            if args.command == 'request':
                if not args.scenario:
                    print("❌ Please provide a scenario description with --scenario")
                    return
                    
                success = cli.send_scenario_request(args.scenario)
                if success:
                    print("💡 To activate the scenario, run:")
                    print("   python3 scenario_cli.py activate")
                    
            elif args.command == 'activate':
                cli.activate_scenario(True)
                
            elif args.command == 'deactivate':
                cli.activate_scenario(False)
                
        except KeyboardInterrupt:
            print("\n🛑 Interrupted")
        except Exception as e:
            print(f"❌ Error: {e}")
        finally:
            if 'cli' in locals():
                cli.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
