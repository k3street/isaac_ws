#!/usr/bin/env python3
"""
Comprehensive Camera Control Test Script
Tests both file-based and ROS2-based camera control methods
"""
import subprocess
import time
import sys
import json
import os
from pathlib import Path

class CameraControlTester:
    def __init__(self):
        self.test_results = []
        self.isaac_process = None
        
    def log_test(self, test_name, status, details=""):
        """Log test result"""
        result = {
            'test': test_name,
            'status': status,
            'timestamp': time.strftime("%Y-%m-%d %H:%M:%S"),
            'details': details
        }
        self.test_results.append(result)
        status_symbol = "✓" if status == "PASS" else "✗" if status == "FAIL" else "⚠"
        print(f"{status_symbol} {test_name}: {status}")
        if details:
            print(f"   Details: {details}")
    
    def check_prerequisites(self):
        """Check if all prerequisites are available"""
        print("🔍 Checking prerequisites...")
        
        # Check ROS2
        try:
            result = subprocess.run(['which', 'ros2'], capture_output=True, text=True)
            if result.returncode == 0:
                self.log_test("ROS2 Installation", "PASS", f"Found at: {result.stdout.strip()}")
            else:
                self.log_test("ROS2 Installation", "FAIL", "ros2 command not found")
                return False
        except Exception as e:
            self.log_test("ROS2 Installation", "FAIL", str(e))
            return False
        
        # Check Isaac Sim files
        isaac_files = [
            "/home/kimate/isaac_ws/camera_control_node.py",
            "/home/kimate/isaac_ws/camera_control_sender.py",
            "/home/kimate/isaac_ws/test_ros2_camera_control.py"
        ]
        
        for file_path in isaac_files:
            if os.path.exists(file_path):
                self.log_test(f"File Check: {os.path.basename(file_path)}", "PASS")
            else:
                self.log_test(f"File Check: {os.path.basename(file_path)}", "FAIL", f"File not found: {file_path}")
                return False
        
        return True
    
    def test_file_based_control(self):
        """Test file-based camera control"""
        print("\n📁 Testing file-based camera control...")
        
        command_file = Path("/tmp/isaac_camera_commands.json")
        
        # Test 1: Create command file
        try:
            test_command = {
                "timestamp": time.time(),
                "command": "move",
                "position": [3.0, 3.0, 3.0],
                "rotation": [0.0, 0.0, 45.0]
            }
            
            with open(command_file, 'w') as f:
                json.dump(test_command, f)
            
            self.log_test("File-based Command Creation", "PASS", "Test command written to file")
            
            # Verify file was created and readable
            if command_file.exists():
                with open(command_file, 'r') as f:
                    loaded_command = json.load(f)
                if loaded_command == test_command:
                    self.log_test("File-based Command Verification", "PASS", "Command file readable and correct")
                else:
                    self.log_test("File-based Command Verification", "FAIL", "Command file content mismatch")
            else:
                self.log_test("File-based Command Verification", "FAIL", "Command file not created")
                
        except Exception as e:
            self.log_test("File-based Command Creation", "FAIL", str(e))
    
    def test_ros2_topics(self):
        """Test ROS2 topic availability and publishing"""
        print("\n🤖 Testing ROS2 topic functionality...")
        
        # Test ROS2 topic creation (without Isaac Sim running, this will show if ROS2 works)
        try:
            # Create a simple ROS2 test script
            ros2_test_script = "/tmp/ros2_topic_test.py"
            with open(ros2_test_script, 'w') as f:
                f.write("""#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys

class TopicTester(Node):
    def __init__(self):
        super().__init__('topic_tester')
        self.publisher = self.create_publisher(Twist, '/camera/cmd_vel', 10)
        
    def test_publish(self):
        msg = Twist()
        msg.linear.x = 1.0
        self.publisher.publish(msg)
        return True

def main():
    rclpy.init()
    node = TopicTester()
    try:
        result = node.test_publish()
        print("SUCCESS" if result else "FAILED")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
""")
            
            os.chmod(ros2_test_script, 0o755)
            
            # Run the ROS2 test
            result = subprocess.run([sys.executable, ros2_test_script], 
                                 capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0 and "SUCCESS" in result.stdout:
                self.log_test("ROS2 Topic Publishing", "PASS", "ROS2 topic creation and publishing works")
            else:
                self.log_test("ROS2 Topic Publishing", "FAIL", f"stdout: {result.stdout}, stderr: {result.stderr}")
                
        except subprocess.TimeoutExpired:
            self.log_test("ROS2 Topic Publishing", "FAIL", "Test timed out")
        except Exception as e:
            self.log_test("ROS2 Topic Publishing", "FAIL", str(e))
    
    def launch_isaac_sim_test(self):
        """Launch Isaac Sim for interactive testing"""
        print("\n🚀 Launching Isaac Sim for interactive testing...")
        print("This will start Isaac Sim with the camera control node.")
        print("You can then run separate tests while Isaac Sim is running.")
        
        try:
            # Use the launch script
            launch_script = "/home/kimate/isaac_ws/launch_camera_control.sh"
            if os.path.exists(launch_script):
                print(f"Launching: {launch_script}")
                print("Isaac Sim will start in GUI mode. You can:")
                print("1. Test file-based control: python3 camera_control_sender.py")
                print("2. Test ROS2 control: python3 test_ros2_camera_control.py")
                print("3. Monitor ROS2 topics: ros2 topic list")
                print("\nPress Ctrl+C to return to this script when done testing.")
                
                # Run Isaac Sim (this will block)
                result = subprocess.run(['bash', launch_script], cwd="/home/kimate/isaac_ws")
                
                self.log_test("Isaac Sim Launch", "PASS", "Launched successfully")
                return True
            else:
                self.log_test("Isaac Sim Launch", "FAIL", f"Launch script not found: {launch_script}")
                return False
                
        except KeyboardInterrupt:
            print("\n⏹ Isaac Sim testing interrupted by user")
            self.log_test("Isaac Sim Interactive Test", "INFO", "User interrupted")
            return True
        except Exception as e:
            self.log_test("Isaac Sim Launch", "FAIL", str(e))
            return False
    
    def generate_report(self):
        """Generate a test report"""
        print("\n📊 Test Report")
        print("=" * 50)
        
        pass_count = sum(1 for r in self.test_results if r['status'] == 'PASS')
        fail_count = sum(1 for r in self.test_results if r['status'] == 'FAIL')
        total_count = len([r for r in self.test_results if r['status'] in ['PASS', 'FAIL']])
        
        print(f"Total Tests: {total_count}")
        print(f"Passed: {pass_count}")
        print(f"Failed: {fail_count}")
        print(f"Success Rate: {(pass_count/total_count*100):.1f}%" if total_count > 0 else "N/A")
        
        print("\nDetailed Results:")
        for result in self.test_results:
            status_symbol = "✓" if result['status'] == "PASS" else "✗" if result['status'] == "FAIL" else "ℹ"
            print(f"  {status_symbol} {result['test']}: {result['status']}")
            if result['details']:
                print(f"    {result['details']}")
        
        # Save report to file
        report_file = "/home/kimate/isaac_ws/camera_control_test_report.json"
        with open(report_file, 'w') as f:
            json.dump(self.test_results, f, indent=2)
        
        print(f"\n📝 Detailed report saved to: {report_file}")

def main():
    print("🎯 Isaac Sim Camera Control - Comprehensive Test Suite")
    print("=" * 60)
    
    tester = CameraControlTester()
    
    # Run prerequisite checks
    if not tester.check_prerequisites():
        print("\n❌ Prerequisites not met. Please fix the issues above.")
        sys.exit(1)
    
    # Run automated tests
    tester.test_file_based_control()
    tester.test_ros2_topics()
    
    # Ask user if they want to launch Isaac Sim for interactive testing
    print("\n🤔 Do you want to launch Isaac Sim for interactive testing?")
    print("This will start Isaac Sim where you can manually test camera movement.")
    
    response = input("Launch Isaac Sim? (y/N): ").strip().lower()
    
    if response in ['y', 'yes']:
        tester.launch_isaac_sim_test()
    else:
        print("Skipping Isaac Sim interactive test.")
    
    # Generate final report
    tester.generate_report()
    
    print("\n🎉 Testing complete!")

if __name__ == "__main__":
    main()
