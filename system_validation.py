#!/usr/bin/env python3
"""
Isaac Sim Camera Control - System Validation Test
Validates the cleaned workspace structure and system functionality
"""
import os
import json
import subprocess
import sys
from pathlib import Path

class SystemValidator:
    def __init__(self):
        self.workspace_path = Path("/home/kimate/isaac_ws")
        self.test_results = []
        
    def log_result(self, test_name, status, details=""):
        """Log a test result"""
        result = {
            'test': test_name,
            'status': status,
            'details': details
        }
        self.test_results.append(result)
        
        # Console output
        symbol = "✅" if status == "PASS" else "❌" if status == "FAIL" else "⚠️"
        print(f"{symbol} {test_name}: {status}")
        if details:
            print(f"   {details}")
            
    def test_file_structure(self):
        """Test that all expected files exist"""
        print("\n🔍 Testing File Structure...")
        
        expected_files = {
            # Core system
            "camera_control_node.py": "Main Isaac Sim camera control node",
            "camera_control_sender.py": "Command-line movement tool",
            
            # Launch scripts
            "launch_camera_control.sh": "Main system launcher",
            "launch_ros2_camera_test.sh": "Launch with automated tests",
            
            # Testing
            "test_camera_control.py": "File-based movement tests",
            "test_ros2_camera_control.py": "ROS2 topic tests",
            "comprehensive_camera_test.py": "Complete test suite",
            
            # Documentation
            "README.md": "Main overview guide",
            "CAMERA_CONTROL_README.md": "Technical documentation",
            "WORKSPACE_STRUCTURE.md": "Workspace organization guide",
            "CLEANUP_SUMMARY.md": "Cleanup documentation"
        }
        
        for filename, description in expected_files.items():
            file_path = self.workspace_path / filename
            if file_path.exists():
                size = file_path.stat().st_size
                self.log_result(f"File: {filename}", "PASS", f"{description} ({size} bytes)")
            else:
                self.log_result(f"File: {filename}", "FAIL", f"Missing: {description}")
                
    def test_obsolete_files_removed(self):
        """Test that obsolete files were properly removed"""
        print("\n🗑️ Testing Obsolete File Removal...")
        
        obsolete_files = [
            "isaac_camera_node_final.py",
            "isaac_camera_working_movable.py", 
            "camera_command_sender.py",
            "launch_complete_system.sh",
            "launch_movable_camera.sh",
            "launch_simple_camera.sh",
            "test_camera_movement.sh",
            "test_movement.sh",
            "isaac_sim_output.log",
            "cleanup_workspace.sh"
        ]
        
        for filename in obsolete_files:
            file_path = self.workspace_path / filename
            if not file_path.exists():
                self.log_result(f"Removed: {filename}", "PASS", "Obsolete file properly removed")
            else:
                self.log_result(f"Removed: {filename}", "FAIL", "Obsolete file still exists")
                
    def test_python_syntax(self):
        """Test that all Python files compile without syntax errors"""
        print("\n🐍 Testing Python Syntax...")
        
        python_files = [
            "camera_control_node.py",
            "camera_control_sender.py",
            "test_camera_control.py",
            "test_ros2_camera_control.py",
            "comprehensive_camera_test.py"
        ]
        
        for filename in python_files:
            file_path = self.workspace_path / filename
            if file_path.exists():
                try:
                    result = subprocess.run([
                        sys.executable, "-m", "py_compile", str(file_path)
                    ], capture_output=True, text=True, cwd=self.workspace_path)
                    
                    if result.returncode == 0:
                        self.log_result(f"Syntax: {filename}", "PASS", "No syntax errors")
                    else:
                        self.log_result(f"Syntax: {filename}", "FAIL", f"Syntax error: {result.stderr}")
                except Exception as e:
                    self.log_result(f"Syntax: {filename}", "FAIL", f"Exception: {e}")
            else:
                self.log_result(f"Syntax: {filename}", "FAIL", "File missing")
                
    def test_launch_scripts(self):
        """Test that launch scripts are executable and well-formed"""
        print("\n🚀 Testing Launch Scripts...")
        
        launch_scripts = [
            "launch_camera_control.sh",
            "launch_ros2_camera_test.sh"
        ]
        
        for script in launch_scripts:
            script_path = self.workspace_path / script
            if script_path.exists():
                # Check if executable
                if os.access(script_path, os.X_OK):
                    self.log_result(f"Executable: {script}", "PASS", "Script has execute permissions")
                else:
                    self.log_result(f"Executable: {script}", "FAIL", "Script missing execute permissions")
                    
                # Check bash syntax
                try:
                    result = subprocess.run([
                        "bash", "-n", str(script_path)
                    ], capture_output=True, text=True)
                    
                    if result.returncode == 0:
                        self.log_result(f"Bash Syntax: {script}", "PASS", "Valid bash syntax")
                    else:
                        self.log_result(f"Bash Syntax: {script}", "FAIL", f"Bash error: {result.stderr}")
                except Exception as e:
                    self.log_result(f"Bash Syntax: {script}", "FAIL", f"Exception: {e}")
            else:
                self.log_result(f"Launch Script: {script}", "FAIL", "Script missing")
                
    def test_documentation_updated(self):
        """Test that documentation reflects the cleaned structure"""
        print("\n📚 Testing Documentation Updates...")
        
        # Check README.md for updated content
        readme_path = self.workspace_path / "README.md"
        if readme_path.exists():
            content = readme_path.read_text()
            
            # Check for updated launch command
            if "launch_camera_control.sh" in content:
                self.log_result("README Launch Command", "PASS", "Updated launch script referenced")
            else:
                self.log_result("README Launch Command", "FAIL", "Old launch script still referenced")
                
            # Check for updated file structure
            if "camera_control_node.py" in content:
                self.log_result("README File References", "PASS", "Updated file names referenced")
            else:
                self.log_result("README File References", "FAIL", "Old file names still referenced")
        else:
            self.log_result("README.md", "FAIL", "README.md missing")
            
    def test_system_integration(self):
        """Test basic system integration points"""
        print("\n🔧 Testing System Integration...")
        
        # Test command file location
        command_file = Path("/tmp/isaac_camera_commands.json")
        try:
            # Create a test command
            test_command = {
                "timestamp": 1234567890,
                "command": "test",
                "position": [1.0, 2.0, 3.0]
            }
            
            with open(command_file, 'w') as f:
                json.dump(test_command, f)
                
            # Verify it can be read back
            with open(command_file, 'r') as f:
                loaded_command = json.load(f)
                
            if loaded_command == test_command:
                self.log_result("Command File I/O", "PASS", "JSON command file read/write works")
            else:
                self.log_result("Command File I/O", "FAIL", "Command file data mismatch")
                
            # Clean up
            command_file.unlink()
            
        except Exception as e:
            self.log_result("Command File I/O", "FAIL", f"Exception: {e}")
            
    def test_ros2_environment(self):
        """Test ROS2 environment availability"""
        print("\n🤖 Testing ROS2 Environment...")
        
        # Check if ROS2 is available
        try:
            result = subprocess.run(['which', 'ros2'], capture_output=True, text=True)
            if result.returncode == 0:
                self.log_result("ROS2 Available", "PASS", f"Found at: {result.stdout.strip()}")
            else:
                self.log_result("ROS2 Available", "FAIL", "ros2 command not found")
        except Exception as e:
            self.log_result("ROS2 Available", "FAIL", f"Exception: {e}")
            
    def generate_report(self):
        """Generate a comprehensive test report"""
        print("\n" + "="*60)
        print("📊 SYSTEM VALIDATION REPORT")
        print("="*60)
        
        # Count results
        total_tests = len(self.test_results)
        passed_tests = len([r for r in self.test_results if r['status'] == 'PASS'])
        failed_tests = len([r for r in self.test_results if r['status'] == 'FAIL'])
        
        print(f"Total Tests: {total_tests}")
        print(f"Passed: {passed_tests}")
        print(f"Failed: {failed_tests}")
        
        if total_tests > 0:
            success_rate = (passed_tests / total_tests) * 100
            print(f"Success Rate: {success_rate:.1f}%")
        
        # Overall status
        if failed_tests == 0:
            print("\n🎉 SYSTEM STATUS: FULLY VALIDATED")
        elif failed_tests <= 2:
            print("\n⚠️ SYSTEM STATUS: MOSTLY FUNCTIONAL (minor issues)")
        else:
            print("\n❌ SYSTEM STATUS: ISSUES DETECTED")
            
        # Save detailed report
        report_file = self.workspace_path / "VALIDATION_REPORT.json"
        with open(report_file, 'w') as f:
            json.dump(self.test_results, f, indent=2)
            
        print(f"\n📝 Detailed report saved to: {report_file}")
        
        return failed_tests == 0

def main():
    print("🎯 Isaac Sim Camera Control - System Validation")
    print("=" * 60)
    
    validator = SystemValidator()
    
    # Run all validation tests
    validator.test_file_structure()
    validator.test_obsolete_files_removed()
    validator.test_python_syntax()
    validator.test_launch_scripts()
    validator.test_documentation_updated()
    validator.test_system_integration()
    validator.test_ros2_environment()
    
    # Generate final report
    system_valid = validator.generate_report()
    
    return 0 if system_valid else 1

if __name__ == "__main__":
    sys.exit(main())
