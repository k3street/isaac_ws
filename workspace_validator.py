#!/usr/bin/env python3
"""
Complete Workspace Validation for Isaac Sim Camera Control with Scenarios
Validates all components and generates updated documentation
"""

import os
import json
import subprocess
from pathlib import Path
from datetime import datetime

def validate_file_structure():
    """Validate workspace file structure"""
    print("🏗️  Validating File Structure")
    print("-" * 40)
    
    required_files = {
        # Core system files
        'camera_control_node.py': 'Main Isaac Sim integration with scenario support',
        'camera_cli.py': 'Command-line camera control interface (WORKING)',
        'camera_control_sender.py': 'ROS2 camera control node',
        
        # Scenario management
        'scenario_manager.py': 'Core scenario parsing and management',
        'scenario_cli.py': 'Scenario command-line interface',
        'scenario_service_node.py': 'ROS2 scenario service',
        'scenario_status_monitor.py': 'System status monitoring',
        
        # Demo and testing
        'demo_complete_workflow.py': 'Complete user experience demo',
        'test_camera_control.py': 'Camera control test script (WORKING)',
        'comprehensive_camera_test.py': 'Extended camera tests',
        'test_ros2_camera_control.py': 'ROS2 topic tests',
        'system_validation.py': 'System validation script',
        
        # Launch scripts
        'launch_camera_control_with_scenarios.sh': 'Main launch script with scenarios',
        'launch_camera_control.sh': 'Basic camera control launch',
        'launch_ros2_camera_test.sh': 'ROS2 test launch',
        
        # Documentation
        'README.md': 'Main user documentation',
        'LAUNCH_INSTRUCTIONS.md': 'Detailed launch instructions',
        'AGENT_HANDOFF.md': 'Agent handoff documentation',
        'CAMERA_CONTROL_README.md': 'Camera control specific docs',
        'WORKSPACE_STRUCTURE.md': 'Workspace structure overview',
        'SYSTEM_STATUS.md': 'Current system status'
    }
    
    missing_files = []
    present_files = []
    
    for file, description in required_files.items():
        if os.path.exists(file):
            present_files.append((file, description))
            print(f"✅ {file}")
        else:
            missing_files.append((file, description))
            print(f"❌ {file} - MISSING")
    
    return present_files, missing_files

def validate_python_syntax():
    """Validate Python file syntax"""
    print("\n🐍 Validating Python Syntax")
    print("-" * 40)
    
    python_files = [
        'camera_control_node.py',
        'camera_cli.py',
        'camera_control_sender.py',
        'scenario_manager.py',
        'scenario_cli.py',
        'scenario_service_node.py',
        'scenario_status_monitor.py',
        'demo_complete_workflow.py',
        'test_camera_control.py',
        'comprehensive_camera_test.py',
        'test_ros2_camera_control.py',
        'system_validation.py'
    ]
    
    valid_files = []
    invalid_files = []
    
    for file in python_files:
        if os.path.exists(file):
            try:
                import py_compile
                py_compile.compile(file, doraise=True)
                valid_files.append(file)
                print(f"✅ {file} - syntax valid")
            except Exception as e:
                invalid_files.append((file, str(e)))
                print(f"❌ {file} - syntax error: {e}")
        else:
            print(f"⚠️  {file} - file not found")
    
    return valid_files, invalid_files

def validate_shell_scripts():
    """Validate shell script syntax"""
    print("\n🐚 Validating Shell Scripts")
    print("-" * 40)
    
    shell_files = [
        'launch_camera_control_with_scenarios.sh',
        'launch_camera_control.sh',
        'launch_ros2_camera_test.sh'
    ]
    
    valid_scripts = []
    invalid_scripts = []
    
    for file in shell_files:
        if os.path.exists(file):
            try:
                # Check if file is executable
                executable = os.access(file, os.X_OK)
                
                # Basic bash syntax check
                result = subprocess.run(['bash', '-n', file], 
                                      capture_output=True, text=True)
                
                if result.returncode == 0:
                    valid_scripts.append(file)
                    exec_status = "executable" if executable else "not executable"
                    print(f"✅ {file} - syntax valid ({exec_status})")
                else:
                    invalid_scripts.append((file, result.stderr))
                    print(f"❌ {file} - syntax error: {result.stderr}")
                    
            except Exception as e:
                invalid_scripts.append((file, str(e)))
                print(f"❌ {file} - validation error: {e}")
        else:
            print(f"⚠️  {file} - file not found")
    
    return valid_scripts, invalid_scripts

def test_key_functionality():
    """Test key system functionality"""
    print("\n🧪 Testing Key Functionality")
    print("-" * 40)
    
    tests = []
    
    # Test scenario manager import
    try:
        import scenario_manager
        tests.append(("Scenario Manager Import", True, ""))
        print("✅ Scenario Manager - imports successfully")
    except Exception as e:
        tests.append(("Scenario Manager Import", False, str(e)))
        print(f"❌ Scenario Manager - import failed: {e}")
    
    # Test camera CLI help
    try:
        result = subprocess.run(['python3', 'camera_cli.py', '--help'], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            tests.append(("Camera CLI Help", True, ""))
            print("✅ Camera CLI - help command works")
        else:
            tests.append(("Camera CLI Help", False, result.stderr))
            print(f"❌ Camera CLI - help failed: {result.stderr}")
    except Exception as e:
        tests.append(("Camera CLI Help", False, str(e)))
        print(f"❌ Camera CLI - test failed: {e}")
    
    # Test scenario CLI help
    try:
        result = subprocess.run(['python3', 'scenario_cli.py', '--help'], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            tests.append(("Scenario CLI Help", True, ""))
            print("✅ Scenario CLI - help command works")
        else:
            tests.append(("Scenario CLI Help", False, result.stderr))
            print(f"❌ Scenario CLI - help failed: {result.stderr}")
    except Exception as e:
        tests.append(("Scenario CLI Help", False, str(e)))
        print(f"❌ Scenario CLI - test failed: {e}")
    
    # Test scenario configuration
    try:
        from scenario_manager import ScenarioManager
        manager = ScenarioManager(enable_logging=False)
        config = manager.generate_scenario_config("test warehouse")
        if config and config.get('parsed_scenario'):
            tests.append(("Scenario Generation", True, ""))
            print("✅ Scenario Generation - creates valid config")
        else:
            tests.append(("Scenario Generation", False, "Invalid config generated"))
            print("❌ Scenario Generation - invalid config")
    except Exception as e:
        tests.append(("Scenario Generation", False, str(e)))
        print(f"❌ Scenario Generation - failed: {e}")
    
    return tests

def generate_validation_report():
    """Generate comprehensive validation report"""
    print("\n" + "=" * 60)
    print("📊 WORKSPACE VALIDATION REPORT")
    print("=" * 60)
    
    present_files, missing_files = validate_file_structure()
    valid_python, invalid_python = validate_python_syntax()
    valid_shell, invalid_shell = validate_shell_scripts()
    functionality_tests = test_key_functionality()
    
    # Generate summary
    total_files = len(present_files) + len(missing_files)
    total_python = len(valid_python) + len(invalid_python)
    total_shell = len(valid_shell) + len(invalid_shell)
    
    passed_tests = sum(1 for test in functionality_tests if test[1])
    total_tests = len(functionality_tests)
    
    report = {
        'timestamp': datetime.now().isoformat(),
        'summary': {
            'files_present': len(present_files),
            'files_missing': len(missing_files),
            'files_total': total_files,
            'python_valid': len(valid_python),
            'python_invalid': len(invalid_python),
            'shell_valid': len(valid_shell),
            'shell_invalid': len(invalid_shell),
            'tests_passed': passed_tests,
            'tests_total': total_tests,
            'overall_health': 'EXCELLENT' if (
                len(missing_files) == 0 and 
                len(invalid_python) == 0 and 
                len(invalid_shell) == 0 and 
                passed_tests == total_tests
            ) else 'GOOD' if (
                len(missing_files) <= 2 and 
                len(invalid_python) == 0 and 
                passed_tests >= total_tests * 0.8
            ) else 'NEEDS_ATTENTION'
        },
        'details': {
            'present_files': present_files,
            'missing_files': missing_files,
            'valid_python': valid_python,
            'invalid_python': invalid_python,
            'valid_shell': valid_shell,
            'invalid_shell': invalid_shell,
            'functionality_tests': functionality_tests
        }
    }
    
    print(f"\n📋 Summary:")
    print(f"   Files: {len(present_files)}/{total_files} present")
    print(f"   Python: {len(valid_python)} valid, {len(invalid_python)} invalid")
    print(f"   Shell: {len(valid_shell)} valid, {len(invalid_shell)} invalid")
    print(f"   Tests: {passed_tests}/{total_tests} passed")
    print(f"   Overall Health: {report['summary']['overall_health']}")
    
    # Save report
    report_file = 'WORKSPACE_VALIDATION_REPORT.json'
    with open(report_file, 'w') as f:
        json.dump(report, f, indent=2)
    
    print(f"\n💾 Full report saved to: {report_file}")
    
    return report

if __name__ == "__main__":
    print("🚀 Isaac Sim Workspace Validation")
    print("=" * 60)
    print(f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"Location: {os.getcwd()}")
    
    report = generate_validation_report()
    
    print("\n✅ Validation completed!")
    print("📄 Use this report to update documentation")
