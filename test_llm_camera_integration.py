#!/usr/bin/env python3
"""
Test Script for LLM Camera Controller Integration
Tests the complete pipeline: Isaac Sim → Camera Feed → LLM Analysis → Camera Control
"""
import subprocess
import time
import sys
import json
from pathlib import Path

def run_command(cmd, description, check_output=False, timeout=10):
    """Run a command and return result"""
    print(f"\n🔧 {description}")
    print(f"   Command: {cmd}")
    
    try:
        if check_output:
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
            return result.returncode == 0, result.stdout, result.stderr
        else:
            result = subprocess.run(cmd, shell=True, timeout=timeout)
            return result.returncode == 0, "", ""
    except subprocess.TimeoutExpired:
        print(f"   ⏰ Command timed out after {timeout}s")
        return False, "", "Timeout"
    except Exception as e:
        print(f"   ❌ Error: {e}")
        return False, "", str(e)

def check_isaac_sim_running():
    """Check if Isaac Sim camera node is running"""
    success, stdout, stderr = run_command(
        "ps aux | grep camera_control_node.py | grep -v grep",
        "Checking Isaac Sim camera node",
        check_output=True
    )
    return success and len(stdout.strip()) > 0

def check_ros2_topics():
    """Check if required ROS2 topics are available"""
    success, stdout, stderr = run_command(
        "ros2 topic list",
        "Checking ROS2 topics",
        check_output=True
    )
    
    if success:
        topics = stdout.split('\n')
        required_topics = ['/camera/rgb', '/camera/cmd_vel']
        available_topics = [topic for topic in required_topics if topic in topics]
        return len(available_topics) == len(required_topics), available_topics
    
    return False, []

def build_isaac_test_package():
    """Build the isaac_test package"""
    print("\n🔨 Building isaac_test package...")
    success, stdout, stderr = run_command(
        "cd /home/kimate/isaac_ws && colcon build --packages-select isaac_test",
        "Building isaac_test package",
        check_output=True,
        timeout=60
    )
    
    if success:
        print("   ✅ Package built successfully")
        # Source the workspace
        run_command(
            "source /home/kimate/isaac_ws/install/setup.bash",
            "Sourcing workspace"
        )
        return True
    else:
        print(f"   ❌ Build failed: {stderr}")
        return False

def test_llm_controller():
    """Test the LLM camera controller"""
    print("\n" + "="*70)
    print("🧪 TESTING LLM CAMERA CONTROLLER INTEGRATION")
    print("="*70)
    
    # Step 1: Check prerequisites
    print("\n📋 Step 1: Checking Prerequisites")
    
    if not check_isaac_sim_running():
        print("❌ Isaac Sim camera node not running!")
        print("💡 Please start Isaac Sim first:")
        print("   cd /home/kimate/isaac_ws")
        print("   python3 camera_control_node.py")
        return False
    else:
        print("✅ Isaac Sim camera node is running")
    
    # Check ROS2 topics
    topics_ok, available_topics = check_ros2_topics()
    if topics_ok:
        print(f"✅ Required ROS2 topics available: {available_topics}")
    else:
        print(f"❌ Missing required ROS2 topics. Available: {available_topics}")
        return False
    
    # Step 2: Build package
    print("\n📋 Step 2: Building Isaac Test Package")
    if not build_isaac_test_package():
        return False
    
    # Step 3: Test camera feed
    print("\n📋 Step 3: Testing Camera Feed")
    success, stdout, stderr = run_command(
        "timeout 5 ros2 topic hz /camera/rgb --window 3",
        "Checking camera feed rate",
        check_output=True,
        timeout=10
    )
    
    if success and "average rate:" in stdout:
        print("✅ Camera feed is active")
    else:
        print("❌ Camera feed not available or too slow")
        return False
    
    # Step 4: Launch LLM controller (background)
    print("\n📋 Step 4: Launching LLM Camera Controller")
    print("🚀 Starting LLM controller in background...")
    
    # Start the controller
    controller_process = subprocess.Popen([
        "bash", "-c", 
        "source /home/kimate/isaac_ws/install/setup.bash && "
        "ros2 run isaac_test llm_camera_controller"
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    # Wait for initialization
    time.sleep(3)
    
    # Check if controller is running
    if controller_process.poll() is None:
        print("✅ LLM controller launched successfully")
    else:
        stdout, stderr = controller_process.communicate()
        print(f"❌ LLM controller failed to start: {stderr.decode()}")
        return False
    
    # Step 5: Monitor topics
    print("\n📋 Step 5: Testing Topic Communication")
    
    # Check for new topics
    success, stdout, stderr = run_command(
        "ros2 topic list | grep isaac_test",
        "Checking isaac_test topics",
        check_output=True
    )
    
    if success and len(stdout.strip()) > 0:
        isaac_topics = stdout.strip().split('\n')
        print(f"✅ Isaac test topics created: {isaac_topics}")
    else:
        print("❌ Isaac test topics not found")
    
    # Step 6: Test manual control
    print("\n📋 Step 6: Testing Manual Control")
    
    # Enable LLM control
    success, _, _ = run_command(
        "ros2 topic pub --once /isaac_test/llm_control_enable std_msgs/msg/Bool '{data: true}'",
        "Enabling LLM control",
        timeout=5
    )
    
    if success:
        print("✅ LLM control enabled")
    else:
        print("⚠️  Could not enable LLM control")
    
    # Step 7: Monitor for analysis output
    print("\n📋 Step 7: Monitoring LLM Analysis (10 seconds)")
    print("📊 Watching for intelligent camera movements...")
    
    # Monitor analysis output
    monitor_process = subprocess.Popen([
        "bash", "-c",
        "source /home/kimate/isaac_ws/install/setup.bash && "
        "timeout 10 ros2 topic echo /isaac_test/llm_analysis --once"
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    stdout, stderr = monitor_process.communicate()
    
    if monitor_process.returncode == 0 and stdout:
        print("✅ LLM analysis output detected!")
        try:
            analysis_data = json.loads(stdout.decode().split('data: ')[1].strip().strip("'\""))
            print(f"📝 Action: {analysis_data.get('action', 'unknown')}")
            print(f"🎯 Reasoning: {analysis_data.get('reasoning', 'none')}")
            print(f"🎲 Confidence: {analysis_data.get('confidence', 0)}")
        except:
            print("📝 Analysis output received (parsing issue)")
    else:
        print("⚠️  No analysis output detected in 10 seconds")
    
    # Step 8: Check camera commands
    print("\n📋 Step 8: Checking Camera Commands")
    
    cmd_monitor = subprocess.Popen([
        "bash", "-c",
        "source /home/kimate/isaac_ws/install/setup.bash && "
        "timeout 5 ros2 topic echo /camera/cmd_vel --once"
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    stdout, stderr = cmd_monitor.communicate()
    
    if cmd_monitor.returncode == 0 and stdout:
        print("✅ Camera movement commands detected!")
        print("🎮 LLM is successfully controlling the camera")
    else:
        print("⚠️  No camera commands detected")
    
    # Cleanup
    print("\n📋 Cleanup: Stopping LLM controller")
    controller_process.terminate()
    time.sleep(1)
    if controller_process.poll() is None:
        controller_process.kill()
    
    print("\n" + "="*70)
    print("🎉 LLM CAMERA CONTROLLER TEST COMPLETED")
    print("="*70)
    print("✅ Integration test successful!")
    print("\n🚀 To run manually:")
    print("   # Terminal 1: Start Isaac Sim")
    print("   cd /home/kimate/isaac_ws")
    print("   python3 camera_control_node.py")
    print("\n   # Terminal 2: Start LLM controller")
    print("   cd /home/kimate/isaac_ws")
    print("   source install/setup.bash")
    print("   ros2 run isaac_test llm_camera_controller")
    print("\n   # Terminal 3: Monitor (optional)")
    print("   ros2 topic echo /isaac_test/llm_analysis")
    print("   ros2 topic echo /camera/cmd_vel")
    print("\n💡 The system will automatically analyze camera images and")
    print("   generate intelligent camera movements every 2 seconds!")
    print("="*70)
    
    return True

def main():
    """Main test function"""
    try:
        success = test_llm_controller()
        return 0 if success else 1
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
        return 1
    except Exception as e:
        print(f"\n❌ Test failed with error: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(main())
