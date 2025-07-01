#!/usr/bin/env python3
"""
Isaac Sim Scenario Demo - Complete User Experience
Shows end-to-end workflow from natural language to running simulation
"""

import time
import json
from pathlib import Path
from scenario_manager import ScenarioManager

def demo_complete_workflow():
    """Demonstrate complete user workflow"""
    print("🎬 Isaac Sim Scenario System - Complete User Experience Demo")
    print("=" * 70)
    
    # Step 1: Initialize the scenario system
    print("\n📋 Step 1: Initialize Scenario System")
    print("-" * 40)
    manager = ScenarioManager(enable_logging=False)  # Quiet for demo
    
    # Step 2: User provides natural language request
    print("\n🗣️  Step 2: User Natural Language Request")
    print("-" * 40)
    
    user_requests = [
        "warehouse scene with carter 2.0 robot that is fully ros controlled",
        "office environment with franka arm for manipulation tasks",
        "simple room with overhead camera view"
    ]
    
    print("Available user requests:")
    for i, request in enumerate(user_requests, 1):
        print(f"  {i}. '{request}'")
    
    # Let's use the first scenario for demo
    selected_request = user_requests[0]
    print(f"\n✅ Selected: '{selected_request}'")
    
    # Step 3: Parse and validate scenario
    print("\n🔍 Step 3: Parse and Validate Scenario")
    print("-" * 40)
    config = manager.generate_scenario_config(selected_request)
    
    # Step 4: Save configuration for Isaac Sim
    print("\n💾 Step 4: Save Configuration for Isaac Sim")
    print("-" * 40)
    manager.save_scenario_config(config)
    print("Configuration ready for Isaac Sim camera control node")
    
    # Step 5: Show what would happen next
    print("\n🚀 Step 5: Launch Isaac Sim with Enhanced Camera Control")
    print("-" * 40)
    print("Now you would run:")
    print("  ./launch_camera_control_with_scenarios.sh")
    print("\nThis will:")
    print("  ✅ Launch Isaac Sim")
    print("  ✅ Load the warehouse environment")
    print("  ✅ Add Carter 2.0 robot with ROS2 control")
    print("  ✅ Enable camera control via ROS2 topics")
    print("  ✅ Start publishing camera data")
    
    # Step 6: Show ROS2 interaction
    print("\n🤖 Step 6: ROS2 Control Interface")
    print("-" * 40)
    scenario = config['parsed_scenario']
    
    print("Available ROS2 topics:")
    print("  📷 Camera Topics:")
    print("    - /camera/rgb (sensor_msgs/Image)")
    print("    - /camera/depth (sensor_msgs/Image)")
    print("    - /camera/camera_info (sensor_msgs/CameraInfo)")
    print("    - /camera/current_pose (geometry_msgs/PoseStamped)")
    
    print("  🎮 Control Topics:")
    print("    - /camera/cmd_vel (geometry_msgs/Twist)")
    print("    - /camera/set_pose (geometry_msgs/PoseStamped)")
    
    if scenario['robots']:
        print(f"  🤖 Robot Topics (for {len(scenario['robots'])} robot(s)):")
        for robot in scenario['robots']:
            if robot['ros_enabled']:
                robot_name = robot['name']
                print(f"    - /{robot_name}/cmd_vel (geometry_msgs/Twist)")
                print(f"    - /{robot_name}/odom (nav_msgs/Odometry)")
    
    # Step 7: Show example commands
    print("\n💡 Step 7: Example Commands")
    print("-" * 40)
    print("Camera control examples:")
    print("  # Move camera forward")
    print("  ros2 topic pub --once /camera/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}}'")
    print("  # Set overhead view")
    print("  ros2 topic pub --once /camera/set_pose geometry_msgs/msg/PoseStamped '{pose: {position: {x: 0.0, y: 0.0, z: 10.0}}}'")
    
    if scenario['robots'] and any(r['ros_enabled'] for r in scenario['robots']):
        print("\nRobot control examples:")
        for robot in scenario['robots']:
            if robot['ros_enabled']:
                print(f"  # Move {robot['type']} robot")
                print(f"  ros2 topic pub --once /{robot['name']}/cmd_vel geometry_msgs/msg/Twist '{{linear: {{x: 0.5}}}}'")
    
    # Step 8: Show scenario CLI usage
    print("\n🛠️  Step 8: Scenario Management CLI")
    print("-" * 40)
    print("Change scenarios on the fly:")
    print("  # Request new scenario")
    print("  python3 scenario_cli.py request -s 'office with franka arm'")
    print("  # Activate scenario")
    print("  python3 scenario_cli.py activate")
    print("  # Monitor status")
    print("  python3 scenario_status_monitor.py")
    
    # Step 9: Show expected Isaac Sim behavior
    print("\n🎯 Step 9: Expected Isaac Sim Behavior")
    print("-" * 40)
    print("When Isaac Sim launches, you'll see:")
    print("  1. 🏗️  Warehouse environment loads")
    print("  2. 🤖 Carter 2.0 robot appears at origin [0,0,0]")
    print("  3. 📷 Camera positioned at [2,2,2] (or scenario-specified position)")
    print("  4. 🔄 Real-time camera movement when you send ROS2 commands")
    print("  5. 📊 Live data streaming to ROS2 topics")
    print("  6. 🎮 Robot responds to ROS2 control commands")
    
    return config

def create_launch_example():
    """Create a specific launch example"""
    print("\n" + "=" * 70)
    print("🚀 LAUNCH EXAMPLE - Ready to Run!")
    print("=" * 70)
    
    print("\n1️⃣  Create scenario configuration:")
    print("   python3 -c \"")
    print("   from scenario_manager import ScenarioManager")
    print("   manager = ScenarioManager(enable_logging=False)")
    print("   config = manager.generate_scenario_config('warehouse with carter 2.0 robot fully ros controlled')")
    print("   manager.save_scenario_config(config)")
    print("   print('✅ Scenario configured')")
    print("   \"")
    
    print("\n2️⃣  Launch Isaac Sim with scenario:")
    print("   ./launch_camera_control_with_scenarios.sh")
    
    print("\n3️⃣  In another terminal, control the camera:")
    print("   # Move camera around the warehouse")
    print("   ros2 topic pub --once /camera/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 2.0}}'")
    print("   ros2 topic pub --once /camera/cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.5}}'")
    
    print("\n4️⃣  Control the Carter robot:")
    print("   # Make the robot move in the warehouse")
    print("   ros2 topic pub --once /carter_2_robot/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}}'")
    
    print("\n5️⃣  Monitor everything:")
    print("   # Watch camera feed")
    print("   ros2 topic echo /camera/rgb")
    print("   # Monitor robot position")
    print("   ros2 topic echo /carter_2_robot/odom")
    print("   # Check system status")
    print("   python3 scenario_status_monitor.py")

def main():
    """Main demo function"""
    try:
        # Run the complete workflow demo
        config = demo_complete_workflow()
        
        # Create launch example
        create_launch_example()
        
        print("\n" + "=" * 70)
        print("✅ DEMO COMPLETE - System Ready for Isaac Sim Launch!")
        print("=" * 70)
        
        print(f"\n📋 Summary:")
        scenario = config['parsed_scenario']
        print(f"   Scenario: {scenario['scene']} environment")
        print(f"   Robots: {len(scenario['robots'])} robot(s)")
        print(f"   ROS2 Enabled: {any(r['ros_enabled'] for r in scenario['robots'])}")
        print(f"   Special Requirements: {scenario['special_requirements']}")
        
        print(f"\n🎯 Next Steps:")
        print(f"   1. Ensure Isaac Sim is installed ($ISAAC_SIM_PATH set)")
        print(f"   2. Source ROS2 environment")
        print(f"   3. Run: ./launch_camera_control_with_scenarios.sh")
        print(f"   4. Use ROS2 commands to control camera and robots")
        
        print(f"\n💡 The magic happens when Isaac Sim loads the scenario and")
        print(f"   you see the warehouse with the Carter robot responding to")
        print(f"   your ROS2 commands in real-time!")
        
    except Exception as e:
        print(f"\n❌ Demo failed: {e}")
        return False
    
    return True

if __name__ == "__main__":
    main()
