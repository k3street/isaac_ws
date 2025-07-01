#!/usr/bin/env python3
"""
Camera Control Test Script
Demonstrates various movement commands for the Isaac Sim Camera Control System
"""

import time
import json
from pathlib import Path

def send_command(command_data, description=""):
    """Send a command to the camera control system"""
    command_file = Path("/tmp/isaac_camera_commands.json")
    
    print(f"📍 {description}")
    print(f"   Command: {json.dumps(command_data, indent=2)}")
    
    # Write command to file
    with open(command_file, 'w') as f:
        json.dump(command_data, f)
    
    print("   ✅ Command sent")
    time.sleep(3)  # Wait for processing
    print()

def main():
    """Run camera movement test sequence"""
    print("🎮 Isaac Sim Camera Control Test Sequence")
    print("=" * 50)
    print("Make sure the camera control node is running!")
    print("Launch with: ./launch_camera_control.sh")
    print()
    
    input("Press Enter to start the test sequence...")
    print()
    
    # Test sequence
    commands = [
        {
            "data": {"type": "position", "data": {"x": 0.0, "y": 0.0, "z": 2.0}},
            "description": "Move to origin (center of scene)"
        },
        {
            "data": {"type": "position", "data": {"x": 5.0, "y": 0.0, "z": 2.5}},
            "description": "Move to right side of scene"
        },
        {
            "data": {"type": "position", "data": {"x": 0.0, "y": 5.0, "z": 3.0}},
            "description": "Move to back of scene (higher view)"
        },
        {
            "data": {"type": "position", "data": {"x": -3.0, "y": -3.0, "z": 1.5}},
            "description": "Move to front-left corner (lower view)"
        },
        {
            "data": {"type": "position", "data": {"x": 2.0, "y": 2.0, "z": 4.0}},
            "description": "Move to elevated diagonal view"
        },
        {
            "data": {"type": "position", "data": {"x": 2.0, "y": 2.0, "z": 2.0}},
            "description": "Return to default position"
        }
    ]
    
    for i, cmd in enumerate(commands, 1):
        print(f"Step {i}/{len(commands)}")
        send_command(cmd["data"], cmd["description"])
    
    print("🎉 Test sequence completed!")
    print("Check the Isaac Sim viewport to see the camera movements.")
    print("Monitor the console output for position feedback.")

if __name__ == "__main__":
    main()
