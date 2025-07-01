#!/usr/bin/env python3
"""
Camera Control CLI Tool
Direct command-line interface for Isaac Sim camera control
"""

import json
import argparse
import time
from pathlib import Path

class CameraControlCLI:
    def __init__(self):
        self.command_file = Path("/tmp/isaac_camera_commands.json")
        
    def send_command(self, command_data, description=""):
        """Send a command to the camera control system"""
        print(f"📍 {description}")
        print(f"   Command: {json.dumps(command_data, indent=2)}")
        
        # Write command to file
        try:
            with open(self.command_file, 'w') as f:
                json.dump(command_data, f)
            print(f"   ✅ Command sent to {self.command_file}")
        except Exception as e:
            print(f"   ❌ Failed to send command: {e}")
            return False
        
        return True
    
    def move(self, x=0.0, y=0.0, z=0.0):
        """Move camera by relative amounts"""
        command = {
            "type": "velocity",
            "data": {
                "linear_x": x,
                "linear_y": y, 
                "linear_z": z,
                "angular_x": 0.0,
                "angular_y": 0.0,
                "angular_z": 0.0
            }
        }
        
        description = f"Move camera: x={x}, y={y}, z={z}"
        return self.send_command(command, description)
    
    def position(self, x=2.0, y=2.0, z=2.0):
        """Set camera to absolute position"""
        command = {
            "type": "position",
            "data": {
                "x": x,
                "y": y,
                "z": z
            }
        }
        
        description = f"Set camera position: ({x}, {y}, {z})"
        return self.send_command(command, description)
    
    def rotate(self, pitch=0.0, yaw=0.0, roll=0.0):
        """Rotate camera"""
        command = {
            "type": "velocity", 
            "data": {
                "linear_x": 0.0,
                "linear_y": 0.0,
                "linear_z": 0.0,
                "angular_x": pitch,
                "angular_y": yaw, 
                "angular_z": roll
            }
        }
        
        description = f"Rotate camera: pitch={pitch}, yaw={yaw}, roll={roll}"
        return self.send_command(command, description)
    
    def reset(self):
        """Reset camera to default position"""
        return self.position(2.0, 2.0, 2.0)

def main():
    parser = argparse.ArgumentParser(description='Isaac Sim Camera Control CLI')
    subparsers = parser.add_subparsers(dest='command', help='Available commands')
    
    # Move command
    move_parser = subparsers.add_parser('move', help='Move camera by relative amounts')
    move_parser.add_argument('--x', type=float, default=0.0, help='Move in X direction')
    move_parser.add_argument('--y', type=float, default=0.0, help='Move in Y direction') 
    move_parser.add_argument('--z', type=float, default=0.0, help='Move in Z direction')
    
    # Position command
    pos_parser = subparsers.add_parser('position', help='Set camera to absolute position')
    pos_parser.add_argument('--x', type=float, default=2.0, help='X coordinate')
    pos_parser.add_argument('--y', type=float, default=2.0, help='Y coordinate')
    pos_parser.add_argument('--z', type=float, default=2.0, help='Z coordinate')
    
    # Rotate command
    rot_parser = subparsers.add_parser('rotate', help='Rotate camera')
    rot_parser.add_argument('--pitch', type=float, default=0.0, help='Pitch rotation (degrees)')
    rot_parser.add_argument('--yaw', type=float, default=0.0, help='Yaw rotation (degrees)')
    rot_parser.add_argument('--roll', type=float, default=0.0, help='Roll rotation (degrees)')
    
    # Reset command
    subparsers.add_parser('reset', help='Reset camera to default position')
    
    # Quick commands
    subparsers.add_parser('up', help='Move camera up')
    subparsers.add_parser('down', help='Move camera down')
    subparsers.add_parser('forward', help='Move camera forward')
    subparsers.add_parser('backward', help='Move camera backward')
    subparsers.add_parser('left', help='Move camera left')
    subparsers.add_parser('right', help='Move camera right')
    subparsers.add_parser('overhead', help='Set overhead view')
    
    args = parser.parse_args()
    
    if not args.command:
        parser.print_help()
        return
    
    cli = CameraControlCLI()
    
    print("🎮 Isaac Sim Camera Control CLI")
    print("=" * 40)
    
    # Execute commands
    if args.command == 'move':
        cli.move(args.x, args.y, args.z)
    elif args.command == 'position':
        cli.position(args.x, args.y, args.z)
    elif args.command == 'rotate':
        cli.rotate(args.pitch, args.yaw, args.roll)
    elif args.command == 'reset':
        cli.reset()
    elif args.command == 'up':
        cli.move(z=1.0)
    elif args.command == 'down':
        cli.move(z=-1.0)
    elif args.command == 'forward':
        cli.move(x=1.0)
    elif args.command == 'backward':
        cli.move(x=-1.0)
    elif args.command == 'left':
        cli.move(y=1.0)
    elif args.command == 'right':
        cli.move(y=-1.0)
    elif args.command == 'overhead':
        cli.position(0.0, 0.0, 10.0)
    
    print("\n✅ Command completed!")
    print("💡 Check Isaac Sim viewport to see the camera movement")

if __name__ == '__main__':
    main()
