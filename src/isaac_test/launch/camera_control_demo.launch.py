#!/usr/bin/env python3
"""
Launch file for Isaac Sim Camera Control Demo
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import FindExecutable

def generate_launch_description():
    return LaunchDescription([
        # Launch the Isaac Sim camera control node
        Node(
            package='isaac_test',
            executable='isaac_camera_controlled',
            name='isaac_camera_control_node',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        ),
        
        # Wait 5 seconds then launch camera subscriber to monitor the feed
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='isaac_test',
                    executable='camera_subscriber',
                    name='camera_subscriber_node',
                    output='screen',
                    parameters=[
                        {'use_sim_time': False}
                    ]
                )
            ]
        ),
        
        # Print usage instructions after 8 seconds
        TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        FindExecutable(name='echo'),
                        '\n=== Camera Control Ready ===\n'
                        'Test with these commands in a new terminal:\n'
                        '1. Move camera forward: ros2 topic pub --once /camera/cmd_vel geometry_msgs/msg/Twist "{\\"linear\\": {\\"x\\": 0.5}}" \n'
                        '2. Rotate camera: ros2 topic pub --once /camera/cmd_vel geometry_msgs/msg/Twist "{\\"angular\\": {\\"z\\": 0.5}}" \n'
                        '3. Stop camera: ros2 topic pub --once /camera/cmd_vel geometry_msgs/msg/Twist "{}" \n'
                        '4. Set specific position: ros2 topic pub --once /camera/set_pose geometry_msgs/msg/PoseStamped "{\\"pose\\": {\\"position\\": {\\"x\\": 3.0, \\"y\\": 3.0, \\"z\\": 3.0}}}" \n'
                        '5. Disable camera: ros2 topic pub --once /camera/enable std_msgs/msg/Bool "{\\"data\\": false}" \n'
                        '6. Enable camera: ros2 topic pub --once /camera/enable std_msgs/msg/Bool "{\\"data\\": true}" \n'
                    ],
                    output='screen'
                )
            ]
        )
    ])
