#!/usr/bin/env python3
"""
Launch file for Isaac Test LLM Camera Controller
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare launch arguments
    analysis_interval_arg = DeclareLaunchArgument(
        'analysis_interval',
        default_value='2.0',
        description='Time interval between image analyses (seconds)'
    )
    
    movement_scale_arg = DeclareLaunchArgument(
        'movement_scale',
        default_value='0.5',
        description='Scale factor for linear movements'
    )
    
    rotation_scale_arg = DeclareLaunchArgument(
        'rotation_scale',
        default_value='0.3',
        description='Scale factor for angular movements'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='Minimum confidence threshold for executing commands'
    )
    
    enable_monitoring_arg = DeclareLaunchArgument(
        'enable_monitoring',
        default_value='true',
        description='Enable topic monitoring output'
    )
    
    # LLM Camera Controller Node
    llm_controller_node = Node(
        package='isaac_test',
        executable='llm_camera_controller',
        name='isaac_test_llm_controller',
        parameters=[{
            'analysis_interval': LaunchConfiguration('analysis_interval'),
            'movement_scale': LaunchConfiguration('movement_scale'),
            'rotation_scale': LaunchConfiguration('rotation_scale'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
        }],
        output='screen'
    )
    
    # Optional: Topic monitoring
    monitor_analysis = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('enable_monitoring')),
        cmd=['ros2', 'topic', 'echo', '/isaac_test/llm_analysis'],
        output='screen',
        prefix='gnome-terminal -- '
    )
    
    monitor_commands = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('enable_monitoring')),
        cmd=['ros2', 'topic', 'echo', '/camera/cmd_vel'],
        output='screen', 
        prefix='gnome-terminal -- '
    )
    
    return LaunchDescription([
        analysis_interval_arg,
        movement_scale_arg,
        rotation_scale_arg,
        confidence_threshold_arg,
        enable_monitoring_arg,
        llm_controller_node,
        # Uncomment to enable automatic monitoring windows
        # monitor_analysis,
        # monitor_commands,
    ])
