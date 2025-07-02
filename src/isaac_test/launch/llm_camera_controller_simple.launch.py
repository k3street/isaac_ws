#!/usr/bin/env python3
"""
Launch file for LLM Camera Controller with different provider configurations
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    llm_provider_arg = DeclareLaunchArgument(
        'llm_provider',
        default_value='simulation',
        description='LLM provider: openai_gpt4.1, gemini_2.5, claude_4_sonnet, or simulation'
    )
    
    openai_api_key_arg = DeclareLaunchArgument(
        'openai_api_key',
        default_value='',
        description='OpenAI API key'
    )
    
    gemini_api_key_arg = DeclareLaunchArgument(
        'gemini_api_key', 
        default_value='',
        description='Google Gemini API key'
    )
    
    anthropic_api_key_arg = DeclareLaunchArgument(
        'anthropic_api_key',
        default_value='',
        description='Anthropic Claude API key'
    )
    
    analysis_interval_arg = DeclareLaunchArgument(
        'analysis_interval',
        default_value='2.0',
        description='Time between analyses in seconds'
    )
    
    # LLM Camera Controller Node
    llm_controller_node = Node(
        package='isaac_test',
        executable='llm_camera_controller_simple',  # Remove .py extension
        name='llm_camera_controller',
        parameters=[{
            'llm_provider': LaunchConfiguration('llm_provider'),
            'openai_api_key': LaunchConfiguration('openai_api_key'),
            'gemini_api_key': LaunchConfiguration('gemini_api_key'),
            'anthropic_api_key': LaunchConfiguration('anthropic_api_key'),
            'analysis_interval': LaunchConfiguration('analysis_interval'),
            'movement_scale': 2.5,  # Increased from 0.5
            'rotation_scale': 1.5,  # Increased from 0.3
            'confidence_threshold': 0.5,
            'llm_temperature': 0.1,
            'llm_max_tokens': 300
        }],
        output='screen'
    )
    
    return LaunchDescription([
        llm_provider_arg,
        openai_api_key_arg,
        gemini_api_key_arg,
        anthropic_api_key_arg,
        analysis_interval_arg,
        llm_controller_node
    ])
