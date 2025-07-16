#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level for all nodes'
        ),
        
        # Log system startup
        LogInfo(msg="Starting System Controller with all nodes..."),
        
        # System Manager Node
        Node(
            package='system_controller',
            executable='system_manager_node',
            name='system_manager',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),
        
        # Command Input Arbiter Node
        Node(
            package='system_controller',
            executable='command_input_arbiter_node',
            name='command_input_arbiter',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),
        
        # Vehicle Adapter Manager Node
        Node(
            package='system_controller',
            executable='vehicle_adapter_manager_node',
            name='vehicle_adapter_manager',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),
        
        # Telemetry Collector Node
        Node(
            package='system_controller',
            executable='telemetry_collector_node',
            name='telemetry_collector',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),
        
        # Mission Service Node
        Node(
            package='system_controller',
            executable='mission_service_node',
            name='mission_service',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),
        
        # Mission Adapter Manager Node
        Node(
            package='system_controller',
            executable='mission_adapter_manager_node',
            name='mission_adapter_manager',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),
    ]) 