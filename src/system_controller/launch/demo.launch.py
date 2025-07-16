#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start system manager and command arbiter for basic demo
        Node(
            package='system_controller',
            executable='system_manager_node',
            name='system_manager_demo',
            output='screen'
        ),
        
        Node(
            package='system_controller',
            executable='command_input_arbiter_node',
            name='command_arbiter_demo',
            output='screen'
        ),
        
        Node(
            package='system_controller',
            executable='vehicle_adapter_manager_node',
            name='vehicle_adapter_demo',
            output='screen'
        ),
    ]) 