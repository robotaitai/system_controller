#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments for configuration
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level for all nodes'
        ),
        DeclareLaunchArgument(
            'vehicle_type',
            default_value='polaris',
            description='Vehicle type: polaris, new_holland, john_deere, case_ih'
        ),
        DeclareLaunchArgument(
            'mission_type',
            default_value='spraying',
            description='Mission type: spraying, mowing, seeding, multi_implement'
        ),
        DeclareLaunchArgument(
            'policy_mode',
            default_value='field_safety',
            description='Policy mode: teleop_only, adas_assistance, field_safety, autonomous_operation'
        ),
        DeclareLaunchArgument(
            'enable_sprayer',
            default_value='true',
            description='Enable sprayer adapter and mission'
        ),
        DeclareLaunchArgument(
            'enable_mower',
            default_value='false',
            description='Enable mower adapter and mission'
        ),
        DeclareLaunchArgument(
            'enable_seeder',
            default_value='false',
            description='Enable seeder adapter and mission'
        ),
        DeclareLaunchArgument(
            'simulation_mode',
            default_value='false',
            description='Run in simulation mode with mock hardware'
        ),
        
        # Log system startup
        LogInfo(msg=['Starting Agricultural Automation System...']),
        LogInfo(msg=['Vehicle Type: ', LaunchConfiguration('vehicle_type')]),
        LogInfo(msg=['Mission Type: ', LaunchConfiguration('mission_type')]),
        LogInfo(msg=['Policy Mode: ', LaunchConfiguration('policy_mode')]),
        
        # Core System Nodes
        GroupAction(
            actions=[
                # System Manager Node
                Node(
                    package='system_controller',
                    executable='system_manager_node',
                    name='system_manager',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('simulation_mode'),
                        'policy_mode': LaunchConfiguration('policy_mode'),
                        'vehicle_type': LaunchConfiguration('vehicle_type')
                    }],
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
                ),
                
                # Command Input Arbiter Node
                Node(
                    package='system_controller',
                    executable='command_input_arbiter_node',
                    name='command_arbiter',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('simulation_mode'),
                        'priority_policy_commands': 100,
                        'priority_teleop_commands': 80,
                        'priority_autonomy_commands': 60
                    }],
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
                ),
                
                # Vehicle Adapter Manager Node
                Node(
                    package='system_controller',
                    executable='vehicle_adapter_manager_node',
                    name='vehicle_adapter_manager',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('simulation_mode'),
                        'vehicle_type': LaunchConfiguration('vehicle_type'),
                        'config_file': '/ros2_ws/src/system_controller/config/vehicle_adapters.yaml'
                    }],
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
                ),
                
                # Mission Adapter Manager Node (Updated)
                Node(
                    package='system_controller',
                    executable='mission_adapter_manager_node',
                    name='mission_adapter_manager',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('simulation_mode'),
                        'mission_config_file': '/ros2_ws/src/system_controller/config/missions.yaml',
                        'enable_sprayer': LaunchConfiguration('enable_sprayer'),
                        'enable_mower': LaunchConfiguration('enable_mower'),
                        'enable_seeder': LaunchConfiguration('enable_seeder')
                    }],
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
                ),
                
                # Mission Service Node
                Node(
                    package='system_controller',
                    executable='mission_service_node',
                    name='mission_service',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('simulation_mode'),
                        'mission_type': LaunchConfiguration('mission_type'),
                        'config_file': '/ros2_ws/src/system_controller/config/missions.yaml'
                    }],
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
                ),
                
                # Telemetry Collector Node
                Node(
                    package='system_controller',
                    executable='telemetry_collector_node',
                    name='telemetry_collector',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('simulation_mode'),
                        'collection_rate': 10.0,  # Hz
                        'log_telemetry': True
                    }],
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
                )
            ]
        ),
        
        # Implement Adapter Nodes (Conditional)
        GroupAction(
            condition=IfCondition(LaunchConfiguration('enable_sprayer')),
            actions=[
                Node(
                    package='system_controller',
                    executable='sprayer_adapter_node',
                    name='sprayer_adapter',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('simulation_mode'),
                        'adapter_id': 'boom_sprayer_24m',
                        'can_interface': 'can0',
                        'simulation_mode': LaunchConfiguration('simulation_mode')
                    }],
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
                )
            ]
        ),
        
        GroupAction(
            condition=IfCondition(LaunchConfiguration('enable_mower')),
            actions=[
                Node(
                    package='system_controller',
                    executable='mower_adapter_node',
                    name='mower_adapter',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('simulation_mode'),
                        'adapter_id': 'rotary_mower_3m',
                        'can_interface': 'can1',
                        'simulation_mode': LaunchConfiguration('simulation_mode')
                    }],
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
                )
            ]
        ),
        
        GroupAction(
            condition=IfCondition(LaunchConfiguration('enable_seeder')),
            actions=[
                Node(
                    package='system_controller',
                    executable='seeder_adapter_node',
                    name='seeder_adapter',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('simulation_mode'),
                        'adapter_id': 'precision_seeder_12m',
                        'can_interface': 'can2',
                        'simulation_mode': LaunchConfiguration('simulation_mode')
                    }],
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
                )
            ]
        ),
        
        # Mission Controller Nodes (Conditional)
        GroupAction(
            condition=IfCondition(LaunchConfiguration('enable_sprayer')),
            actions=[
                Node(
                    package='system_controller',
                    executable='sprayer_mission_node',
                    name='sprayer_mission',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('simulation_mode'),
                        'mission_config_file': '/ros2_ws/src/system_controller/config/missions.yaml',
                        'weather_monitoring': True,
                        'gps_monitoring': True,
                        'tree_detection': True
                    }],
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
                )
            ]
        ),
        
        GroupAction(
            condition=IfCondition(LaunchConfiguration('enable_mower')),
            actions=[
                Node(
                    package='system_controller',
                    executable='mower_mission_node',
                    name='mower_mission',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('simulation_mode'),
                        'mission_config_file': '/ros2_ws/src/system_controller/config/missions.yaml',
                        'terrain_following': True,
                        'obstacle_detection': True
                    }],
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
                )
            ]
        ),
        
        GroupAction(
            condition=IfCondition(LaunchConfiguration('enable_seeder')),
            actions=[
                Node(
                    package='system_controller',
                    executable='seeder_mission_node',
                    name='seeder_mission',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('simulation_mode'),
                        'mission_config_file': '/ros2_ws/src/system_controller/config/missions.yaml',
                        'variable_rate_seeding': True,
                        'soil_monitoring': True
                    }],
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
                )
            ]
        ),
        
        # Policy Controller Nodes
        GroupAction(
            actions=[
                Node(
                    package='system_controller',
                    executable='teleop_only_policy_node',
                    name='teleop_policy',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('simulation_mode'),
                        'policy_config_file': '/ros2_ws/src/system_controller/config/policies.yaml',
                        'active': TextSubstitution(text='true') if LaunchConfiguration('policy_mode').perform() == 'teleop_only' else TextSubstitution(text='false')
                    }],
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
                ),
                
                Node(
                    package='system_controller',
                    executable='adas_policy_node',
                    name='adas_policy',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('simulation_mode'),
                        'policy_config_file': '/ros2_ws/src/system_controller/config/policies.yaml',
                        'active': TextSubstitution(text='true') if LaunchConfiguration('policy_mode').perform() == 'adas_assistance' else TextSubstitution(text='false')
                    }],
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
                )
            ]
        ),
        
        # System Status and Monitoring
        LogInfo(msg="Agricultural Automation System fully started!"),
        LogInfo(msg="Available topics:"),
        LogInfo(msg="  - /VehicleCommand - Vehicle control commands"),
        LogInfo(msg="  - /SprayerCommand - Sprayer control commands"),
        LogInfo(msg="  - /MowerCommand - Mower control commands"),
        LogInfo(msg="  - /SeederCommand - Seeder control commands"),
        LogInfo(msg="  - /PolicyState - Current policy state"),
        LogInfo(msg="  - /MissionStatus - Mission progress and status"),
        LogInfo(msg="Ready for field operations!")
    ]) 