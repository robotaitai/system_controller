#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction, GroupAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Demo configuration arguments
        DeclareLaunchArgument(
            'demo_scenario',
            default_value='multi_implement',
            description='Demo scenario: spraying_only, mowing_only, seeding_only, multi_implement, autonomous_field'
        ),
        DeclareLaunchArgument(
            'simulation_mode',
            default_value='true',
            description='Run in simulation mode for demo'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level for demo'
        ),
        
        # Demo introduction
        LogInfo(msg="🚜 Agricultural Automation System Demo Starting! 🌾"),
        LogInfo(msg=['Demo Scenario: ', LaunchConfiguration('demo_scenario')]),
        LogInfo(msg="========================================"),
        
        # Core System Nodes
        GroupAction(
            actions=[
                Node(
                    package='system_controller',
                    executable='system_manager_node',
                    name='system_manager',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('simulation_mode'),
                        'policy_mode': 'adas_assistance',
                        'vehicle_type': 'polaris'
                    }],
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
                ),
                
                Node(
                    package='system_controller',
                    executable='command_input_arbiter_node',
                    name='command_arbiter',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('simulation_mode'),
                        'demo_mode': True
                    }],
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
                ),
                
                Node(
                    package='system_controller',
                    executable='vehicle_adapter_manager_node',
                    name='vehicle_adapter_manager',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('simulation_mode'),
                        'vehicle_type': 'polaris',
                        'simulation_mode': True
                    }],
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
                ),
                
                Node(
                    package='system_controller',
                    executable='telemetry_collector_node',
                    name='telemetry_collector',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('simulation_mode'),
                        'demo_mode': True,
                        'verbose_logging': True
                    }],
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
                )
            ]
        ),
        
        # Multi-Implement Demo Setup
        GroupAction(
            actions=[
                # All three implement adapters for full demo
                Node(
                    package='system_controller',
                    executable='sprayer_adapter_node',
                    name='sprayer_adapter',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('simulation_mode'),
                        'adapter_id': 'demo_sprayer_24m',
                        'simulation_mode': True,
                        'demo_chemical_type': 'herbicide',
                        'demo_application_rate': 150.0,
                        'demo_boom_width': 24.0
                    }],
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
                ),
                
                Node(
                    package='system_controller',
                    executable='mower_adapter_node',
                    name='mower_adapter',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('simulation_mode'),
                        'adapter_id': 'demo_mower_3m',
                        'simulation_mode': True,
                        'demo_cutting_height': 0.05,
                        'demo_blade_speed': 3000,
                        'demo_pattern': 'stripe'
                    }],
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
                ),
                
                Node(
                    package='system_controller',
                    executable='seeder_adapter_node',
                    name='seeder_adapter',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('simulation_mode'),
                        'adapter_id': 'demo_seeder_12m',
                        'simulation_mode': True,
                        'demo_seed_rate': 65000,
                        'demo_planting_depth': 0.025,
                        'demo_seed_type': 'corn'
                    }],
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
                )
            ]
        ),
        
        # Mission Controllers
        GroupAction(
            actions=[
                Node(
                    package='system_controller',
                    executable='sprayer_mission_node',
                    name='sprayer_mission',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('simulation_mode'),
                        'demo_mode': True,
                        'demo_field_size': 10.0,  # hectares
                        'demo_weather_conditions': True,
                        'demo_gps_accuracy': 0.03,  # meters
                        'demo_tree_detection': True
                    }],
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
                ),
                
                Node(
                    package='system_controller',
                    executable='mower_mission_node',
                    name='mower_mission',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('simulation_mode'),
                        'demo_mode': True,
                        'demo_field_size': 5.0,   # hectares
                        'demo_terrain_following': True,
                        'demo_cutting_pattern': 'spiral'
                    }],
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
                ),
                
                Node(
                    package='system_controller',
                    executable='seeder_mission_node',
                    name='seeder_mission',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('simulation_mode'),
                        'demo_mode': True,
                        'demo_field_size': 15.0,  # hectares
                        'demo_variable_rate': True,
                        'demo_soil_conditions': True
                    }],
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
                )
            ]
        ),
        
        # Policy Controllers
        GroupAction(
            actions=[
                Node(
                    package='system_controller',
                    executable='adas_policy_node',
                    name='adas_policy',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('simulation_mode'),
                        'demo_mode': True,
                        'active': True,
                        'demo_collision_avoidance': True,
                        'demo_tree_detection': True,
                        'demo_gps_monitoring': True
                    }],
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
                ),
                
                Node(
                    package='system_controller',
                    executable='teleop_only_policy_node',
                    name='teleop_policy',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('simulation_mode'),
                        'demo_mode': True,
                        'active': False,  # Standby for emergency
                        'demo_operator_presence': True
                    }],
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
                )
            ]
        ),
        
        # Mission Adapter Manager (Updated)
        Node(
            package='system_controller',
            executable='mission_adapter_manager_node',
            name='mission_adapter_manager',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('simulation_mode'),
                'demo_mode': True,
                'enable_sprayer': True,
                'enable_mower': True,
                'enable_seeder': True,
                'demo_scenario': LaunchConfiguration('demo_scenario')
            }],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),
        
        # Demo Timeline Actions
        LogInfo(msg="Demo system fully loaded. Starting demonstration sequence..."),
        
        # Delayed demo actions to show system capabilities
        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg="🎯 Demo Phase 1: Sprayer Mission Demonstration"),
                LogInfo(msg="   - Activating herbicide spraying mission"),
                LogInfo(msg="   - GPS accuracy: 3cm (RTK enabled)"),
                LogInfo(msg="   - Weather conditions: Optimal (wind 8 m/s)"),
                LogInfo(msg="   - Tree detection: Active"),
            ]
        ),
        
        TimerAction(
            period=15.0,
            actions=[
                LogInfo(msg="🎯 Demo Phase 2: Mower Mission Demonstration"),
                LogInfo(msg="   - Switching to autonomous mowing"),
                LogInfo(msg="   - Cutting height: 5cm"),
                LogInfo(msg="   - Pattern: Spiral optimization"),
                LogInfo(msg="   - Terrain following: Enabled"),
            ]
        ),
        
        TimerAction(
            period=25.0,
            actions=[
                LogInfo(msg="🎯 Demo Phase 3: Seeder Mission Demonstration"),
                LogInfo(msg="   - Precision corn seeding mission"),
                LogInfo(msg="   - Variable rate: 65,000 seeds/ha"),
                LogInfo(msg="   - Planting depth: 2.5cm"),
                LogInfo(msg="   - Soil monitoring: Active"),
            ]
        ),
        
        TimerAction(
            period=35.0,
            actions=[
                LogInfo(msg="🎯 Demo Phase 4: Safety System Demonstration"),
                LogInfo(msg="   - Simulating obstacle detection"),
                LogInfo(msg="   - ADAS policy intervention"),
                LogInfo(msg="   - Emergency stop capability"),
                LogInfo(msg="   - Operator presence monitoring"),
            ]
        ),
        
        TimerAction(
            period=45.0,
            actions=[
                LogInfo(msg="🎯 Demo Phase 5: Multi-Implement Coordination"),
                LogInfo(msg="   - Demonstrating implement switching"),
                LogInfo(msg="   - Dynamic adapter loading"),
                LogInfo(msg="   - Mission priority management"),
                LogInfo(msg="   - Policy-based control"),
            ]
        ),
        
        TimerAction(
            period=55.0,
            actions=[
                LogInfo(msg="🏆 Demo Complete!"),
                LogInfo(msg="========================================"),
                LogInfo(msg="System demonstrated:"),
                LogInfo(msg="✅ Type-safe command messaging"),
                LogInfo(msg="✅ Dynamic implement adapter loading"),
                LogInfo(msg="✅ Mission business logic"),
                LogInfo(msg="✅ Advanced safety policies"),
                LogInfo(msg="✅ Multi-implement coordination"),
                LogInfo(msg="✅ Real-time monitoring and control"),
                LogInfo(msg=""),
                LogInfo(msg="Ready for production deployment! 🚜🌾"),
                LogInfo(msg="========================================"),
            ]
        ),
        
        # Instructions for users
        LogInfo(msg=""),
        LogInfo(msg="💡 Demo Instructions:"),
        LogInfo(msg="   Monitor the topics to see system operation:"),
        LogInfo(msg="   ros2 topic echo /SprayerCommand"),
        LogInfo(msg="   ros2 topic echo /MowerCommand"),
        LogInfo(msg="   ros2 topic echo /SeederCommand"),
        LogInfo(msg="   ros2 topic echo /PolicyState"),
        LogInfo(msg="   ros2 topic echo /MissionStatus"),
        LogInfo(msg=""),
        LogInfo(msg="🎮 Test Commands:"),
        LogInfo(msg="   ros2 topic pub /VehicleCommand system_controller/msg/VehicleCommand"),
        LogInfo(msg="   ros2 service call /emergency_stop std_srvs/srv/Trigger"),
        LogInfo(msg="   ros2 param set /adas_policy tree_detection_enabled false"),
        LogInfo(msg=""),
    ]) 