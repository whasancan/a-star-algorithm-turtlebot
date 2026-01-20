#!/usr/bin/env python3
"""
Nav2 Launch File

Launches the complete Nav2 stack:
1. Gazebo simulation
2. Map Server + AMCL
3. Nav2 Nodes (Planner, Controller, Behavior)
4. RViz

The A* plugin is loaded by the planner_server here.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    pkg_share = get_package_share_directory('my_astar_planner')
    turtlebot3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    
    # Configuration files
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'nav2_rviz.rviz')
    
    # Map file
    home_dir = os.path.expanduser('~')
    map_yaml = os.path.join(home_dir, 'turtlebot3_map.yaml')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    return LaunchDescription([
        # ========================================
        # 1. GAZEBO SIMULATION
        # ========================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(turtlebot3_gazebo_share, 'launch', 'turtlebot3_world.launch.py')
            ]),
            launch_arguments={'use_sim_time': 'true'}.items()
        ),
        
        # ========================================
        # 2. MAP SERVER
        # ========================================
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    parameters=[{
                        'yaml_filename': map_yaml,
                        'use_sim_time': True
                    }]
                )
            ]
        ),
        
        # ========================================
        # 3. AMCL (Localization)
        # ========================================
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='nav2_amcl',
                    executable='amcl',
                    name='amcl',
                    output='screen',
                    parameters=[nav2_params]
                )
            ]
        ),
        
        # ========================================
        # 4. NAV2 LIFECYCLE MANAGER (Localization)
        # ========================================
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_localization',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'autostart': True,
                        'node_names': ['map_server', 'amcl']
                    }]
                )
            ]
        ),
        
        # ========================================
        # 5. PLANNER SERVER (A* Plugin)
        # ========================================
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    parameters=[nav2_params]
                )
            ]
        ),
        
        # ========================================
        # 6. CONTROLLER SERVER (DWA)
        # ========================================
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    output='screen',
                    parameters=[nav2_params]
                )
            ]
        ),
        
        # ========================================
        # 7. BEHAVIOR SERVER
        # ========================================
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='nav2_behaviors',
                    executable='behavior_server',
                    name='behavior_server',
                    output='screen',
                    parameters=[nav2_params]
                )
            ]
        ),
        
        # ========================================
        # 8. BT NAVIGATOR
        # ========================================
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',
                    parameters=[nav2_params]
                )
            ]
        ),
        
        # ========================================
        # 9. NAV2 LIFECYCLE MANAGER (Navigation)
        # ========================================
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'autostart': True,
                        'node_names': [
                            'planner_server',
                            'controller_server',
                            'behavior_server',
                            'bt_navigator'
                        ]
                    }]
                )
            ]
        ),
        
        # ========================================
        # 10. RVIZ
        # ========================================
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config],
                    output='screen'
                )
            ]
        ),
        
        # ========================================
        # 11. GOAL BRIDGE (RViz Goal -> Nav2 Action)
        # ========================================
        # Bridge node for RViz "2D Goal Pose" button
        TimerAction(
            period=14.0,
            actions=[
                Node(
                    package='my_astar_planner',
                    executable='goal_bridge.py',
                    name='goal_bridge',
                    output='screen'
                )
            ]
        ),
    ])
