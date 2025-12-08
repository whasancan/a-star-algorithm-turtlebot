#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paket yolları
    pkg_share = get_package_share_directory('my_astar_planner')
    turtlebot3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    
    # RViz config dosyası
    rviz_config = os.path.join(pkg_share, 'config', 'astar_rviz.rviz')
    
    # Harita dosyası (home dizininde)
    home_dir = os.path.expanduser('~')
    map_file = os.path.join(home_dir, 'turtlebot3_map.yaml')
    
    # TurtleBot3 model
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    
    return LaunchDescription([
        # 1. Gazebo Simülasyonu
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(turtlebot3_gazebo_share, 'launch', 'turtlebot3_world.launch.py')
            ]),
            launch_arguments={'use_sim_time': 'true'}.items()
        ),
        
        # 2. Map Server (2 saniye bekle)
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    parameters=[{
                        'yaml_filename': map_file,
                        'use_sim_time': True
                    }]
                )
            ]
        ),
        
        # 3. Map Server Lifecycle Configure (4 saniye bekle)
        TimerAction(
            period=4.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/map_server', 'configure'],
                    output='screen'
                )
            ]
        ),
        
        # 4. Map Server Lifecycle Activate (5 saniye bekle)
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/map_server', 'activate'],
                    output='screen'
                )
            ]
        ),
        
        # 5. Haritayı Yükle (6 saniye bekle)
        TimerAction(
            period=6.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'service', 'call', '/map_server/load_map', 
                         'nav2_msgs/srv/LoadMap', 
                         '{map_url: ' + map_file + '}'],
                    output='screen'
                )
            ]
        ),
        
        # 6. Static TF Publisher (map -> base_footprint)
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_map_to_base',
                    arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_footprint'],
                    output='screen'
                )
            ]
        ),
        
        # 7. A* Planner Node - AYRI TERMİNALDE (7 saniye bekle)
        TimerAction(
            period=7.0,
            actions=[
                ExecuteProcess(
                    cmd=['gnome-terminal', '--', 'bash', '-c', 
                         'source /opt/ros/humble/setup.bash && source ~/turtlebot3_ws/install/setup.bash && ros2 run my_astar_planner astar_node; exec bash'],
                    output='screen'
                )
            ]
        ),
        
        # 8. RViz2 (8 saniye bekle)
        TimerAction(
            period=8.0,
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
    ])
