#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    nodes = []

    # args
    courier_count = max(min(int(LaunchConfiguration('courier_count').perform(context)), 5), 1)
    loader_count = max(min(int(LaunchConfiguration('loader_count').perform(context)), 5), 1)
    unloader_count = max(min(int(LaunchConfiguration('unloader_count').perform(context)), 5), 1)

    # === Loader nodes ===
    loader_positions = [
        (-1.9, -7.0, -1.67),
        (-1.5, 6.0, -1.67),
        (3.7, 6.2, -1.67),
        (3.67, -3.9, 0.0),
        (-11.0, 5.7, 0.0)
    ]
   
    for i, (x, y, yaw) in enumerate(loader_positions[:loader_count]):
        loader_id = f'loader_{i+1}'
        nodes.append(Node(
            package='vikings_bot_agent_abstraction_layer',
            executable='loader.py',
            name=loader_id,
            output='screen',
            emulate_tty=True,
            parameters=[{
                'loader_id': loader_id,
                'loc_x': x,
                'loc_y': y,
                'loc_yaw': yaw,
                'broadcast_interval': 0.5
            }]
        ))

    # === Unloader nodes ===
    unloader_positions = [
        (11.7, -2.0, 3.14),
        (11.7, 1.85, 3.14),
        (-12.0, -6.5, 0.0),
        (-11.6, 0.6, 0.0),
        (12.0, 6.0, 3.14)

    ]
  
    for i, (x, y, yaw) in enumerate(unloader_positions[:unloader_count]):
        unloader_id = f'unloader_{i+1}'
        nodes.append(Node(
            package='vikings_bot_agent_abstraction_layer',
            executable='unloader.py',
            name=unloader_id,
            output='screen',
            emulate_tty=True,
            parameters=[{
                'unloader_id': unloader_id,
                'loc_x': x,
                'loc_y': y,
                'loc_yaw': yaw,
                'broadcast_interval': 0.5
            }]
        ))

    # === Courier agents ===
    courier_positions = [
        (10.0, 6.0, -1.7),
        (10.0, -6.0, 1.7),
        (-11.0, -5.0, 0.0),
        (-7.0, 0.5, 0.0),
        (2.75, -0.6, 3.14)
    ]
    for i, (x, y, yaw) in enumerate(courier_positions[:courier_count]):
        ugv_id = f'ugv_{i+1}'
        ns = f'vikings_bot_{i+1}'  # Namespace for navigation
        nodes.append(Node(
            package='vikings_bot_agent_abstraction_layer',
            executable='courier.py',
            namespace=ns,
            name=ugv_id,
            output='screen',
            emulate_tty=True,
            parameters=[{
                'ugv_id': ugv_id,
                'start_x': x,
                'start_y': y,
                'start_yaw': yaw,
            }]
        ))
    
    
    return nodes


def generate_launch_description():
    

    return LaunchDescription([
        DeclareLaunchArgument('courier_count', default_value='1'),
        DeclareLaunchArgument('loader_count', default_value='5'),
        DeclareLaunchArgument('unloader_count', default_value='5'),
        OpaqueFunction(function=launch_setup),
    ])
