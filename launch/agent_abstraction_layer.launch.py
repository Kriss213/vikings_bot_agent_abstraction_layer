#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # === Loader nodes ===
    loader_positions = [(-2.0, -2.0), (-2.0, 0.0), (-2.0, 2.0)]
    for i, (x, y) in enumerate(loader_positions):
        loader_id = f'loader_{i+1}'
        ld.add_action(Node(
            package='vikings_bot_agent_abstraction_layer',
            executable='loader.py',
            name=loader_id,
            output='screen',
            emulate_tty=True,
            parameters=[{
                'loader_id': loader_id,
                'loc_x': x,
                'loc_y': y,
                'broadcast_interval': 0.5
            }]
        ))

    # === Unloader nodes ===
    unloader_positions = [(4.0, -2.0), (4.0, 0.0), (4.0, 2.0)]
    for i, (x, y) in enumerate(unloader_positions):
        unloader_id = f'unloader_{i+1}'
        ld.add_action(Node(
            package='vikings_bot_agent_abstraction_layer',
            executable='unloader.py',
            name=unloader_id,
            output='screen',
            emulate_tty=True,
            parameters=[{
                'unloader_id': unloader_id,
                'loc_x': x,
                'loc_y': y,
                'broadcast_interval': 0.5
            }]
        ))

    # === Courier agents ===
    courier_positions = [(-4.0, -2.0), (-4.0, -2.5), (-4.0, -1.0)]
    for i, (x, y) in enumerate(courier_positions):
        ugv_id = f'ugv_{i+1}'
        ns = f'vikings_bot_{i+1}'  # Namespace for navigation
        ld.add_action(Node(
            package='vikings_bot_agent_abstraction_layer',
            executable='courier.py',
            namespace=ns,
            name=ugv_id,
            output='screen',
            emulate_tty=True,
            parameters=[{
                'ugv_id': ugv_id,
                'start_x': x,
                'start_y': y
            }]
        ))

    return ld
