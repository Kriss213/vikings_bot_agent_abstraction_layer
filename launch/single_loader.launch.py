#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    ld = LaunchDescription()

    loader_id_arg = DeclareLaunchArgument(
        'id',
        default_value='loader_1',
        description='Loader ID'
    )
    ld.add_action(loader_id_arg)

    loader_loc_x_arg = DeclareLaunchArgument(
        'loc_x',
        default_value='-2.0',
        description='Loader location x coordinate'
    )
    loader_loc_y_arg = DeclareLaunchArgument(
        'loc_y',
        default_value='-2.0',
        description='Loader location y coordinate'
    )
    ld.add_action(loader_loc_x_arg)
    ld.add_action(loader_loc_y_arg)

    loader_id = LaunchConfiguration('id')
    ld.add_action(Node(
        package='vikings_bot_agent_abstraction_layer',
        executable='loader.py',
        name=loader_id,
        output='screen',
        emulate_tty=True,
        parameters=[{
            'loader_id': loader_id,
            'loc_x': PythonExpression([LaunchConfiguration('loc_x')]),
            'loc_y': PythonExpression([LaunchConfiguration('loc_y')]),
            'broadcast_interval': 0.5
        }]
    ))

    return ld