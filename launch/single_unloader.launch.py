#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    ld = LaunchDescription()
    
    unloader_id_arg = DeclareLaunchArgument(
        'id',
        default_value='unloader_1',
        description='Unloader ID'
    )
    ld.add_action(unloader_id_arg)

    unloader_loc_x_arg = DeclareLaunchArgument(
        'loc_x',
        default_value='4.0',
        description='Unloader location x coordinate'
    )
    unloader_loc_y_arg = DeclareLaunchArgument(
        'loc_y',
        default_value='-2.0',
        description='Unloader location y coordinate'
    )
    ld.add_action(unloader_loc_x_arg)
    ld.add_action(unloader_loc_y_arg)

    unloader_id = LaunchConfiguration('id')

    ld.add_action(Node(
        package='vikings_bot_agent_abstraction_layer',
        executable='unloader.py',
        name=unloader_id,
        output='screen',
        emulate_tty=True,
        parameters=[{
            'unloader_id': unloader_id,
            'loc_x': PythonExpression([LaunchConfiguration('loc_x')]),
            'loc_y': PythonExpression([LaunchConfiguration('loc_y')]),
            'broadcast_interval': 0.5
        }]
    ))

    return ld