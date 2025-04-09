#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()
        
    courier_id_arg = DeclareLaunchArgument(
        'id',
        default_value='courier_1',
        description='Courier ID'
    )
    ld.add_action(courier_id_arg)

    ns_arg = DeclareLaunchArgument(
        'vb_ns',
        default_value='vikings_bot_1',
        description='Vikings Bot namespace'
    )
    ld.add_action(courier_id_arg)
    ld.add_action(ns_arg)

    courier_id = LaunchConfiguration('id')
    ns = LaunchConfiguration('vb_ns')
    ld.add_action(Node(
        package='vikings_bot_agent_abstraction_layer',
        executable='courier.py',
        namespace=ns,
        name=courier_id,
        output='screen',
        emulate_tty=True,
        parameters=[{
            'ugv_id': courier_id,
            'start_x': -4.0,
            'start_y': -2.0
        }]
    ))

    return ld