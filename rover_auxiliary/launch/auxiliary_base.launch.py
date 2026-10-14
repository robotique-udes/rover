import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    with_sik_radio_arg = DeclareLaunchArgument(
        'with_sik_radio',
        default_value='true',
        description='Whether to launch SiK 900MHz telemetry radio node'
    )
    serial_port_arg = DeclareLaunchArgument(
        'sik_serial_port',
        default_value='/dev/ttySikRadio',
        description='Serial port for SiK radio'
    )

    node_antenna = Node(
        package="rover_auxiliary",
        namespace="/rover/antenna",
        executable="antenna",
        name="antenna"
    )

    node_sik_base = Node(
        package="rover_auxiliary",
        namespace="/rover/auxiliary",
        executable="sik_radio",
        name="sik_radio_base",
        parameters=[{
            "is_base": True,
            "serial_port": LaunchConfiguration('sik_serial_port'),
            "baud_rate": 115200
        }],
        condition=IfCondition(LaunchConfiguration('with_sik_radio'))
    )

    return LaunchDescription([
        with_sik_radio_arg,
        serial_port_arg,
        node_antenna,
        node_sik_base
    ])
