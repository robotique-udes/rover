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
        default_value='/dev/ttyUSB0',
        description='Serial port for SiK radio'
    )

    node_ddb_control = Node(
        package="rover_auxiliary",
        namespace="/rover/auxiliary",
        executable="ddb_control",
        name="ddb_control"
    )
    
    node_camera_manager = Node(
        package="rover_auxiliary",
        namespace="/rover/camera_manager",
        executable="camera_manager",
        name="camera_manager"
    )

    node_bms_info = Node(
        package="rover_auxiliary",
        namespace="/rover/auxiliary",
        executable="bms_info",
        name="bms_info"
    )

    node_sik_rover = Node(
        package="rover_auxiliary",
        namespace="/rover/auxiliary",
        executable="sik_radio",
        name="sik_radio_rover",
        parameters=[{
            "is_base": False,
            "serial_port": LaunchConfiguration('sik_serial_port'),
            "baud_rate": 115200
        }],
        condition=IfCondition(LaunchConfiguration('with_sik_radio'))
    )

    return LaunchDescription([
        with_sik_radio_arg,
        serial_port_arg,
        node_ddb_control,
        node_camera_manager,
        node_bms_info,
        node_sik_rover
    ])
