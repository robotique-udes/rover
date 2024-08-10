import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Create a LaunchDescription object
    ld = LaunchDescription()

    # Define the nodes
    node_lights_main = Node(
        package="rover_auxiliary",
        namespace="/rover/auxiliary",
        executable="light_control",
        name="light_controller"
    )

    node_compass_main = Node(
        package="rover_auxiliary",
        namespace="/rover/auxiliary",
        executable="compass_calibrator",
        name="compass_calibrator"
    )

    node_redistribute_stream = Node(
        package="rover_auxiliary",
        namespace="/rover/auxiliary",
        executable="redistribute_stream.py",
        name="redistribute_stream",
        parameters=[
            {"stream_ip": "127.0.0.2"},
            {"stream_port": 69},
            {"server_port": 72}
            ]
    )

    # Add the nodes to the launch description
    ld.add_action(node_lights_main)
    ld.add_action(node_compass_main)
    ld.add_action(node_redistribute_stream)

    # Return the launch description
    return ld
