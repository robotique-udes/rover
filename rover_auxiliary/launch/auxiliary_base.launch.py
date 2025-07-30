import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    node_antenna = Node(
        package="rover_auxiliary",
        namespace="/rover/antenna",
        executable="antenna",
        name="antenna"
    )
    return ld
