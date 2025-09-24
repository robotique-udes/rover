import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    node_ddb_control = Node(
        package="rover_auxiliary",
        namespace="/rover/auxiliary",
        executable="ddb_control",
        name="ddb_control"
    )
    ld.add_action(node_ddb_control)
    
    node_camera_manager = Node(
        package="rover_auxiliary",
        namespace="/rover/camera_manager",
        executable="camera_manager",
        name="camera_manager"
    )
    ld.add_action(node_camera_manager)

    return ld
