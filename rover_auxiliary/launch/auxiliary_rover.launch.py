import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    node_compass_calibrator = Node(
        package="rover_auxiliary",
        namespace="/rover/auxiliary",
        executable="compass_calibrator",
        name="compass_calibrator"
    )
    
    node_light_control = Node(
        package="rover_auxiliary",
        namespace="/rover/auxiliary",
        executable="light_control",
        name="light_control"
    )
    
    ld.add_action(node_compass_calibrator)
    ld.add_action(node_light_control)

    return ld
