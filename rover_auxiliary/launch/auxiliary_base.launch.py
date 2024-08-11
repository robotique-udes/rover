import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    node_goal_manager = Node(
        package="rover_auxiliary",
        namespace="/rover/auxiliary",
        executable="goal_manager",
        name="goal_manager"
    )
    
    ld.add_action(node_goal_manager)

    return ld
