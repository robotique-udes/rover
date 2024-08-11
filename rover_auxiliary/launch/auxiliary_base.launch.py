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
    
    node_redistribute_stream = Node(
            package="rover_auxiliary",
            namespace="/rover/auxiliary",
            executable="redistribute_stream.py",
            name="redistribute_stream",
            parameters=[
                {"streamIP": "192.168.144.62"},
                {"streamPort": 69},
                {"serverPort1": 8554},  
                {"outputIP1": "192.168.144.103"},  
                {"serverPort2": 8555},  
                {"outputIP2": "192.168.144.103"},  
            ]
    )

    node_screenshot = Node(
        package="rover_auxiliary",
        namespace="/rover/auxiliary",
        executable="screenshot_service.py",
        name="screenshot_service"
    )
    
    
    ld.add_action(node_goal_manager)
    ld.add_action(node_redistribute_stream)
    ld.add_action(node_screenshot)

    return ld
