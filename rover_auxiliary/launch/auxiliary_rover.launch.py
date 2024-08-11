import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # node_aruco = Node(
    #     package="rover_auxiliary",
    #     namespace="/rover/auxiliary",
    #     executable="aruco_main.py",
    #     name="aruco"
    # )
    
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
    
    node_redistribute_stream = Node(
            package="rover_auxiliary",
            namespace="/rover/auxiliary",
            executable="redistribute_stream.py",
            name="redistribute_stream",
            parameters=[
                {"streamIP": "192.168.144.62"},
                {"streamPort": 69},
                {"serverPort1": 10001},  
                {"outputIP1": "192.168.144.21"},  
                {"serverPort2": 10002},  
                {"outputIP2": "192.168.144.22"},  
            ]
    )
    
    # ld.add_action(node_aruco)
    ld.add_action(node_compass_calibrator)
    ld.add_action(node_light_control)
    ld.add_action(node_redistribute_stream)

    return ld
