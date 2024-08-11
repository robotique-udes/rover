import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

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
    
    #node_panorama = Node(
        #package="rover_auxiliary",
        #namespace="/rover/auxiliary",
        #executable="panorama_service.py",
        #name="panorama_service"
    #)

    node_screenshot = Node(
        package="rover_auxiliary",
        namespace="/rover/auxiliary",
        executable="screenshot_service.py",
        name="control_screenshot"
    )

    node_redistribute_stream = Node(
            package="rover_auxiliary",
            namespace="/rover/auxiliary",
            executable="redistribute_stream.py",
            name="redistribute_stream",
            parameters=[
                {"streamIP": "127.0.0.2"},
                {"streamPort": 69},
                {"serverPort1": 8554},  
                {"outputIP1": "192.168.144.198"},  
                {"serverPort2": 8555},  
                {"outputIP2": "192.168.144.199"},  
            ]
        )

    ld.add_action(node_lights_main)
    ld.add_action(node_compass_main)
    ld.add_action(node_redistribute_stream)
    #ld.add_action(node_panorama)
    ld.add_action(node_screenshot)

    return ld
