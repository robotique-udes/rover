import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld: LaunchDescription = LaunchDescription()

    node_lights_main = Node(package="rover_navigation",
                         namespace="/rover/auxiliary",
                         executable="light_control",
                         name="light_controller")
    
    node_compass_main = Node(package="rover_navigation",
                        namespace="/rover/auxiliary",
                        executable="compass_calibrator",
                        name="compass_calibrator")
    
    node_panorama = Node(package="rover_navigation",
                        namespace="/rover/auxiliary",
                        executable="panorama",
                        name="panorama")

    ld.add_action(node_lights_main)
    ld.add_action(node_compass_main)
    ld.add_action(node_panorama)

    return LaunchDescription([
                              node_lights_main,
                              node_compass_main,
                              node_panorama
                             ])
