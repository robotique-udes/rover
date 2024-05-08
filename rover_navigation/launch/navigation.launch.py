import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld: LaunchDescription = LaunchDescription()

    node_lights_main = Node(package="rover_navigation",
                         namespace="/rover/auxiliarty",
                         executable="light_control",
                         name="light_controller")

    ld.add_action(node_lights_main)

    return LaunchDescription([
                              node_lights_main
                             ])
