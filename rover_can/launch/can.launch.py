import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld: LaunchDescription = LaunchDescription()

    node_joy_main = Node(package="rover_can",
                         namespace="/rover/canbus",
                         executable="can_master",
                         name="can_master")

    ld.add_action(node_joy_main)

    return LaunchDescription([
                              node_joy_main
                             ])
