import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld: LaunchDescription = LaunchDescription()

    debug_printer = Node(package="rover_helper",
                           namespace="/debug",
                           executable="debug_printer",
                           name="debug_printer",
                           parameters=[{"debug_topic": "rovus_debug"}]
                           )

    ld.add_action(debug_printer)

    return LaunchDescription([debug_printer])