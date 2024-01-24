import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
        ld: LaunchDescription = LaunchDescription()

        node_heartbeat_base = Node (package="rover_security",
                                    namespace="/base",
                                    executable="heartbeat",
                                    name="base_heartbeat",
                                    parameters=[{"heartbeat_frequency": 2}],
                                    remappings=[("security", "raw")])
        
        node_heartbeat_base.cmd
        
        node_heartbeat_rover = Node (package="rover_security",
                                    namespace="/rover",
                                    executable="heartbeat",
                                    name="rover_heartbeat",
                                    parameters=[{"heartbeat_frequency": 2}],
                                    remappings=[("security", "raw")])
        
        ld.add_action(node_heartbeat_base)
        ld.add_action(node_heartbeat_rover)

        return ld
