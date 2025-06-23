import os
from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():
        ld: LaunchDescription = LaunchDescription()
        
        goal_manager_node = Node(package="rover_autonomous",
                                        namespace="/rover/auto",
                                        executable="goal_manager_node",
                                        name="goal_manager",
                                        )
        
        ld.add_action(goal_manager_node)

        return ld
