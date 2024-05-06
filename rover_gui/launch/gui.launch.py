from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
        ld: LaunchDescription = LaunchDescription()
 
        node_heartbeat_rover = Node (package="rover_gui",
                                    executable="main_gui",
                                    name="gui")
        
        ld.add_action(node_heartbeat_rover)

        return ld
