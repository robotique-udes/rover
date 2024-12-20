from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
        ld: LaunchDescription = LaunchDescription()
 
        node_heartbeat_rover = Node (package="rover_security",
                                    namespace="/rover",
                                    executable="heartbeat",
                                    name="heartbeat",
                                    parameters=[{"heartbeat_frequency": 4}],
                                    remappings=[("security", "raw")])
        
        ld.add_action(node_heartbeat_rover)

        return ld
