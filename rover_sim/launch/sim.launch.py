from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
        ld: LaunchDescription = LaunchDescription()

        sim_node = Node (package="rover_sim",
                                    namespace="/rover/arm",
                                    executable="sim.py",
                                    name="sim"
                                    )
        
        ld.add_action(sim_node)

        return ld
