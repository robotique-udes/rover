from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
        ld: LaunchDescription = LaunchDescription()

        teleop_node = Node (package="rover_arm",
                                    namespace="/rover/arm",
                                    executable="teleop",
                                    name="teleop"
                                    )
        arm_simulation = Node(
            package="rover_arm",
            namespace="/rover/arm",
            executable="arm_sim.py",
            name="arm_simulation",
            output="screen",             
        )

        
        ld.add_action(teleop_node)
        ld.add_action(arm_simulation)

        return ld
