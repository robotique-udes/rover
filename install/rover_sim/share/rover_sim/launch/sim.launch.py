from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    ld = LaunchDescription()
    
    simulation_arg = DeclareLaunchArgument(
        'simulate_arm',
        default_value='false',
        description='Flag to enable arm simulation'
    )
    
    simulate_arm = LaunchConfiguration('simulate_arm')
 
    rover_sim_node = Node(
        package="rover_sim",
        executable="rover_sim",
        name="sim",
        condition=IfCondition(simulate_arm)
    )
    
    ld.add_action(simulation_arg)
    ld.add_action(rover_sim_node)

    return ld
