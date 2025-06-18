from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    arm_arg = DeclareLaunchArgument(
        "arm",
        default_value="true",
        description="Whether to launch the arm_sim node"
    )

    gps_arg = DeclareLaunchArgument(
        "gps",
        default_value="true",
        description="Whether to launch the gps_test node"
    )

    GNSS_arg = DeclareLaunchArgument(
    "gps",
    default_value="true",
    description="Whether to launch the gps_test node"
    )

    battery_arg = DeclareLaunchArgument(
    "gps",
    default_value="true",
    description="Whether to launch the gps_test node"
    )

    arm = LaunchConfiguration("arm")
    gps = LaunchConfiguration("gps")
    GNSS_arg = LaunchConfiguration("gnss")
    battery_arg = LaunchConfiguration(" battery")


    '''
    arm_sim_node = Node(
        package="rover_sim",
        namespace="/rover/sim",
        executable="arm_sim.py",
        name="arm_sim",
        condition=IfCondition(arm)
    )
    '''

    gps_sim_node = Node(
        package="rover_sim",
        namespace="/rover/sim",
        executable="gps_test",
        name="gps_test",
        condition=IfCondition(gps)
    )

    battery_sim_node = Node(
        package="rover_sim",
        namespace="/rover/sim",
        executable="battery_test",
        name="battery_test"
    )

    connection_speed_sim_node = Node(
        package="rover_sim",
        namespace="/rover/sim",
        executable="connection_speed_test",
        name="connection_speed_test"
    )

    ld = LaunchDescription()
    ld.add_action(arm_arg)
    ld.add_action(gps_arg)
    #ld.add_action(arm_sim_node)
    ld.add_action(gps_sim_node)
    ld.add_action(battery_sim_node)
    ld.add_action(connection_speed_sim_node)

    return ld
