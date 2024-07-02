import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    
    urdf_file = "robot.urdf"
    pkg_desc = "rover_sim"

    robot_desc_path = os.path.join(get_package_share_directory(pkg_desc), "rover_urdf", urdf_file)

    with open(robot_desc_path, 'r') as file:
        robot_desc_path = file.read()

    pkg_desc = "rover_sim"
    rviz_config_dir = os.path.join(get_package_share_directory(pkg_desc), 'rover_rviz', 'rover.rviz')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publiser_node',
        emulate_tty=True,
        parameters=[{'use_time_sim': True, 'robot_description': robot_desc_path}],
        output="screen"
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
        )
    
    sim_node = Node(
                package="rover_sim",
                namespace="/rover/arm",
                executable="simulation",
                name="simulation"
                    )
    encoder_sim_node = Node(
            package="rover_sim",
            namespace="/rover/arm",
            executable="encoder_simulation",
            name="encoder_simulation"
                )            

    return LaunchDescription(
        [
            robot_state_publisher_node,
            rviz_node,
            joint_state_publisher,
            sim_node,
            encoder_sim_node
        ]
    )