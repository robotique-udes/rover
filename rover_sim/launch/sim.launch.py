import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
# from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
# from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = get_package_share_directory('rover_sim')

    description_file = os.path.join(pkg_name, 'model', 'new_rover.urdf')
    with open(description_file, 'r') as infp:
        robot_desc = infp.read()
        
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[robot_desc],
        output=['screen']
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        arguments=[
            {'use_sim_time': True},
            {'robot_description': robot_desc}
        ]
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_name, 'config', 'rover.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    return LaunchDescription(
        [
            DeclareLaunchArgument('rviz', default_value='true'),
            joint_state_publisher_gui,
            robot_state_publisher,
            rviz
        ]
    )
    
