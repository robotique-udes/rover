import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    ld: LaunchDescription = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='false'))
    ld.add_action(DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'))


    # Lidar
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('unitree_lidar_ros2'),
                'launch',
                'launch.py',  # use your actual filename
            )
        ),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'port': LaunchConfiguration('lidar_port'),
        }.items(),
    )

    lidar_group = GroupAction([
        PushRosNamespace('/rover/lidar'),
        lidar_launch,
    ])

    ld.add_action(lidar_group)

    return ld