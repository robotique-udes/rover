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
        unitree_l1_nnode = Node(package='rover_autonomous',
                                executable='unitree_L1',
                                name='unitree_L1',
                                output='screen',
                                parameters= [
                                        {'port': '/dev/ttyUSB0'},
                                        {'rotate_yaw_bias': 0.0},
                                        {'range_scale': 0.001},
                                        {'range_bias': 0.0},
                                        {'range_max': 50.0},
                                        {'range_min': 0.0},
                                        {'cloud_frame': "unilidar_lidar"},
                                        {'cloud_topic': "unilidar/cloud"},
                                        {'cloud_scan_num': 18},
                                        {'imu_frame': "unilidar_imu"},
                                        {'imu_topic': "unilidar/imu"}]
  )
        
        ld.add_action(goal_manager_node)
        ld.add_action(unitree_l1_nnode)

        return ld
