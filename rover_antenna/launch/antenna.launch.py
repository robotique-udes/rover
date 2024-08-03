import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld: LaunchDescription = LaunchDescription()

    node_antenna_abtr = Node(package="rover_antenna",
                         namespace="/base/antenna",
                         executable="arbitration",
                         name="arbitration")

    node_antenna_jog = Node(package="rover_antenna",
                                  namespace="/base/antenna",
                                  executable="joy_mode",
                                  name="teleop",
                                  parameters=[{"max_speed": 3.14159/8}],
                                  )

    node_antenna_auto = Node(package="rover_antenna",
                              namespace="/base/antenna",
                              executable="auto_mode",
                              name="autonomus",
                              parameters=[{"max_speed": 3.14159/8}],
                              )
    
    node_udp = Node(package="rover_antenna",
                              namespace="/base/antenna",
                              executable="udp",
                              name="udp",
                              )

    ld.add_action(node_antenna_abtr)
    ld.add_action(node_antenna_jog)
    ld.add_action(node_antenna_auto)
    ld.add_action(node_udp)

    return LaunchDescription([
                              node_antenna_abtr,
                              node_antenna_jog,
                              node_antenna_auto,
                              node_udp,
                             ])
