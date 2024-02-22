import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld: LaunchDescription = LaunchDescription()

    node_antenna_abtr = Node(package="rover_antenna",
                         namespace="/antenna/abtr",
                         executable="arbitration",
                         name="antenna_abtr")

    node_antenna_jog = Node(package="rover_antenna",
                                  namespace="/antenna/jog",
                                  executable="jog_antenna",
                                  name="jog_antenna",
                                  parameters=[{"max_speed": 3.14159/8}],
                                  #remappings=[]
                                  )

    node_antenna_auto = Node(package="rover_antenna",
                              namespace="/antenna/auto",
                              executable="autonomus",
                              name="autonomus",
                              parameters=[{"max_speed": 3.14159/8}],
                              #remappings=[]
                              )

    ld.add_action(node_antenna_abtr)
    ld.add_action(node_antenna_jog)
    ld.add_action(node_antenna_auto)

    return LaunchDescription([
                              node_antenna_abtr,
                              node_antenna_jog,
                              node_antenna_auto,
                             ])
