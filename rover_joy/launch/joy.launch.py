import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld: LaunchDescription = LaunchDescription()

# Si le controller n'est pas trouver, faire la commande  "ros2 run joy joy_node" et il va trouver le nom de la manette brancher
# Pour une manette de PS4 brancher, essayer "Sony Interactive Entertainment Wireless Controller"
# Pour une manette de PS4 bluethoot, le device_name est "Wireless Controller"

    node_joy_main = Node(package="joy",
                         namespace="/joy/main",
                         executable="joy_node",
                         name="joy_node",
                         parameters=[{"autorepeat_rate": 20.0,
                                      "device_name": "Wireless Controller"}],
                                    #   "device_name": "Sony Interactive Entertainment Wireless Controller"}],
                         remappings=[("joy", "raw")])

    node_joy_main_formator = Node(package="rover_joy",
                                  namespace="/joy/main",
                                  executable="joy_formator",
                                  name="joy_formator",
                                  parameters=[{"controller_type": "DS4"}],
                                  remappings=[("raw/joy", "raw"),
                                              ("formated/joy", "formated")])

    node_joy_secondary = Node(package="joy",
                              namespace="/joy/secondary",
                              executable="joy_node",
                              name="joy_node",
                              parameters=[{"autorepeat_rate": 20.0,
                                        #    "device_name": "Logitech RumblePad 2 USB"}],
                                           "device_name": "Sony Interactive Entertainment Wireless Controller"}],
                              remappings=[("joy", "raw")])

    node_joy_secondary_formator = Node(package="rover_joy",
                                       namespace="/joy/secondary",
                                       executable="joy_formator",
                                       name="joy_formator",
                                       parameters=[{"controller_type": "Logitech"}],
                                       remappings=[("raw/joy", "raw"),
                                                   ("formated/joy", "formated")])
    
    node_joy_demux = Node(package="rover_joy",
                          namespace="/joy",
                          executable="joy_demux",
                          name="joy_demux",
                          parameters=[{"controller_type": "Logitech"}],
                          remappings=[("main_joy", "main/formated"),
                                      ("secondary_joy", "secondary/formated")])

    ld.add_action(node_joy_main)
    ld.add_action(node_joy_secondary)
    ld.add_action(node_joy_main_formator)
    ld.add_action(node_joy_secondary_formator)
    ld.add_action(node_joy_demux)

    return LaunchDescription([
                              node_joy_main,
                              node_joy_secondary,
                              node_joy_main_formator,
                              node_joy_secondary_formator,
                              node_joy_demux
                             ])
