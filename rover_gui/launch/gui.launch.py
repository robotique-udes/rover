import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():

  use_valgrind = LaunchConfiguration("use_valgrind")

  ld = LaunchDescription([
      DeclareLaunchArgument("use_valgrind", default_value="false"),
      Node(
          package="rover_gui",
          namespace="base/gui",
          executable="main_gui",
          name="main_gui",
          prefix=PythonExpression([
              "'valgrind --leak-check=full --log-file=/tmp/main_gui.log' if '",
              use_valgrind, "' == 'true' else ''"
          ]),
      ),
  ])
  return ld
