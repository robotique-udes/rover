import os
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


class UINode(Node):
    def __init__(self):
        super().__init__('ui_node')

    def get_resources_directory(self, package_name):
        package_share_directory = get_package_share_directory(package_name)
        resource_directory = package_share_directory +"/../../resource/"
        return resource_directory