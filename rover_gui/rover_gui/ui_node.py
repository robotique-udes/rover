from rclpy.node import Node


class UINode(Node):
    def __init__(self):
        super().__init__('ui_node')