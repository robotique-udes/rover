import threading
from rover_msgs.srv import PanoControl
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class PanoramaClient:
    def __init__(self, node):
        self.node = node
        self.cb_group = ReentrantCallbackGroup()
        self.pano_control_client = self.node.create_client(PanoControl, 'control_panorama', callback_group=self.cb_group)
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

    def send_request(self, request, callback):
        def internal_callback(future):
            try:
                response = future.result()
                if response is not None:
                    self.node.get_logger().info(f'{response.status_message}')
                    callback(True, response.status_message)
                else:
                    self.node.get_logger().error('Service call failed')
                    callback(False, 'Service call failed')
            except Exception as e:
                self.node.get_logger().error(f'Service call failed with exception: {str(e)}')
                callback(False, f'Service call failed: {str(e)}')

        future = self.pano_control_client.call_async(request)
        future.add_done_callback(internal_callback)

    def shutdown(self):
        self.executor.shutdown()