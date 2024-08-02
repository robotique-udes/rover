import rclpy
from rclpy.node import Node
from rover_msgs.msg import ArmCmd


class SimulationStatus(Node):
    
    def __init__(self):
        super().__init__("simulation_status")
        self.simulation_staus = self.create_subscription(
            ArmCmd, "/rover/arm/cmd/in/teleop", self.simulationCallback, 10)
        
    def simulationCallback(self, ArmMsg: ArmCmd):
        self.get_logger().info(str(ArmMsg))

def main(args=None):
    rclpy.init(args=args)
    node = SimulationStatus()
    rclpy.spin(node)
    rclpy.shutdown()