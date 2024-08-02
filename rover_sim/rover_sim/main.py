import rclpy
from rclpy.node import Node
from rover_msgs.msg import ArmCmd
import math as m
from math import pi as PI
import numpy as np
import matplotlib.pyplot as plt

class SimulationStatus(Node):
    
    def __init__(self):
        super().__init__("simulation_status")
        self.simulation_staus = self.create_subscription(
            ArmCmd, "/rover/arm/cmd/in/teleop", self.simulationCallback, 10)
        
    def simulationCallback(self, ArmMsg: ArmCmd):
        self.get_logger().info(str(ArmMsg))
        
        self.JL_pos = ArmMsg.position[ArmCmd.JL]
        self.J0_pos = ArmMsg.position[ArmCmd.J0]
        self.J1_pos = ArmMsg.position[ArmCmd.J1]
        self.J2_pos = ArmMsg.position[ArmCmd.J2]
        self.GripperTilt_pos = ArmMsg.position[ArmCmd.GRIPPERTILT]
        
        
        
def main(args=None):
    rclpy.init(args=args)
    node = SimulationStatus()
    rclpy.spin(node)
    rclpy.shutdown()